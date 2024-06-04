#!/usr/bin/env python
"""
This script parses and converts Trimble GSOF messages incoming from a receiver and publishes the relevant ROS messages.
It  has been adapted from https://kb.unavco.org/kb/article/trimble-netr9-receiver-gsof-messages-806.html

@author: Michael Hutchinson (mhutchinson@sagarobotics.com)
@auther: Geesara (ggeesara@gmail.com)
"""

from struct import unpack
import rclpy.duration
import rclpy.time
from trimble_gnss_driver_ros2.scripts.parser import parse_maps
from trimble_gnss_driver_ros2.scripts.gps_qualities import gps_qualities
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy, QoSDurabilityPolicy
import socket
import sys
import math
import rclpy
import math

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu # For lat lon h

"""
tf2_ros API: https://docs.ros.org/en/kinetic/api/tf2_ros/html/python/
"""
from tf2_ros import TransformException
from tf2_ros import BufferInterface
from tf2_ros.buffer import Buffer
from tf2_ros.buffer_client import BufferClient
from tf2_ros.transform_listener import TransformListener

from rclpy.node import Node
from rclpy.parameter import Parameter 
"""
GSOF messages from https://www.trimble.com/OEM_ReceiverHelp/#GSOFmessages_Overview.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257COverview%257C_____0
"""
# Records we most care about
ATTITUDE = 27              # Attitude information with errors
LAT_LON_H = 2              # Position lat lon h
POSITION_SIGMA = 12        # Errors in position
POSITION_TYPE = 38
BASE_POSITION_QUALITY = 41 # Needed for gps quality indicator
INS_FULL_NAV = 49          # INS fused full nav info pose, attittude etc
INS_RMS = 50               # RMS errors from reported fused position
RECEIVED_BASE_INFO = 35    # Received base information

# Others (but still not the entire list of GSOF msgs available.)
VELOCITY = 8
SERIAL_NUM = 15
GPS_TIME = 1
UTC_TIME = 16
ECEF_POS = 3
LOCAL_DATUM = 4
LOCAL_ENU= 5


class GSOFDriver(Node):
    """ A class to parse GSOF messages from a TCP stream. """

    def __init__(self):

        super().__init__('gsof_driver') 

        self.declare_parameter('rtk_port'                   , rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('rtk_ip'                     , rclpy.Parameter.Type.STRING) 
        self.declare_parameter('output_frame_id'            , rclpy.Parameter.Type.STRING) 
        self.declare_parameter('apply_dual_antenna_offset'  , rclpy.Parameter.Type.BOOL) 
        self.declare_parameter('gps_main_frame_id'          , rclpy.Parameter.Type.STRING) 
        self.declare_parameter('gps_aux_frame_id'           , rclpy.Parameter.Type.STRING) 
        self.declare_parameter('heading_offset'             , rclpy.Parameter.Type.DOUBLE) 
        self.declare_parameter("prefix"                     , rclpy.Parameter.Type.STRING)
       
        self.rtk_port = self.get_parameter_or("rtk_port", Parameter('int', Parameter.Type.INTEGER, 21098)).value
        self.rtk_ip = self.get_parameter_or("rtk_ip", Parameter('str', Parameter.Type.STRING, "192.168.0.50")).value 
        self.output_frame_id = self.get_parameter_or("output_frame_id", Parameter('str', Parameter.Type.STRING, "base_link")).value
        self.apply_dual_antenna_offset = self.get_parameter_or("apply_dual_antenna_offset", Parameter('bool', Parameter.Type.BOOL, False)).value
        self.prefix = self.get_parameter_or("prefix", Parameter('str', Parameter.Type.STRING, "gps")).value
        self.gps_main_frame_id = self.get_parameter_or("gps_main_frame_id", Parameter('str', Parameter.Type.STRING, "back_antenna_link")).value
        self.gps_aux_frame_id = self.get_parameter_or("gps_aux_frame_id", Parameter('str', Parameter.Type.STRING, "front_antenna_link")).value
        self.heading_offset = self.get_parameter_or("heading_offset", Parameter('double', Parameter.Type.DOUBLE, 0.0 )).value
        
        if self.apply_dual_antenna_offset == False:
            self.heading_offset = 0

        self.get_logger().info("Heading offset is {}".format(self.heading_offset) )

        self.latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", qos_profile=self.latching_qos)
        # For attitude, use IMU msg to keep compatible with robot_localization
        # But note that this is not only from an IMU
        self.attitude_pub = self.create_publisher(Imu,  "/attitude", qos_profile=self.latching_qos)
        # yaw from the dual antennas fills an Imu msg simply to keep consistent
        # with the current setup
        # Keep separate from attitude to avoid accidentally fusing zeros when
        # we don't measure roll/pitch
        self.yaw_pub = self.create_publisher(Imu, '/yaw', qos_profile=self.latching_qos)

        self.client = self.setup_connection(self.rtk_ip , self.rtk_port)

        self.msg_dict = {}
        self.msg_bytes = None
        self.checksum = True
        self.rec_dict = {}

        self.current_time = self.get_clock().now()
        self.ins_rms_ts = 0
        self.pos_sigma_ts = 0
        self.quality_ts = 0
        self.error_info_timeout = 1.0
        self.base_info_timeout = 5.0
        
       
        while rclpy.ok():
            # READ GSOF STREAM
            self.records = []
            self.current_time = self.get_clock().now()
            self.get_message_header()
            self.get_records()

            if not self.checksum:
                self.get_logger().warn("Invalid checksum, skipping")
                continue

            # Make sure we have the information required to publish msgs and
            # that its not too old
            if INS_FULL_NAV in self.records:
                if INS_RMS in self.records or self.current_time - self.ins_rms_ts < self.error_info_timeout:
                    # print "Full INS info, filling ROS messages"
                    self.send_ins_fix()
                    self.send_ins_attitude()
                else:
                    self.get_logger().warn("Skipping INS output as no matching errors within the timeout. Current time: {}, last error msg {}".format(self.current_time, self.ins_rms_ts))
            else:
                if LAT_LON_H in self.records:
                    if (POSITION_SIGMA in self.records or self.current_time - self.pos_sigma_ts < self.error_info_timeout) and (POSITION_TYPE in self.records or self.current_time - self.quality_ts < self.base_info_timeout):
                        self.send_fix()
                    else:
                        self.get_logger().warn("Skipping fix output as no corresponding sigma errors or gps quality within the timeout. Current time: {}, last sigma msg {}, last gps quality msg {}".format(self.current_time, self.pos_sigma_ts, self.quality_ts))
                if ATTITUDE in self.records:
                    self.send_yaw()

            # if RECEIVED_BASE_INFO in self.records or LOCAL_DATUM in self.records or LOCAL_ENU in self.records:
            #     print(self.rec_dict)

                # print("Base Info: \n", self.rec_dict['BASE_NAME_1'], self.rec_dict['BASE_ID_1'], self.rec_dict['BASE_LATITUDE'], self.rec_dict['BASE_LONGITUDE'], self.rec_dict['BASE_HEIGHT'])
            # if INS_FULL_NAV in self.records and LAT_LON_H in self.records:
            #     print("Altitude INS: ", self.rec_dict['FUSED_ALTITUDE'], "Height LLH (WGS84): ", self.rec_dict['HEIGHT_WGS84'])
                    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [w, x, y, z]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def send_ins_fix(self):
        if self.rec_dict['FUSED_LATITUDE'] == 0 and self.rec_dict['FUSED_LONGITUDE']  == 0 and self.rec_dict['FUSED_ALTITUDE'] == 0:
            self.get_logger().warn("Invalid fix, skipping")
            return
        current_time = self.get_clock().now() # Replace with GPS time?
        fix = NavSatFix()

        fix.header.stamp = current_time.to_msg()
        fix.header.frame_id = self.output_frame_id

        gps_qual = gps_qualities[self.rec_dict['GPS_QUALITY']]
        fix.status.service = NavSatStatus.SERVICE_GPS # TODO: Fill correctly
        fix.status.status = gps_qual[0]
        fix.position_covariance_type = gps_qual[1]

        fix.latitude = self.rec_dict['FUSED_LATITUDE']
        fix.longitude = self.rec_dict['FUSED_LONGITUDE']

        # To follow convention set in the NavSatFix definition, altitude should be:
        # Altitude [m]. Positive is above the WGS 84 ellipsoid
        # Ref - http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

        fix.altitude = self.rec_dict['FUSED_ALTITUDE']  # <-- CHECK

        fix.position_covariance[0] = self.rec_dict['FUSED_RMS_LONGITUDE'] ** 2
        fix.position_covariance[4] = self.rec_dict['FUSED_RMS_LATITUDE'] ** 2
        fix.position_covariance[8] = self.rec_dict['FUSED_RMS_ALTITUDE'] ** 2

        self.fix_pub.publish(fix)


    def send_ins_attitude(self):
        """
        We send the GNSS fused attitude information as an IMU msg to
        keep compatible with the robot_localization package. Although it should
        be noted that this is not just from an IMU but fused with dual gnss
        antennas information.
        """

        if self.rec_dict['FUSED_ROLL'] == 0 and self.rec_dict['FUSED_PITCH']  == 0 and self.rec_dict['FUSED_YAW'] == 0:
            self.get_logger().warn("Invalid yaw, skipping")
            return

        current_time = self.get_clock().now() # Replace with GPS time?

        # print "current time: ", current_time, "INS time: ", self.ins_rms_ts
        attitude = Imu()

        attitude.header.stamp = current_time.to_msg()
        attitude.header.frame_id = self.output_frame_id  # Assume transformation handled by receiver

        heading_enu = 2*math.pi - self.normalize_angle(math.radians(self.rec_dict['FUSED_YAW']) + 3*math.pi/2)
        orie_quat = self.quaternion_from_euler(math.radians(self.rec_dict['FUSED_ROLL']),     #  roll sign stays the same
                                             - math.radians(self.rec_dict['FUSED_PITCH']),  # -ve for robots coord system (+ve down)
                                             heading_enu)
        # print 'r p y_enu receiver_heading [degs]: ', self.rec_dict['FUSED_ROLL'], self.rec_dict['FUSED_PITCH'], math.degrees(heading_enu), self.rec_dict['FUSED_YAW']

        attitude.orientation = Quaternion(x=orie_quat[1], y=orie_quat[2], z=orie_quat[3], w=orie_quat[0])

        attitude.orientation_covariance[0] = math.radians(self.rec_dict['FUSED_RMS_ROLL']) ** 2  # [36] size array
        attitude.orientation_covariance[4] = math.radians(self.rec_dict['FUSED_RMS_PITCH']) ** 2  # [36] size array
        attitude.orientation_covariance[8] = math.radians(self.rec_dict['FUSED_RMS_YAW']) ** 2 # [36] size array

        self.attitude_pub.publish(attitude)


    def send_fix(self):
        if self.rec_dict['LATITUDE'] == 0 and self.rec_dict['LONGITUDE']  == 0 and self.rec_dict['HEIGHT_WGS84'] == 0:
            self.get_logger().warn("Invalid fix, skipping")
            return

        current_time = self.get_clock().now() # Replace with GPS time?
        fix = NavSatFix()

        fix.header.stamp = current_time.to_msg()
        fix.header.frame_id = self.output_frame_id
        gps_qual = self.get_gps_quality()
        fix.status.service = NavSatStatus.SERVICE_GPS # TODO: Fill correctly
        fix.status.status = gps_qual[0]
        fix.position_covariance_type = gps_qual[1]

        fix.latitude = math.degrees(self.rec_dict['LATITUDE'])
        fix.longitude = math.degrees(self.rec_dict['LONGITUDE'])

        # To follow convention set in the NavSatFix definition, altitude should be:
        # Altitude [m]. Positive is above the WGS 84 ellipsoid
        # Ref - http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

        fix.altitude = self.rec_dict['HEIGHT_WGS84']  # <-- CHECK

        fix.position_covariance[0] = self.rec_dict['SIG_EAST'] ** 2  # Check east north order
        fix.position_covariance[4] = self.rec_dict['SIG_NORT'] ** 2
        fix.position_covariance[8] = self.rec_dict['SIG_UP'] ** 2

        self.fix_pub.publish(fix)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


    def send_yaw(self):
        """
        We send yaw (without ins) as an IMU message for compatibility with our
        other software
        """
        if self.rec_dict['ROLL'] == 0 and self.rec_dict['PITCH']  == 0 and self.rec_dict['YAW'] == 0:
            self.get_logger().warn("Invalid yaw, skipping")
            return

        current_time = self.get_clock().now() # Replace with GPS time?

        # print "current time: ", current_time, "INS time: ", self.ins_rms_ts
        yaw = Imu()

        yaw.header.stamp = current_time.to_msg()
        yaw.header.frame_id = self.output_frame_id  # Assume transformation handled by receiver

        heading_ned = self.normalize_angle(self.rec_dict['YAW'] + self.heading_offset)
        heading_enu = 2*math.pi - self.normalize_angle(heading_ned + 3*math.pi/2)

        orie_quat = self.quaternion_from_euler(self.rec_dict['ROLL'],     #  roll sign stays the same
                                             - self.rec_dict['PITCH'],  # -ve for robots coord system (+ve down)
                                             heading_enu)
        # print 'r p y receiver_heading [rads]: ', self.rec_dict['ROLL'], self.rec_dict['PITCH'], heading_enu, self.rec_dict['YAW']

        yaw.orientation = Quaternion(x=orie_quat[1], y=orie_quat[2], z=orie_quat[3], w=orie_quat[0])

        yaw.orientation_covariance[0] = self.rec_dict['ROLL_VAR']  # [9]
        yaw.orientation_covariance[4] = self.rec_dict['PITCH_VAR']  # [9]
        yaw.orientation_covariance[8] = self.rec_dict['YAW_VAR'] # [9]

        self.yaw_pub.publish(yaw)


    def get_gps_quality(self):
        """Get ROS NavSatStatus position type from trimbles
        """
        trimble_position_type = self.rec_dict['POSITION_TYPE']

        if trimble_position_type >= 9:
            position_type = 4 # fix
        elif trimble_position_type >= 7:
            position_type = 5 # float
        elif trimble_position_type >= 1:
            position_type = 2
        else:
            position_type = 0

        return gps_qualities[position_type]


    @staticmethod
    def normalize_angle(angle_in):
        while angle_in > 2*math.pi:
            angle_in = angle_in - 2*math.pi
        while angle_in < 0:
            angle_in = angle_in + 2*math.pi
        return angle_in

    def setup_connection(self, _ip, _port):
        self.rtk_port = _port
        self.rtk_ip = None
        attempts_limit = 10
        current_attempt = 0
        connected = False

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while not connected and current_attempt < attempts_limit:
            current_attempt += 1
            try:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5)
                self.rtk_ip = socket.gethostbyname(_ip)
                address = (self.rtk_ip, self.rtk_port)
                self.get_logger().info("Attempting connection to {}:{} ".format(self.rtk_ip, self.rtk_port))
                client.connect(address)
                self.get_logger().info("=====================================")
                self.get_logger().info("Connected to {}:{} ".format(self.rtk_ip, self.rtk_port))
                self.get_logger().info("=====================================")
                connected = True
            except Exception as e:
                self.get_logger().warn("Connection to IP: " + self.rtk_ip + ": " + e.__str__() +
                              ".\nRetrying connection: Attempt: {}/{}".format(current_attempt, attempts_limit))

        if not connected:
            self.get_logger().error("No connection established. Node shutting down")
            sys.exit()

        return client

    def read_from_client(self, num_bytes):
        num_read_bytes = 0
        read_bytes = b''
        while num_read_bytes != num_bytes and rclpy.ok():
            if num_read_bytes:
                # rospy.logwarn_throttle(10, "Significant network latency detected. Enable DEBUG for more details.")
                self.get_logger().warn('Significant network latency detected. Enable DEBUG for more details.', throttle_duration_sec=1)
                self.get_logger().debug("Only received {}/{} bytes.".format(num_read_bytes, num_bytes))
            try:
                new_bytes = self.client.recv(num_bytes-num_read_bytes)
            except socket.timeout as timeout:
                self.get_logger().warn("Socket timed out during read: {}".format(timeout))
                continue
            read_bytes += new_bytes
            num_read_bytes += len(new_bytes)
        return read_bytes

    def get_message_header(self):
        data = self.read_from_client(7)
        msg_field_names = ('STX', 'STATUS', 'TYPE', 'LENGTH',
                           'T_NUM', 'PAGE_INDEX', 'MAX_PAGE_INDEX')
        self.msg_dict = dict(zip(msg_field_names, unpack('>7B', data)))
        # print "msg dict: ", self.msg_dict
        self.msg_bytes = self.read_from_client(self.msg_dict['LENGTH'] - 3)
        (checksum, etx) = unpack('>2B', self.read_from_client(2))

        if checksum-self.checksum256(self.msg_bytes+data[1:]) == 0:
            self.checksum = True
        else:
            self.checksum = False
        # print("Checksum: ", self.checksum)


    def checksum256(self, st):
        """Calculate checksum"""
        if sys.version_info[0] >= 3:
            return sum(st) % 256
        else:
            return reduce(lambda x, y: x+y, map(ord, st)) % 256


    def get_records(self):
        self.byte_position = 0
        while self.byte_position < len(self.msg_bytes):
            # READ THE FIRST TWO BYTES FROM RECORD HEADER
            record_type, record_length = unpack('>2B', self.msg_bytes[self.byte_position:self.byte_position + 2])
            self.byte_position += 2
            self.records.append(record_type)
            # print "Record type: ", record_type, " Length: ", record_length
            # self.select_record(record_type, record_length)
            if record_type in parse_maps:
                self.rec_dict.update(dict(zip(parse_maps[record_type][0], unpack(parse_maps[record_type][1], self.msg_bytes[self.byte_position:self.byte_position + record_length]))))
            else:
                self.get_logger().warn("Record type {} is not in parse maps.".format(record_type))
            self.byte_position += record_length

        if INS_RMS in self.records:
            self.ins_rms_ts = self.current_time
        if POSITION_SIGMA in self.records:
            self.pos_sigma_ts = self.current_time
        if POSITION_TYPE in self.records:
            self.quality_ts = self.current_time

def main(args=None): 

    rclpy.init(args=args) 
    gsof_driver = GSOFDriver() 
    rclpy.spin(gsof_driver) 

    gsof_driver.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()
