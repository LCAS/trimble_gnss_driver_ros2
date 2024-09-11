# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: Geesara
# @email: ggeesara@gmail.com
# @date:
# ----------------------------------

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    rtk_ip = LaunchConfiguration('rtk_ip', default='192.168.5.50')
    rtk_port = LaunchConfiguration('rtk_port', default='28009')
    prefix = LaunchConfiguration('prefix', default='gps_base')
    output_frame_id = LaunchConfiguration('output_frame_id', default='gps_base_link')
    apply_dual_antenna_offset = LaunchConfiguration('apply_dual_antenna_offset', default='False')
    heading_offset = LaunchConfiguration('heading_offset', default='0.0')
    gps_main_frame_id = LaunchConfiguration('gps_main_frame_id', default='back_antenna_link')
    gps_aux_frame_id = LaunchConfiguration('gps_aux_frame_id', default='front_antenna_link')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    covariance_threshold = LaunchConfiguration('covariance_threshold', default='0.2')
    permit_high_covariance_data = LaunchConfiguration('permit_high_covariance_data', default='True')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'rtk_ip', default_value=rtk_ip, description='IP address of RTK'),
        DeclareLaunchArgument(
            'rtk_port', default_value=rtk_port, description='RTK port'),
        DeclareLaunchArgument(
            'prefix', default_value=prefix, description='node name'),
        DeclareLaunchArgument(
            'output_frame_id', default_value=output_frame_id, description='Output frame id'),
        DeclareLaunchArgument(
            'apply_dual_antenna_offset', default_value=apply_dual_antenna_offset,
            description='Apply dual antenna offset'),
        DeclareLaunchArgument(
            'heading_offset', default_value=heading_offset,
            description='GPS antennas headding correction offset in radians w.r.t the main antenna'),
        DeclareLaunchArgument(
            'covariance_threshold', default_value=covariance_threshold,
            description='This is the variance threshold of the position'),
        DeclareLaunchArgument(
            'permit_high_covariance_data', default_value=permit_high_covariance_data,
            description='Allow publishing even if the covariance get out of bound'),
        DeclareLaunchArgument(
            'gps_main_frame_id', default_value=gps_main_frame_id, description='GPS main frame id'),
        DeclareLaunchArgument(
            'gps_aux_frame_id', default_value=gps_aux_frame_id,
            description='gps_aux_frame_id'),
        Node(
            package='trimble_gnss_driver_ros2',
            executable='gsof_driver',
            output='screen',
            parameters=[{
                'rtk_ip': rtk_ip,
                'rtk_port': rtk_port,
                'prefix':prefix,
                'output_frame_id':output_frame_id,
                'apply_dual_antenna_offset':apply_dual_antenna_offset,
                'gps_main_frame_id': gps_main_frame_id,
                'gps_aux_frame_id': gps_aux_frame_id,
                'use_sim_time': use_sim_time,
                'heading_offset': heading_offset,
                'covariance_threshold': covariance_threshold,
                'permit_high_covariance_data': permit_high_covariance_data,
            }],
            remappings=[("/fix", "/gps_base/fix"), ("/yaw", "/gps_base/yaw"),
                        ("/attitude", "/gps_base/attitude")],
        ),
    ])


