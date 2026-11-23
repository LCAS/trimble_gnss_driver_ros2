# Trimble RTK-GNSS driver for ROS2 


## Configuration Parameters

The following are the configuration parameters available for the Trimble GNSS Driver ROS 2 package. Each parameter can be set through the launch file with its corresponding default value.

### Parameters

- **`rtk_ip`**
  - **Description**: IP address of the RTK GNSS receiver.
  - **Default**: `192.168.0.50`

- **`rtk_port`**
  - **Description**: Port number for the RTK GNSS receiver.
  - **Default**: `28009`

- **`output_frame_id`**
  - **Description**: Frame ID for the output data.
  - **Default**: `gps_base_link`

- **`apply_dual_antenna_offset`**
  - **Description**: Boolean flag to apply dual antenna offset.
  - **Default**: `False`

- **`heading_offset`**
  - **Description**: Heading offset value.
  - **Default**: `0.0`

- **`use_sim_time`**
  - **Description**: Use simulation time.
  - **Default**: `False`

- **`covariance_threshold`**
  - **Description**: Variance threshold used to filter high-covariance fixes when high-covariance data is not permitted.
  - **Default**: `0.2`

- **`permit_high_covariance_data`**
  - **Description**: Publish fixes even when position covariance exceeds `covariance_threshold`.
  - **Default**: `True`

- **`use_gnss_time`**
  - **Description**: Stamp `/fix` with the receiver's measurement epoch (GSOF record 1, *Position Time*) and `/yaw` with the attitude record's own time, instead of the arrival time. Enable record 1 on the receiver's GSOF port and keep the host clock synchronised to UTC (e.g. chrony). Messages without an epoch, or whose epoch differs from the host clock by more than `max_gnss_time_offset`, are stamped with the arrival time and a warning is logged. The driver logs the arrival − epoch latency every 600 messages.
  - **Default**: `True`

- **`gps_leap_seconds`**
  - **Description**: GPS − UTC leap seconds used to convert GPS time to UTC.
  - **Default**: `18`

- **`max_gnss_time_offset`**
  - **Description**: Largest accepted |host clock − GNSS epoch| in seconds before falling back to the arrival time.
  - **Default**: `0.5`

The driver reconnects when the receiver closes the GSOF stream (for example after its port configuration changes), and records longer than their parse map (optional trailing fields) are parsed by their known prefix instead of stopping the driver.
