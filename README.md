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
