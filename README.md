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

- **`prefix`**
  - **Description**: Prefix used for naming the nodes.
  - **Default**: `gps_base`

- **`output_frame_id`**
  - **Description**: Frame ID for the output data.
  - **Default**: `gps_base_link`

- **`apply_dual_antenna_offset`**
  - **Description**: Boolean flag to apply dual antenna offset.
  - **Default**: `False`

- **`heading_offset`**
  - **Description**: Heading offset value.
  - **Default**: `0.0`

- **`gps_main_frame_id`**
  - **Description**: Frame ID for the main GPS antenna.
  - **Default**: `back_antenna_link`

- **`gps_aux_frame_id`**
  - **Description**: Frame ID for the auxiliary GPS antenna.
  - **Default**: `front_antenna_link`

- **`use_sim_time`**
  - **Description**: Use simulation time.
  - **Default**: `False`
