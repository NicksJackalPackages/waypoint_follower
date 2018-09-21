# waypoint_follower

This package provides waypoint following in ROS. Sends target waypoints to move_base and displays the waypoints in RVIZ.

## 1. Nodes
### 1.1 waypoint_follower2_node
#### 1.1.1 Subscribed Topics

`set_waypoints` ([`waypoint_follower_msgs/WaypointArray`](waypoint_follower_msgs/msg/WaypointArray.msg))

Clears the waypoint list, then sets this as new waypoints.

#### 1.1.2 Published Topics

`jackal_velocity_controller/cmd_vel` ([`geometry_msgs/Twist`](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

The command velocity to be sent to the robot.

`waypoint_paths` ([`nav_msgs/Path`](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

The incomplete waypoints.

#### 1.1.3 Parameters

`~baselink_frame` (`string`, default: `"base_link"`)

The `base_link` frame of the robot. This is only used if `include_robot_path` is true.

`~odom_frame` (`string`, default: `"odom"`)

The `odom` frame of the robot.

`~map_frame` (`string`, default: `"map"`)

The `map` frame of the robot.

`~launch_frame` (`string`, default: `odom_frame`)

The frame of waypoints specified in the launch file.

`~include_start_pose` ('bool`, default: `true`)

Whether to include the current position of the robot as part of the waypoint path.

`~waypoints` (`array`, default: `[]`)

Sets waypoints. In the form position(x, y, z) followed by orientation (x, y, z, w).

`~waypoint_order` (`array`, default: `[1,2,...]`)

The order to complete waypoints given by the parameter `waypoints`.

`~gps_waypoints` (`array`, default: `[]`)

Sets waypoints. In the form position(lat, long). Requires the UTM transform provided by robot_localization's navsat_transform_node.

`~gps_waypoint_order` (`array`, default: `[1,2,...]`)

The order to complete waypoints given by the parameter `gps_waypoints`.

`~lin_vel` (`double`, default: `0.5`)

The linear velocity of the robot.

`~ang_vel` (`double`, default: `0.5`)

The maximum angular velocity of the robot.

`~dist_req` (`double`, default: `1`)

The distance in metres required for waypoint to be deemed completed.

`~start_delay` (`double`, default: `0`)

The time in seconds to wait before moving.
