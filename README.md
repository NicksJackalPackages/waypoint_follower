# waypoint_follower

This package provides waypoint following in ROS. Sends target waypoints to move_base and displays the waypoints in RVIZ. Waypoints can be added dynamically (like through RVIZ), or statically in the launch file. 

The position of waypoints can be dynamically updated e.g. if a dynamically added
waypoint has the same ID as a previously added waypoint, the waypoint will be
updated.

The waypoint is deemed completed when move_base sends a succeed message.

Each waypoint can have a different frame, but must have a transformation to the odom frame in order to be sent to move_base.

## 1. Nodes
### 1.1 waypoint_follower_node
#### 1.1.1 Subscribed Topics

`add_waypoint` ([`geometry_msgs/PoseStamped`](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

Adds a single waypoint to the waypoint list. Designed for the output of RVIZ '2D Nav Goal'. Uses a frame of `~rviz_frame`.

`set_waypoints` ([`waypoint_follower_msgs/WaypointArray`](waypoint_follower_msgs/msg/WaypointArray.msg))

Clears the waypoint list, then sets this as new waypoints.

`update_waypoints` ([`waypoint_follower_msgs/WaypointArray`](waypoint_follower_msgs/msg/WaypointArray.msg))

Adds each waypoint to the list. If the waypoint already exists in the list, its values are updated. If a waypoint has a new ID, it is added to the end of the list. 

`observed_waypoints` ([`waypoint_follower_msgs/WaypointArray`](waypoint_follower_msgs/msg/WaypointArray.msg))

Updates each waypoint in the list. If the waypoint already exists in the list, its values are updated. If a waypoint has a new ID, it is ignored. 

`move_base/result` ([`move_base_msgs/MoveBaseActionResult`](http://docs.ros.org/fuerte/api/move_base_msgs/html/msg/MoveBaseActionResult.html))

Reports when a waypoint has been reached.

#### 1.1.2 Published Topics

`waypoint_paths` ([`nav_msgs/Path`](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))

The incomplete waypoints. Published whenever the waypoint list is updated. Uses the frame `~rviz_frame`.

#### 1.1.3 Parameters

`~move_base_name` (`string`, default: `"move_base"`)

The `move_base` client service name.

`~baselink_frame` (`string`, default: `"base_link"`)

The `base_link` frame of the robot. This is only used if `include_robot_path` is true.

`~odom_frame` (`string`, default: `"odom"`)

The `odom` frame of the robot.

`~rviz_frame` (`string`, default: `~odom_frame`)

The frame of the subscribed `add_waypoint` topic and published `waypoint_paths` topic.

`~waypoints` (`array`, default: `[]`)

Sets waypoints. In the form position(x, y, z) followed by orientation (x, y, z, w).

`~waypoints_frame` (`string`, default: `~odom_frame`)

The frame of waypoints given by the parameter `waypoints`.

`~include_robot_path` (`bool`, default: `false`)

Whether to display the path from the robots `base_link` to the first waypoint. 

## 2. TO-DO
- Option to sit (or spin) at waypoint until an external succeed message is also sent
