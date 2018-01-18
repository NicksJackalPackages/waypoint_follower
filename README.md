# waypoint_follower

This package provides waypoint following in ROS. Sends target waypoints to 
move_base. Waypoints can be added dynamically (like through RVIZ), or statically
in the launch file. 

The position of waypoints can be dynamically updated e.g. if a dynamically added
waypoint has the same ID as a previously added waypoint, the waypoint will be
updated.

The waypoint is deemed completed when move_base sends a succeed message.

# TO-DO
- Test installation and write up
- Option to sit (or spin) at waypoint until an external succeed message is also sent
