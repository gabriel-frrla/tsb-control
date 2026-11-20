# stamp: time this reference was generated, using the ROS clock.
# frame_id: reference frame for heading; initially "odom".
std_msgs/Header header

# Desired forward velocity along the robot's body x-axis, in m/s.
float64 surge_velocity

# Desired heading in radians, measured in header.frame_id.
# Zero points along that frame's positive x-axis.
# Positive rotation is counterclockwise about its positive z-axis.
float64 heading