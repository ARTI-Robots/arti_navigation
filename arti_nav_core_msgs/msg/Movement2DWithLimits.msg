# Combination of a pose (position and orientation) and a twist (linear and angular velocities). The twist is given
# relative to the pose (i.e., in the robot coordinate frame).
#
# For example, a robot driving forward along the x axis of the coordinate frame would have pose.theta = 0 and
# twist.x = v_lin; a robot driving forward along the y axis of the coordinate frame would have pose.theta = pi/2 and
# twist.x = v_lin; and a robot driving sideways along the x axis of the coordinate frame would have pose.theta = pi/2
# and twist.y = -v_lin.

arti_nav_core_msgs/Pose2DWithLimits pose
arti_nav_core_msgs/Twist2DWithLimits twist