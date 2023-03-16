#!/usr/bin/env python
from __future__ import print_function
import angles
import math
import os.path
import rospy

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovariance, Quaternion
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

POSE_FORMAT = """\
  - pos_x: {pos_x:5f}
    pos_y: {pos_y:5f}
    yaw: {yaw:5f}
    linear_x: {linear_x:5f}
    angular_z: {angular_z:5f}
"""


class RecordTrajectory(object):
    def __init__(self):
        self.write_data = True
        self.dir = rospy.get_param('~directory', os.path.join(os.path.expanduser('~'), 'arti', 'recording'))
        self.file = rospy.get_param('~file', os.path.join(self.dir, 'trajectory_recorded.yaml'))
        self.distance_thresh = rospy.get_param('~distance_thresh', 0.3)  # in meter
        self.angular_thresh_rad = rospy.get_param('~angular_thresh', 5.0) * math.pi / 180.0  # in degree
        self.prev_pose = None

        rospy.loginfo("saving trajectory to: %s", self.file)

        self.data_file = open(self.file, 'w')
        self.cx = []
        self.cy = []
        self.cyaw = []

        self.path_pub = rospy.Publisher('~path', Path, latch=True, queue_size=1)
        self.odom_subscriber = rospy.Subscriber("/ukf_pose", Odometry, self.odom_callback, queue_size=1)
        self.pose_subscriber = rospy.Subscriber("~pose", PoseStamped, self.pose_callback, queue_size=1)

    def odom_callback(self, msg):
        """
        :type msg: Odometry
        """
        orientation = msg.pose.pose.orientation
        _roll, _pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        if not self.cx:
            self.data_file.write("frame_id: {frame_id}\n".format(frame_id=msg.header.frame_id))
            self.data_file.write("trajectory:\n")
            self.add_pose(msg, yaw)
        else:
            prev_o = self.prev_pose.pose.pose.orientation
            _prev_roll, _prev_pitch, prev_yaw = euler_from_quaternion([prev_o.x, prev_o.y, prev_o.z, prev_o.w])

            angle_diff = angles.normalize_angle(yaw - prev_yaw)

            dist = math.hypot((msg.pose.pose.position.x - self.prev_pose.pose.pose.position.x),
                              (msg.pose.pose.position.y - self.prev_pose.pose.pose.position.y))
            # check if traveled distance increased relevant enough
            if dist >= self.distance_thresh or abs(angle_diff) > self.angular_thresh_rad:
                self.add_pose(msg, yaw)

    def pose_callback(self, msg):
        """
        :type msg: PoseStamped
        """
        self.odom_callback(Odometry(header=msg.header, pose=PoseWithCovariance(pose=msg.pose)))

    def add_pose(self, msg, yaw):
        self.cx.append(msg.pose.pose.position.x)
        self.cy.append(msg.pose.pose.position.y)
        self.cyaw.append(yaw)

        self.data_file.write(POSE_FORMAT.format(pos_x=msg.pose.pose.position.x, pos_y=msg.pose.pose.position.y, yaw=yaw,
                                                linear_x=msg.twist.twist.linear.x, angular_z=msg.twist.twist.angular.z))

        self.prev_pose = msg
        rospy.loginfo("saved %d poses to trajectory", len(self.cx))
        self.publish_plan(msg.header)

    def publish_plan(self, header):
        """
        publish the global plan
        """
        msg = Path(header=header)
        for x, y, yaw in zip(self.cx, self.cy, self.cyaw):
            orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
            msg.poses.append(PoseStamped(header=header, pose=Pose(position=Point(x=x, y=y), orientation=orientation)))

        self.path_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('record_trajectory')
    try:
        RecordTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
