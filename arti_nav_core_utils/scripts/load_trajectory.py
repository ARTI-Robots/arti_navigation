#!/usr/bin/env python
from __future__ import print_function
import actionlib
import angles
import math
import os.path
import rospy
import yaml

from actionlib_msgs.msg import GoalStatus
from arti_move_base_msgs.msg import FollowTrajectoryAction, FollowTrajectoryGoal, FollowTrajectoryFeedback
from arti_nav_core_msgs.msg import Trajectory2DWithLimits, Movement2DWithLimits
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


class LoadTrajectory(object):
    def __init__(self):
        self.dir = rospy.get_param('~directory', os.path.join(os.path.expanduser('~'), 'arti', 'recording'))
        self.file = rospy.get_param('~file', os.path.join(self.dir, 'trajectory_recorded.yaml'))

        self.transform_base_link = rospy.get_param('~transform_base_link', False)
        self.reverse = rospy.get_param('~reverse', False)

        if self.transform_base_link:
            rospy.loginfo("trajectory will be transformed to base link!")

        if self.reverse:
            rospy.loginfo("trajectory will be reversed!")

        self.frame_id = None
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.cxv = []
        self.cav = []

        self.path_pub = rospy.Publisher('~path', Path, latch=True, queue_size=1)

        self.follow_trajectory_action_client = actionlib.SimpleActionClient('/follow_trajectory',
                                                                            FollowTrajectoryAction)
        rospy.loginfo("waiting for action server: %s",
                      rospy.remap_name(self.follow_trajectory_action_client.action_client.ns))
        self.follow_trajectory_action_client.wait_for_server()

        self.load_file()
        self.path = self.create_path()
        self.publish_path()

        self.follow_trajectory()

    def load_file(self):
        rospy.loginfo("file to load: %s", self.file)
        with open(self.file) as parameters:
            loaded_dict = yaml.safe_load(parameters)
            trajectory = loaded_dict["trajectory"]

            if self.transform_base_link:
                self.frame_id = 'base_link'
                initial_pose = trajectory[-1 if self.reverse else 0]
            else:
                self.frame_id = loaded_dict["frame_id"]
                initial_pose = {'pos_x': 0.0, 'pos_y': 0.0, 'yaw': 0.0}
            initial_yaw = float(initial_pose["yaw"])
            sin_inv_yaw = math.sin(-initial_yaw)
            cos_inv_yaw = math.cos(-initial_yaw)

            for v in reversed(trajectory) if self.reverse else trajectory:
                dx = float(v["pos_x"]) - float(initial_pose["pos_x"])
                dy = float(v["pos_y"]) - float(initial_pose["pos_y"])
                self.cx.append(cos_inv_yaw * dx - sin_inv_yaw * dy)
                self.cy.append(sin_inv_yaw * dx + cos_inv_yaw * dy)
                self.cyaw.append(angles.normalize_angle(float(v["yaw"]) - float(initial_pose["yaw"])))
                self.cxv.append(float(v["linear_x"]) * (-1.0 if self.reverse else 1.0))
                self.cav.append(float(v["angular_z"]))
        rospy.loginfo("finished loading file: %s", self.file)

    def create_path(self):
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        path = Path(header=header)
        for x, y, yaw in zip(self.cx, self.cy, self.cyaw):
            orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
            path.poses.append(PoseStamped(header=header, pose=Pose(position=Point(x=x, y=y), orientation=orientation)))
        return path

    @staticmethod
    def create_movement(x, y, theta, v_x):
        msg_movement = Movement2DWithLimits()
        msg_movement.pose.point.x.value = x
        msg_movement.pose.point.x.has_limits = True
        msg_movement.pose.point.x.upper_limit = 0.1
        msg_movement.pose.point.x.lower_limit = -0.1
        msg_movement.pose.point.y.value = y
        msg_movement.pose.point.y.has_limits = True
        msg_movement.pose.point.y.upper_limit = 0.1
        msg_movement.pose.point.y.lower_limit = -0.1
        msg_movement.pose.theta.value = theta
        msg_movement.pose.theta.has_limits = True
        msg_movement.pose.theta.upper_limit = 0.01
        msg_movement.pose.theta.lower_limit = -0.01
        msg_movement.twist.x.value = v_x
        msg_movement.twist.x.has_limits = True
        msg_movement.twist.x.upper_limit = max(0.0, -v_x)
        msg_movement.twist.x.lower_limit = min(0.0, -v_x)
        return msg_movement

    def publish_path(self):
        """
        publish the global plan
        """
        self.path_pub.publish(self.path)

    def follow_trajectory(self):
        trajectory = Trajectory2DWithLimits(header=Header(stamp=rospy.Time.now(), frame_id=self.frame_id))
        for (wx, wy, wyaw, wxv) in zip(self.cx, self.cy, self.cyaw, self.cxv):
            trajectory.movements.append(self.create_movement(wx, wy, wyaw, wxv))

        self.follow_trajectory_action_client.send_goal(FollowTrajectoryGoal(trajectory=trajectory),
                                                       done_cb=self.process_follow_trajectory_action_done)
        rospy.loginfo("--------- sent goal to move base ---------")

    @staticmethod
    def process_follow_trajectory_action_done(state, result):
        rospy.loginfo("follow trajectory action finished with state %s and result %s",
                      actionlib.get_name_of_constant(GoalStatus, state), result)
        rospy.signal_shutdown("done")


if __name__ == '__main__':
    rospy.init_node('load_trajectory')
    try:
        LoadTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
