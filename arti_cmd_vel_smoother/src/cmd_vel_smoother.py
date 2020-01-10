#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from collections import deque

from threading import RLock


class Smoother:
    """
    This class is used to smooth the cmd vel with an low-pass-like filter.
    """

    def __init__(self, frequency=10, queue_len=10, cmd_sub_topic='cmd_vel_raw', cmd_pub_topic='cmd_vel'):
        """
        Constructor for cmd vel smoother
        :param frequency: frequency of the published cmd vel msgs
        :param queue_len: len of queue of the linear x velocity
        :param cmd_sub_topic: name of subscribed topic
        :param cmd_pub_topic: name of published topic
        """
        self.x_queue = deque(maxlen=queue_len)
        self.cmd_vel = Twist()
        self.theta = 0.0
        self.lock = RLock()
        self.enabled = False

        self.x_current = 0.0
        self.theta_current = 0.0

        self.rate = rospy.Rate(frequency)

        self.pub = rospy.Publisher(cmd_pub_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(cmd_sub_topic, Twist, self.cb)

    def pub_mean(self):
        """
        Calculate mean of linear x velocity and publish new Twist msgs.
        """
        with self.lock:
            if len(self.x_queue):
                self.cmd_vel.linear.x = float(sum(self.x_queue) / self.x_queue.maxlen)
            else:
                self.cmd_vel.linear.x = 0.0

            self.cmd_vel.angular.z = float(self.theta_current)

            # Only publish cmds if necessary
            if self.cmd_vel.linear.x == 0.0 and self.cmd_vel.angular.z == 0.0:
                self.enabled = False

        if self.enabled:
            self.pub.publish(self.cmd_vel)

    def cb(self, msg):
        """
        Callback for cmd vel msgs.
        :param msg: new topic msg
        """
        with self.lock:
            self.x_current = msg.linear.x
            self.theta_current = msg.angular.z
            self.enabled = True

    def run(self):
        """
        Main loop. It runs with a defined frequency and adds the current cmd vel to the queue.
        Therefore, it works like a low-pass filter. Afterwords, it calculates/publishes the
        mean cmd vel and publishes it.
        """
        while not rospy.is_shutdown():
            self.x_queue.append(self.x_current)

            self.pub_mean()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('arti_cmd_vel_smoother', anonymous=True)
    rospy.loginfo("started")
    smoother = Smoother(frequency=20, queue_len=10, cmd_sub_topic='cmd_vel_raw', cmd_pub_topic='cmd_vel_smooth')
    smoother.run()
