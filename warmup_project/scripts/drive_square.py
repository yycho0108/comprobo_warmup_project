#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

class SquareDriver(object):
    def __init__(self):
        # receive ROS parameters
        self.rate_ = rospy.get_param('~rate', 50.0)
        self.odom_from_tf = rospy.get_param('~odom_from_tf', False)

        # create ROS handle
        if self.odom_from_tf:
            self.tfl_ = tf.TransformListener()
        else:
            self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odom_cb)
    
    def odom_cb(self, msg):
        pass

    def run(self):
        rate = rospy.Rate(self.rate_)
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    app = SquareDriver()
    app.run()

if __name__ == "__main__":
    main()
