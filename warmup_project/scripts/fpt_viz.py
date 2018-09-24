#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Polygon, PolygonStamped, Point32

class FptViz(object):
    """
    Footprint Visualization Node.
    Current footprint information is hardcoded for the neato.
    """
    def __init__(self):
        # TODO : non-hardcoded footprint (possibly load from .yaml files given as a rosparam)
        self.fpt_ = np.asarray([[ 0.25649121, -0.17213032],
            [ 0.17213032, -0.17468672],
            [ 0.0562406 , -0.17468672],
            [-0.00596491, -0.14315788],
            [-0.05027569, -0.09117794],
            [-0.06987468, -0.00426065],
            [-0.05027569,  0.07243107],
            [ 0.00170426,  0.13804510],
            [ 0.05624060,  0.16616541],
            [ 0.16957393,  0.16957393],
            [ 0.25478697,  0.16786967]], dtype=np.float32)

        self.fpt_msg_  = PolygonStamped()
        self.fpt_msg_.header.stamp = rospy.Time.now()
        self.fpt_msg_.header.frame_id = 'base_link'
        self.fpt_msg_.polygon = Polygon([Point32(x,y,0) for (x,y) in self.fpt_])
        self.fpt_pub_  = rospy.Publisher('fpt', PolygonStamped, queue_size=2)

    def step(self):
        self.fpt_msg_.header.stamp = rospy.Time.now()
        self.fpt_pub_.publish(self.fpt_msg_)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main():
    rospy.init_node('fpt_viz')
    node = FptViz()
    node.run()

if __name__ == "__main__":
    main()
