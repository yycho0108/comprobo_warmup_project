#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class VizPt(object):
    def __init__(self):
        self.pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.msg_ = Marker()
        self.build(self.msg_)
        self.rate_ = rospy.get_param('~rate', default=10)

    def build(self, m):
        m.header.frame_id='base_link'
        m.type = m.SPHERE
        m.action = m.ADD
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0

        m.pose.position.x = 1.0
        m.pose.position.y = 2.0
        m.pose.position.z = 0.0

    def run(self):
        r = rospy.Rate(self.rate_)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg_)
            r.sleep()

def main():
    rospy.init_node('viz_pt')
    app = VizPt()
    app.run()

if __name__ == "__main__":
    main()
