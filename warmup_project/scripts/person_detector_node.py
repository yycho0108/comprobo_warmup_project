from warmup_project.person_detector import PersonDetector
import warmup_project.utils as U
import numpy as np

import rospy

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class PersonDetectorNode(object):
    def __init__(self):
        min_x = rospy.get_param('~min_x', 0.3)
        max_x = rospy.get_param('~max_x', 2.0)

        max_y = rospy.get_param('~max_y', 1.0) # note: get max_y first
        min_y = rospy.get_param('~min_y', -max_y)

        self.scan_ = None
        self.new_scan_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None
        self.init_ = False

        self.detector_ = PersonDetector(min_x,max_x,min_y,max_y)
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)

    def publish(self, info):
        c0, c1 = info
        p0x,p0y = np.mean(c0,axis=0) # centroid
        p1x,p1y = np.mean(c1,axis=0) # centroid

        p0 = Point(x=p0x,y=p0y)
        p1 = Point(x=p1x,y=p1y)
        col = ColorRGBA(1.0,1.0,1.0,1.0)

        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'base_link'

        msg.type = msg.POINTS
        msg.action = msg.ADD
        msg.points = [p0, p1]
        msg.colors = [col,col]
        msg.scale.x = msg.scale.y = msg.scale.z = 0.2

        self.viz_pub_.publish(msg)

    def scan_cb(self, msg):
        self.new_scan_ = True
        if not self.init_:
            self.angle_ = U.anorm(np.linspace(0,2*np.pi,len(msg.ranges),endpoint=True))
            self.dmin_ = msg.range_min
            self.dmax_ = msg.range_max
            self.init_ = True
        self.scan_ = np.asarray(msg.ranges, dtype=np.float32)

    def step(self):
        if self.new_scan_:
            rq = np.stack([self.scan_, self.angle_], axis=-1)
            mask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)
            rq_valid = rq[mask]
            suc, info = self.detector_(rq_valid)
            if suc:
                self.publish(info)
            self.new_scan_ = False

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

def main():
    rospy.init_node('person_detector_node')
    node = PersonDetectorNode()
    node.run()

if __name__ == "__main__":
    main()
