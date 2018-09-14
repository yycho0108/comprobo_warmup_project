#!/usr/bin/env python2

import rospy
import tf

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from std_srvs.srv import Empty

def xq2p(x,q):
    res = Pose()
    res.position.x = x[0]
    res.position.y = x[1]
    res.position.z = x[2]

    res.orientation.x = q[0]
    res.orientation.y = q[1]
    res.orientation.z = q[2]
    res.orientation.w = q[3]
    return res

class TrajectoryPublisher(object):
    def __init__(self):
        rospy.init_node('plot_trajectory')
        self._frame = rospy.get_param('~frame_id', 'map')

        self._path = Path()
        self._path.header = Header(frame_id=self._frame, stamp=rospy.Time.now())
        self._path.header.frame_id = self._frame
        self._path.header.stamp = rospy.Time.now()

        self._tfl = tf.TransformListener()
        self._pub = rospy.Publisher('path', Path, queue_size=10)
        self._reset_srv = rospy.Service('reset', Empty, self.reset)

    def reset(self, *args, **kwargs):
        self._path.poses = []

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                x, q = self._tfl.lookupTransform(self._frame, 'base_link', rospy.Time(0))
                self._path.poses.append(PoseStamped(
                    header=Header(frame_id=self._frame, stamp=rospy.Time.now()),
                    pose=xq2p(x,q)))
                self._pub.publish(self._path)
            except Exception as e:
                print e
            rate.sleep()

def main():
    app = TrajectoryPublisher()
    app.run()

if __name__ == '__main__':
    main()
