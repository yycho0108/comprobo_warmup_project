#!/usr/bin/env python2

import rospy
import numpy as np

from warmup_project.pid import PID
import warmup_project.utils as U

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker


class ObstacleAvoider(object):
    def __init__(self):

        # initialize scan data fields
        self.scan_ = None
        self.new_scan_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None
        self.init_ = False

        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)

        self.pid_v_ = PID(kp=0.5, ki=0.03, max_u=0.5,max_i=0.25)
        #self.k_v_ = 0.25 #??
        self.pid_w_ = PID(kp=1.0, ki=0.08, max_u=1.0,max_i=0.5)
        self.last_cmd_ = rospy.Time.now()
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_ = Twist()
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)

    def scan_cb(self, msg):
        self.new_scan_ = True
        if not self.init_:
            self.angle_ = U.anorm(np.linspace(0,2*np.pi,len(msg.ranges),endpoint=True))
            self.dmin_ = msg.range_min
            self.dmax_ = msg.range_max
            self.init_ = True
        self.scan_ = np.asarray(msg.ranges, dtype=np.float32)

    def cmd_viz(self, v):
        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'base_link'

        px, py = v
        p = Point(x=px,y=py)

        msg.type = msg.SPHERE#msg.POINT
        msg.action = msg.ADD
        msg.pose.position = p
        #msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = 0.2
        msg.color = ColorRGBA(1.0,1.0,1.0,1.0)

        self.viz_pub_.publish(msg)


    def step(self):
        if self.scan_ is not None:
            now = rospy.Time.now()
            dt = (now - self.last_cmd_).to_sec()
            rq = np.stack([self.scan_, self.angle_], axis=-1)
            mask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)
            #mask &= (-np.pi/16 < self.angle_) & (self.angle_ < np.pi/16)
            rq = rq[mask]
            if np.size(rq) <= 0:
                return

            r, q = rq.T

            weight  = (1.0 / np.abs(r))
            dx = np.cos(q)
            dy = np.sin(q)

            uv = np.reshape(weight, [-1,1]) * np.stack([-dx,-dy], axis=-1) #N,2
            uv = np.sum(uv, axis=0, keepdims=True)
            uv /= np.linalg.norm(uv, axis=-1)

            self.cmd_viz(uv[0])

            _, v_q = U.xy2rq(uv)[0]

            v = self.pid_v_(0.4 / np.mean(r) ,dt) #self.k_v_ * (np.min(r))#, dt)
            w = self.pid_w_(v_q, dt)

            self.cmd_vel_.linear.x = v
            self.cmd_vel_.angular.z = w
            self.cmd_pub_.publish(self.cmd_vel_)

            self.last_cmd_ = now

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

def main():
    rospy.init_node('obstacle_avoider')
    node = ObstacleAvoider()
    node.run()

if __name__ == "__main__":
    main()
