#!/usr/bin/env python

import numpy
import rospy
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from tf_conversions import posemath as pm
from warmup_project.pid import PID
from warmup_project.utils import anorm, adiff

class WallFollower(object):
    def __init__(self):
        # loop rate
        self.rate_ = rospy.get_param('~rate', 50.0)

        # default speeds
        self.v_ = rospy.get_param('~v', 0.1)
        self.w_ = rospy.get_param('~w', 0.1)

        # distance threshold constants
        self.radius_ = rospy.get_param('~r', 0.3)
        self.danger_ = rospy.get_param('~q', 0.6)
        self.target_ = rospy.get_param('~d', 0.4)

        # pid parameters
        kp = rospy.get_param('~kp', 1.0)
        ki = rospy.get_param('~ki', 0.0)
        kd = rospy.get_param('~kd', 0.0)
        max_u = rospy.get_param('~max_u', 1.0)
        max_i = rospy.get_param('~max_i', 0.0)
        self.pid_ = PID(kp,ki,kd,max_u,max_i)

        # data (static @ initialization)
        self.init_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None

        # data (dynamic)
        self.scan_ = None
        self.now_ = rospy.Time.now()

        # ros handles
        self.tfl_ = tf.TransformListener()
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)

    def check_forward(range, angle, mask, radius, danger):
        """ check if something exists in front of the robot within collision range """
        amask = np.abs(anorm(angle)) < (np.pi / 2.0)
        dmask = range * np.sin(angle) < radius # in sweep path
        dmask &= range * np.cos(angle) < danger
        return np.any(mask & amask & dmask)

    def min_distance(range, angle, mask, danger):
        """ check distance to right wall """
        amask = (angle < 0) # right-wall
        dmask = (range < danger)
        valid = range[mask & amask & dmask]
        if len(valid) > 0:
            dmin = np.min(valid)
        else:
            dmin = None # wall does not exist
        return dmin

    def scan_cb(self, msg):
        """ get scan data """
        # TODO: move frame to base_link
        #p, q = self.tfl_.lookupTransform('base_link', 'laser', rospy.Time(0))
        if not self.init_:
            self.init_ = True
            self.angle_ = anorm( np.linspace(msg.angle_min, msg.angle_max,
                    len(msg.ranges), endpoint=True) )
            self.dmin_ = msg.range_min
            self.dmax_ = msg.range_max
        self.scan_ = msg.range

    def step(self, cmd_vel):
        # keep track of elapsed time (PID)
        now = rospy.Time.now()
        dt = (now - self.now_).to_sec()
        self.now_ = now

        if self.scan_ is None:
            # no data yet!
            return

        # mask for valid data
        mask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)

        turn_flag = self.check_forward(self.scan_, self.angle_,
                mask, self.d_robot_)
        v,w = 0.,0.

        if turn_flag:
            # turn if something exists in front
            v = 0
            w = self.w_
        else:
            # otherwise follow **right** wall
            v = self.v_ # constant velocity

            dmin = self.min_distance(self.scan_, self.angle_,
                    mask, self.d_wall_)
            if dmin is None:
                # no wall to follow
                w = 0.0
            else:
                derr = (self.target_ - dmin)
                w = self.pid_(derr, dt)

        cmd_vel.linear.x = v
        cmd_vel.angular.z = w

    def run(self):
        cmd_vel = Twist()
        rate = rospy.Rate(self.rate_)
        while not rospy.is_shutdown():
            self.step(cmd_vel)
            self.cmd_pub_.publish(cmd_vel)
            rate.sleep()

def main():
    rospy.init_node('comprobo_wall_follower')
    node = WallFollower()
    node.run()

if __name__ == "__main__":
    main()
