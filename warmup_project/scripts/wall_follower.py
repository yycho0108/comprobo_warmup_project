#!/usr/bin/env python

import numpy as np
import rospy
import tf

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion

from tf_conversions import posemath as pm
from warmup_project.pid import PID
import warmup_project.utils as U
from warmup_project.wall_finder import WallFinderHough as WallFinder
from visualization_msgs.msg import Marker

class WallFollower(object):
    """
    Wall Following based on PID control on distance to nearest wall.
    TODO(@yoonyoungcho): incorporate Hough Transform based wall detection and following
    """
    
    def __init__(self):
        # loop rate
        self.rate_ = rospy.get_param('~rate', 50.0)

        # default speeds
        self.v_ = rospy.get_param('~v', 0.2)
        self.w_ = rospy.get_param('~w', 0.4)

        # distance threshold constants
        self.radius_ = rospy.get_param('~r', 0.3)
        self.target_ = rospy.get_param('~d', 0.7)
        self.danger_ = rospy.get_param('~q', 0.6)

        # pid parameters
        kp = rospy.get_param('~kp', 0.5)
        ki = rospy.get_param('~ki', 0.05)
        kd = rospy.get_param('~kd', 0.0)
        max_u = rospy.get_param('~max_u', 0.4)
        max_i = rospy.get_param('~max_i', 0.1)
        self.pid_ = PID(kp,ki,kd,max_u,max_i)

        # wall detector
        self.wall_finder_ = WallFinder()

        # data (static @ initialization)
        self.init_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None

        # data (dynamic)
        self.scan_ = None
        self.scan_t_ = None
        self.now_ = rospy.Time.now()

        # ros handles
        self.tfl_ = tf.TransformListener()
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.scan_sub_ = rospy.Subscriber('stable_scan', LaserScan, self.scan_cb)
        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.wall_pub_ = rospy.Publisher('wall', Marker, queue_size=2)

    def footprint(self):
        # TODO : build parametric footprint at all 360'
        pass

    def publish_quad(self, range, angle):
        """ Publish four points front,back,left,right to the robot """
        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'base_link'

        fwd = range[ np.argmin( np.abs(U.adiff(angle, 0.0 * np.pi))) ]
        fwd = Point(x=fwd)

        lft = range[ np.argmin( np.abs(U.adiff(angle, 0.5 * np.pi))) ]
        lidx = np.argmin( np.abs(U.adiff(angle, 0.5 * np.pi)))
        print 'lft', lft, lidx
        lft = Point(y=lft)

        bwd = range[ np.argmin( np.abs(U.adiff(angle, 1.0 * np.pi))) ]
        bwd = Point(x=-bwd)

        rgt = range[ np.argmin( np.abs(U.adiff(angle, -0.5 * np.pi))) ]
        rgt = Point(y=-rgt)

        col = ColorRGBA(1.0,1.0,1.0,1.0)

        msg.type = msg.POINTS
        msg.action = msg.ADD
        msg.points = [fwd,lft,bwd,rgt]
        msg.colors = [col,col,col,col]
        msg.scale.x = msg.scale.y = msg.scale.z = 0.2

        self.viz_pub_.publish(msg)

    def publish_wall(self, xy_valid):
        ps = self.wall_finder_(xy_valid)
        if ps is None:
            return
        rospy.loginfo_throttle(1.0, '# Walls Detected : {}'.format(len(ps)))
        #r, t = rt

        ### points
        #po = np.stack([r * np.cos(t), r * np.sin(t)],axis=-1).reshape(-1,1,2) # (n,1,2)

        ### vectors
        #u = np.stack([np.sin(t), -np.cos(t)], axis=-1).reshape(-1,1,2)
        #ps = po + ((3.0*u) * np.reshape([-1.0, 1.0], (1,2,1)) )
        ps = np.asarray(ps,dtype=np.float32)

        col = ColorRGBA(1.0,0.0,1.0,1.0)

        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.scan_t_#rospy.Time.now()
        msg.type = msg.LINE_LIST
        msg.action = msg.ADD
        msg.points = []
        msg.pose.position = Point()
        msg.pose.orientation = Quaternion(0,0,0,1)

        for (p0, p1) in ps:
            msg.points.extend([
                Point(x=p0[0],y=p0[1]),
                Point(x=p1[0],y=p1[1])])
        msg.scale.x = msg.scale.y = msg.scale.z = 0.1
        msg.colors = [col for _ in msg.points]
        self.wall_pub_.publish(msg)

    def publish_closest(self, range, angle):
        """ Publish the closest point detected from the LIDAR """
        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'base_link'

        px  = np.cos(angle) * range
        py  = np.sin(angle) * range
        p = Point(x=px,y=py)

        msg.type = msg.SPHERE#msg.POINT
        msg.action = msg.ADD
        msg.pose.position = p
        #msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = 0.2
        msg.color = ColorRGBA(1.0,1.0,1.0,1.0)

        self.viz_pub_.publish(msg)

    @staticmethod
    def check_forward(range, angle, mask, radius, danger, thresh=2):
        """ check if something exists in front of the robot within collision range """
        amask = np.abs(U.anorm(angle)) < (np.pi / 4.0)
        dmask =  (range * np.sin(angle)) < radius # in sweep path
        dmask &= (range * np.cos(angle)) < danger # danger
        #print 'huh?', range[ (mask & amask) & dmask]
        #print 'fw_check', np.sum(mask & amask & dmask)
        return np.sum(mask & amask & dmask) >= thresh

    @staticmethod
    def min_distance(range, angle, mask, danger):
        """ check distance to right wall """
        amask = (-3*np.pi/4 < angle) & (angle < -np.pi/4) # right-wall
        dmask = (range < danger)

        sel   = (mask & amask)

        valid = range[sel]# & dmask]
        if len(valid) > 0:
            v_idx = np.where(sel)[0]
            didx = v_idx[np.argmin(valid)]
            dmin = np.min( np.abs(valid * angle[sel]))
        else:
            dmin = None # wall does not exist
            didx = None
        return dmin, didx

    def scan_cb(self, msg):
        """ get scan data """
        # TODO: move frame to base_link
        #p, q = self.tfl_.lookupTransform('base_link', 'laser', rospy.Time(0))
        if not self.init_:
            self.init_ = True
            #self.angle_ = U.anorm( np.linspace(msg.angle_min, msg.angle_max,
            #        len(msg.ranges), endpoint=True) )
            self.angle_ = U.anorm(np.linspace(0, 2*np.pi, len(msg.ranges), endpoint=True))
            self.dmin_ = msg.range_min
            self.dmax_ = msg.range_max
        self.scan_ = np.asarray(msg.ranges, dtype=np.float32)
        self.scan_t_ = msg.header.stamp

    def step(self, cmd_vel):
        """ Run Single Steop """
        # keep track of elapsed time (PID)
        now = rospy.Time.now()
        dt = (now - self.now_).to_sec()
        self.now_ = now

        if self.scan_ is None:
            # no data yet!
            return

        #self.publish_quad(self.scan_, self.angle_)
        rq = np.stack([self.scan_, self.angle_], axis=-1)
        vmask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)
        rq_valid = rq[vmask]
        xy_valid = U.rq2xy(rq_valid)
        self.publish_wall(xy_valid)

        # mask for valid data
        mask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)

        turn_flag = self.check_forward(self.scan_, self.angle_,
                mask, self.radius_, self.danger_)
        v,w = 0.,0.
        rospy.loginfo_throttle(1.0, 'turn : {}'.format(turn_flag))

        if turn_flag:
            # turn if something exists in front
            v = 0
            w = self.w_
            self.pid_.reset()
        else:
            # otherwise follow **right** wall
            v = self.v_ # constant velocity

            dmin, didx = self.min_distance(self.scan_, self.angle_,
                    mask, self.danger_)
            rospy.loginfo_throttle(0.2, 'min dist: {}'.format(dmin))
            if dmin is None or dmin > 2.0:
                # no wall to follow - trace very large circle
                w = 0.03
                self.pid_.reset()
            else:
                self.publish_closest(self.scan_[didx], self.angle_[didx])
                derr = (self.target_ - dmin)
                w = self.pid_(derr, dt)

        cmd_vel.linear.x = v
        cmd_vel.angular.z = w

    def run(self):
        """ main loop """
        cmd_vel = Twist()
        rate = rospy.Rate(self.rate_)
        while not rospy.is_shutdown():
            self.step(cmd_vel)
            #self.cmd_pub_.publish(cmd_vel)
            rate.sleep()

def main():
    rospy.init_node('comprobo_wall_follower')
    node = WallFollower()
    node.run()

if __name__ == "__main__":
    main()
