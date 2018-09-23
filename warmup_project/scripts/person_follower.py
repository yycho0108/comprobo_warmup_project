#!/usr/bin/env python2

from warmup_project.pid import PID
from warmup_project.person_detector import PersonDetector
from warmup_project.person_tracker import PersonTracker
from warmup_project import utils as U

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import tf
import rospy
import numpy as np
from tf_conversions import posemath as pm
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32, Twist

class PersonFollowerNode(object):
    def __init__(self):

        # detector args
        min_x = rospy.get_param('~min_x', 0.3)
        max_x = rospy.get_param('~max_x', 2.0)
        max_y = rospy.get_param('~max_y', 0.5) # note: get max_y first
        min_y = rospy.get_param('~min_y', -max_y)

        # tracker args
        self.v_max_ = rospy.get_param('~vmax', 6.0)
        self.d_targ_ = rospy.get_param('~d_target', 0.6)

        self.detector_ = PersonDetector(min_x,max_x,min_y,max_y)

        now = rospy.Time.now().to_sec()
        self.time_ = now
        self.tracker_  = PersonTracker(vmax=self.v_max_)

        self.scan_ = None
        self.new_scan_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None
        self.init_ = False

        self.pid_v_ = PID(kp=1.0,max_u=0.5,max_i=0.0)
        self.pid_w_ = PID(kp=1.0,max_u=0.5,max_i=0.0)

        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.tfl_ = tf.TransformListener()
        self.calib_srv_ = rospy.Service("calibrate", Empty, self.calibrate)
        self.calibrated_ = False
        self.calibrate_req_ = False
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)

        # control
        self.cmd_vel_ = Twist()
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def calibrate(self, _):
        rospy.loginfo("Stand in front of the robot until calibration is complete")
        self.calibrate_req_ = True
        self.calibrated_ = False
        return EmptyResponse()

    def publish(self, suc, info):
        # area ...
        # self.ar_msg_.header.stamp = rospy.Time.now()
        # self.ar_msg_.header.frame_id = 'base_link'
        # self.ar_msg_.polygon = Polygon([Point32(x,y,0) for (x,y) in self.ar_])
        # self.ar_pub_.publish(self.ar_msg_)

        # detections ...
        if not suc:
            return

        p0, p1 = info
        p0x,p0y = p0 # np.mean(c0,axis=0) # centroid
        p1x,p1y = p1 # np.mean(c1,axis=0) # centroid

        p0 = Point(x=p0x,y=p0y)
        p1 = Point(x=p1x,y=p1y)
        col = ColorRGBA(1.0,1.0,1.0,1.0)

        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'odom'

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

    def apply_scan_tf(self, pts, source='odom', target='base_link'):
        # pts = [N,2]
        try:
            #T_tf = self.tfl_.lookupTransform(target, source, rospy.Time(0))
            T_tf = self.tfl_.lookupTransform(source, target, rospy.Time(0))
        except tf.Exception as e:
            rospy.logerr('Scan tf failed : {}'.format(e))
            return None
        T_msg = pm.toMsg(pm.fromTf(T_tf))
        x,y = T_msg.position.x, T_msg.position.y
        q   = 2.0 * np.arctan2(T_msg.orientation.z, T_msg.orientation.w)
        pts_odom = np.dot(pts, U.R(q).T) + np.reshape([x,y], [-1,2])
        return pts_odom

    @staticmethod
    def to_pose2d(msg):
        x, y  = [msg.position.x, msg.position.y]
        rz = 2.0 * np.arctan2(msg.orientation.z, msg.orientation.w)
        pose = Pose2D(x=x,y=y,theta=rz)
        return pose

    def step(self):
        if (self.scan_ is not None):# and self.new_scan_:
            rq = np.stack([self.scan_, self.angle_], axis=-1)
            mask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)
            rq_valid = rq[mask]
            self.new_scan_ = False

        if self.calibrate_req_:
            suc, info = self.detector_(rq_valid)
            self.calibrated_ = suc
            rospy.loginfo('Calibration Success : {}'.format(self.calibrated_))
            if suc:
                self.calibrate_req_ = False
                now = rospy.Time.now().to_sec()
                self.time_ = now
                info_global = self.apply_scan_tf(info)
                self.tracker_.initialize(info_global)
                self.pid_v_.reset()
                self.pid_w_.reset()

        if self.calibrated_:
            now = rospy.Time.now().to_sec()
            dt = (now - self.time_)
            xy_valid = U.rq2xy(rq_valid)
            xy_valid_global = self.apply_scan_tf(xy_valid)
            p_l, p_r, l_lost, r_lost = self.tracker_(xy_valid_global, dt)

            if (l_lost > 3 or r_lost > 3): # lost for 3 successive frames
                self.calibrated_ = False
                self.calibrate_req_ = True
                self.cmd_pub_.publish(Twist())
                return

            # controls computation
            p = np.mean([p_l,p_r], axis=0)
            p = self.apply_scan_tf([p], 'base_link', 'odom')
            r, q = U.xy2rq(p)[0]

            err_d = (r - self.d_targ_)
            err_q = (q - 0.0)

            u_v = self.pid_v_(err_d, dt)
            u_w = self.pid_w_(err_q, dt)
            self.cmd_vel_.linear.x = u_v
            self.cmd_vel_.angular.z = u_w
            self.cmd_pub_.publish(self.cmd_vel_)

            # logging
            self.publish(True, (p_l, p_r))
            self.time_ = now

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()
        

def main():
    rospy.init_node('person_follower')
    node = PersonFollowerNode()
    node.run()

if __name__ == "__main__":
    main()
