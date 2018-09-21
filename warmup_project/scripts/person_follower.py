from warmup_project.pid import PID
from warmup_project.person_detector import PersonDetector
from warmup_project import utils as U

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import tf
import rospy
import numpy as np
from tf_conversions import posemath as pm
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32

## TRACKING ##
# TODO : incorporate more complex walking model?

def ukf_hx(s):
    return s[:2]

def ukf_fx(s, dt):
    x,y,vx,vy = s
    x += vx * dt
    y += vy * dt
    return np.asarray([x,y,vx,vy])

class PersonTracker(object):
    def __init__(self):

        self.vmax_ = rospy.get_param('~vmax', 3.0)

        sigma_points = MerweScaledSigmaPoints(4,1e-3,2,-2)
        #sigma_points = JulierSigmaPoints(4, 5-2, sqrt_method=np.linalg.cholesky)

        self.ukf_l_ = UKF(
                dim_x=4, # x,y,vx,vy
                dim_z=2, # x,y
                dt=0.01, # note: dynamic
                hx=ukf_hx,
                fx=ukf_fx,
                points=sigma_points
                )

        self.ukf_r_ = UKF(
                dim_x=4, # x,y,vx,vy
                dim_z=2, # x,y
                dt=0.01, # note: dynamic
                hx=ukf_hx,
                fx=ukf_fx,
                points=sigma_points
                )

        self.time_ = rospy.Time.now()

        # note ukf state coordinates are expressed in absolute (odom/map) frame,
        # not relative (base_link) frame; this is to subtract self-motion from tracking data.

    def initialize(self, info):
        p0, p1 = info

        # reset data with initial pose
        self.ukf_l_.x[:] = [p0[0],p0[1],0,0]
        self.ukf_r_.x[:] = [p1[0],p1[1],0,0]

        # reset other cached data
        for ukf in [self.ukf_l_, self.ukf_r_]:
            ukf.P = np.diag(np.square([0.3,0.3,3.0,3.0])) #TODO: hardcoded
            ukf.x_prior = ukf.x.copy()
            ukf.P_prior = ukf.P.copy()
            ukf.x_post  = ukf.x.copy()
            ukf_P_post  = ukf.P.copy()

        self.time_ = rospy.Time.now()


    def __call__(self, pts, dt):
        # predict ...
        # note scans ~5Hz

        for ukf in [self.ukf_l_, self.ukf_r_]:
            ukf.P = (ukf.P + ukf.P.T) / 2.0
            ukf.predict(dt)

        p_l = self.ukf_l_.x.copy()[:2]
        p_r = self.ukf_r_.x.copy()[:2]

        p_pred = np.reshape([p_l, p_r], (-1, 1, 2)) #(2,1,2)
        p_pts  = np.reshape(pts, (1, -1, 2)) #(1,N,2)
        cost = np.linalg.norm(p_pred - p_pts, axis=-1) #(2,N)

        sel  = (cost < (dt * self.vmax_)) # filter by distance
        sel0 = (cost[0,:] < cost[1,:]) # i.e. 1 when cost(0)>cost(1)
        sel1 = np.logical_not(sel0)

        pts_l = pts[sel[0] & sel0]
        pts_r = pts[sel[1] & sel1]

        # update ...
        if pts_l.size > 0:
            pt_l = np.mean(pts_l, axis=0)
            self.ukf_l_.update(pt_l)
        if pts_r.size > 0:
            pt_r = np.mean(pts_r, axis=0)
            self.ukf_r_.update(pt_r)

        p_l = self.ukf_l_.x.copy()[:2]
        p_r = self.ukf_r_.x.copy()[:2]

        return p_l, p_r

class PersonFollowerNode(object):
    def __init__(self):

        # detector args
        min_x = rospy.get_param('~min_x', 0.3)
        max_x = rospy.get_param('~max_x', 2.0)
        max_y = rospy.get_param('~max_y', 0.5) # note: get max_y first
        min_y = rospy.get_param('~min_y', -max_y)

        self.detector_ = PersonDetector(min_x,max_x,min_y,max_y)
        self.tracker_  = PersonTracker()

        self.scan_ = None
        self.new_scan_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None
        self.init_ = False

        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.tfl_ = tf.TransformListener()
        self.calib_srv_ = rospy.Service("calibrate", Empty, self.calibrate)
        self.calibrated_ = False
        self.calibrate_req_ = False
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)

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
            self.calibrate_req_ = False
            rospy.loginfo('Calibration Success : {}'.format(self.calibrated_))
            #### TODO : REMEMBER TO CONVERT TO ODOM FRAME!! ####
            info_global = self.apply_scan_tf(info)
            self.tracker_.initialize(info_global)

        if self.calibrated_:
            now = rospy.Time.now()
            dt = (now - self.tracker_.time_).to_sec()
            #### TODO : REMEMBER TO CONVERT TO ODOM FRAME!! ####
            xy_valid = U.rq2xy(rq_valid)
            xy_valid_global = self.apply_scan_tf(xy_valid)
            p_l, p_r = self.tracker_(xy_valid_global, dt)
            self.publish(True, (p_l, p_r))
            self.tracker_.time_ = now

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
