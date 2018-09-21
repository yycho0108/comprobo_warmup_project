from warmup_project.pid import PID
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

import sklearn.clust

## DETECTION ##
class PersonDetector(object):
    def __init__(self, min_r, max_r, max_q=np.deg2rad(45)):
        self.min_r_ = min_r
        self.max_r_ = max_q
        self.max_q_ = max_q
    @staticmethod
    def rq2xy(self, rq):
        # (n,2) -> (n,2)
        r, q = rq.T
        x = r * np.cos(q)
        y = r * np.sin(q)
        return np.stack([x,y], axis=-1)
    def __call__(self, rq):
        r, q = rq.T
        pass

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

        # note ukf state coordinates are expressed in absolute (odom/map) frame,
        # not relative (base_link) frame; this is to subtract self-motion from tracking data.

        pass

    def initialize(self, info):
        self.info_ = info

    def __call__(self, pts, dt):
        # predict ...
        self.ukf_.P = (self.ukf_.P + self.ukf_.P.T) / 2.0
        self.ukf_.predict(dt)
        p0 = self.ukf_.x.copy()[np.newaxis, :2]
        p0 = np.reshape([x0,y0], (-1,2))

        # update ...
        np.subtract(pts, 

class PersonFollower(object):
    def __init__(self):
        self.detector_ = PersonDetector()
        self.tracker_  = PersonTracker()

        self.calib_srv_ = rospy.Service("calibrate", Empty, self.calibrate)
        self.calibrated_ = False
        self.calibrate_req_ = False

    def calibrate(self, _):
        rospy.loginfo("Stand in front of the robot until calibration is complete")
        self.calibrate_req_ = True
        self.detector_(self.last_scan_)
        self.calibrated_ = False
        pass

    def step(self):
        if self.calibrate_req_:
            suc, info = self.detector_(self.scan_)
            self.calibrated_ = suc
            self.calibrate_req_ = False
            rospy.loginfo('Calibration Success : {}'.format(self.calibrated_))
            self.tracker_.initialize(info)

        if self.calibrated_:
            pass
