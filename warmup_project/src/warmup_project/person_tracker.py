from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints
import numpy as np

# TODO : more complex walking model?

def ukf_hx(s):
    return s[:2]

def ukf_fx(s, dt):
    x,y,vx,vy = s
    x += vx * dt
    y += vy * dt
    return np.asarray([x,y,vx,vy])


class PersonTracker(object):
    def __init__(self, vmax):

        self.vmax_ = vmax # rospy.get_param('~vmax', 6.0)

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

        self.l_lost_ = 0
        self.r_lost_ = 0

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

            ukf.Q = np.diag(np.square([0.05,0.05,0.5,0.5])) # TODO : tune
            ukf.R = np.diag(np.square([0.09,0.09]))


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
            self.l_lost_ = 0
        else:
            self.l_lost_ += 1

        if pts_r.size > 0:
            pt_r = np.mean(pts_r, axis=0)
            self.ukf_r_.update(pt_r)
            self.r_lost_ = 0
        else:
            self.r_lost_ += 1

        p_l = self.ukf_l_.x.copy()[:2]
        p_r = self.ukf_r_.x.copy()[:2]

        return p_l, p_r, self.l_lost_, self.r_lost_
