#!/usr/bin/env python2

import rospy
import tf
import numpy as np

from tf_conversions import posemath as pm

from warmup_project.pid import PID
import warmup_project.utils as U

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Point32, Pose, PoseStamped, Polygon, PolygonStamped
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from sklearn.cluster import DBSCAN

class ObstacleAvoider(object):
    def __init__(self):
        # hardcoded neato footprint
        self.fpt_ = np.asarray([[ 0.25649121, -0.17213032],
            [ 0.17213032, -0.17468672],
            [ 0.0562406 , -0.17468672],
            [-0.00596491, -0.14315788],
            [-0.05027569, -0.09117794],
            [-0.06987468, -0.00426065],
            [-0.05027569,  0.07243107],
            [ 0.00170426,  0.13804510],
            [ 0.05624060,  0.16616541],
            [ 0.16957393,  0.16957393],
            [ 0.25478697,  0.16786967]], dtype=np.float32)

        self.radius_ = rospy.get_param('~radius', 0.2) # todo : use "footprint" instead

        # potential field params
        self.a_ = rospy.get_param('~a', 1.0) # ??
        self.b_ = rospy.get_param('~b', 0.01) # ??
        self.r_ = rospy.get_param('~r', 0.25)
        self.gr_ = rospy.get_param('~gr', 0.4) # goal
        self.s_ = rospy.get_param('~s', 2.0)

        # initialize scan data fields
        self.scan_fpt_ = None
        self.scan_ = None
        self.new_scan_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None
        self.init_ = False

        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.tfl_ = tf.TransformListener()

        self.pid_v_ = PID(kp=0.4, ki=0.03, max_u=0.5,max_i=0.3)
        self.k_v_ = 0.2 #??
        self.pid_w_ = PID(kp=5.0, ki=0.02, max_u=5.0, max_i=1.0)
        self.last_cmd_ = rospy.Time.now()
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_ = Twist()
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)

        self.goal_ = None
        self.goal_sub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        self.goal_viz_pub_ = rospy.Publisher('goal_area', PolygonStamped, queue_size=5)
        self.goal_viz_msg_ = PolygonStamped()

        self.obs_viz_pub_ = rospy.Publisher('obs_field', PoseArray, queue_size=2)
        self.obs_viz_msg_ = PoseArray()

        self.cls_viz_pub_ = rospy.Publisher('clusters', Marker, queue_size=2)

    def goal_cb(self, msg):
        self.goal_ = msg.pose.position

    def get_heading(self, source='odom', target='base_link'):
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
        #pts_odom = np.dot(pts, U.R(q).T) + np.reshape([x,y], [-1,2])
        return q

    def scan_cb(self, msg):
        self.new_scan_ = True
        if not self.init_:
            self.angle_ = U.anorm(np.linspace(0,2*np.pi,len(msg.ranges),endpoint=True))
            self.dmin_ = msg.range_min
            #self.dmin_ = max(msg.range_min, self.radius_) # -- radius
            self.dmax_ = msg.range_max
            self.init_ = True
            self.scan_fpt_ = U.ScanFootprint(self.fpt_, self.angle_)
        self.scan_ = np.asarray(msg.ranges, dtype=np.float32)

    def goal_viz(self):
        if self.goal_ is None:
            return

        # viz goal
        gx, gy = self.goal_.x, self.goal_.y
        q = np.linspace(-np.pi, np.pi, num=10)
        x = gx + self.gr_ * np.cos(q)
        y = gy + self.gr_ * np.sin(q)
        ar_r = np.stack([x,y], axis=-1)

        x = gx + (self.gr_+self.s_) * np.cos(q)
        y = gy + (self.gr_+self.s_) * np.sin(q)
        ar_s = np.stack([x,y], axis=-1)

        ar_g = np.concatenate([ar_r, ar_s], axis=0)

        [cx,cy,_], _ = self.tfl_.lookupTransform('odom', 'base_link',  rospy.Time(0))
        x = cx + self.r_ * np.cos(q)
        y = cy + self.r_ * np.sin(q)
        ar_r = np.stack([x,y], axis=-1)

        x = cx + (self.r_+self.s_) * np.cos(q)
        y = cy + (self.r_+self.s_) * np.sin(q)
        ar_s = np.stack([x,y], axis=-1)

        ar_b = np.concatenate([ar_r, ar_s], axis=0)

        ar = np.concatenate([ar_g, ar_b], axis=0)

        self.goal_viz_msg_.header.stamp = rospy.Time.now()
        self.goal_viz_msg_.header.frame_id = 'odom'
        self.goal_viz_msg_.polygon = Polygon([Point32(x,y,0) for (x,y) in ar])
        self.goal_viz_pub_.publish(self.goal_viz_msg_)

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

    def obs_viz(self, dx, dy, h):
        uv = np.stack([dx,dy], axis=-1) # [n,2]
        uv = uv.dot(U.R(h))

        self.obs_viz_msg_.header.frame_id = 'base_link'
        self.obs_viz_msg_.header.stamp = rospy.Time.now()

        self.obs_viz_msg_.poses = []

        uv = sorted(uv, key=np.linalg.norm, reverse=True)

        for (x,y) in uv[:5]:
            # show the most important <=10 contributors
            p = Pose()
            h = np.arctan2(y,x)
            p.orientation.z=np.sin(h/2.)
            p.orientation.w=np.cos(h/2.)
            p.position.x=0
            p.position.y=0
            self.obs_viz_msg_.poses.append(p)

        self.obs_viz_pub_.publish(self.obs_viz_msg_)

    def cls_viz(self, cls):
        col = ColorRGBA(0.0,0.0,1.0,1.0)

        msg = Marker()
        msg.lifetime = rospy.Duration(1.0)
        msg.header.frame_id = 'base_link'

        msg.type = msg.POINTS
        msg.action = msg.ADD
        msg.points = [Point(x=x,y=y) for (x,y) in cls]
        msg.colors = [col for _ in msg.points]
        msg.scale.x = msg.scale.y = msg.scale.z = 0.1

        self.cls_viz_pub_.publish(msg)

    def step(self):
        if self.goal_ is None:
            return

        self.goal_viz()

        if self.scan_ is not None:
            scan_offset = self.scan_fpt_(self.scan_, scale=0.9)
            now = rospy.Time.now()
            dt = (now - self.last_cmd_).to_sec()
            rq = np.stack([self.scan_, self.angle_], axis=-1)
            mask = (self.dmin_ < self.scan_) & (self.scan_ < self.dmax_)


            mask &= (scan_offset > 0) # internal-points rectification

            rq = rq[mask]
            if np.size(rq) <= 0:
                return

            h = self.get_heading()

            # apply goal potential
            [cx,cy,_], _ = self.tfl_.lookupTransform('odom', 'base_link',  rospy.Time(0))
            gx, gy = self.goal_.x, self.goal_.y
            d = np.linalg.norm([gx-cx,gy-cy])
            q = np.arctan2(gy-cy,gx-cx)

            gdx,gdy = 0,0
            if d < self.gr_:
                gdx=0
                gdy=0
                rospy.loginfo("Reached Goal!")
                self.goal_ = None
                self.cmd_pub_.publish(Twist())
                return
            elif d <= self.gr_ + self.s_:
                gdx = self.a_ * (d - self.r_) * np.cos(q)
                gdy = self.a_ * (d - self.r_) * np.sin(q)
            else:
                gdx = self.a_ * self.s_ * np.cos(q)
                gdy = self.a_ * self.a_ * np.sin(q)

            # --- goal part done.

            # cluster obstacles

            r, q = rq.T

            # collect obstacle potentials
            dx = np.zeros_like(r)
            dy = np.zeros_like(r)

            xy = U.rq2xy(rq)
            db = DBSCAN(eps=0.2, min_samples=3).fit(xy)
            labels = db.labels_
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            print n_clusters
            cls = []
            for i in range(n_clusters):
                cl = xy[labels==i]
                p  = np.mean(cl, axis=0)
                # apply tangential potential field
                # (moving away from cluster center)
                dx[labels==i] += (1.0 / r[labels==i]) * (xy[labels==i][:,0] - p[0])
                dy[labels==i] += (1.0 / r[labels==i]) * (xy[labels==i][:,1] - p[1])
                cls.append(p)
            self.cls_viz(cls)

            q = U.anorm(q+h) # rotate to align obstacle points with odom coordinates

            obs = scan_offset[mask] < 0.05
            if np.any(obs):
                rospy.loginfo("obs!")
            n_obs = np.logical_not(obs)
            out = (r > (self.s_ + self.r_))

            dx[obs] = - 9. * np.sign(np.cos(q[obs]))
            dy[obs] = - 9. * np.sign(np.sin(q[obs]))

            dx[n_obs] = - self.b_ * (1.0 / (r[n_obs])) * np.cos(q[n_obs])
            dy[n_obs] = - self.b_ * (1.0 / (r[n_obs])) * np.sin(q[n_obs])

            dx[out] *= 0
            dy[out] *= 0

            self.obs_viz(dx,dy,h)

            uv = np.stack([dx,dy], axis=-1)
            #uv = sorted(uv, key=np.linalg.norm, reverse=True)
            uv = np.sum(uv, axis=0, keepdims=True)
            uv += [[gdx,gdy]]
            #uv /= np.linalg.norm(uv, axis=-1)

            #uv = np.reshape(weight, [-1,1]) * np.stack([dx,dy], axis=-1) #N,2
            #uv = np.sum(uv, axis=0, keepdims=True)
            #uv += [[gdx,gdy]] # (1,2)
            #uv /= np.linalg.norm(uv, axis=-1)

            #weight[obs] = np.sign(np.cos(q))

            #if np.any(obs):
            #    rospy.loginfo("Obstacle!")
            #    uq = np.mean( U.anorm(q[obs]) )
            #    uv = np.reshape([-np.cos(uq), -np.sin(uq)], [1,2])
            #else:
            #    weight = self.b_ * (self.s_ + self.r_ - r)
            #    weight *= (r < (self.s_ + self.r_) ) # out of influence
            #    dx = np.cos(q)
            #    dy = np.sin(q)

            #    uv = np.reshape(weight, [-1,1]) * np.stack([dx,dy], axis=-1) #N,2
            #    uv = np.sum(uv, axis=0, keepdims=True)
            #    uv += [[gdx,gdy]] # (1,2)
            #    uv /= np.linalg.norm(uv, axis=-1)

            uv = uv.dot(U.R(h)) # back to base_link - note R(h) transposed twice
            self.cmd_viz(uv[0])

            v_v, v_q = U.xy2rq(uv)[0]
            if uv[0,0] < 0.0: # negative - back up
                v_v *= -1.0
                v_q = U.anorm(v_q + np.pi)

            #v = self.pid_v_(0.4 / np.mean(r) ,dt) #self.k_v_ * (np.min(r))#, dt)
            #if uv[0,0] < 0.0: # negative
            #    v = 0.0
            #else:
            #    v = self.k_v_# * np.min(r)
            v = self.pid_v_(v_v, dt)
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
