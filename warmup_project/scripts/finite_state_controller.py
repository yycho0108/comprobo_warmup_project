#!/usr/bin/env python

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import tf
import rospy
import numpy as np
from tf_conversions import posemath as pm
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32, Twist

from warmup_project import utils as U
from warmup_project.person_detector import PersonDetector
from warmup_project.person_tracker import PersonTracker
from warmup_project.pid import PID

from abc import abstractmethod, ABCMeta

class FiniteStateController(object):
    def __init__(self, states):
        self.states_ = states
        self.state_ = ''

    def state(self):
        return self.states_[self.state_]

    def start(self, init_state, arg):
        self.state_ = init_state
        self.state().start(arg)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(2.0, 'Current State : {}'.format(self.state_))
            next = self.state().step()
            if self.state_ is not next:
                # transition
                data = self.state().stop()
                self.state_ = next
                self.state().start(data)
            rate.sleep()

class State(object):
    __metaclass__ = ABCMeta
    def __init__(self): pass

    @abstractmethod
    def start(self, arg):
        """
        Performs initialization operations,
        such as creating / starting all processing handles
        and resetting cache.
        Accepts arguments from previous state.
        """
        pass

    @abstractmethod
    def stop(self):
        """
        Performs cleanup operations,
        such as destroying callbacks and shutting down processing handles.
        Returns arguments for next state.
        """
        return None

    @abstractmethod
    def step(self):
        """
        Performs all iterative actions.
        Returns next state identifier
        """
        return ''

class Idle(State):
    def __init__(self):
        super(Idle, self).__init__()
        self.rec_req_ = False
        self.rec_srv_ = None
        self.cmd_pub_ = None

    def start(self, arg):
        self.rec_srv_ = rospy.Service("record", Empty, self.rec_cb)
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def stop(self):
        if self.rec_srv_ is not None:
            self.rec_srv_.shutdown('Idle State Finished')
        self.rec_srv_ = None

        if self.cmd_pub_ is not None:
            self.cmd_pub_.unregister()
        self.cmd_pub_ = None
        return None

    def rec_cb(self,_):
        self.rec_req_ = True

    def step(self):
        self.cmd_pub_.publish(Twist())
        if self.rec_req_:
            return 'Record'
        else:
            return 'Idle'

class Record(State):
    """
    record trajectories from exemplar.
    """
    def __init__(self,
            min_x=0.3, max_x=2.0, min_y=-0.5, max_y=0.5,
            v_max=6.0
            ):
        super(Record, self).__init__()

        # processor handles
        self.detector_ = PersonDetector(min_x,max_x,min_y,max_y)
        self.tracker_  = PersonTracker(vmax=v_max)

        # reset all cache
        self.reset()

        # Create empty ROS handles
        self.tfl_ = tf.TransformListener()
        self.viz_pub_ = None # rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.calib_srv_ = None # rospy.Service("calibrate", Empty, self.calibrate)
        self.scan_sub_ = None # rospy.Subscriber('scan', LaserScan, self.scan_cb)
        self.play_srv_ = None # rospy.Service('play', Empty, self.play_cb)

    def play_cb(self, _):
        self.play_req_ = True
        return EmptyResponse()

    def reset(self):
        # time
        now = rospy.Time.now().to_sec()
        self.time_ = now

        # scan data
        self.scan_ = None
        self.new_scan_ = False
        self.angle_ = None
        self.dmin_ = None
        self.dmax_ = None
        self.init_ = False

        # calibration
        self.calibrated_ = False
        self.calibrate_req_ = False

        # transition
        self.play_req_ = False

        # output data
        self.trajectory_=[]

    def start(self, arg):
        print 'Starting!'
        self.reset()
        # create ros handles
        self.viz_pub_ = rospy.Publisher('viz_pt', Marker, queue_size=2)
        self.calib_srv_ = rospy.Service("calibrate", Empty, self.calibrate)
        self.play_srv_ = rospy.Service('play', Empty, self.play_cb)
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)

    def stop(self):
        if self.scan_sub_ is not None:
            self.scan_sub_.unregister()
        self.scan_sub_ = None

        if self.play_srv_ is not None:
            self.play_srv_.shutdown('Record State Finished')
        self.play_srv_ = None

        if self.calib_srv_ is not None:
            self.calib_srv_.shutdown('Record State Finished')
        self.calib_srv_ = None

        if self.viz_pub_ is not None:
            self.viz_pub_.unregister()
        self.viz_pub_ = None

        return np.copy(self.trajectory_)

    def calibrate(self, _):
        rospy.loginfo("Stand in front of the robot until calibration is complete")
        self.calibrate_req_ = True
        self.calibrated_ = False
        return EmptyResponse()

    def publish(self, suc, info):
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

    def step(self):
        if self.play_req_:
            return 'Replay'

        if (self.scan_ is not None):
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

        if self.calibrated_:
            now = rospy.Time.now().to_sec()
            dt = (now - self.time_)
            xy_valid = U.rq2xy(rq_valid)
            xy_valid_global = self.apply_scan_tf(xy_valid)
            p_l, p_r, l_lost, r_lost = self.tracker_(xy_valid_global, dt)

            if (l_lost > 3 or r_lost > 3): # lost for 3 successive frames
                self.calibrated_ = False
                self.calibrate_req_ = True
                return 'Record'

            # controls computation
            p = np.mean([p_l,p_r], axis=0)

            self.trajectory_.append(p)

            # logging
            self.publish(True, (p_l, p_r))
            self.time_ = now

        return 'Record'

def to_pose2d(msg):
    x, y  = [msg.position.x, msg.position.y]
    rz = 2.0 * np.arctan2(msg.orientation.z, msg.orientation.w)
    pose = Pose2D(x=x,y=y,theta=rz)
    return pose

class Replay(object):
    """
    replay trajectories from exemplar.
    """
    def __init__(self):
        super(Replay, self).__init__()

        # static handles
        self.pid_v_ = PID(kp=0.5, ki=0.01, max_i=0.5, max_u=1.0)
        self.pid_w_ = PID(kp=1.0, ki=0.03, max_i=1.0, max_u=5.0)
        self.tfl_ = tf.TransformListener()

        self.stop_srv_ = None
        self.cmd_pub_ = None

        # reset cache
        self.reset()

    def reset(self):
        self.pid_v_.reset()
        self.pid_w_.reset()
        self.wpt_idx_ = 0 # waypoint index
        self.stop_req_ = False
        self.time_ = rospy.Time.now()
        self.cmd_vel_ = Twist()

    def start(self, arg):
        self.reset()
        self.stop_srv_ = rospy.Service("stop", Empty, self.stop_cb)
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.trajectory_ = arg

    def stop(self):
        if self.stop_srv_ is not None:
            self.stop_srv_.shutdown('Replay State Finished')
        self.stop_srv_ = None

        if self.cmd_pub_ is not None:
            self.cmd_pub_.unregister()
        self.cmd_pub_ = None
        return None

    def stop_cb(self, _):
        self.stop_req_ = True

    def step(self):
        if self.stop_req_:
            # stop requested
            return 'Idle'

        if self.wpt_idx_ >= len(self.trajectory_):
            # reached goal
            return 'Idle'

        now = rospy.Time.now()
        target = self.trajectory_[self.wpt_idx_] # = (x,y)
        try:
            t, q   = self.tfl_.lookupTransform('odom', 'base_link', rospy.Time(0))
        except Exception as e:
            rospy.logerr('?? : {}'.format(e))
            self.time_ = rospy.Time.now()
            return 'Replay'

        source = (t[0], t[1])
        heading =  2.0 * np.arctan2(q[2], q[3])

        d_pos = np.subtract(target, source)
        dr, dq = U.xy2rq(d_pos)

        d_pos = U.R(-heading).dot(d_pos) # delta in base_link frame
        d_r, d_q = U.xy2rq(d_pos)

        if np.abs(d_r) < 0.1:
            # hit current waypoint, get next one
            self.wpt_idx_ += 1
        else:
            # continue reaching current waypoint
            if d_pos[0] < 0.0: # negative - back up
                d_r *= -1.0
                d_q = U.anorm(d_q + np.pi)

            dt = (now - self.time_).to_sec()

            v = self.pid_v_(d_r, dt)
            w = self.pid_w_(d_q, dt)

            self.cmd_vel_.linear.x = v
            self.cmd_vel_.angular.z = w
            self.cmd_pub_.publish(self.cmd_vel_)
        self.time_ = now
        return 'Replay'

def main():
    rospy.init_node('fsm')
    fsm = FiniteStateController(
            states={
                'Idle' : Idle(),
                'Record' : Record(),
                'Replay' : Replay()
                })

    # test record ...
    # fsm.start('Record', None)
    # fsm.run()

    # test replay ...
    # h = -np.linspace(-np.pi, np.pi, num=20)
    # c = np.cos(h)
    # s = np.sin(h)
    # trajectory = 2.0 * np.stack([c,s], axis=-1)
    # print trajectory
    # fsm.start('Replay', trajectory)
    # fsm.run()

    fsm.start('Idle', None)
    fsm.run()

if __name__ == "__main__":
    main()
