#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np
from tf_conversions import posemath as pm
from geometry_msgs.msg import Pose, Pose2D, Twist
from warmup_project.utils import anorm, adiff, R

class SquareDriver(object):
    def __init__(self):
        # receive ROS parameters
        self.rate_ = rospy.get_param('~rate', 50.0)
        self.use_tf_ = rospy.get_param('~use_tf', False)
        self.gtol_ = rospy.get_param('~gtol', 5e-2) # goal tolerance
        self.atol_ = rospy.get_param('~atol', np.deg2rad(10)) # waypoint angle tolerance

        rospy.loginfo('Parameters : ')
        rospy.loginfo('loop rate : {}'.format(self.rate_))
        rospy.loginfo('goal tolerance : {}'.format(self.gtol_))
        rospy.loginfo('control loop angular tolerance : {}'.format(self.atol_))
        rospy.loginfo('use tf for odometry : {}'.format(self.use_tf_))

        self.pose_ = None

        # create ROS handle
        if self.use_tf_:
            self.tfl_ = tf.TransformListener()
        else:
            self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=5)


        self.n_phase_ = 4
        self.phase_  = 0
        self.phase_name_ = ['right', 'top', 'left', 'bottom']
        self.target_ = [(1.0,-1.0),(1.0,1.0), (-1.0,1.0),(-1.0,-1.0)]
        self.target_ = np.asarray(self.target_) * 0.5
        self.origin_ = None

    @staticmethod
    def to_pose2d(msg):
        x, y  = [msg.position.x, msg.position.y]
        rz = 2.0 * np.arctan2(msg.orientation.z, msg.orientation.w)
        pose = Pose2D(x=x,y=y,theta=rz)
        return pose

    def get_odom(self):
        try:
            pose_tf = self.tfl_.lookupTransform('base_link', 'odom', rospy.Time(0))
        except tf.Exception as e:
            rospy.loginfo_throttle(1.0, 'Failed TF Transform : {}'.format(e) )
            return None
        msg = pm.toMsg(pm.fromTf(pose_tf))
        pose = self.to_pose2d(msg)
        return pose
    
    def odom_cb(self, msg):
        # nav_msgs/Odometry --> geometry_msgs/Pose2D
        pose = self.to_pose2d(msg.pose.pose)
        if self.origin_ is None:
            self.origin_ = Pose2D(x=pose.x,y=pose.y,theta=pose.theta)
        self.pose_ = Pose2D(x=pose.x,y=pose.y,theta=pose.theta)

    def run(self):
        rate = rospy.Rate(self.rate_)
        cmd_vel = Twist()
        while not rospy.is_shutdown():
            if (self.origin_ is None):
                # need to initialize origin
                if self.use_tf_:
                    pose = self.get_odom()
                    if pose is not None:
                        self.pose_   = Pose2D(x=pose.x,y=pose.y,theta=pose.theta)
                        self.origin_ = Pose2D(x=pose.x,y=pose.y,theta=pose.theta)
                else:
                    # tf disabled, wait for odom callback
                    pass
            else:
                # has origin
                ox, oy = self.origin_.x, self.origin_.y
                gx, gy = R(self.origin_.theta).dot(self.target_[self.phase_])
                dx, dy = (gx+ox)-self.pose_.x, (gy+oy)-self.pose_.y
                theta = np.arctan2(dy, dx)

                if np.linalg.norm([dx,dy]) < self.gtol_:
                    # reached goal!
                    next_phase = (self.phase_ + 1) % self.n_phase_
                    rospy.loginfo('Reached Goal! {} -> {}'.format(
                        self.phase_name_[self.phase_],
                        self.phase_name_[next_phase]))
                    self.phase_ = next_phase
                else:
                    d_theta = adiff(theta, self.pose_.theta)
                    if np.abs(d_theta) > self.atol_:
                        # turn to align with goal first
                        v = 0
                        w = 0.2 * np.sign(d_theta) # TODO : pid or some loop closing action
                    else:
                        v = 0.2 # TODO : pid or some loop closing action
                        w = 0.05 * d_theta # TODO : pid or some loop closing action
                    cmd_vel.linear.x = v
                    cmd_vel.angular.z = w
                    self.cmd_pub_.publish(cmd_vel)

            rate.sleep()

def main():
    rospy.init_node('comprobo_square_driver')
    node = SquareDriver()
    node.run()

if __name__ == "__main__":
    main()
