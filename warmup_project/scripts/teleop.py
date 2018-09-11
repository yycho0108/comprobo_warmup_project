#/usr/bin/env python

# currently just holds sample key handling code

import tty
import select
import sys
import termios

import rospy
from geometry_msgs.msg import Twist

class Teleop(object):
    def __init__(self):
        # sys
        self.settings_ = termios.tcgetattr(sys.stdin)

        # handle ros paramters
        self.period_  = rospy.get_param('~period', 0.0)
        self.v_scale_ = rospy.get_param('~v_scale', 1.0)
        self.w_scale_ = rospy.get_param('~w_scale', 1.0)
        self.timeout_ = rospy.get_param('~timeout', 1.0)
        self.vwmap_ = {
                'i' : [1.0, 0.0],
                'j' : [0.0, 1.0],
                'k' : [0.0, 0.0],
                'l' : [0.0, -1.0],
                ',' : [-1.0, 0.0],
                'u' : [1.0, 1.0],
                'o' : [1.0, -1.0],
                'm' : [-1.0, -1.0],
                '.' : [-1.0, 1.0]
                }

        # track commands
        self.last_cmd_ = rospy.Time(0)
        self.last_pub_ = rospy.Time(0)
        self.cmd_pub_  = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def get_key(self):
        """ non-blocking keyboard input """
        tty.setraw(sys.stdin.fileno())
        rdy = select.select([sys.stdin], [], [], 0)[0]
        if rdy:
            #key = sys.stdin.read(1)
            key = sys.stdin.read(1)
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)
        else:
            key = None
        return key

    def run(self):
        repeat_flag = (self.period_ > 0)
        cmd_vel     = Twist()

        while not rospy.is_shutdown():
            # current time (technically has some delay)
            now = rospy.Time.now()

            # handle key input
            k = self.get_key()
            if k == '\x03': # SIGINT
                break
            if k in self.vwmap_:
                v, w = self.vwmap_[k]
                cmd_vel.linear.x  = v * self.v_scale_
                cmd_vel.angular.z = w * self.w_scale_
                self.last_cmd_ = now
                self.last_pub_ = now
                self.cmd_pub_.publish(cmd_vel)
            else:
                if k is not None:
                    print 'k', k

            # handle timeout
            if (now - self.last_cmd_).to_sec() > (self.timeout_):
                cmd_vel.linear.x  = 0
                cmd_vel.angular.z = 0

            # handle publishing
            if repeat_flag and (now - self.last_pub_).to_sec() > (self.period_):
                self.cmd_pub_.publish(cmd_vel)
                self.last_pub_ = now

def main():
    rospy.init_node('comprobo_teleop_key')
    node = Teleop()
    node.run()

if __name__ == "__main__":
    main()

