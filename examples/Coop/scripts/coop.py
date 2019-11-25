#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
import argparse

# keyboard
import sys
import select
import tty
import termios

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def teleop(args):
    pub = rospy.Publisher('/simple_robot/servo_cf', Wrench, queue_size=1)
    rospy.init_node('coop', anonymous=True)
    rate = rospy.Rate(10) # 10Hz

    print("move along x using 4 and 6")
    print("move along y using 8 and 2")
    print("move along z using 7 and 9")

    # increment
    increment = 0.5

    while not rospy.is_shutdown():
        # Pose
        t = Wrench()
        # t.header.frame_id = "map"

        if args.constz:
            t.force.z = -increment
        else:
            if isData():
                c = sys.stdin.read(1)
                if c == '\x1b':         # x1b is ESC
                    break
                elif c == '4':
                    t.force.x = -increment
                elif c == '6':
                    t.force.x = increment
                elif c == '8':
                    t.force.y = increment
                elif c == '2':
                    t.force.y = -increment
                elif c == '7':
                    t.force.z = increment
                elif c == '9':
                    t.force.z = -increment

        # t.header.stamp = rospy.Time.now()
        pub.publish(t)

        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--constz', action="store_true", default=False)
    args = parser.parse_args()
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        teleop(args)
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
