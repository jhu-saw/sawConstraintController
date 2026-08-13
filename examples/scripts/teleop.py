#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

# keyboard
import sys
import select
import tty
import termios

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def teleop():
    pub = rospy.Publisher('/simple_robot/servo_cp', PoseStamped, queue_size=1)
    rospy.init_node('teleop', anonymous=True)
    rate = rospy.Rate(10) # 10Hz

    print("move along x using 4 and 6")
    print("move along y using 8 and 2")
    print("move along z using 7 and 9")

    # Pose
    t = PoseStamped()
    t.pose.orientation.w = 1.0
    t.header.frame_id = "map"

    # increment
    increment = 0.005 # m

    while not rospy.is_shutdown():
        if isData():
            c = sys.stdin.read(1)
            if c == '\x1b':         # x1b is ESC
                break
            elif c == '4':
                t.pose.position.x -= increment
            elif c == '6':
                t.pose.position.x += increment
            elif c == '8':
                t.pose.position.y += increment
            elif c == '2':
                t.pose.position.y -= increment
            elif c == '7':
                t.pose.position.z -= increment
            elif c == '9':
                t.pose.position.z += increment

        t.header.stamp = rospy.Time.now()
        pub.publish(t)

        rate.sleep()


if __name__ == "__main__":
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        teleop()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
