#!/usr/bin/env python3
"""Simple motion commander using cmd_vel."""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MotionCommander:
    def __init__(self):
        rospy.init_node('motion_commander')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/command', String, self.cmd_cb)
        self.default_speed = rospy.get_param('~default_speed', 0.2)
        self.rotation_duration = rospy.get_param('~rotation_duration', 2.0)

    def cmd_cb(self, msg):
        twist = Twist()
        if msg.data == 'forward':
            twist.linear.x = self.default_speed
        elif msg.data == 'backward':
            twist.linear.x = -self.default_speed
        elif msg.data == 'rotate_left':
            twist.angular.z = self.default_speed
        elif msg.data == 'rotate_right':
            twist.angular.z = -self.default_speed
        else:
            rospy.logwarn('Unknown command: %s', msg.data)
        self.cmd_pub.publish(twist)

    def spin(self):
        rospy.loginfo('Motion commander started')
        rospy.spin()


if __name__ == '__main__':
    MotionCommander().spin()
