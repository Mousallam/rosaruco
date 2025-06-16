#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

class OdomControlNode(object):
    """Drive the robot using odometry until it reaches the detected marker."""
    def __init__(self):
        rospy.init_node('odom_control_movement', anonymous=True)

        self.path_sub = rospy.Subscriber('/trajectory', Path, self.path_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.target_dist = None
        self.start_x = None
        self.current_x = None

        self.kp = rospy.get_param('~kp', 0.5)
        self.tolerance = rospy.get_param('~tolerance', 0.05)
        self.max_speed = rospy.get_param('~max_speed', 0.2)

    def path_cb(self, msg):
        if msg.poses:
            self.target_dist = msg.poses[-1].pose.position.x
            self.start_x = None
            rospy.loginfo('New target distance: %.2f m', self.target_dist)

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            if self.target_dist is not None and self.current_x is not None:
                if self.start_x is None:
                    self.start_x = self.current_x
                travelled = self.current_x - self.start_x
                error = self.target_dist - travelled
                if abs(error) > self.tolerance:
                    speed = max(-self.max_speed, min(self.max_speed, self.kp * error))
                    twist.linear.x = speed
                else:
                    twist.linear.x = 0.0
                    self.target_dist = None
                    rospy.loginfo('Target reached within tolerance')
            else:
                rospy.loginfo_throttle(5, '[odom_control_movement] Waiting for trajectory and odom...')

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    node = OdomControlNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
