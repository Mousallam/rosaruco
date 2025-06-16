#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class ControlMovementNode(object):
    """
    ROS node that subscribes to a Path and commands the robot to move
    forward if the marker's y-position is negative, backward if positive,
    and stops when y is within ±tolerance.
    """
    def __init__(self):
        rospy.init_node('control_movement', anonymous=True)

        # Subscribe to trajectory Path
        self.path_sub = rospy.Subscriber('/trajectory', Path, self.path_cb)
        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Latest target position
        self.latest = None

        # Control parameters
        self.kp = rospy.get_param('~kp', 0.5)              # proportional gain
        self.tolerance = rospy.get_param('~tolerance', 0.05)  # 5 cm
        self.max_speed = rospy.get_param('~max_speed', 0.2)   # m/s

        rospy.loginfo("[control_movement] Ready: forward/back control on y-axis."
                      " y< -tol -> forward; y> tol -> backward; |y|<=tol -> stop.")

    def path_cb(self, msg):
        """Store the latest point from the received Path."""
        if msg.poses:
            self.latest = msg.poses[-1].pose.position

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()

            if self.latest is not None:
                y_err = self.latest.y
                if y_err < -self.tolerance:
                    # Marker is left (negative y): move forward
                    speed = min(self.max_speed, -self.kp * y_err)
                    twist.linear.x = speed
                    rospy.loginfo("y_err=%.3f < -tol: forward at %.3f m/s", y_err, speed)
                elif y_err > self.tolerance:
                    # Marker is right (positive y): move backward
                    speed = max(-self.max_speed, -self.kp * y_err)
                    twist.linear.x = speed
                    rospy.loginfo("y_err=%.3f > tol: backward at %.3f m/s", y_err, speed)
                else:
                    # Within tolerance
                    twist.linear.x = 0.0
                    rospy.loginfo("y_err=%.3f within tol: stopping.", y_err)
            else:
                rospy.loginfo_throttle(5, "[control_movement] Waiting for /trajectory messages...")

            # Always zero angular velocity
            twist.angular.z = 0.0
            # Publish the command
            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    node = ControlMovementNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
