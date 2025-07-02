#!/usr/bin/env python3
"""
State-machine based MotionCommander:
1) Wait for robot pose, goal ID, and goal theta
2) Compute dx, dy, distance, heading_to_goal, rot1, rot2
3) TURN: rotate toward goal
4) DRIVE: move straight to goal
5) FINAL_TURN: rotate to final heading
6) LOST: if pose update lost, rotate in place until new update
7) DONE: stop motion and exit
"""
import math
import yaml
import rospy
import rospkg
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist, Pose2D


def wrap_to_pi(angle):
    """Wrap any angle to [-pi, +pi]."""
    return ((angle + math.pi) % (2 * math.pi)) - math.pi


class MotionCommander:
    def __init__(self):
        rospy.init_node('motion_commander')

        # Load map of marker positions
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('aruco_localization')
        with open(pkg_path + '/maps/map.yaml') as f:
            self.map = yaml.safe_load(f)

        # Parameters
        self.speed = rospy.get_param('~drive_speed', 0.1)          # m/s
        self.turn_speed = rospy.get_param('~turn_speed', 0.5)      # rad/s
        self.pos_tol = rospy.get_param('~pos_tolerance', 0.05)     # meters
        self.ang_tol = rospy.get_param('~ang_tolerance', 0.05)     # radians
        self.lost_timeout = rospy.get_param('~lost_timeout', 0.5)  # seconds

        # State
        self.pose = None            # latest Pose2D
        self.last_pose_time = None  # rospy.Time
        self.goal_id = None         # int
        self.goal_theta = None      # float (radians)
        self.goal_x = None          # float
        self.goal_y = None          # float
        self.rot1 = 0.0
        self.rot2 = 0.0

        # State machine
        self.state = 'INIT'
        self.prev_state = None

        # ROS interfaces
        rospy.Subscriber('/robot_pose', Pose2D, self.pose_cb)
        rospy.Subscriber('/goal_id',    Int32,   self.goal_id_cb)
        rospy.Subscriber('/goal_theta', Float32, self.goal_theta_cb)
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    def pose_cb(self, msg):
        self.pose = msg
        self.last_pose_time = rospy.Time.now()

    def goal_id_cb(self, msg):
        self.goal_id = msg.data
        rospy.loginfo("New goal_id: %d", self.goal_id)
        self.try_init()

    def goal_theta_cb(self, msg):
        self.goal_theta = wrap_to_pi(msg.data)
        rospy.loginfo("New goal_theta: %.3f rad", self.goal_theta)
        self.try_init()

    def try_init(self):
        # If we have pose, goal_id and goal_theta, initialize motion
        if self.state == 'INIT' and self.pose and self.goal_id is not None and self.goal_theta is not None:
            # lookup goal position
            key = f"marker_{self.goal_id}"
            pos = self.map.get(key)
            if pos is None:
                rospy.logerr("Unknown goal_id %d", self.goal_id)
                return
            self.goal_x, self.goal_y = pos
            rospy.loginfo("Goal position set: %s (%.2f, %.2f)", key, self.goal_x, self.goal_y)

            # compute initial rotation and travel
            dx = self.goal_x - self.pose.x
            dy = self.goal_y - self.pose.y
            self.rot1 = wrap_to_pi(math.atan2(dy, dx) - self.pose.theta)
            self.rot2 = wrap_to_pi(self.goal_theta - self.pose.theta - self.rot1)

            rospy.loginfo("Initial rot1=%.3f, rot2=%.3f", self.rot1, self.rot2)
            self.transition('TURN')

    def transition(self, new_state):
        rospy.loginfo("State -> %s", new_state)
        self.prev_state = self.state
        self.state = new_state

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def check_lost(self):
        if not self.last_pose_time:
            return False
        return (rospy.Time.now() - self.last_pose_time).to_sec() > self.lost_timeout

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.state != 'DONE':
            # LOST check
            if self.state != 'LOST' and self.check_lost():
                rospy.logwarn("Pose lost: entering LOST state")
                self.transition('LOST')

            if self.state == 'TURN':
                # rotate toward goal
                err = wrap_to_pi((math.pi/2) - math.atan2(-self.goal_y + self.pose.y,
                                            -self.goal_x + self.pose.x) - self.pose.theta)
                rospy.loginfo("The rotation error is %s", err)
                if abs(err) < self.ang_tol:
                    self.publish_zero()
                    self.transition('DRIVE')
                else:
                    tw = Twist()
                    tw.angular.z = self.turn_speed * math.copysign(1.0, err)
                    self.cmd_pub.publish(tw)

            elif self.state == 'DRIVE':
                # compute distance and heading error
                dx = -self.goal_x + self.pose.x
                dy = -self.goal_y + self.pose.y
                dist = math.hypot(dx, dy)
                heading_err = wrap_to_pi((math.pi/2) - math.atan2(dy, dx) - self.pose.theta)
                if dist < self.pos_tol:
                    self.publish_zero()
                    self.transition('FINAL_TURN')
                elif abs(heading_err) > self.ang_tol:
                    self.publish_zero()
                    self.transition('TURN')
                else:
                    tw = Twist()
                    tw.linear.x = self.speed
                    self.cmd_pub.publish(tw)

            elif self.state == 'FINAL_TURN':
                # rotate to final heading
                err = wrap_to_pi(self.goal_theta - self.pose.theta)
                if abs(err) < self.ang_tol:
                    self.publish_zero()
                    rospy.loginfo("Goal reached with final orientation.")
                    self.transition('DONE')
                else:
                    tw = Twist()
                    tw.angular.z = self.turn_speed * math.copysign(1.0, err)
                    self.cmd_pub.publish(tw)

            elif self.state == 'LOST':
                # spin in place until pose update
                tw = Twist()
                tw.angular.z = 0.2
                self.cmd_pub.publish(tw)
                if not self.check_lost():
                    rospy.loginfo("Pose regained: returning to %s", self.prev_state)
                    self.transition(self.prev_state)
            elif self.state == 'INIT':
                self.try_init()

            rate.sleep()

        # final stop
        self.publish_zero()
        rospy.loginfo("MotionCommander done.")


if __name__ == '__main__':
    MotionCommander().run()
