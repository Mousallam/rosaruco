#!/usr/bin/env python3
import math
import yaml
import rospy
import rospkg
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Pose2D

class MotionCommander:
    def __init__(self):
        rospy.init_node('motion_commander')

        # load map of marker positions
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('aruco_localization')
        with open(pkg_path + '/maps/map.yaml') as f:
            self.map = yaml.safe_load(f)

        # drive speed (m/s)
        self.speed = 0.1
        # current robot pose from aruco_detector
        self.pose = None
        # latest goal marker ID
        self.goal_id = None

        # subscribers & publishers
        rospy.Subscriber('/robot_pose', Pose2D, self.pose_cb)
        rospy.Subscriber('/goal_id',     Int32,  self.goal_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def pose_cb(self, msg):
        self.pose = msg

    def goal_cb(self, msg):
        self.goal_id = msg.data
        rospy.loginfo("New goal_id set: %d", self.goal_id)

    def run(self):
        rate = rospy.Rate(5)
        est_pose = None

        while not rospy.is_shutdown():
            # wait until both pose & goal_id have been received
            if self.pose is None or self.goal_id is None:
                rate.sleep()
                continue

            goal = self.map.get(f'marker_{self.goal_id}')
            if goal is None:
                rospy.logwarn("Unknown goal_id %d", self.goal_id)
                rate.sleep()
                continue

            # smooth estimate of current pose
            if est_pose is None:
                est_pose = [self.pose.x, self.pose.y]
            else:
                est_pose[0] = 0.5 * est_pose[0] + 0.5 * self.pose.x
                est_pose[1] = 0.5 * est_pose[1] + 0.5 * self.pose.y

            # compute distance to goal
            dx = goal[0] - est_pose[0]
            dy = goal[1] - est_pose[1]
            dist = math.hypot(dx, dy)

            # if close enough → stop
            if dist < 0.05:
                self.cmd_pub.publish(Twist())
                rospy.loginfo("Reached marker_%d", self.goal_id)
                break

            # otherwise, step forward
            tw = Twist()
            tw.linear.x = self.speed
            self.cmd_pub.publish(tw)

            # sleep for exactly the travel time
            travel_time = min(dist, 0.1) / self.speed
            rospy.sleep(travel_time)

            # stop briefly and wait for new pose
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.2)
            rate.sleep()

if __name__ == '__main__':
    MotionCommander().run()
