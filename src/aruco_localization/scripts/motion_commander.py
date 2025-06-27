#!/usr/bin/env python3
"""Navigation controller that moves the robot towards a marker ID.

The controller subscribes to ``/robot_pose`` published by ``aruco_detector``
and sends ``Twist`` commands on ``/cmd_vel``.  The goal marker id is provided
via the ``~goal_id`` parameter.  The robot moves with a speed of ``0.1`` and
requests a new pose estimate after each small step.
"""
import math
import os
import yaml
import rospy
import rospkg
from geometry_msgs.msg import Twist, Pose2D


class MotionCommander:
    def __init__(self):
        rospy.init_node('motion_commander')
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('aruco_localization')
        map_file = os.path.join(pkg_path, 'maps', 'map.yaml')
        with open(map_file) as f:
            self.map = yaml.safe_load(f)

        self.goal_id = rospy.get_param('~goal_id', 0)
        self.speed = 0.1
        self.pose = None
        self.last_update = rospy.Time(0)

        rospy.Subscriber('/robot_pose', Pose2D, self.pose_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def pose_cb(self, msg):
        self.pose = msg
        self.last_update = rospy.Time.now()

    def run(self):
        goal = self.map.get(f'marker_{self.goal_id}')
        if goal is None:
            rospy.logerr('Unknown goal id %s', self.goal_id)
            return
        rate = rospy.Rate(5)
        est_pose = None
        while not rospy.is_shutdown():
            if self.pose is not None:
                if est_pose is None:
                    est_pose = [self.pose.x, self.pose.y]
                else:
                    est_pose[0] = 0.5 * est_pose[0] + 0.5 * self.pose.x
                    est_pose[1] = 0.5 * est_pose[1] + 0.5 * self.pose.y
            if est_pose is None:
                rate.sleep()
                continue
            dx = goal[0] - est_pose[0]
            dy = goal[1] - est_pose[1]
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self.cmd_pub.publish(Twist())
                rospy.loginfo('Goal reached')
                break
            step = min(dist, 0.1)
            tw = Twist()
            tw.linear.x = self.speed
            self.cmd_pub.publish(tw)
            rospy.sleep(step / self.speed)
            self.cmd_pub.publish(Twist())
            # predict next pose
            est_pose[0] += math.cos(0) * step
            est_pose[1] += math.sin(0) * step
            # wait a bit for a new pose estimate
            rospy.sleep(0.2)
            rate.sleep()


if __name__ == '__main__':
    MotionCommander().run()
