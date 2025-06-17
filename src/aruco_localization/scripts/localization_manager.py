#!/usr/bin/env python3
"""Matches detected markers with a map and publishes robot pose."""
import yaml
import rospy
import rospkg
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Int32MultiArray


class LocalizationManager:
    def __init__(self):
        rospy.init_node('localization_manager')
        map_file = rospy.get_param('~map_file', 'maps/map.yaml')
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('aruco_localization')
        with open(pkg_path + '/' + map_file) as f:
            self.map = yaml.safe_load(f)

        rospy.Subscriber('aruco/poses', PoseArray, self.pose_cb)
        rospy.Subscriber('aruco/ids', Int32MultiArray, self.id_cb)
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=1)
        self.ids = []
        self.poses = []

    def pose_cb(self, msg):
        self.poses = msg.poses
        self.update_pose()

    def id_cb(self, msg):
        self.ids = msg.data
        self.update_pose()

    def update_pose(self):
        if not self.ids or not self.poses:
            return
        # TODO: compute robot pose from detected markers and map
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        self.pose_pub.publish(pose)

    def spin(self):
        rospy.loginfo('Localization manager started with map entries: %s', list(self.map.keys()))
        rospy.spin()


if __name__ == '__main__':
    LocalizationManager().spin()
