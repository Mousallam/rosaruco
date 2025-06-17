#!/usr/bin/env python3
"""Detects ArUco markers and publishes corrected robot pose."""
import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray


class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector')
        self.bridge = CvBridge()
        self.cam_info = None

        rospy.Subscriber('/camera/image_raw', Image, self.image_cb)
        rospy.Subscriber('/camera_info', CameraInfo, self.info_cb)
        self.pose_pub = rospy.Publisher('aruco/poses', PoseArray, queue_size=1)
        self.id_pub = rospy.Publisher('aruco/ids', Int32MultiArray, queue_size=1)

        self.dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

    def info_cb(self, msg):
        self.cam_info = msg

    def image_cb(self, msg):
        if self.cam_info is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dict)
        if ids is None:
            return

        pose_msg = PoseArray()
        pose_msg.header.stamp = rospy.Time.now()
        id_msg = Int32MultiArray()
        id_msg.data = ids.flatten().tolist()
        # TODO: estimate pose using self.cam_info and compensate 30cm offset
        self.pose_pub.publish(pose_msg)
        self.id_pub.publish(id_msg)

    def spin(self):
        rospy.loginfo('Aruco detector started')
        rospy.spin()


if __name__ == '__main__':
    ArucoDetector().spin()
