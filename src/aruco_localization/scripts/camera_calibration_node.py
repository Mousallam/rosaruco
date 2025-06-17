#!/usr/bin/env python3
"""Camera calibration node.

Captures chessboard images and stores intrinsic parameters to a YAML file.
"""
import rospy
import cv2
from sensor_msgs.msg import CameraInfo


def main():
    rospy.init_node('camera_calibration_node')

    # TODO: implement chessboard calibration using OpenCV
    # Store results to `camera_info.yaml` under package `config` folder
    rospy.loginfo('Camera calibration node started. Not yet implemented.')

    # Placeholder publish loop
    pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(CameraInfo())
        rate.sleep()


if __name__ == '__main__':
    main()
