#!/usr/bin/env python3

import os
import yaml
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import rospy
import rospkg
from geometry_msgs.msg import Pose2D

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector')
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('aruco_localization')

        # Load camera intrinsics
        info_file = os.path.join(pkg_path, 'config', 'camera_info.yaml')
        fs = cv2.FileStorage(info_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode('camera_matrix').mat()
        self.dist_coeffs = fs.getNode('distortion_coefficients').mat()
        fs.release()

        # Load world map of marker positions (marker_i -> [x, y])
        map_file = os.path.join(pkg_path, 'maps', 'map.yaml')
        with open(map_file) as f:
            self.map = yaml.safe_load(f)

        self.marker_length = 0.1  # meters
        # Offset to shift the computed position (x, y)
        self.camera_offset = rospy.get_param('~camera_offset', [0.0, 0.0])

        # Video capture
        self.cap = cv2.VideoCapture(0)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.pose_pub = rospy.Publisher('/robot_pose', Pose2D, queue_size=1)

    def compute_pose(self, ids, corners):
        xs, ys, thetas = [], [], []
        # Estimate pose for each detected marker
        for i, marker_id in enumerate(ids.flatten()):
            # Only consider markers 0-3
            key = f'marker_{marker_id}'
            if key not in self.map:
                continue

            # estimatePoseSingleMarkers expects a list of corners
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                [corners[i]],
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )
            # extract single pose
            rvec = rvecs[0, 0]
            tvec = tvecs[0, 0]

            # world position of the marker (z ignored)
            wx, wy = self.map[key]

            # Rotation: marker -> camera
            R_mc, _ = cv2.Rodrigues(rvec)
            # transform camera -> marker (inverse)
            R_cm = R_mc.T

            # Camera position in world: marker_pos - R_cm * tvec
            dott = R_cm.dot(tvec)
            cam_pos = np.array([wx, wy, 0.0]) - [dott[0], -dott[1], dott[2]]
            xs.append(cam_pos[0])
            ys.append(cam_pos[1])

            # Yaw of camera in world frame
            yaw = math.atan2(R_cm[1, 0], R_cm[0, 0])
            thetas.append(yaw)

        if not xs:
            return None

        # Average if multiple markers seen
        x_avg = sum(xs) / len(xs)
        y_avg = sum(ys) / len(ys)
        theta_avg = sum(thetas) / len(thetas)

        return x_avg, y_avg, theta_avg

    def spin(self):
        if not self.cap.isOpened():
            rospy.logerr('Camera not available')
            return
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rate.sleep()
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

            if ids is not None:
                pose = self.compute_pose(ids, corners)
                aruco.drawDetectedMarkers(frame, corners, ids)

                if pose:
                    x, y, theta = pose
                    print(pose)
                    # apply offset
                    x += self.camera_offset[0]
                    y += self.camera_offset[1]

                    # Publish 2D pose
                    msg = Pose2D()
                    msg.x = x
                    msg.y = y
                    msg.theta = theta
                    self.pose_pub.publish(msg)

            # Display for debugging
            cv2.imshow('aruco_detection', frame)
            cv2.waitKey(1)
            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ArucoDetector().spin()
