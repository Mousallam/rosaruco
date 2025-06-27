#!/usr/bin/env python3
"""Interactive chessboard camera calibration.

OpenCV based calibration that captures frames from the default camera.
Press ``c`` to capture a chessboard view and ``q`` when done. Results
are written to the package ``config`` directory and published on the
``/camera_info`` topic.
"""
import os
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo


def main():
    rospy.init_node("camera_calibration_node")
    board_size = (7, 6)  # inner corners
    square_size = rospy.get_param("~square_size", 0.025)  # meters
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points = []
    img_points = []

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Could not open camera")
        return

    rospy.loginfo(
        "Calibration started. Press 'c' to capture, 'q' to finish")
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, board_size, None)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, board_size, corners, found)
        cv2.imshow("calibration", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') and found:
            obj_points.append(objp.copy())
            img_points.append(corners)
            rospy.loginfo("Captured image %d", len(obj_points))
        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

    if len(obj_points) < 3:
        rospy.logwarn("Not enough samples for calibration")
        return

    ret, mtx, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None)
    if not ret:
        rospy.logerr("Calibration failed")
        return

    rp = rospkg.RosPack()
    pkg_path = rp.get_path('aruco_localization')
    yaml_path = os.path.join(pkg_path, 'config', 'camera_info.yaml')
    data = {
        'image_width': int(gray.shape[1]),
        'image_height': int(gray.shape[0]),
        'camera_name': 'aruco_camera',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': mtx.flatten().tolist()},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1, 'cols': len(dist.flatten()),
            'data': dist.flatten().tolist()
        }
    }
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    for k, v in data.items():
        fs.write(k, np.array(v) if isinstance(v, list) else v)
    fs.release()
    rospy.loginfo("Calibration saved to %s", yaml_path)

    pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=1)
    msg = CameraInfo()
    msg.width = data['image_width']
    msg.height = data['image_height']
    msg.K = mtx.flatten().tolist()
    msg.D = dist.flatten().tolist()
    msg.distortion_model = 'plumb_bob'
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
