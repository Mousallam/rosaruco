#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

# Node to calibrate camera using chessboard captures and publish intrinsics

def main():
    rospy.init_node('camera_intrinsics_publisher')

    # Publisher for the calibrated intrinsics
    caminfo_pub = rospy.Publisher(
        '/camera/camera_info', CameraInfo, queue_size=1)

    # Chessboard parameters: inner corners count (7 x 4) and square size (30 mm)
    pattern_size = (7, 4)  # inner corners per row, col
    square_size = 0.03     # meters

    # Prepare object points: (0,0,0), (1,0,0), ... scaled by square_size
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2) * square_size

    # Lists to store object and image points from all views
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    bridge = CvBridge()

    rospy.loginfo("Press 'c' to capture frame, 'q' to finish and calibrate")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Frame grab failed")
            continue

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Live chessboard detection for visual feedback
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            # refine corner locations
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            # draw detected corners
            cv2.drawChessboardCorners(display, pattern_size, corners2, True)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(10) & 0xFF

        if key != 0xFF:
            rospy.loginfo("Key pressed: 0x%02X (%s)", key, chr(key) if 32<key<127 else '')

        if key == ord('c'):
            # capture for calibration
            if found:
                objpoints.append(objp)
                imgpoints.append(corners2)
                rospy.loginfo("Captured image %d", len(objpoints))
            else:
                rospy.logwarn("Chessboard not found, try again")

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()

    if len(objpoints) < 3:
        rospy.logerr("Not enough captures (%d) for calibration", len(objpoints))
        return

    # Run calibration
    image_size = gray.shape[::-1]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None)
    rospy.loginfo("Calibration RMS error: %.4f", ret)
    rospy.loginfo("Camera matrix:\n%s", mtx)
    rospy.loginfo("Distortion coeffs: %s", dist.ravel())

    # Prepare CameraInfo message with intrinsics
    cam_info = CameraInfo()
    cam_info.header.frame_id = "camera"
    cam_info.height = image_size[1]
    cam_info.width  = image_size[0]
    cam_info.distortion_model = "plumb_bob"
    cam_info.D = dist.ravel().tolist()
    cam_info.K = [mtx[0,0], mtx[0,1], mtx[0,2],
                  mtx[1,0], mtx[1,1], mtx[1,2],
                  mtx[2,0], mtx[2,1], mtx[2,2]]
    cam_info.R = [1.,0.,0., 0.,1.,0., 0.,0.,1.]
    cam_info.P = [mtx[0,0], 0.,        mtx[0,2], 0.,
                   0.,       mtx[1,1], mtx[1,2], 0.,
                   0.,       0.,        1.,       0.]

    rate = rospy.Rate(1)  # publish at 1 Hz
    rospy.loginfo("Publishing CameraInfo at 1 Hz")
    while not rospy.is_shutdown():
        cam_info.header.stamp = rospy.Time.now()
        caminfo_pub.publish(cam_info)
        rate.sleep()

if __name__ == '__main__':
    main()
