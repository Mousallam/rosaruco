#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CameraNode(object):
    """
    ROS node: detect ArUco ID 0, overlay coords and planned path on realtime frame,
    plot trajectory, provide UI to adjust speeds, and publish trajectory.
    """
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        # Subscribers
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_cb)
        # Publishers
        self.traject_pub = rospy.Publisher('/trajectory', Path, queue_size=1)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.bridge = CvBridge()
        # State
        self.cam_info = None
        self.trajectory = []  # list of (x, y)
        # UI controls for speed
        self.max_lin_speed = rospy.get_param('/control_movement/linear_speed', 0.2)
        self.max_ang_speed = rospy.get_param('/control_movement/angular_speed', 1.0)
        cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('LinSpeed%', 'Controls', int(100*self.max_lin_speed), 200, self.on_lin_speed)
        cv2.createTrackbar('AngSpeed%', 'Controls', int(100*self.max_ang_speed), 500, self.on_ang_speed)
        # Frame display
        cv2.namedWindow('Frame', cv2.WINDOW_NORMAL)
        # Matplotlib for trajectory
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('ArUco ID 0 Trajectory')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.line, = self.ax.plot([], [], '-o')

    def camera_info_cb(self, msg):
        if self.cam_info is None:
            self.cam_info = msg
            rospy.loginfo('Camera info received')

    def on_lin_speed(self, val):
        self.max_lin_speed = val/100.0
        rospy.set_param('/control_movement/linear_speed', self.max_lin_speed)
        rospy.loginfo('Linear speed set to %.2f m/s', self.max_lin_speed)

    def on_ang_speed(self, val):
        self.max_ang_speed = val/100.0
        rospy.set_param('/control_movement/angular_speed', self.max_ang_speed)
        rospy.loginfo('Angular speed set to %.2f rad/s', self.max_ang_speed)

    def run(self):
        # Wait for camera info
        while not rospy.is_shutdown() and self.cam_info is None:
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return

        # Setup ArUco
        try:
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        except AttributeError:
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        params = aruco.DetectorParameters_create()

        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            rospy.logerr('Cannot open camera')
            return

        # Prepare path message
        path_msg = Path()
        path_msg.header.frame_id = self.cam_info.header.frame_id
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                continue

            # Detect markers
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
            h, w = frame.shape[:2]
            center_pt = (w//2, h//2)

            if ids is not None and 0 in ids.flatten():
                # Draw all detected markers
                aruco.drawDetectedMarkers(frame, corners, ids)
                idx = list(ids.flatten()).index(0)

                # Estimate pose of all markers
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.1,
                    np.array(self.cam_info.K).reshape(3,3),
                    np.array(self.cam_info.D)
                )
                tvec = tvecs[idx][0]

                # Marker center in image
                pts = corners[idx][0]
                px, py = int(pts[:,0].mean()), int(pts[:,1].mean())

                # Draw center and line to image center
                cv2.circle(frame, (px, py), 5, (0,255,0), -1)
                cv2.line(frame, center_pt, (px, py), (255,0,0), 2)

                # Annotate coordinates
                text = 'x:{:.2f}m y:{:.2f}m'.format(tvec[0], tvec[1])
                cv2.putText(frame, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0,255,0), 2)

                # Append trajectory
                self.trajectory.append((tvec[0], tvec[1]))
                xs, ys = zip(*self.trajectory)
                self.line.set_data(xs, ys)
                self.ax.relim(); self.ax.autoscale_view()
                self.fig.canvas.draw(); self.fig.canvas.flush_events()

                # Publish path
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.cam_info.header.frame_id
                pose.pose.position.x = tvec[0]
                pose.pose.position.y = tvec[1]
                pose.pose.position.z = 0
                path_msg.header.stamp = rospy.Time.now()
                path_msg.poses.append(pose)
                self.traject_pub.publish(path_msg)

            # Display frame
            cv2.imshow('Frame', frame)

            # Publish raw image with fallback
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = rospy.Time.now()
            except Exception:
                img_msg = Image()
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = self.cam_info.header.frame_id
                height, width, ch = frame.shape
                img_msg.height = height; img_msg.width = width
                img_msg.encoding = 'bgr8'; img_msg.is_bigendian = False
                img_msg.step = width*ch; img_msg.data = frame.tobytes()
            self.image_pub.publish(img_msg)

            # UI events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

        cap.release()
        plt.close(self.fig)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = CameraNode()
    node.run()
