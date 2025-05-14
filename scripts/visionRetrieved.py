#!/usr/bin/env python3

import os
import cv2  # to display the images
from cv_bridge import CvBridge, CvBridgeError  # to convert ros image messages to OpenCV images
import rospy  # ROS Python Interface
from sensor_msgs.msg import CompressedImage  # Allows for faster transportation of the image
import numpy as np
import imutils

class display(object):
    def __init__(self):
        rospy.init_node("MiRo_Display", anonymous = True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        self.bridge = CvBridge()
        self.miro_cams = [None, None]

        self.left_cam_sub = rospy.Subscriber(topic_base_name + "sensors/caml/compressed",
                                             CompressedImage,
                                             self.callback_left_cam,
                                             queue_size=1,
                                             tcp_nodelay=True)

        self.right_cam_sub = rospy.Subscriber(topic_base_name + "sensors/camr/compressed",
                                              CompressedImage,
                                              self.callback_right_cam,
                                              queue_size=1,
                                              tcp_nodelay=True)


        # Calibration features for un-distorting the MiRo fish-eye images
        # Taken from MiRo-projects/basic_functions/miro_constants.py
        self.mtx = np.array([
            [1.04358065e+03, 0, 3.29969935e+02],
            [0, 1.03845278e+03, 1.68243114e+02],
            [0, 0, 1]])
        self.dist = np.array([[-3.63299415e+00, 1.52661324e+01, -7.23780207e-03, -7.48630198e-04, -3.20700124e+01]])
        self.focal_length = 330

        # For FPS counter
        self.fps_count = 0
        self.cur_fps = 0
        self.stop = False

        rospy.on_shutdown(self.shutdown_hook)

    def left_cam(self):
        left_eye = self.miro_cams[0]
        return left_eye

    def right_cam(self):
        right_eye = self.miro_cams[1]
        return right_eye

    def callback_left_cam(self, ros_image):
        self.callback_cam(ros_image, 0)
        rospy.sleep(0.05)

    def callback_right_cam(self, ros_image):
        self.callback_cam(ros_image, 1)
        rospy.sleep(0.05)

    # General Camera Callback, used by both the left and right cams
    def callback_cam(self, ros_image, index):
        try:
            # Converts compressed ROS image to raw CV image
            image = self.bridge.compressed_imgmsg_to_cv2(ros_image, "bgr8")

            # Undistorts the fish eye image retrieved from the MiRo cameras
            # Returns image with, height = 360, width = 640
            unfisheye_img = cv2.undistort(image, self.mtx, self.dist, None)

            # Allows for testing with different image crop values
            crop_width = 50
            crop_height = 0

            # Left
            if index == 0:
                cropped_img = unfisheye_img[crop_height:360, crop_width:640]
            # Right
            else:
                cropped_img = unfisheye_img[crop_height:360, 0:640 - crop_width]

            # Ignores empty images
            if cropped_img.all != None:
                self.miro_cams[index] = cropped_img

        # Ignores corrupted frames
        except CvBridgeError as e:
            pass

    def shutdown_hook(self):
        print("Program Done!")
        self.stop = True

if __name__ == '__main__':
    node = display()

    while not rospy.is_shutdown():
        if node.miro_cams[0] is not None and node.miro_cams[1] is not None:

            time_now = rospy.get_time()
            # Counts up how many frames are shown in 5 seconds
            # then averages out to find the current fps
            node.fps_count = 0
            while rospy.get_time() < time_now + 5:
                if node.stop == False:
                    left_eye = node.left_cam()
                    right_eye = node.right_cam()
                    stereo_frame = np.hstack((left_eye, right_eye))
                    cv2.imshow("Stereo View", stereo_frame)
            node.cur_fps = (node.fps_count / 5)


