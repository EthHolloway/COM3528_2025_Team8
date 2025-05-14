# visionRetrieve.py
#!/usr/bin/env python3

import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np

class Display:
    def __init__(self):
        rospy.init_node("MiRo_Display", anonymous=True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro")

        self.bridge = CvBridge()
        self.miro_cams = [None, None]

        rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
                         CompressedImage,
                         self.callback_left_cam,
                         queue_size=1,
                         tcp_nodelay=True)

        rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
                         CompressedImage,
                         self.callback_right_cam,
                         queue_size=1,
                         tcp_nodelay=True)

        self.mtx = np.array([
            [1.04358065e+03, 0, 3.29969935e+02],
            [0, 1.03845278e+03, 1.68243114e+02],
            [0, 0, 1]])
        self.dist = np.array([[-3.63299415e+00, 1.52661324e+01, -7.23780207e-03, -7.48630198e-04, -3.20700124e+01]])
        self.stop = False

        rospy.on_shutdown(self.shutdown_hook)

    def left_cam(self):
        return self.miro_cams[0]

    def right_cam(self):
        return self.miro_cams[1]

    def callback_left_cam(self, ros_image):
        self.callback_cam(ros_image, 0)

    def callback_right_cam(self, ros_image):
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(ros_image, "bgr8")
            unfisheye_img = cv2.undistort(image, self.mtx, self.dist, None)

            crop_width = 50
            crop_height = 0

            if index == 0:
                cropped_img = unfisheye_img[crop_height:360, crop_width:640]
            else:
                cropped_img = unfisheye_img[crop_height:360, 0:640 - crop_width]

            if cropped_img is not None and cropped_img.size > 0:
                self.miro_cams[index] = cropped_img

        except CvBridgeError as e:
            rospy.logwarn(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logwarn(f"General camera callback error: {e}")

    def shutdown_hook(self):
        self.stop = True
        print("[ROS] Shutdown triggered.")
