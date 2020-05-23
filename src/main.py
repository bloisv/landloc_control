#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import image_capture
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion


if __name__ == '__main__':
    controller = image_capture.ThymioController()
    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
