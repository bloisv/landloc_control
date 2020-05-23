#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
PI = 3.14159

class ThymioController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
            self.name + '/cmd_vel',  # name of the topic
            Twist,  # message type
            queue_size=10  # queue size
        )

        # create image subscriber
        self.image_subscriber = rospy.Subscriber(
            self.name + '/camera/image_raw/compressed',  # name of the topic
            CompressedImage,  # message type
            self.capture_image  # function that hanldes incoming messages
        )

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)
        self.photo = None
        self.bridge = CvBridge()

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def move_ahead(self, vel = 0.2):
        return Twist(
            linear=Vector3(
                vel,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                .0,
            )
        )

    def capture_image(self,data):
        self.photo = data

    def transform_image(self, img):
        flat = img[:,:,0]
        return np.where(flat < 200, 0, 1)

    def run(self):
        #speed = input("photo sec:")
        i = 0
        while i < 10:
            velocity = self.move_ahead(0)
            self.velocity_publisher.publish(velocity)
            i += 1
            self.rate.sleep()

        velocity = self.move_ahead(0)
        self.velocity_publisher.publish(velocity)

        speed = 10
        i = 0
        while i < speed:
            i += 0.5
            self.rate.sleep()
        img = self.bridge.compressed_imgmsg_to_cv2(self.photo)
        #img = self.transform_image(img)
        print(type(img))
        print(sum(sum(img)))
        print(img.shape)
        print(img)
        plt.imsave('/home/usi/catkin_ws/src/landloc_control/src/binary_image.png', img, cmap=cm.gray)
        #cv2.imwrite('test_immagine.jpeg', img)

if __name__ == '__main__':
    controller = ThymioController()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass

