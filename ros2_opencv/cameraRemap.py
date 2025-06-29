import cv2

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import re
import yaml
from sensor_msgs.msg import CameraInfo
import os
import numpy as np


def yaml_to_CameraInfo(calib_yaml):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    f = open(calib_yaml)
    calib_data = yaml.safe_load(f)

    # calib_data = yaml.load(calib_yaml)
    # print("-------------------")
    # print(calib_data)
    # print("-------------------")
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.k = calib_data["camera_matrix"]["data"]
    camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.r = calib_data["rectification_matrix"]["data"]
    camera_info_msg.p = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

class PublisherNodeClass(Node):

    def __init__(self):

        super().__init__('publisher_node')
        self.cameraDeviceNumber=0
        # self.camera = cv2.VideoCapture('tcpclientsrc host=192.168.1.245 port=7001 ! decodebin ! queue ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720 ! appsink drop=1', cv2.CAP_GSTREAMER)
        self.bridgeObject = CvBridge()
        self.topicNameFrames='image/uncompressed'
        self.topicCamerInfo='/camera_info'
        self.queueSize=20
        self.publisher=self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.publisherCamInfo = self.create_publisher(CameraInfo, "camera_info", self.queueSize)

        # self.periodCommunication=0.02
        # self.timer=self.create_timer(self.periodCommunication,self.timer_callbackFunction)
        self.i=0
    
        # calib_yaml = rclpy.get_param("/home/ahmed/.ros/camera_info/rapoo_camera:_rapoo_camera")
        calib_yaml = "/home/ahmed/moveit2/ws_moveit_humble_control/src/ros2_opencv/ros2_opencv/rapoo_camera:_rapoo_camera2.yaml"
        self.camera_info_msg = yaml_to_CameraInfo(calib_yaml)

        # Params
        self.image = None
        # Subscribers
        self.subscriber = self.create_subscription(Image,"/camera/image_raw",self.callback, self.queueSize)

    def callback(self, msg):
        # self.get_logger().info('Image received...')
        frame = self.bridgeObject.imgmsg_to_cv2(msg)
        ROS2ImageMessage=self.bridgeObject.cv2_to_imgmsg(frame, encoding='bgr8')
        self.camera_info_msg.header.stamp = ROS2ImageMessage.header.stamp
        self.publisher.publish(ROS2ImageMessage)
        self.publisherCamInfo.publish(self.camera_info_msg)

        # self.get_logger().info('Publishing image number %d' % self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisherObject=PublisherNodeClass()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

