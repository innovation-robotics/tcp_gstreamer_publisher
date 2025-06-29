import cv2

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import re

import yaml
from sensor_msgs.msg import CameraInfo

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
        self.camera = cv2.VideoCapture('tcpclientsrc host=192.168.1.245 port=7001 ! decodebin ! queue ! videoconvert ! videoscale ! video/x-raw,width=1280,height=720 ! appsink drop=1', cv2.CAP_GSTREAMER)
        self.bridgeObject = CvBridge()
        self.topicNameFrames='image/uncompressed'
        self.topicCamerInfo='/camera_info'
        self.queueSize=20
        self.publisher=self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.publisherCamInfo = self.create_publisher(CameraInfo, "camera_info", self.queueSize)

        self.periodCommunication=0.02
        self.timer=self.create_timer(self.periodCommunication,self.timer_callbackFunction)
        self.i=0
    
        # calib_yaml = rclpy.get_param("/home/ahmed/.ros/camera_info/rapoo_camera:_rapoo_camera")
        calib_yaml = "/home/ros/mobile_moveo/src/tcp_gstreamer_publisher/ros2_opencv/rapoo_camera:_rapoo_camera.yaml"
        self.camera_info_msg = yaml_to_CameraInfo(calib_yaml)

    def timer_callbackFunction(self):
        success,frame=self.camera.read()
        if success==True:
            print('yes image')
            frame=cv2.resize(frame,(1280,720), interpolation=cv2.INTER_CUBIC)
            ROS2ImageMessage=self.bridgeObject.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ROS2ImageMessage)
            self.publisherCamInfo.publish(self.camera_info_msg)
        else:
            print('no image')
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

####################################################################################################

# import cv2

# import rclpy
# from sensor_msgs.msg import Image
# from rclpy.node import Node
# from cv_bridge import CvBridge

# class PublisherNodeClass(Node):

#     def __init__(self):

#         super().__init__('publisher_node')
#         self.cameraDeviceNumber=0
#         self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
#         self.bridgeObject = CvBridge()
#         self.topicNameFrames='topic_camera_image'
#         self.queueSize=20
#         self.publisher=self.create_publisher(Image, self.topicNameFrames, self.queueSize)
#         self.periodCommunication=0.02
#         self.timer=self.create_timer(self.periodCommunication,self.timer_callbackFunction)
#         self.i=0
    
#     def timer_callbackFunction(self):
#         success,frame=self.camera.read()
#         frame=cv2.resize(frame,(320,240), interpolation=cv2.INTER_CUBIC)
#         if success==True:
#             ROS2ImageMessage=self.bridgeObject.cv2_to_imgmsg(frame, encoding='rgb8')
#             self.publisher.publish(ROS2ImageMessage)
#         self.get_logger().info('Publishing image number %d' % self.i)
#         self.i += 1

# def main(args=None):
#     rclpy.init(args=args)
#     publisherObject=PublisherNodeClass()
#     rclpy.spin(publisherObject)
#     publisherObject.destroy_node()
#     rclpy.shutdown()

# if __name__=='__main__':
#     main()

####################################################################################################

# import cv2

# # Check for gstreamer support from opencv
# import re
# print('GStreamer support: %s' % re.search(r'GStreamer\:\s+(.*)', cv2.getBuildInformation()).group(1))


# # Create capture (you would edit for setting your Jetson's IP)
# # cap = cv2.VideoCapture('tcpclientsrc host=0.0.0.0 port=7001 ! decodebin ! queue ! videoconvert ! videoscale ! video/x-raw,format=BGR ! queue ! appsink drop=1', cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture('tcpclientsrc host=0.0.0.0 port=7001 ! decodebin ! queue ! videoconvert ! videoscale ! video/x-raw,width=1920,height=1080 ! appsink drop=1', cv2.CAP_GSTREAMER)
# if not cap.isOpened():
# 	print('Failed to open camera')
# 	exit(-1)
# w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# fps = cap.get(cv2.CAP_PROP_FPS)
# print('camera opened, framing %dx%d@%f fps' % (w,h,fps))

# # Loop reading frame and displaying	
# while True:
# 	ret,frame = cap.read()
# 	if not ret:
# 		print('Failed to read from camera')
# 		cap.release()
# 		exit(-3)
# 	cv2.imshow('Test', frame)
# 	cv2.waitKey(1)
# cap.release()
