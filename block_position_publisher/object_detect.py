#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Subscribe to monochrome camera feed
        self.subscription = self.create_subscription(
            Image,
            '/world/movable_shapes_world/model/quadcopter/link/base_link/sensor/mono_camera/image',
            self.image_callback,
            10
        )
        
        # Bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        
        # Publisher for processed/annotated image
        self.processed_pub = self.create_publisher(
            Image,
            '/object_detector/processed_image',
            10
        )
        
        self.get_logger().info('Object Detector Node Started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            # For L8 (monochrome), this gives a single-channel numpy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            # ----------------------------------------------------------
            # OPENCV PROCESSING AREA
            # ----------------------------------------------------------
            # cv_image is a numpy array (height x width), uint8, 0-255
            # 
            # TODO: Add your OpenCV detection code here
            # Example placeholder:
            # processed_image = your_detection_function(cv_image)
            # bounding_boxes = your_detection_function(cv_image)
            #
            # For now, just pass the original image through
            processed_image = cv_image
            # ----------------------------------------------------------
            
            # Convert back to ROS Image and publish
            output_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='mono8')
            output_msg.header = msg.header
            self.processed_pub.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()