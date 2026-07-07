#!/usr/bin/env python3
"""
Front Camera: YOLOv8 Object Detection (local model)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.bridge = CvBridge()
        
        # Load local model
        model_path = os.path.expanduser(
            '~/ros2_ws/src/block_position_publisher/models/yolov8n.pt'
        )
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model loaded!')
        
        self.image_sub = self.create_subscription(
            Image,
            '/world/movable_shapes_world/model/quadcopter/link/base_link/sensor/mono_camera/image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('YOLO detector running...')
    
    def image_callback(self, msg):
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        h, w = display.shape[:2]
        
        # Run YOLO
        results = self.model(gray, verbose=False)
                # Debug: print all detections
        for result in results:
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    class_name = self.model.names[int(box.cls[0])]
                    confidence = float(box.conf[0])
                    self.get_logger().info(f'Found: {class_name} ({confidence:.2f})')
            else:
                self.get_logger().info('No detections in this frame')
        # Draw detections
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = float(box.conf[0])
                    class_name = self.model.names[int(box.cls[0])]
                    
                    center_y = (y1 + y2) // 2
                    is_close = center_y > h // 2
                    color = (0, 0, 255) if is_close else (0, 255, 0)
                    
                    cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(display, f'{class_name} {confidence:.2f}', 
                               (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.4, color, 1)
        
        cv2.line(display, (0, h // 2), (w, h // 2), (255, 0, 0), 1)
        
        cv2.imshow('YOLO Detection', display)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()