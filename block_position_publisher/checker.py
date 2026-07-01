#!/usr/bin/env python3
"""
Step 6: Average pixel flow from optical flow tracking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np


class AverageFlow(Node):
    def __init__(self):
        super().__init__('average_flow')
        
        self.bridge = CvBridge()
        self.current_altitude = 10.0
        
        self.prev_gray = None
        self.prev_points = None
        
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=7
        )
        
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/world/movable_shapes_world/model/quadcopter/link/base_link/sensor/mono1_camera/image',
            self.image_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Step 6: Average flow running...')
    
    def odom_callback(self, msg):
        self.current_altitude = msg.pose.pose.position.z
    
    def image_callback(self, msg):
        current_gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        display = cv2.cvtColor(current_gray, cv2.COLOR_GRAY2BGR)
        
        tracked = 0
        avg_flow_x = 0.0
        avg_flow_y = 0.0
        
        # Track previous features if available
        if self.prev_gray is not None and self.prev_points is not None:
            current_points, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, current_gray, self.prev_points, None, **self.lk_params
            )
            
            if current_points is not None:
                status = status.reshape(-1)
                good_mask = status == 1
                tracked = int(np.sum(good_mask))
                
                # Draw tracked (green) and lost (red)
                for i in range(len(self.prev_points)):
                    pt1 = tuple(self.prev_points[i].ravel().astype(int))
                    if good_mask[i]:
                        pt2 = tuple(current_points[i].ravel().astype(int))
                        cv2.arrowedLine(display, pt1, pt2, (0, 255, 0), 2, tipLength=0.3)
                        cv2.circle(display, pt1, 3, (0, 255, 0), -1)
                    else:
                        cv2.circle(display, pt1, 5, (0, 0, 255), 1)
                
                # STEP 6: Calculate average pixel flow
                if tracked > 5:
                    prev_good = self.prev_points[good_mask]
                    curr_good = current_points[good_mask]
                    flow_vectors = (curr_good - prev_good).reshape(-1, 2)
                    avg_flow = np.median(flow_vectors, axis=0)
                    avg_flow_x = float(avg_flow[0])
                    avg_flow_y = float(avg_flow[1])
                    
                    # Big red arrow from center showing average flow
                    h, w = display.shape[:2]
                    center = (w // 2, h // 2)
                    end_pt = (center[0] + int(avg_flow_x * 3), center[1] + int(avg_flow_y * 3))
                    cv2.arrowedLine(display, center, end_pt, (0, 0, 255), 3, tipLength=0.2)
        
        # Always detect fresh features (blue dots)
        new_points = cv2.goodFeaturesToTrack(current_gray, mask=None, **self.feature_params)
        new_count = len(new_points) if new_points is not None else 0
        
        if new_points is not None:
            for pt in new_points:
                x, y = pt.ravel()
                cv2.circle(display, (int(x), int(y)), 4, (255, 0, 0), -1)
        
        # Display info
        cv2.putText(display, f'Tracked: {tracked} | New: {new_count}', (5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(display, f'Flow: [{avg_flow_x:.1f}, {avg_flow_y:.1f}] px', (5, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(display, f'Alt: {self.current_altitude:.2f}m', (5, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow('Step 6 - Average Flow', display)
        cv2.waitKey(1)
        
        # Store for next frame
        self.prev_gray = current_gray.copy()
        self.prev_points = new_points
        
        self.get_logger().info(f'Flow: [{avg_flow_x:.1f}, {avg_flow_y:.1f}] px | Tracked: {tracked}')


def main(args=None):
    rclpy.init(args=args)
    node = AverageFlow()
    
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