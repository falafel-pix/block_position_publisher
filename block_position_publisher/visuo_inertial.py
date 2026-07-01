#!/usr/bin/env python3
"""
Step 9: Position from visual velocity integration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisualPosition(Node):
    def __init__(self):
        super().__init__('visual_position')
        
        self.bridge = CvBridge()
        self.current_altitude = 10.0
        self.focal_length = 160.0
        
        self.prev_gray = None
        self.prev_points = None
        self.prev_time = None
        
        # Velocity
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        
        # Position (starts at origin)
        self.position_x = 0.0
        self.position_y = 0.0
        
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
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/visual_velocity', 10)
        self.pos_pub = self.create_publisher(Point, '/visual_position', 10)
        
        self.get_logger().info('Step 9: Visual position running...')
    
    def odom_callback(self, msg):
        self.current_altitude = msg.pose.pose.position.z
        if self.current_altitude < 0.5:
            self.current_altitude = 0.5
    
    def image_callback(self, msg):
        current_gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        display = cv2.cvtColor(current_gray, cv2.COLOR_GRAY2BGR)
        
        # Timestamp
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None:
            dt = current_time - self.prev_time
        else:
            dt = 0.0
        self.prev_time = current_time
        
        tracked = 0
        avg_flow_x = 0.0
        avg_flow_y = 0.0
        
        # Track previous features
        if self.prev_gray is not None and self.prev_points is not None:
            current_points, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, current_gray, self.prev_points, None, **self.lk_params
            )
            
            if current_points is not None:
                status = status.reshape(-1)
                good_mask = status == 1
                tracked = int(np.sum(good_mask))
                
                for i in range(len(self.prev_points)):
                    pt1 = tuple(self.prev_points[i].ravel().astype(int))
                    if good_mask[i]:
                        pt2 = tuple(current_points[i].ravel().astype(int))
                        cv2.arrowedLine(display, pt1, pt2, (0, 255, 0), 2, tipLength=0.3)
                        cv2.circle(display, pt1, 3, (0, 255, 0), -1)
                    else:
                        cv2.circle(display, pt1, 5, (0, 0, 255), 1)
                
                if tracked > 5 and dt > 0:
                    prev_good = self.prev_points[good_mask]
                    curr_good = current_points[good_mask]
                    flow_vectors = (curr_good - prev_good).reshape(-1, 2)
                    avg_flow = np.median(flow_vectors, axis=0)
                    avg_flow_x = float(avg_flow[0])
                    avg_flow_y = float(avg_flow[1])
                    
                    # Pixel to meters
                    scale = self.current_altitude / self.focal_length
                    dx_meters = -avg_flow_x * scale
                    dy_meters = -avg_flow_y * scale
                    
                    # Velocity
                    self.velocity_x = dx_meters / dt
                    self.velocity_y = dy_meters / dt
                    
                    # STEP 9: Integrate position
                    self.position_x += self.velocity_x * dt
                    self.position_y += self.velocity_y * dt
                    
                    # Publish
                    vel_msg = Twist()
                    vel_msg.linear.x = self.velocity_x
                    vel_msg.linear.y = self.velocity_y
                    self.vel_pub.publish(vel_msg)
                    
                    pos_msg = Point()
                    pos_msg.x = self.position_x
                    pos_msg.y = self.position_y
                    pos_msg.z = self.current_altitude
                    self.pos_pub.publish(pos_msg)
                    
                    # Red arrow
                    h, w = display.shape[:2]
                    center = (w // 2, h // 2)
                    end_pt = (center[0] + int(avg_flow_x * 3), center[1] + int(avg_flow_y * 3))
                    cv2.arrowedLine(display, center, end_pt, (0, 0, 255), 3, tipLength=0.2)
        
        # Fresh features
        new_points = cv2.goodFeaturesToTrack(current_gray, mask=None, **self.feature_params)
        new_count = len(new_points) if new_points is not None else 0
        
        if new_points is not None:
            for pt in new_points:
                x, y = pt.ravel()
                cv2.circle(display, (int(x), int(y)), 4, (255, 0, 0), -1)
        
        # Display
        cv2.putText(display, f'Velocity: [{self.velocity_x:.2f}, {self.velocity_y:.2f}] m/s', (5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(display, f'Position: [{self.position_x:.2f}, {self.position_y:.2f}] m', (5, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(display, f'Altitude: {self.current_altitude:.2f} m', (5, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(display, f'Tracked: {tracked} | New: {new_count}', (5, 80),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow('Visual Position', display)
        cv2.waitKey(1)
        
        self.prev_gray = current_gray.copy()
        self.prev_points = new_points
        
        self.get_logger().info(
            f'Pos: [{self.position_x:.2f}, {self.position_y:.2f}] m | '
            f'Vel: [{self.velocity_x:.2f}, {self.velocity_y:.2f}] m/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisualPosition()
    
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