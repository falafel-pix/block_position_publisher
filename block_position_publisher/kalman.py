#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation

class KalmanFilterFusion(Node):
    def __init__(self):
        super().__init__('kalman_filter_fusion')
        
        # Storage for latest sensor data
        self.imu_data = None
        self.visual_velocity = None
        self.visual_position = None
        self.ground_truth = None
        self.current_orientation = None
        
        # Flags to track when both measurements are ready
        self.velocity_ready = False
        self.position_ready = False
        
        # Dead-reckoning state (for comparison)
        self.dr_position = {'x': 0.0, 'y': 0.0}
        self.dr_velocity = {'x': 0.0, 'y': 0.0}
        
        # Kalman filter state: [x_pos, x_vel, y_pos, y_vel]
        self.x = np.zeros((4, 1))
        
        # State covariance matrix
        self.P = np.eye(4) * 0.1
        self.P[1, 1] = 1.0
        self.P[3, 3] = 1.0
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01])
        
        # Measurement noise covariance for combined update
        self.R_pos = 1
        self.R_vel = 0.01
        
        # Combined measurement matrix
        self.H_combined = np.array([
            [1, 0, 0, 0],  # X position
            [0, 0, 1, 0],  # Y position
            [0, 1, 0, 0],  # X velocity
            [0, 0, 0, 1]   # Y velocity
        ])
        
        # Combined measurement noise matrix
        self.R_combined = np.diag([self.R_pos, self.R_pos, self.R_vel, self.R_vel])
        
        self.last_imu_time = None
        self.initialized = False
        
        # Track updates
        self.update_count = 0
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.vel_sub = self.create_subscription(
            Twist, '/visual_velocity', self.velocity_callback, 10)
        
        self.pos_sub = self.create_subscription(
            Point, '/visual_position', self.position_callback, 10)
        
        self.gt_sub = self.create_subscription(
            Odometry, '/model/quadcopter/odometry', self.ground_truth_callback, 10)
        
        # Timer for logging
        self.create_timer(1.0, self.log_status)
        
        self.get_logger().info('Kalman Filter Fusion Node Started (Combined Update + Orientation)')
    
    def rotate_accel_to_world(self, accel_x, accel_y, accel_z, orientation):
        """Rotate acceleration from body frame to world frame and remove gravity"""
        q = orientation
        rot = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']])
        
        # Acceleration in body frame
        accel_body = np.array([accel_x, accel_y, accel_z])
        
        # Rotate to world frame
        accel_world = rot.apply(accel_body)
        
        # Subtract gravity from world Z
        accel_world[2] -= 9.8
        
        # Return world X and Y only
        return accel_world[0], accel_world[1]
    
    def rotate_velocity_to_world(self, vel_x, vel_y, orientation):
        """Rotate velocity from body frame to world frame"""
        q = orientation
        rot = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']])
        
        # Velocity in body frame (Z velocity is 0 for optical flow)
        vel_body = np.array([vel_x, vel_y, 0.0])
        
        # Rotate to world frame
        vel_world = rot.apply(vel_body)
        
        # Return world X and Y only
        return vel_world[0], vel_world[1]
    
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        
        self.imu_data = {
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        
        # Store orientation
        self.current_orientation = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        }
        
        if not self.initialized:
            self.last_imu_time = current_time
            self.initialized = True
            return
        
        # Calculate dt
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        self.last_imu_time = current_time
        
        if dt <= 0 or dt > 0.1:
            return
        
        # Get IMU accelerations in body frame
        accel_x_body = self.imu_data['linear_acceleration']['x']
        accel_y_body = self.imu_data['linear_acceleration']['y']
        accel_z_body = self.imu_data['linear_acceleration']['z']
        
        # Rotate to world frame and remove gravity
        if self.current_orientation is not None:
            accel_x, accel_y = self.rotate_accel_to_world(
                accel_x_body, accel_y_body, accel_z_body, 
                self.current_orientation
            )
        else:
            accel_x, accel_y = accel_x_body, accel_y_body
        
        # === Dead Reckoning (for comparison) ===
        old_vel_x = self.dr_velocity['x']
        old_vel_y = self.dr_velocity['y']
        self.dr_velocity['x'] += accel_x * dt
        self.dr_velocity['y'] += accel_y * dt
        self.dr_position['x'] += 0.5 * (old_vel_x + self.dr_velocity['x']) * dt
        self.dr_position['y'] += 0.5 * (old_vel_y + self.dr_velocity['y']) * dt
        
        # === Kalman Filter Prediction ===
        F = np.array([
            [1, dt, 0,  0],
            [0,  1, 0,  0],
            [0,  0, 1, dt],
            [0,  0, 0,  1]
        ])
        
        B = np.array([
            [0.5*dt*dt, 0],
            [dt,        0],
            [0, 0.5*dt*dt],
            [0,        dt]
        ])
        
        u = np.array([[accel_x], [accel_y]])
        
        # Predict state
        self.x = F @ self.x + B @ u
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def velocity_callback(self, msg):
        self.visual_velocity = {
            'x': msg.linear.x,
            'y': msg.linear.y
        }
        
        self.velocity_ready = True
        
        # Try combined update if both measurements are available
        self.try_combined_update()
    
    def position_callback(self, msg):
        self.visual_position = {
            'x': msg.x,
            'y': msg.y
        }
        
        self.position_ready = True
        
        # Try combined update if both measurements are available
        self.try_combined_update()
    
    def try_combined_update(self):
        """Perform combined update when both position and velocity are available"""
        if not self.initialized:
            return
        
        if not (self.velocity_ready and self.position_ready):
            return
        
        # Rotate optical flow velocity to world frame
        if self.current_orientation is not None:
            of_world_x, of_world_y = self.rotate_velocity_to_world(
                self.visual_velocity['x'], 
                self.visual_velocity['y'],
                self.current_orientation
            )
        else:
            of_world_x = self.visual_velocity['x']
            of_world_y = self.visual_velocity['y']
        
        # Combined measurement vector: [x_pos, y_pos, x_vel, y_vel]
        z = np.array([
            [self.visual_position['x']],
            [self.visual_position['y']],
            [of_world_x],
            [of_world_y]
        ])
        
        # Innovation
        y = z - self.H_combined @ self.x
        
        # Innovation covariance
        S = self.H_combined @ self.P @ self.H_combined.T + self.R_combined
        
        # Kalman gain
        K = self.P @ self.H_combined.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        self.P = (np.eye(4) - K @ self.H_combined) @ self.P
        
        self.update_count += 1
        
        # Reset flags
        self.velocity_ready = False
        self.position_ready = False
    
    def ground_truth_callback(self, msg):
        self.ground_truth = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y
        }
    
    def log_status(self):
        if self.ground_truth is not None and self.initialized:
            # Kalman estimates
            kf_x_pos = self.x[0, 0]
            kf_x_vel = self.x[1, 0]
            kf_y_pos = self.x[2, 0]
            kf_y_vel = self.x[3, 0]
            
            # Dead reckoning
            dr_x_pos = self.dr_position['x']
            dr_y_pos = self.dr_position['y']
            
            # Ground truth
            gt_x = self.ground_truth['x']
            gt_y = self.ground_truth['y']
            gt_vx = self.ground_truth['vx']
            gt_vy = self.ground_truth['vy']
            
            # Optical flow for comparison
            of_vx = self.visual_velocity['x'] if self.visual_velocity else 0.0
            of_vy = self.visual_velocity['y'] if self.visual_velocity else 0.0
            of_px = self.visual_position['x'] if self.visual_position else 0.0
            of_py = self.visual_position['y'] if self.visual_position else 0.0
            
            # Errors
            kf_error_x = gt_x - kf_x_pos
            kf_error_y = gt_y - kf_y_pos
            dr_error_x = gt_x - dr_x_pos
            dr_error_y = gt_y - dr_y_pos
            
            kf_total_error = np.sqrt(kf_error_x**2 + kf_error_y**2)
            dr_total_error = np.sqrt(dr_error_x**2 + dr_error_y**2)
            
            # Covariance diagonals
            p_diag = np.diag(self.P)
            
            self.get_logger().info(f'Combined Updates: {self.update_count}')
            self.get_logger().info(
                f'KF  Pos: x={kf_x_pos:.4f}, y={kf_y_pos:.4f} | Err: {kf_total_error:.4f}'
            )
            self.get_logger().info(
                f'DR  Pos: x={dr_x_pos:.4f}, y={dr_y_pos:.4f} | Err: {dr_total_error:.4f}'
            )
            self.get_logger().info(
                f'GT  Pos: x={gt_x:.4f}, y={gt_y:.4f}'
            )
            self.get_logger().info(
                f'OF  Pos: x={of_px:.4f}, y={of_py:.4f}'
            )
            self.get_logger().info(
                f'OF  Vel: x={of_vx:.4f}, y={of_vy:.4f}'
            )
            self.get_logger().info(
                f'GT  Vel: x={gt_vx:.4f}, y={gt_vy:.4f}'
            )
            self.get_logger().info(
                f'KF  Vel: x={kf_x_vel:.4f}, y={kf_y_vel:.4f}'
            )
            self.get_logger().info(
                f'Covariance: [{p_diag[0]:.4f}, {p_diag[1]:.4f}, {p_diag[2]:.4f}, {p_diag[3]:.4f}]'
            )
            self.get_logger().info('---')

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()