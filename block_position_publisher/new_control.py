import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


class NewController(Node):
    
    def __init__(self):
        super().__init__('new_controller')
        # ----tunable parameters
        self.kp_z = 2
        self.kd_z = 4.0
        self.kp_psi = 10
        self.kd_psi = 4
        self.kptheta = 10
        self.kdtheta =4 
        self.dt = 0.01
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # ── Physical parameters ────────────────────────────────────
        self.m = 0.031
        self.g = 9.81
        self.J = np.diag([6.4e-4, 6.4e-4, 1.28e-3])
        self.get_logger().info(f"Controller mass = {self.m}")
       
        # ── Current state (updated by subscribers) ─────────────────
        self.pz     = 0.0                     # position
        self.vz    = 0.0                        # linear velocity in z
        self.q     = np.array([1.0, 0.0, 0.0, 0.0])   # quaternion [w, x, y, z]
        self.psi     = 0.0   # yaw angle
        self.phi     = 0.0
        self.theta   = 0.0
        self.vx=0.0
        self.omega = np.zeros(3)                        # angular velocity
        self.acc=np.zeros(3)
        self.phi_fixed = 0.0    # desired roll angle
        self.theta_fixed = 0.0  # desired pitch angle
       
        

        #----desired trajectory parameters
        # self.create_subscription()
        self.pd=5.0 #z desired position
        self.vd=0.0
        self.ad=0.0
        self.psi_d = 0.0    # desired yaw angle
        self.F_x=0

       

        # ── susbscriptions ─────────────────────────────────────────────
        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            1
        )

        

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/desired_trajectory',
            self.command_callback,
            10
        )
        


        self.force_pub = self.create_publisher(Wrench, '/block/force', 1)
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
    def odom_callback(self, msg: Odometry):

        self.pz = msg.pose.pose.position.z

        self.q = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])

        self.vz = msg.twist.twist.linear.z

        self.omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])

    def command_callback(self, msg):

        self.pd    = msg.data[0]
        self.vd    = msg.data[1]
        self.ad    = msg.data[2]
        self.psi_d = msg.data[3]
        self.F_x   = msg.data[4]

        self.get_logger().info(
            f"Updated command: "
            f"pd={self.pd:.2f}, "
            f"vd={self.vd:.2f}, "
            f"ad={self.ad:.2f}, "
            f"psi={self.psi_d:.2f}, "
            f"Fx={self.F_x:.2f}"
    )

    def imu_callback(self, msg: Imu):
        """Extract orientation, angular velocity, and linear acceleration from /imu/data."""
        # 1. Save orientation quaternion matching his [w, x, y, z] order style
        """
        self.q = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])
        
 # 2. Save angular velocity (omega) from the IMU
        self.omega = np.array([ 
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
       """
        # 3. Save linear acceleration from the IMU
        self.accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        
    def quat_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        w, x, y, z = q
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])
        return R
    def yaw_pitch_roll_from_quaternion(self, q: np.ndarray) -> tuple:
        """Convert quaternion to yaw, pitch, roll angles (Yaw is 0 to 2*pi)."""
        w, x, y, z = q
        
        # Yaw (psi)
        psi = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        # Wrap yaw from [-pi, pi] to [0, 2*pi]
       # psi = psi % (2 * np.pi)
        
        # Pitch (theta)
        theta = np.arcsin(2*(w*y - z*x))
        
        # Roll (phi)
        phi = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    
        return phi, theta, psi
    def z_control(self, t):
        #pid control for z-axis
        ep = self.pz - self.pd
        ev = self.vz - self.vd
        acc_req = self.ad - self.kp_z * ev - self.kd_z * ep  # PD control
        f_z = self.m * (self.g + acc_req)  # total thrust in z
        return f_z
    def psi_control(self, t):
        #pid control for psi (yaw) angle
        ep = self.psi - self.psi_d
        ev = self.omega[2]  # angular velocity around z-axis
        acc_req = -self.kp_psi * ep - self.kd_psi * ev  # PD control
        tau_psi = self.J[2, 2] * acc_req  # torque around z-axis
        print(self.psi,self.psi_d,tau_psi)
        return tau_psi
    def attitude_balancer (self, t):
        #balance the quadcopter
        #calculate the required roll and pitch angles to maintain stability
        # desired pitch angle

        ep_phi = self.phi - self.phi_fixed
        ep_theta = self.theta - self.theta_fixed
        ev_phi = self.omega[0]  # angular velocity around x-axis
        ev_theta = self.omega[1]  # angular velocity around y-axis
        acc_req_phi = -self.kp_psi * ep_phi - self.kd_psi * ev_phi  # PD control for roll
        acc_req_theta = -self.kp_psi * ep_theta - self.kd_psi * ev_theta
        tau_phi = self.J[0, 0] * acc_req_phi  # torque around x-axis

        return tau_phi

    # for control motion in x only
    def x_control(self,f_z, t):
        self.theta_fixed=np.arctan2(self.F_x,f_z)
        ep=self.theta-self.theta_fixed
        ev = self.omega[1]
        a = self.ad-self.kptheta*ep-self.kdtheta*ev
        tautheta=self.J[1,1]*a
        return tautheta
    


    def control_loop(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        R = self.quat_to_rotation_matrix(self.q)
        self.phi, self.theta, self.psi = self.yaw_pitch_roll_from_quaternion(self.q)
        self.omega = R @ self.omega

        f_z = self.z_control(t)
      
        tau_psi = self.psi_control(t)
        tau_phi = self.attitude_balancer(t)
        tau_theta = self.x_control(f_z,t)

        f_s=f_z/np.cos(self.theta)
        f_x=f_s*np.sin(self.theta)
        f_des = np.array([f_x, 0.0, f_z])
        thrust=R@f_des
        #print(f"thrust: {thrust}, tau_phi: {tau_phi}, tau_theta: {tau_theta}, tau_psi: {tau_psi}" )

       # thrust = float(f_des @ (R @ np.array([0.0, 0.0, 1.0])))
        thrust = (np.clip(thrust, -1.5, 1.5))

        torque = np.array([tau_phi, tau_theta, tau_psi])
        torque = np.clip(torque, -0.1, 0.1)
        torque_r=R@torque
        wrench_msg = Wrench()
        wrench_msg.force.x = thrust[0]
        wrench_msg.force.y = thrust[1]
        wrench_msg.force.z = thrust[2]
        wrench_msg.torque.x = torque_r[0]
        wrench_msg.torque.y = torque_r[1]
        wrench_msg.torque.z = torque_r[2]
        self.force_pub.publish(wrench_msg)


def main():
    rclpy.init()
    controller = NewController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()