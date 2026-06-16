import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Wrench
from nav_msgs.msg import Odometry



class GeometricController(Node):
    """
    Geometric tracking controller for quadcopter.
    Based on Lee et al. 2010 - geometric tracking control on SE(3).

    Subscribes:
        /model/quadcopter/pose      → position + orientation (quaternion)
        /model/quadcopter/odometry  → linear + angular velocity

    Publishes:
        /block/force                → thrust (force.z) + torque (torque.xyz)
    """

    def __init__(self):
        super().__init__('geometric_controller')

            # ── Gains ──────────────────────────────────────────────────
        self.Kp = np.diag([0.1, 0.3, 8.0])

        self.Kd = np.diag([1.0, 1.0, 2.0])

        self.Ki = np.zeros((3,3))

        self.KR = np.diag([0.3, 0.3, 0.05])

        self.Kw = np.diag([0.05, 0.05, 0.02])
        self.log_data = []



        # ── Physical parameters ────────────────────────────────────
        self.m = 0.031
        self.g = 9.81

        self.get_logger().info(f"Controller mass = {self.m}")
        self.J = np.diag([
            1.395e-5,
            1.395e-5,
            2.173e-5
        ])

        # Odometry ≈ 40 Hz
        self.dt = 0.01
        

        # ── Current state (updated by subscribers) ─────────────────
        self.p     = np.array([0.0, 0.0, 0])                       # position
        self.q     = np.array([1.0, 0.0, 0.0, 0.0])   # quaternion [w, x, y, z]
        self.v     = np.zeros(3)                        # linear velocity
        self.omega = np.zeros(3)                        # angular velocity

        # ── Setpoints (edit here or make ROS params) ───────────────
        self.pd = np.array([19.0, 0.0, 2.0]) # desired position (hover at 1m)
        self.vd    = np.zeros(3)                  # desired velocity
        self.ad    = np.zeros(3)                  # desired acceleration
        self.psi_d = 0.0                          # desired yaw angle (rad)

        # ── Subscribers ────────────────────────────────────────────
        self.ep_int = np.zeros(3)
        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            1
        )

        # ── Publisher ──────────────────────────────────────────────
        self.force_pub = self.create_publisher(Wrench, '/block/force', 1)

        # ── Control loop @ 1 kHz ───────────────────────────────────
        #self.create_timer(self.dt, self.control_loop)

        #self.get_logger().info('Geometric controller started. Hovering at z=1.0m')
        self.control_timer = self.create_timer(
            0.01,      # 100 Hz
            self.control_loop
        )
    # ──────────────────────────────────────────────────────────────
    # Subscriber callbacks
    # ──────────────────────────────────────────────────────────────

    

    def odom_callback(self, msg: Odometry):
        """Extract linear and angular velocity from /model/quadcopter/odometry."""
        self.p = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
     
        self.q = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])
        
        self.v = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
       
        self.omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])
       
        

        
        #
        

    # ──────────────────────────────────────────────────────────────
    # Math helpers
    # ──────────────────────────────────────────────────────────────

    def quat_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
        w, x, y, z = q
        R = np.array([
            [1 - 2*y**2 - 2*z**2,   2*x*y - 2*z*w,       2*x*z + 2*y*w    ],
            [2*x*y + 2*z*w,          1 - 2*x**2 - 2*z**2,  2*y*z - 2*x*w    ],
            [2*x*z - 2*y*w,          2*y*z + 2*x*w,        1 - 2*x**2 - 2*y**2],
        ])
        return R

    def vee(self, S: np.ndarray) -> np.ndarray:
        """Vee map: extracts the 3-vector from a skew-symmetric 3x3 matrix."""
        return np.array([S[2, 1], S[0, 2], S[1, 0]])

    # ──────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────
    #-----------converting omega to body frame _________________
    

    def control_loop(self):
        # ── Position & velocity errors ─────────────────────────────
        if self.p is None:
            return
        ep = self.p - self.pd        # position error
        ev = self.v - self.vd
        print("vz =", self.v[2])         # velocity error
        self.ep_int += ep * self.dt    # integral of position error
        print("p =", self.p)
        print("pd =", self.pd)
        print("ep =", ep)
        # ── Desired acceleration ───────────────────────────────────
        a_des = (self.ad
                 - self.Kp @ ep
                 - self.Kd @ ev
                 + self.Ki @ self.ep_int)
        print("a_des =", a_des)
        # ── Desired thrust vector (world frame) ────────────────────
        #f_des = self.m * (a_des - np.array([0.0, 0.0, -self.g]))
        f_des = self.m * (a_des + np.array([0.0, 0.0, self.g]))  # gravity compensation
        print("f_des =", f_des)
        # ── Rotation matrix from current quaternion ────────────────
        R = self.quat_to_rotation_matrix(self.q)
        

        # ── Collective thrust (project onto body z-axis) ──────────
        thrust = float(f_des @ (R @ np.array([0.0, 0.0, 1.0])))  # project desired force onto current body z-axis
        thrust = float(np.clip(thrust, 0.0, 1.5))

        # ── Desired attitude (Rd) ──────────────────────────────────
        b3_des = f_des / np.linalg.norm(f_des)
        b1_des = np.array([
        np.cos(self.psi_d),
        np.sin(self.psi_d),
        0.0
        ])

        b2_des = np.cross(b3_des, b1_des)
        b2_des = b2_des / np.linalg.norm(b2_des)

        Rd = np.column_stack([
            b1_des,
            b2_des,
            b3_des
        ])

        # ── Attitude error ─────────────────────────────────────────
        eR = 0.5 * self.vee(Rd.T @ R - R.T @ Rd)

        # ── Angular velocity error ─────────────────────────────────
        eomega = R.T @ self.omega                          # desired omega = 0

        # ── Control torque ─────────────────────────────────────────
        torque = (- self.KR @ eR
                  - self.Kw @ eomega
                  - np.cross(eomega, self.J @ eomega))
        torque = np.clip(torque, -0.02, 0.02)
        print("thrust =", thrust)
        #------------converter----------------
        thrust_vector = R @ np.array([0.0, 0.0, thrust]) 
        print("R =\n", R)
        print("thrust_vector =", thrust_vector) # thrust in world frame
        torquer = R @ torque  # torque in world frame
        #print(thrust_vector,torquer);
        # ── Publish Wrench ─────────────────────────────────────────
        msg = Wrench()
        msg.force.x = thrust_vector[0]
        msg.force.y = thrust_vector[1]
        msg.force.z = thrust_vector[2]
        msg.torque.x = torquer[0]
        msg.torque.y = torquer[1]
        msg.torque.z = torquer[2]
        self.force_pub.publish(msg)

        # Optional debug log (comment out for performance)
       # self.get_logger().info(f'thrust={thrust:.3f}  torque={torque}  pos={self.p}')
        # )


# ──────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = GeometricController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
       
        rclpy.shutdown()


if __name__ == '__main__':
    main()
