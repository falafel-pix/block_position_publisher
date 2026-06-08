#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gz.transport13 import Node as GzNode
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.wrench_pb2 import Wrench as GzWrench
from gz.msgs10.entity_pb2 import Entity

class ForcePublisher(Node):
    def __init__(self):
        super().__init__('force_publisher')
        
        # Subscribe to ROS topic
        self.subscription = self.create_subscription(
            Wrench,
            '/block/force',
            self.force_callback,
            10
        )
        
        # Create Gazebo Transport node
        self.gz_node = GzNode()
        
        self.get_logger().info('Force Publisher ready. Publish to /block/force')
        
    def force_callback(self, ros_msg):
        # Create Gazebo EntityWrench message
        gz_msg = EntityWrench()
        
        # Set entity (which model to apply force to)
        gz_msg.entity.name = 'block'
        gz_msg.entity.type = Entity.MODEL
        
        # Copy force and torque from ROS message
        gz_msg.wrench.force.x = ros_msg.force.x
        gz_msg.wrench.force.y = ros_msg.force.y
        gz_msg.wrench.force.z = ros_msg.force.z
        gz_msg.wrench.torque.x = ros_msg.torque.x
        gz_msg.wrench.torque.y = ros_msg.torque.y
        gz_msg.wrench.torque.z = ros_msg.torque.z
        
        # Publish to Gazebo
        publisher = self.gz_node.advertise('/world/block_world/wrench', EntityWrench)
        publisher.publish(gz_msg)
        
        self.get_logger().info(f'Applied force: x={ros_msg.force.x}, z={ros_msg.force.z}')

def main(args=None):
    rclpy.init(args=args)
    node = ForcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()