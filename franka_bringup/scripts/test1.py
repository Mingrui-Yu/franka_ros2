#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class JointImpedancePublisher(Node):
    def __init__(self):
        super().__init__('joint_impedance_publisher')
        
        # Create a publisher for the 'franka/joint_impedance_command' topic
        self.publisher_ = self.create_publisher(Float64MultiArray, 'franka/joint_impedance_command', 10)
        
        # Set a timer to call the 'publish_message' function at a regular interval
        self.timer = self.create_timer(0.1, self.publish_message)  # 1 second interval

        self.joint_pos = np.array([0, -np.pi/4, 0, -3.0/4.0*np.pi, 0, np.pi/2.0, np.pi/4.0])

    def publish_message(self):
        # Create a Float64MultiArray message
        msg = Float64MultiArray()

        self.joint_pos[6] += 0.02

        target_joint_pos = self.joint_pos.tolist()

        # target_joint_pos[6] += 1.0

        msg.data =  [float(i) for i in target_joint_pos]  # Example 7-dimensional vector
        
        # Publish the message to the topic
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the publisher node
    node = JointImpedancePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the ROS 2 client library
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
