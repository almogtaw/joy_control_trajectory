import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import yaml
import os

class JoyToMultiArrayNode(Node):
    def __init__(self):
        super().__init__('joy_to_multiarray_node')

        # Load the YAML configuration file for joystick mapping
        self.declare_parameter('config_file', 'config/joy_velocity.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.joint_mappings = self.load_config(config_file)

        # Publisher for joint velocities
        self.joint_vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def load_config(self, file_path):
        # Load the YAML configuration for joint mappings
        if not os.path.exists(file_path):
            self.get_logger().error(f"Config file {file_path} does not exist")
            return []

        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['joint_mappings']

    def joy_callback(self, joy_msg):
        # Create a Float64MultiArray message
        joint_vel_msg = Float64MultiArray()

        velocities = []

        # Map joystick axes to joint velocities using the configuration
        for mapping in self.joint_mappings:
            axis_value = joy_msg.axes[mapping['axis']]
            
            if mapping['invert']:
                axis_value = -axis_value
            
            # Scale velocity to joint's min/max limits
            min_vel = mapping['min_velocity']
            max_vel = mapping['max_velocity']
            velocity = axis_value * (max_vel - min_vel) / 2.0 + (min_vel + max_vel) / 2.0

            velocities.append(velocity)

        # Assign velocities to the Float64MultiArray data field
        joint_vel_msg.data = velocities
        
        # Publish the message
        self.joint_vel_pub.publish(joint_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToMultiArrayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
