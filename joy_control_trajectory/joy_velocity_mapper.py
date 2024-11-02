import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
import os

class JoyToJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('joy_to_joint_trajectory_node')
        
        # Load the YAML configuration file
        self.declare_parameter('config_file', 'config/joystick_mapping.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.joy_mappings = self.load_yaml(config_file)
        
        # Create publisher
        self.joint_traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Initialize joint mapping and limits
        self.joint_limits = {}
        self.setup_mappings()
        
    def load_yaml(self, file_path):
        # Load the YAML configuration
        if not os.path.exists(file_path):
            self.get_logger().error(f"YAML file {file_path} does not exist")
            return {}
        
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def setup_mappings(self):
        # Initialize the mappings and limits from the config file
        self.joint_limits = {}
        for mapping in self.joy_mappings['joint_mappings']:
            joint_name = mapping['joint']
            self.joint_limits[joint_name] = {
                'axis': mapping['axis'],
                'min_velocity': mapping.get('min_velocity', -1.0),
                'max_velocity': mapping.get('max_velocity', 1.0),
                'invert': mapping.get('invert', False)
            }

    def joy_callback(self, joy_msg):
        # Create the JointTrajectory message
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.joint_names = []
        joint_traj_point = JointTrajectoryPoint()

        velocities = []
        
        # Map joystick axes to joint velocities
        for joint_name, limits in self.joint_limits.items():
            axis_value = joy_msg.axes[limits['axis']]
            
            if limits['invert']:
                axis_value = -axis_value
            
            # Scale velocity to joint's min/max limits
            min_vel = limits['min_velocity']
            max_vel = limits['max_velocity']
            velocity = axis_value * (max_vel - min_vel) / 2.0 + (min_vel + max_vel) / 2.0
            
            # Append the joint and its velocity
            joint_traj_msg.joint_names.append(joint_name)
            velocities.append(velocity)
        
        # Add empty positions to match the joint_names size
        # joint_traj_point.positions = [0.0] * len(joint_traj_msg.joint_names)
        joint_traj_point.positions = velocities
        
        # joint_traj_point.velocities = velocities
        joint_traj_point.time_from_start.sec = 0
        joint_traj_point.time_from_start.nanosec = 100000000  # 0.1 sec
        
        # Publish the message
        joint_traj_msg.points.append(joint_traj_point)
        self.joint_traj_pub.publish(joint_traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
