#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.last_positions = [0.0] * 6
        
        # Subscribe to joint_states from GUI
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client for trajectory
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        self.get_logger().info('Joint Command Bridge started')
    
    def joint_state_callback(self, msg):
        """Convert joint_state messages to trajectory commands"""
        try:
            # Extract positions for our 6 joints
            positions = []
            for joint_name in self.joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    positions.append(msg.position[idx])
                else:
                    positions.append(0.0)
            
            # Only send if positions changed
            if positions != self.last_positions:
                self.send_trajectory(positions)
                self.last_positions = positions
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def send_trajectory(self, positions):
        """Send trajectory to robot"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        
        goal.trajectory.points.append(point)
        
        self.client.wait_for_server()
        self.client.send_goal_async(goal)
        self.get_logger().info(f'Sent: {[f"{p:.2f}" for p in positions]}')

def main():
    rclpy.init()
    node = JointCommandBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
