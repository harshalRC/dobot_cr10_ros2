#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
import threading

class FastJointControl(Node):
    def __init__(self):
        super().__init__('fast_joint_control')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.last_positions = [0.0] * 6
        self.lock = threading.Lock()
        self.pending_goal = None
        
        # Subscribe to joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        # Timer to batch commands (10 Hz = 100ms)
        self.timer = self.create_timer(0.1, self.send_batched_command)
        
        self.get_logger().info('Fast Joint Control started')
    
    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        try:
            positions = []
            for joint_name in self.joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    positions.append(msg.position[idx])
                else:
                    positions.append(0.0)
            
            with self.lock:
                if positions != self.last_positions:
                    self.pending_goal = positions
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def send_batched_command(self):
        """Send batched command at fixed rate"""
        with self.lock:
            if self.pending_goal is None:
                return
            
            positions = self.pending_goal
            self.pending_goal = None
        
        # Cancel previous goal if still executing
        if hasattr(self, '_last_goal_handle') and self._last_goal_handle:
            try:
                self._last_goal_handle.cancel_goal_async()
            except:
                pass
        
        # Send new goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 200ms
        
        goal.trajectory.points.append(point)
        
        if self.client.wait_for_server(timeout_sec=0.1):
            future = self.client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
            self.last_positions = positions
    
    def goal_response_callback(self, future):
        try:
            self._last_goal_handle = future.result()
        except:
            pass

def main():
    rclpy.init()
    node = FastJointControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
