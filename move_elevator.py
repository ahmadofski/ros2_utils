#!/usr/bin/env python3
"""
Simple ROS2 client to actuate an elevator platform with ros2_control's joint_trajectory_controller
  Author: Ahmad Kurdi
  Usage: python3 ./move_elevator.py <up/down>
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
import argparse


ELEVATOR_RANGE = 0.034
MAX_EFFORT = 10.0
HOLD_EFFORT = 1.0
HOLD_TIME = 86400
ACTUATION_TIME = 4
NUM_POINTS = 11
ACTION_SERVER = 'rb1_robot/joint_trajectory_controller/follow_joint_trajectory'
JOINT_NAME = "robot_elevator_platform_joint"

class MoveElevatorClient(Node):
"""
ROS2 client node

  Args:
    action_server (str): path to follow_joint_trajectory action server
    elevator_range (float): range of elevator prismatic joint in meters     
"""

    def __init__(self,action_server,elevator_range):
        super().__init__('move_elevator_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, action_server)
        self.elevator_range = elevator_range

    
    def gen_points(self,num_points,upwards):
    """
    generate a set of points within elevator_range where spacing increases then decreases logarithmically 
    
        Args:
            num_points (int): number of points to generate
            upwards (bool): indicates increasing/decreasing direction
        Returns:
            Numpy array of float points of size 'num_points' ('num_points' + 1 if even)           
    """

        accel_range = int(num_points*0.5)
        accel = np.insert(np.logspace(0.1,1,accel_range)*self.elevator_range/20 ,0,0)
        dists = np.append(accel,self.elevator_range -  accel[-2::-1])
        return dists if upwards else np.flipud(dists)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Add optional feedback here')
    
    def send_goal(self, num_points, upwards):
    """
    Send action goal to client 
    
        Args:
            num_points (int): number of trajectory points
            upwards (bool): indicates direction
        Returns:
            rclpy action future          
    """
        dist_points = self.gen_points(num_points,upwards)
        steps = dist_points.size
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.goal_time_tolerance = Duration(seconds=1).to_msg()
        goal_msg.trajectory.joint_names = [JOINT_NAME]
        step_times=np.linspace(0,ACTUATION_TIME,num=steps)

        start_effort = MAX_EFFORT if upwards else 0
        end_effort = MAX_EFFORT * 0.75 if upwards else HOLD_EFFORT * 2
        torques=np.linspace(start_effort,end_effort,num=steps)

        # holding hack
        step_times = np.append(step_times,HOLD_TIME)
        torques = np.append(torques,HOLD_EFFORT)
        dist_points = np.append(dist_points,dist_points[-1])


        for i in range(steps):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=step_times[i]).to_msg()
            point.positions = [dist_points[i]]
            point.effort = [torques[i]]
            goal_msg.trajectory.points = [point]
        self._action_client.wait_for_server()

        self.get_logger().info('Sending elevator trajectory:')
        self.get_logger().info('\t- Time steps (s): \t{}'.format('\t'.join([f"{x:.3f}" for x in step_times])))
        self.get_logger().info('\t- Positions (m): \t{}'.format('\t'.join([f"{x:.3f}" for x in dist_points])))
        self.get_logger().info('\t- Effort (N): \t\t{}'.format('\t'.join([f"{x:.1f}" for x in torques])))
        

        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)


def main(args=None):
    
    parser = argparse.ArgumentParser()
    parser.add_argument('direction', default='down')
    args = parser.parse_args()

    rclpy.init()
    action_client = MoveElevatorClient(ACTION_SERVER,ELEVATOR_RANGE)
    future = action_client.send_goal(NUM_POINTS,args.direction=='up')
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
