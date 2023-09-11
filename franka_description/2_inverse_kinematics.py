#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os 
import roboticstoolbox as rtb

# Near to the ground to check grab
#2.1 0 1.94
## full strech right
#3.33 0.05 0.66
## Full strech up
#0.47 0 3.78
## Random
# 0.63 -0.148 2.39

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory,publish_topic, 10);timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6', 'panda_joint7']
    

        panda = rtb.models.Panda()        
        ets = panda.ets()
        Tep = panda.fkine([0, -0.3, 0, -2.2, 0, 2, 0.7854])        
        print("\nTransformation Matrix :\n",Tep)
        solver = rtb.IK_LM()
        ik_solution = solver.solve(ets, Tep)
        print(ik_solution)
        angles = ik_solution.q
        self.goal_positions = angles    
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)
 
   
    def timer_callback(self):
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(bazu_trajectory_msg)
        print("\nTrajectory Sent !\n")
        
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()