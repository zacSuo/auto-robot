#!/usr/bin/env python3
import rospy
import actionlib
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion
import numpy as np

class TrajectoryPlan:
    def __init__(self):
        rospy.init_node('traje_plan')
        
        # 动作客户端连接到move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")
        
        # 订阅路径和机器人位姿
        self.path_sub = rospy.Subscriber('/target_points', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 当前状态
        self.current_path = None
        self.current_pose = None
        self.current_goal_index = 0
        self.path_received = False
        
        # 控制参数
        self.goal_tolerance = 0.2  # 目标点容差 (米)
        self.angle_tolerance = 0.2  # 角度容差 (弧度)
        
        # 主循环
        self.control_loop()
        
    def path_callback(self, msg):
        """接收新路径时的回调函数"""
        if not self.path_received or len(msg.poses) > len(self.current_path.poses):
            self.current_path = msg
            self.current_goal_index = 0
            self.path_received = True
            rospy.loginfo("Received new path with %d points", len(msg.poses))
    
    def odom_callback(self, msg):
        """更新当前机器人位姿"""
        self.current_pose = msg.pose.pose
    
    def distance_to_goal(self, goal_pose):
        """计算当前位置到目标点的距离"""
        if self.current_pose is None:
            return float('inf')
            
        dx = goal_pose.position.x - self.current_pose.position.x
        dy = goal_pose.position.y - self.current_pose.position.y
        return np.sqrt(dx**2 + dy**2)
    
    def angle_to_goal(self, goal_pose):
        """计算当前位置到目标点的角度差"""
        if self.current_pose is None:
            return float('inf')
            
        # 获取当前偏航角
        _, _, current_yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])
        
        # 获取目标偏航角
        _, _, goal_yaw = euler_from_quaternion([
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w
        ])
        
        return abs(current_yaw - goal_yaw)
    
    def send_goal(self, pose_stamped):
        """发送单个目标点到move_base"""
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        self.client.send_goal(goal)
    
    def control_loop(self):
        """主控制循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.current_path is not None and self.current_pose is not None:
                # 检查是否完成所有路径点
                if self.current_goal_index >= len(self.current_path.poses):
                    rospy.loginfo("Completed all points!")
                    self.current_path = None
                    self.path_received = False
                    continue
                
                # 获取当前目标点
                current_goal = self.current_path.poses[self.current_goal_index]
                
                # 检查是否到达当前目标点
                distance = self.distance_to_goal(current_goal.pose)
                angle_diff = self.angle_to_goal(current_goal.pose)
                
                if distance < self.goal_tolerance and angle_diff < self.angle_tolerance:
                    rospy.loginfo("Reached point %d/%d", 
                                 self.current_goal_index+1, 
                                 len(self.current_path.poses))
                    self.current_goal_index += 1
                    
                    # 发送下一个目标点
                    if self.current_goal_index < len(self.current_path.poses):
                        next_goal = self.current_path.poses[self.current_goal_index]
                        self.send_goal(next_goal)
                else:
                    # 如果当前没有活动目标，发送第一个目标点
                    if not self.client.get_state() in [actionlib.GoalStatus.ACTIVE, 
                                                      actionlib.GoalStatus.PENDING]:
                        self.send_goal(current_goal)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        trajectory = TrajectoryPlan()
    except rospy.ROSInterruptException:
        pass