#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class ExplorationController:
    def __init__(self):
        rospy.init_node('exploration_controller')
        
        # 连接到move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # 当前目标索引
        self.goal_index = 0
        self.exploration_goals = [
            (1.0, 0.0, 0.0),    # 前方1米
            (0.0, 1.0, 1.57),   # 左侧1米，90度
            (-1.0, 0.0, 3.14),  # 后方1米，180度
            (0.0, -1.0, -1.57)  # 右侧1米，-90度
        ]
        
        # 启动探索循环
        self.explore()
    
    def send_goal(self, x, y, yaw):
        """发送导航目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # 转换偏航角为四元数
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        self.client.send_goal(goal)
        rospy.loginfo(f"Sent exploration goal: ({x:.2f}, {y:.2f}, {yaw:.2f})")
    
    def explore(self):
        """自主探索循环"""
        rate = rospy.Rate(0.2)  # 每5秒检查一次
        
        while not rospy.is_shutdown():
            # 如果当前目标已完成，发送下一个目标
            if self.client.get_state() in [actionlib.GoalStatus.SUCCEEDED, 
                                         actionlib.GoalStatus.ABORTED,
                                         actionlib.GoalStatus.PREEMPTED]:
                x, y, yaw = self.exploration_goals[self.goal_index]
                self.send_goal(x, y, yaw)
                
                # 更新目标索引
                self.goal_index = (self.goal_index + 1) % len(self.exploration_goals)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = ExplorationController()
    except rospy.ROSInterruptException:
        pass