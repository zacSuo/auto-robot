#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal 
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

def send_goal(pub, x, y, orientation):
     # 构建消息
    goal_msg = MoveBaseActionGoal()
    
    # 填充 header (frame_id 通常为 "map")
    goal_msg.header = Header()
    goal_msg.header.seq = 0
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    
    # 填充 goal_id (可为空)
    goal_msg.goal_id = GoalID()
    goal_msg.goal_id.stamp = rospy.Time.now()
    goal_msg.goal_id.id = ""
    
    # 填充目标位姿
    goal_msg.goal = MoveBaseGoal()
    goal_msg.goal.target_pose = PoseStamped()
    
    # 目标位姿的 header
    goal_msg.goal.target_pose.header = Header()
    goal_msg.goal.target_pose.header.seq = 0
    goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
    goal_msg.goal.target_pose.header.frame_id = "map"
    
    # 目标位置
    goal_msg.goal.target_pose.pose.position = Point(x, y, 0)
    
    # 目标方向 (四元数)
    goal_msg.goal.target_pose.pose.orientation = Quaternion(
        *orientation  # 解包传入的四元数列表 [x, y, z, w]
    )
    
    # 发布消息
    pub.publish(goal_msg)
    rospy.loginfo("导航目标已发布！")

if __name__ == '__main__':
    rospy.init_node('set_goal_node', anonymous=True)
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.sleep(1)  
    # 示例：依次发送两个目标点
    send_goal(pub, 1.0, 0.5, [0.0, 0.0, 0.7071, 0.7071])  # 点1
    rospy.sleep(20)            # 等待到达
    send_goal(pub, -2.0, 3.0, [0.0, 0.0, 0.0, 0.0])  # 点2
    
