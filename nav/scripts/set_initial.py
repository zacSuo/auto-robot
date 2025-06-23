#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Pose
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
        
    # 填充位姿
    goal_msg.pose = PoseWithCovariance()
    goal_msg.pose.pose = Pose()
        
    # 位置
    goal_msg.goal.pose.pose.position = Point(x, y, 0)
    
    # 目标方向 (四元数)
    goal_msg.goal.pose.pose.orientation = Quaternion(
        *orientation  # 解包传入的四元数列表 [x, y, z, w]
    )
    
    # 发布消息
    pub.publish(goal_msg)
    rospy.loginfo("initial position set")

if __name__ == '__main__':
    rospy.init_node('set_initial_node', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  
    # 发送点
    send_goal(pub, -3.0, 1.0, [0.0, 0.0, 0, 0])  
    
