#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf

class MultTarget:
    def __init__(self):
        rospy.init_node('mult_target_points')
        
        # 创建路径发布者
        self.path_pub = rospy.Publisher('/target_points', Path, queue_size=10)
        
        # 定义路径点 (x, y, theta)
        self.points = [
            (-3.0, 1.0, 0.0),   # start
            (7.0, -1.0, 1.57),  # 点2 (90度)
            (1.5, 2.5, 3.14),  # 点3 (180度)
            (-7.0, -1.0, -1.57), # 点4 (-90度)
            (1.0, 0.5, 0.0)    # end
        ]
        
    def create_path(self):
        """创建包含所有路径点的Path消息"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        for i, (x, y, yaw) in enumerate(self.points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.seq = i
            pose.header.stamp = rospy.Time.now()
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            # 将偏航角转换为四元数
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path.poses.append(pose)
        
        return path

    def run(self):
        rate = rospy.Rate(0.1)  # 每10秒发布一次
        while not rospy.is_shutdown():
            path_msg = self.create_path()
            self.path_pub.publish(path_msg)
            rospy.loginfo("Published path with %d points", len(self.points))
            rate.sleep()

if __name__ == '__main__':
    try:
        target = MultTarget()
        target.run()
    except rospy.ROSInterruptException:
        pass