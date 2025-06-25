#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid
import threading

class DynamicMapServer:
    def __init__(self):
        rospy.init_node('dynamic_map_server')
        
        # 订阅gmapping发布的地图
        self.current_map = None
        self.map_lock = threading.Lock()
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # 创建动态地图服务
        self.service = rospy.Service('/dynamic_map', GetMap, self.handle_map_request)
        rospy.loginfo("Dynamic map server ready")
        
    def map_callback(self, msg):
        """接收gmapping更新的地图"""
        with self.map_lock:
            self.current_map = msg
    
    def handle_map_request(self, req):
        """处理地图请求服务"""
        with self.map_lock:
            if self.current_map is None:
                rospy.logwarn("Map requested but not yet available")
                return GetMapResponse()
            
            response = GetMapResponse()
            response.map = self.current_map
            return response

if __name__ == '__main__':
    try:
        server = DynamicMapServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass