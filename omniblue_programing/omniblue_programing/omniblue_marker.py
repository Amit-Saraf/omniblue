#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import math
import time
import threading

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        
        # 创建标记发布器
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/omniblue/waypoint_markers',
            10)
        
        # 存储当前正在导航的点
        self.current_point = -1
        
        # 存储航点
        self.waypoints = []
        
        self.get_logger().info('标记发布器已启动')
    
    def set_waypoints(self, waypoints):
        """设置要显示的航点"""
        self.waypoints = waypoints
        self.get_logger().info(f'设置了 {len(waypoints)} 个航点')
        self.publish_markers()
    
    def set_current_waypoint(self, index):
        """设置当前导航点"""
        self.current_point = index
        self.publish_markers()
        
    def publish_markers(self):
        """发布所有航点标记"""
        if not self.waypoints:
            return
            
        marker_array = MarkerArray()
        
        # 发布所有航点
        for i, (x, y, theta) in enumerate(self.waypoints):
            # 当前航点用红色，其他用黑色
            is_current = (i == self.current_point)
            
            # 创建航点标记
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 设置位置和大小
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.2
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # 设置颜色(当前点红色，其他点黑色)
            if is_current:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker.color.a = 0.8
            
            # 设置标记永久可见
            marker.lifetime.sec = 0
            
            marker_array.markers.append(marker)
            
            # 添加航点编号
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = 0.5
            text_marker.text = str(i+1)
            
            text_marker.scale.z = 0.3
            
            # 白色文本
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # 设置标记永久可见
            text_marker.lifetime.sec = 0
            
            marker_array.markers.append(text_marker)
        
        # 发布标记
        self.marker_publisher.publish(marker_array)

# 全局变量存储节点实例
marker_publisher = None

def initialize(initialize_rclpy=True):
    """初始化节点"""
    global marker_publisher
    if initialize_rclpy:
        try:
            rclpy.init()
        except RuntimeError:
            # ROS2已经初始化，忽略错误
            pass
    
    marker_publisher = MarkerPublisher()
    # 不启动线程，改为在主程序中手动调用spin_once
    
def set_waypoints(waypoints):
    """设置要显示的航点"""
    global marker_publisher
    if marker_publisher is not None:
        marker_publisher.set_waypoints(waypoints)

def set_current_waypoint(index):
    """设置当前导航点"""
    global marker_publisher
    if marker_publisher is not None:
        marker_publisher.set_current_waypoint(index)
        
def update():
    """手动更新标记节点"""
    global marker_publisher
    if marker_publisher is not None:
        rclpy.spin_once(marker_publisher, timeout_sec=0.001)
        
def shutdown():
    """关闭节点"""
    global marker_publisher
    if marker_publisher is not None:
        marker_publisher.destroy_node()

# 简单的测试代码
def main():
    print("标记发布器已启动，显示测试航点...")
    initialize()
    
    test_waypoints = [
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 0.0),
        (0.0, 2.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0)
    ]
    
    set_waypoints(test_waypoints)
    
    try:
        for i in range(len(test_waypoints)):
            print(f"设置当前点为: {i}")
            set_current_waypoint(i)
            # 手动更新节点
            for _ in range(20):  # 更新20次，大约持续0.02秒
                update()
            time.sleep(2)
        
        print("循环测试中，按Ctrl+C退出...")
        while True:
            for i in range(len(test_waypoints)):
                set_current_waypoint(i)
                # 手动更新节点
                for _ in range(10):
                    update()
                time.sleep(1)
    except KeyboardInterrupt:
        print("\n标记发布器被中断")
    finally:
        shutdown()
    return 0

if __name__ == '__main__':
    main()
