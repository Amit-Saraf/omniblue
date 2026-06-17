#!/usr/bin/env python3
import rclpy
from omniblue_programing.omniblue_action import omniblue_cmd
from omniblue_programing.omniblue_marker import initialize, set_waypoints, set_current_waypoint, shutdown
import time

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 初始化标记发布器
    initialize(initialize_rclpy=False)
    
    # 创建导航器
    omniblue = omniblue_cmd()
    
    # 获取当前位置，确认初始坐标
    current_x, current_y, current_theta = omniblue.get_current_position()
    print(f"机器人初始位置: ({current_x}, {current_y}, {current_theta})")
    
    # 设置参考位置
    ref_x, ref_y = omniblue.set_reference_position()
    
    # 定义要访问的全局坐标点
    waypoints = [
        (1.0, 2.5, 0.0),
        (1.1, 9.6, 0.0),
        (-3.8, 9.0, 0.0),
        (-5.5, 1.3, 0.0),
        (0.0, -7.5, 0.0)
    ]
    
    # 转换为地图坐标显示
    map_waypoints = []
    for x, y, theta in waypoints:
        map_x = x - ref_x
        map_y = y - ref_y
        map_waypoints.append((map_x, map_y, theta))
    
    # 设置航点以显示
    set_waypoints(map_waypoints)
    print("已设置航点标记，黑色显示所有点")
    
    print("3秒后开始导航...")
    for i in range(3, 0, -1):
        print(f"{i}...")
        time.sleep(1)
    
    print("开始按顺序访问五个目标点...")
    
    # 开始导航到每个点
    for i, (x, y, theta) in enumerate(waypoints):
        # 先标记当前目标点为红色，然后再开始导航
        set_current_waypoint(i)
        
        # 导航到当前点
        print(f"导航到点 {i+1}: ({x}, {y}, {theta})")
        omniblue.move_global(x, y, theta)
        
        # 等待一下
        time.sleep(1)
    
    # 回到起始位置
    print("所有目标点访问完成，返回起始点...")
    set_current_waypoint(-1)  # 清除当前点高亮
    omniblue.move_map(0.0, 0.0, 0.0)  # 回到坐标原点
    
    # 完成导航
    print("导航完成，退出程序")
    
    # 关闭标记发布器
    shutdown()
    
    # 关闭ROS2
    omniblue.navigator.destroyNode()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()
