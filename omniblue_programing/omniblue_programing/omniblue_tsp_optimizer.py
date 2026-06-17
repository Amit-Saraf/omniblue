#!/usr/bin/env python3
import numpy as np
import math

class TspOptimizer:
    """TSP路径优化器，用于优化多个航点的访问顺序"""
    
    @staticmethod
    def compute_distance(pose1, pose2):
        """计算两个位置之间的欧氏距离"""
        x1, y1, _ = pose1
        x2, y2, _ = pose2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    @staticmethod
    def optimize_waypoints(waypoints, start_position=None, method="nearest_neighbor"):
        """
        优化航点访问顺序
        
        参数:
            waypoints: 航点列表，每个航点是(x, y, theta)元组
            start_position: 起始位置(x, y, theta)，如果为None则使用第一个航点
            method: 优化方法，支持 "nearest_neighbor", "2opt", "brute_force"
            
        返回:
            优化后的航点列表
        """
        if len(waypoints) <= 1:
            return waypoints.copy()
            
        # 准备完整的位置列表，包括起始位置
        positions = []
        if start_position is not None:
            positions.append(start_position)
        positions.extend(waypoints)
        
        # 计算距离矩阵
        n = len(positions)
        distance_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    distance_matrix[i, j] = TspOptimizer.compute_distance(positions[i], positions[j])
                else:
                    distance_matrix[i, j] = float('inf')  # 自己到自己设为无穷大
        
        # 根据选择的方法优化路径
        if method == "nearest_neighbor":
            path = TspOptimizer._nearest_neighbor(distance_matrix, 0)
        elif method == "2opt":
            path = TspOptimizer._two_opt(distance_matrix, 0)
        elif method == "brute_force":
            path = TspOptimizer._brute_force(distance_matrix, 0)
        else:
            raise ValueError(f"不支持的优化方法: {method}")
        
        # 如果包含起点，则从路径中移除起点
        if start_position is not None:
            path = path[1:]
            # 调整索引，考虑起点的偏移
            path = [i-1 for i in path]
        
        # 按优化后的顺序重排航点
        optimized_waypoints = [waypoints[i] for i in path]
        return optimized_waypoints
    
    @staticmethod
    def _nearest_neighbor(distance_matrix, start_idx):
        """最近邻算法"""
        n = len(distance_matrix)
        path = [start_idx]
        unvisited = set(range(n))
        unvisited.remove(start_idx)
        
        while unvisited:
            last = path[-1]
            # 找到距离最后一个访问点最近的未访问点
            next_idx = min(unvisited, key=lambda x: distance_matrix[last, x])
            path.append(next_idx)
            unvisited.remove(next_idx)
            
        return path
    
    @staticmethod
    def _two_opt(distance_matrix, start_idx):
        """2-opt算法 - 先用最近邻生成初始路径，然后应用2-opt改进"""
        # 使用最近邻算法获取初始路径
        path = TspOptimizer._nearest_neighbor(distance_matrix, start_idx)
        n = len(path)
        
        # 2-opt改进
        improved = True
        while improved:
            improved = False
            for i in range(1, n-2):
                for j in range(i+1, n-1):
                    # 计算当前路径成本
                    current_cost = (distance_matrix[path[i-1], path[i]] + 
                                   distance_matrix[path[j], path[j+1]])
                    # 计算交换后路径成本
                    new_cost = (distance_matrix[path[i-1], path[j]] + 
                               distance_matrix[path[i], path[j+1]])
                    
                    if new_cost < current_cost:
                        # 反转子路径
                        path[i:j+1] = reversed(path[i:j+1])
                        improved = True
                        break
                if improved:
                    break
                    
        return path
    
    @staticmethod
    def _brute_force(distance_matrix, start_idx):
        """暴力搜索算法 - 仅适用于小规模问题"""
        n = len(distance_matrix)
        
        # 如果点数过多，自动降级到2-opt
        if n > 10:
            print(f"警告: 点数({n})过多，暴力搜索计算量过大，自动降级到2-opt算法")
            return TspOptimizer._two_opt(distance_matrix, start_idx)
            
        from itertools import permutations
        
        # 构建除起点外的所有点的排列
        other_points = list(range(n))
        other_points.remove(start_idx)
        
        best_path = None
        best_cost = float('inf')
        
        # 枚举所有可能的路径
        for p in permutations(other_points):
            path = [start_idx] + list(p)
            
            # 计算路径成本
            cost = 0
            for i in range(n-1):
                cost += distance_matrix[path[i], path[i+1]]
            
            # 更新最佳路径
            if cost < best_cost:
                best_cost = cost
                best_path = path
                
        return best_path
