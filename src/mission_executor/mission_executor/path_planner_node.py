#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from uav_msgs.srv import PlanPath
import numpy as np
import math

import random

class RRTStar:
    def __init__(self, start, goal, obstacles, bounds, max_iter=500, step_size=2.0):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles # List of (x, y, r)
        self.bounds = bounds # (min_x, max_x, min_y, max_y, min_z, max_z)
        self.max_iter = max_iter
        self.step_size = step_size
        self.nodes = [self.start]
        self.parents = {0: None}
        self.costs = {0: 0.0}

    def plan(self):
        for i in range(self.max_iter):
            rnd_point = self.sample_point()
            nearest_idx = self.get_nearest_idx(rnd_point)
            nearest_node = self.nodes[nearest_idx]
            
            new_node = self.steer(nearest_node, rnd_point)
            
            if self.is_collision_free(nearest_node, new_node):
                new_idx = len(self.nodes)
                self.nodes.append(new_node)
                self.parents[new_idx] = nearest_idx
                self.costs[new_idx] = self.costs[nearest_idx] + np.linalg.norm(new_node - nearest_node)
                
                # Rewire (simplified for brevity)
                if np.linalg.norm(new_node - self.goal) < self.step_size:
                    if self.is_collision_free(new_node, self.goal):
                        self.nodes.append(self.goal)
                        self.parents[len(self.nodes)-1] = new_idx
                        return self.extract_path(len(self.nodes)-1)
                        
        return [self.start, self.goal] # Fallback: direct line

    def sample_point(self):
        if random.random() < 0.1:
            return self.goal
        return np.array([
            random.uniform(self.bounds[0], self.bounds[1]),
            random.uniform(self.bounds[2], self.bounds[3]),
            random.uniform(self.bounds[4], self.bounds[5])
        ])

    def get_nearest_idx(self, point):
        dists = [np.linalg.norm(point - node) for node in self.nodes]
        return np.argmin(dists)

    def steer(self, from_node, to_node):
        vec = to_node - from_node
        dist = np.linalg.norm(vec)
        if dist > self.step_size:
            vec = vec / dist * self.step_size
        return from_node + vec

    def is_collision_free(self, p1, p2):
        # Check collision with cylindrical obstacles
        # Discretize line segment
        steps = int(np.linalg.norm(p2 - p1) / 0.5) + 1
        for i in range(steps):
            p = p1 + (p2 - p1) * i / steps
            for ox, oy, r in self.obstacles:
                # 2D distance check
                dist_xy = math.hypot(p[0] - ox, p[1] - oy)
                if dist_xy < r + 1.0:
                    # If inside XY radius, check Z clearance
                    # Assume obstacles are 5m tall cylinders
                    if p[2] < 5.0: 
                        return False
        return True

    def extract_path(self, end_idx):
        path = []
        curr = end_idx
        while curr is not None:
            path.append(self.nodes[curr])
            curr = self.parents[curr]
        return self.smooth_path(path[::-1])

    def smooth_path(self, path):
        """Simple shortcutting smoothing."""
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            last_valid_idx = current_idx + 1
            # Try to connect to furthest possible node
            for i in range(len(path) - 1, current_idx + 1, -1):
                if self.is_collision_free(path[current_idx], path[i]):
                    last_valid_idx = i
                    break
            
            smoothed.append(path[last_valid_idx])
            current_idx = last_valid_idx
            
        return smoothed

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.srv = self.create_service(PlanPath, 'mission_planner/plan_path', self.plan_path_callback)
        self.get_logger().info('Path Planner Node ready.')
        
        # Mock grid parameters for A* (100x100m area, 1m resolution)
        self.grid_size = 100
        self.resolution = 1.0
        self.origin = np.array([-50.0, -50.0]) # Center at 0,0
        
        # Simple obstacles (list of circles: center_x, center_y, radius)
        self.obstacles = [
            (10.0, 5.0, 2.0),
            (20.0, 0.0, 3.0),
            (15.0, 15.0, 2.0)
        ]

    def plan_path_callback(self, request, response):
        start_pos = request.start.position
        goal_pos = request.goal.position
        
        self.get_logger().info(f"Planning path from ({start_pos.x:.1f}, {start_pos.y:.1f}) to ({goal_pos.x:.1f}, {goal_pos.y:.1f})")
        
        # 1. Run A* (Global 2D)
        astar_points = self.astar_search(
            (start_pos.x, start_pos.y),
            (goal_pos.x, goal_pos.y)
        )
        
        if not astar_points:
            response.success = False
            self.get_logger().warn("No A* path found")
            return response

        # 2. Refine with RRT* (Local 3D for Z-axis)
        full_path = []
        current_z = start_pos.z
        
        for i in range(len(astar_points) - 1):
            p1 = np.array([astar_points[i][0], astar_points[i][1], current_z])
            p2 = np.array([astar_points[i+1][0], astar_points[i+1][1], goal_pos.z]) # Target goal Z
            
            # Define bounds for local RRT*
            margin = 5.0
            bounds = (
                min(p1[0], p2[0]) - margin, max(p1[0], p2[0]) + margin,
                min(p1[1], p2[1]) - margin, max(p1[1], p2[1]) + margin,
                0.0, 10.0 # Z bounds
            )
            
            rrt = RRTStar(p1, p2, self.obstacles, bounds)
            segment_path = rrt.plan()
            
            # Append segment (skip first point to avoid duplicates)
            if i == 0:
                full_path.extend(segment_path)
            else:
                full_path.extend(segment_path[1:])
            
            current_z = full_path[-1][2]

        response.success = True
        response.path = []
        for p in full_path:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            response.path.append(pose)
            
        self.get_logger().info(f"Path found with {len(full_path)} waypoints (A* + RRT*)")
        return response

    def astar_search(self, start, goal):
        # Discretize to grid
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)
        
        open_set = {start_node}
        came_from = {}
        
        g_score = {start_node: 0}
        f_score = {start_node: self.heuristic(start_node, goal_node)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            
            if current == goal_node:
                return self.reconstruct_path(came_from, current, goal)
            
            open_set.remove(current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_node)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        
        return None # No path

    def world_to_grid(self, pos):
        ix = int((pos[0] - self.origin[0]) / self.resolution)
        iy = int((pos[1] - self.origin[1]) / self.resolution)
        return (ix, iy)

    def grid_to_world(self, node):
        x = node[0] * self.resolution + self.origin[0]
        y = node[1] * self.resolution + self.origin[1]
        return (x, y)

    def get_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0: continue
                nx, ny = node[0] + dx, node[1] + dy
                if self.is_valid(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors

    def is_valid(self, ix, iy):
        # Check bounds
        if ix < 0 or ix >= self.grid_size or iy < 0 or iy >= self.grid_size:
            return False
        
        # Check obstacles (A* only checks 2D clearance)
        wx, wy = self.grid_to_world((ix, iy))
        for ox, oy, r in self.obstacles:
            if math.hypot(wx - ox, wy - oy) < r + 1.0: # +1.0 clearance
                return False
        return True

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(self, came_from, current, real_goal):
        total_path = [self.grid_to_world(current)]
        while current in came_from:
            current = came_from[current]
            total_path.append(self.grid_to_world(current))
        total_path.reverse()
        # Replace last point with exact goal
        total_path[-1] = real_goal
        return total_path

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
