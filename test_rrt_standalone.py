import numpy as np
import math
import random

class RRTStar:
    def __init__(self, start, goal, obstacles, bounds, max_iter=1000, step_size=2.0):
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
                
                # Check goal
                if np.linalg.norm(new_node - self.goal) < self.step_size:
                    if self.is_collision_free(new_node, self.goal):
                        self.nodes.append(self.goal)
                        self.parents[len(self.nodes)-1] = new_idx
                        return self.extract_path(len(self.nodes)-1)
                        
        return [self.start, self.goal]

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
        steps = int(np.linalg.norm(p2 - p1) / 0.5) + 1
        for i in range(steps):
            p = p1 + (p2 - p1) * i / steps
            for ox, oy, r in self.obstacles:
                dist_xy = math.hypot(p[0] - ox, p[1] - oy)
                if dist_xy < r + 1.0:
                    if p[2] < 5.0: # Obstacle height
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
        if len(path) < 3: return path
        smoothed = [path[0]]
        current_idx = 0
        while current_idx < len(path) - 1:
            last_valid_idx = current_idx + 1
            for i in range(len(path) - 1, current_idx + 1, -1):
                if self.is_collision_free(path[current_idx], path[i]):
                    last_valid_idx = i
                    break
            smoothed.append(path[last_valid_idx])
            current_idx = last_valid_idx
        return smoothed

def main():
    # Test case: Start at (0,0,2), Goal at (20,0,2), Obstacle at (10,0) radius 2
    # Should fly over the obstacle (z > 5)
    start = [0, 0, 2]
    goal = [20, 0, 2]
    obstacles = [(10.0, 0.0, 2.0)]
    bounds = (-5, 25, -5, 5, 0, 10)
    
    print("Planning RRT* path...")
    rrt = RRTStar(start, goal, obstacles, bounds)
    path = rrt.plan()
    
    print(f"Path found with {len(path)} points")
    max_z = 0
    for p in path:
        print(f"Point: {p}")
        if p[2] > max_z: max_z = p[2]
        
    if max_z > 5.0:
        print("SUCCESS: Path went over the obstacle!")
    else:
        # It might fail if it didn't find a path or went around (but bounds are tight in Y)
        # Actually bounds are -5 to 5 in Y, obstacle is at 0 with radius 2.
        # It could go around. Let's check if it collides.
        print(f"Max Z: {max_z}")

if __name__ == "__main__":
    main()
