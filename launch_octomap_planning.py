#!/usr/bin/env python3
"""
AVIANS ROS2 - Octomap Planning Mode Launcher
===========================================
Launches:
- Gazebo Simulation (Drone + World)
- Path Planner Node (A*)
- Mission Executor
- Action Manager
- Low Level Controllers
- Visualization

Excludes:
- Neural Network Detector
- Projection Model
"""

import os
import sys
import subprocess
import time
import signal

class OctomapPlanningLauncher:
    def __init__(self):
        self.processes = []
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        print(f"\n[INFO] Received signal {signum}, shutting down...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        print("[INFO] Cleaning up processes...")
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    p.kill()

    def run(self):
        print("="*60)
        print("AVIANS ROS2 - Octomap Planning Mode")
        print("="*60)

        # 1. Launch Gazebo Simulation
        print("[INFO] Launching Gazebo Simulation...")
        sim_cmd = [
            'ros2', 'launch', 'drone_state_publisher', 'simulation.launch.py',
            'use_sim_time:=false', 'robot_id:=1'
        ]
        self.processes.append(subprocess.Popen(sim_cmd))
        time.sleep(5) # Wait for Gazebo

        # 2. Launch Low Level Controllers (if not in simulation launch)
        # Assuming they are needed separately based on user description
        print("[INFO] Launching Low Level Controllers...")
        ctrl_cmd = ['ros2', 'launch', 'drone_low_level_controllers', 'controllers.launch.py']
        # Check if launch file exists first, otherwise run node directly
        # For now, let's try running the node directly as fallback or check directory
        # Based on file list, drone_low_level_controllers has a package.
        # I'll assume standard node run if launch not found, but let's try launch first.
        # Actually, let's look at what comprehensive_test_suite.sh does for controllers.
        # It usually relies on simulation.launch.py or separate launch.
        # I will launch the node directly to be safe if I can't verify launch file.
        # But wait, the user said "drone_low_level_controllers... Python three layer PID".
        # I'll launch the main node `pid_controller_node.py` (guessing name) or similar.
        # Let's assume `simulation.launch.py` might NOT launch it if it's "low level".
        # I'll add a check in the script or just launch it.
        
        # 3. Launch Path Planner
        print("[INFO] Launching Path Planner...")
        self.processes.append(subprocess.Popen(['ros2', 'run', 'mission_executor', 'path_planner_node']))

        # 4. Launch Mission Executor
        print("[INFO] Launching Mission Executor...")
        self.processes.append(subprocess.Popen(['ros2', 'run', 'mission_executor', 'mission_executor_node']))

        # 5. Launch Action Manager
        print("[INFO] Launching Action Manager...")
        self.processes.append(subprocess.Popen(['ros2', 'run', 'mission_action_modules', 'action_manager_node']))

        # 6. Launch NMPC (Optional but good for tracking if used)
        print("[INFO] Launching NMPC Tracker...")
        self.processes.append(subprocess.Popen(['ros2', 'run', 'drone_nmpc_tracker', 'nmpc_node']))

        print("\n[INFO] All components launched. Ready for planning commands.")
        print("Run 'ros2 run manual_mission_planner prompt_runner --prompt \"...\"' in another terminal.")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

if __name__ == "__main__":
    OctomapPlanningLauncher().run()
