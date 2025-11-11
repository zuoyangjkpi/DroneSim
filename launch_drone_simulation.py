#!/usr/bin/env python3
"""
AVIANS ROS2 PORT1 - Drone Person Detection & Tracking Simulation (Fixed)
========================================================================

This script launches the complete drone simulation with:
- Gazebo world with drone and people
- YOLO12 person detection
- 3D projection model
- Distributed Kalman filter tracking
- Drone waypoint controller

Usage:
    chmod +x launch_drone_simulation_fixed.py
    ./launch_drone_simulation_fixed.py

Requirements:
    - ROS2 Humble/Jazzy
    - Gazebo Garden/Harmonic
    - All packages compiled successfully
"""

import os
import sys
import subprocess
import time
import signal
from pathlib import Path

class DroneSimulationLauncher:
    def __init__(self):
        self.processes = []
        self.setup_signal_handlers()
        
    def setup_signal_handlers(self):
        """Setup signal handlers for clean shutdown"""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\n[INFO] Received signal {signum}, shutting down...")
        self.cleanup()
        sys.exit(0)
        
    def cleanup(self):
        """Clean up all spawned processes"""
        print("[INFO] Cleaning up processes...")
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        
    def check_environment(self):
        """Check if ROS2 and required packages are available"""
        print("[INFO] Checking environment...")
        
        # Check ROS2 - more robust check
        ros_distro = os.environ.get('ROS_DISTRO')
        if not ros_distro:
            print("[ERROR] ROS_DISTRO not set. Please source ROS2 setup.bash")
            return False
        print(f"[OK] ROS2 {ros_distro} detected")
        
        # Check if ros2 command works
        try:
            result = subprocess.run(['which', 'ros2'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                print("[ERROR] ros2 command not found in PATH")
                return False
            print(f"[OK] ros2 command found at: {result.stdout.strip()}")
        except Exception as e:
            print(f"[ERROR] Failed to check ros2 command: {e}")
            return False
            
        # Check if packages are built
        required_packages = [
            'drone_description',
            'drone_state_publisher', 
            'neural_network_detector',
            'projection_model',
            'target_tracker_distributed_kf'
        ]
        
        print("[INFO] Checking required packages...")
        for package in required_packages:
            try:
                result = subprocess.run(['ros2', 'pkg', 'prefix', package],
                                      capture_output=True, text=True, timeout=10)
                if result.returncode != 0:
                    print(f"[WARNING] Package {package} not found - {result.stderr.strip()}")
                else:
                    print(f"[OK] Package {package} found")
            except Exception as e:
                print(f"[WARNING] Failed to check package {package}: {e}")
                
        return True
        
    def launch_gazebo_simulation(self):
        """Launch the main Gazebo simulation"""
        print("\n[INFO] Launching Gazebo simulation...")
        
        # Try the full simulation launch first
        cmd = [
            'ros2', 'launch', 
            'drone_state_publisher', 
            'simulation.launch.py',
            'use_sim_time:=false',
            'robot_id:=1',
            'num_robots:=1'
        ]
        
        print(f"[INFO] Running command: {' '.join(cmd)}")
        
        try:
            process = subprocess.Popen(cmd, 
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT,
                                     universal_newlines=True,
                                     bufsize=1)
            self.processes.append(process)
            print("[OK] Simulation process started")
            return process
        except Exception as e:
            print(f"[ERROR] Failed to launch simulation: {e}")
            return None
            
    def launch_simple_gazebo(self):
        """Launch simple Gazebo simulation as fallback"""
        print("\n[INFO] Launching simple Gazebo simulation...")
        
        cmd = [
            'ros2', 'launch',
            'drone_description',
            'gz.launch.py'
        ]
        
        print(f"[INFO] Running command: {' '.join(cmd)}")
        
        try:
            process = subprocess.Popen(cmd,
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.STDOUT,
                                     universal_newlines=True,
                                     bufsize=1)
            self.processes.append(process)
            print("[OK] Simple simulation process started")
            return process
        except Exception as e:
            print(f"[ERROR] Failed to launch simple simulation: {e}")
            return None
            
    def launch_individual_nodes(self):
        """Launch individual nodes as fallback"""
        print("\n[INFO] Launching individual nodes...")
        
        # Launch YOLO detector
        try:
            cmd = ['ros2', 'run', 'neural_network_detector', 'yolo12_detector_node']
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            self.processes.append(process)
            print("[OK] YOLO detector started")
        except Exception as e:
            print(f"[WARNING] Failed to start YOLO detector: {e}")
            
        # Launch projection model
        try:
            cmd = ['ros2', 'run', 'projection_model', 'projection_model_node']
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            self.processes.append(process)
            print("[OK] Projection model started")
        except Exception as e:
            print(f"[WARNING] Failed to start projection model: {e}")
            
        return len(self.processes) > 0
            
    def monitor_processes(self):
        """Monitor running processes and show status"""
        print("\n" + "="*60)
        print("SIMULATION RUNNING")
        print("="*60)
        print("Expected components:")
        print("  ✓ Gazebo simulation with drone")
        print("  ✓ YOLO12 person detection")
        print("  ✓ 3D projection model") 
        print("  ✓ Kalman filter tracking")
        print("  ✓ RViz visualization")
        print("\nUseful commands to run in another terminal:")
        print("  ros2 topic list                    - List all topics")
        print("  ros2 topic echo /camera/image_raw  - Monitor camera")
        print("  ros2 topic echo /person_detections - Monitor detections")
        print("  ros2 node list                     - List all nodes")
        print("  rviz2                              - Open RViz")
        print("\nPress Ctrl+C to stop simulation")
        print("="*60)
        
        try:
            while True:
                # Check if any process has died
                active_processes = 0
                for i, process in enumerate(self.processes):
                    if process.poll() is None:
                        active_processes += 1
                    else:
                        # Show some output from terminated process
                        try:
                            stdout, stderr = process.communicate(timeout=1)
                            if stdout:
                                print(f"\n[Process {i} output]: {stdout[-200:]}")  # Last 200 chars
                        except:
                            pass
                        
                if active_processes == 0:
                    print("\n[WARNING] All processes have terminated")
                    break
                    
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\n[INFO] Keyboard interrupt received")
            
    def run(self):
        """Main execution function"""
        print("="*60)
        print("AVIANS ROS2 PORT1 - Drone Simulation Launcher (Fixed)")
        print("="*60)
        
        # Check environment
        if not self.check_environment():
            print("[ERROR] Environment check failed")
            return False
            
        # Launch simulation - try multiple approaches
        print("\n[INFO] Starting simulation components...")
        
        # Try full simulation first
        process = self.launch_gazebo_simulation()
        if not process:
            print("[INFO] Full simulation failed, trying simple simulation...")
            process = self.launch_simple_gazebo()
            
        if not process:
            print("[INFO] Gazebo launch failed, trying individual nodes...")
            if not self.launch_individual_nodes():
                print("[ERROR] Failed to launch any components")
                return False
                
        # Wait for simulation to start
        print("[INFO] Waiting for simulation to initialize...")
        time.sleep(5)
        
        # Show some initial output
        if self.processes:
            print("\n[INFO] Process output (first few seconds):")
            time.sleep(3)
            for i, process in enumerate(self.processes):
                if process.poll() is None:
                    print(f"[Process {i}] Still running...")
                else:
                    print(f"[Process {i}] Terminated early")
        
        # Monitor
        self.monitor_processes()
        
        # Cleanup
        self.cleanup()
        return True

def main():
    # Show environment info
    print("Environment Information:")
    print(f"  ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'Not set')}")
    print(f"  AMENT_PREFIX_PATH: {os.environ.get('AMENT_PREFIX_PATH', 'Not set')[:100]}...")
    print(f"  Current directory: {os.getcwd()}")
    print()
    
    launcher = DroneSimulationLauncher()
    success = launcher.run()
    
    if success:
        print("\n[INFO] Simulation completed")
    else:
        print("\n[ERROR] Simulation failed")
        print("\nTroubleshooting tips:")
        print("1. Make sure all packages are compiled: colcon build")
        print("2. Source the workspace: source install/setup.bash")
        print("3. Try individual launch files:")
        print("   ros2 launch drone_description gz.launch.py")
        print("   ros2 run neural_network_detector yolo12_detector_node")
        sys.exit(1)

if __name__ == "__main__":
    main()
