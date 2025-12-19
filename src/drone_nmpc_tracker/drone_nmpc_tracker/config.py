#!/usr/bin/env python3
"""
NMPC Configuration for Drone Person Tracking
Based on formation_config.py from AirshipMPC
Adapted for ROS2 Jazzy and Python 3.12
"""

import math
import numpy as np

class NMPCConfig:
    """Configuration class for NMPC drone person tracking"""
    
    def __init__(self):
        # ========== NMPC Parameters ==========
        self.TIMESTEP = 0.35  # Time step in seconds (increased from 0.25 for larger steps)
        self.LOOKAHEAD = 2.1  # Look ahead time in seconds
        self.N = int(self.LOOKAHEAD / self.TIMESTEP)  # Number of prediction steps = 6
        
        # ========== Drone Parameters ==========
        self.MACHINES = 1  # Number of drones (single drone tracking)
        self.OBSTACLES = 0  # Number of static obstacles
        
        # Drone physical parameters
        self.DRONE_MASS = 5.0  # kg (octocopter)
        self.DRONE_MAX_VELOCITY = 5.0  # m/s - INCREASED for faster tracking
        self.DRONE_MAX_ACCELERATION = 3.5  # m/s^2 - INCREASED to match Gazebo limits
        self.DRONE_MAX_ANGULAR_VELOCITY = 3.0  # rad/s

        # Drone moment of inertia
        self.DRONE_INERTIA_XX = 0.45  # kg*m^2
        self.DRONE_INERTIA_YY = 0.45  # kg*m^2
        self.DRONE_INERTIA_ZZ = 0.9   # kg*m^2
        
        # ========== State Vector Definition ==========
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        self.STATE_SIZE = 12
        self.CONTROL_SIZE = 4  # [thrust, roll_cmd, pitch_cmd, yaw_rate_cmd]
        
        # State indices
        self.STATE_X = 0
        self.STATE_Y = 1
        self.STATE_Z = 2
        self.STATE_VX = 3
        self.STATE_VY = 4
        self.STATE_VZ = 5
        self.STATE_ROLL = 6
        self.STATE_PITCH = 7
        self.STATE_YAW = 8
        self.STATE_WX = 9
        self.STATE_WY = 10
        self.STATE_WZ = 11
        
        # ========== Cost Function Weights ==========
        # Position / dynamics tracking weights
        # VERY AGGRESSIVE: Extremely high W_POSITION to force rapid movement
        self.W_POSITION = np.array([80.0, 80.0, 25.0])  # DOUBLED AGAIN - absolutely prioritize position tracking
        self.W_VELOCITY = np.array([0.15, 0.15, 0.1])   # REDUCED further - allow max speeds
        self.W_ACCELERATION = np.array([1.8, 1.8, 0.6]) # REDUCED more - allow aggressive acceleration
        self.TARGET_ATTITUDE_SMOOTHING = 0.15           # Reduced from 0.2 for faster attitude response

        # Camera stability - limit pitch/roll to keep person steady in frame
        self.W_CAMERA_TILT = 15.0                       # Penalty for excessive tilt angles

        # Person tracking specific weights - prioritize distance over everything
        self.W_TRACKING_DISTANCE = 20.0  # DOUBLED from 10.0 - force maintaining tracking distance
        self.W_SMOOTH_TRACKING = 0.5     # Further reduced from 0.8 to react even quicker
        self.W_YAW_ALIGNMENT = 12.0      # Slightly increased from 10.0 for better camera pointing
        
        # ========== Constraints ==========
        # State constraints
        self.STATE_MIN = np.array([
            -50.0, -50.0, 0.5,      # Position limits [x, y, z]
            -self.DRONE_MAX_VELOCITY, -self.DRONE_MAX_VELOCITY, -self.DRONE_MAX_VELOCITY,  # Velocity limits
            -math.pi/4, -math.pi/4, -math.pi,  # Attitude limits [roll, pitch, yaw]
            -self.DRONE_MAX_ANGULAR_VELOCITY, -self.DRONE_MAX_ANGULAR_VELOCITY, -self.DRONE_MAX_ANGULAR_VELOCITY  # Angular rate limits
        ])
        
        self.STATE_MAX = np.array([
            50.0, 50.0, 20.0,       # Position limits [x, y, z]
            self.DRONE_MAX_VELOCITY, self.DRONE_MAX_VELOCITY, self.DRONE_MAX_VELOCITY,   # Velocity limits
            math.pi/4, math.pi/4, math.pi,     # Attitude limits [roll, pitch, yaw]
            self.DRONE_MAX_ANGULAR_VELOCITY, self.DRONE_MAX_ANGULAR_VELOCITY, self.DRONE_MAX_ANGULAR_VELOCITY   # Angular rate limits
        ])
        
        # Control constraints
        self.CONTROL_MIN = np.array([
            0.0,          # Minimum thrust
            -math.pi/4.5, # Minimum roll command (40°) - INCREASED for aggressive tracking
            -math.pi/4.5, # Minimum pitch command (40°) - INCREASED for aggressive tracking
            -2.5          # Minimum yaw rate command (rad/s) - INCREASED
        ])

        self.CONTROL_MAX = np.array([
            80.0,         # Maximum thrust (N) for heavier platform
            math.pi/4.5,  # Maximum roll command (40°) - INCREASED for aggressive tracking
            math.pi/4.5,  # Maximum pitch command (40°) - INCREASED for aggressive tracking
            2.5           # Maximum yaw rate command (rad/s) - INCREASED
        ])
        
        # ========== Person Tracking Parameters ==========
        # Track a fixed horizontal offset while keeping the camera pointed at the person
        self.DESIRED_TRACKING_DISTANCE_XY = 3.5     # Desired horizontal spacing (m)
        self.TRACKING_HEIGHT_OFFSET = 1.5          # Increased height offset for better downward view
        self.TRACKING_FIXED_ALTITUDE = 3.0         # Consistent with takeoff altitude for unified flight height (m)
        self.TARGET_POSITION_SMOOTHING = 0.95      # High alpha = more new data, less smoothing
        self.PERSON_POSITION_FILTER_ALPHA = 0.75   # Much higher to reduce lag and track people faster
        self.MIN_WAYPOINT_SPACING = 0.25           # Minimum spacing between generated waypoints (m)
        self.WAYPOINT_PLAN_CHANGE_THRESHOLD = 0.4  # Distance threshold to treat a plan as new

        # Camera parameters - updated for X3 drone configuration
        self.CAMERA_FOV_HORIZONTAL = 1.2        # 68.75 degrees horizontal FOV (from model.sdf)
        self.CAMERA_FOV_VERTICAL = 0.9          # Estimated 51.6 degrees vertical FOV
        self.CAMERA_TILT_ANGLE = -0.5236        # 30 degrees down tilt (from model.sdf)
        self.CAMERA_FORWARD_OFFSET = 0.2        # Camera 0.2m forward from drone center
        
        # ========== Environmental Parameters ==========
        self.GRAVITY = 9.81  # m/s^2
        self.AIR_DENSITY = 1.225  # kg/m^3
        
        # Wind parameters (can be updated dynamically)
        self.WIND_VELOCITY = np.array([0.0, 0.0, 0.0])  # [wx, wy, wz] m/s
        self.WIND_TURBULENCE = 0.1  # Wind turbulence factor
        
        # ========== Solver Parameters ==========
        self.MAX_ITERATIONS = 25  # INCREASED for better convergence
        self.CONVERGENCE_TOLERANCE = 5e-3  # Relaxed for faster convergence and stability
        self.STEP_SIZE = 0.08  # INCREASED 4x - optimizer was too conservative
        self.REGULARIZATION = 1e-4  # Increased for better numerical stability
        
        # ========== ROS2 Topic Names ==========
        self.TOPIC_DRONE_STATE = '/X3/odometry'
        self.TOPIC_TARGET_DETECTIONS = '/target_detections'
        # Backwards compatibility alias
        self.TOPIC_PERSON_DETECTIONS = self.TOPIC_TARGET_DETECTIONS
        self.TOPIC_CONTROL_OUTPUT = '/drone/control/waypoint_command'
        self.TOPIC_WAYPOINT_CMD = '/drone/control/waypoint_command'
        self.TOPIC_ATTITUDE_CMD = '/drone/control/attitude_command'
        self.TOPIC_TRAJECTORY_VIS = '/drone/trajectory'
        self.TOPIC_TARGET_VIS = '/drone/target'
        self.TOPIC_STATUS = '/drone/controller/status'
        
        # ========== Logging and Debug ==========
        self.DEBUG_MODE = True
        self.LOG_LEVEL = 'INFO'  # DEBUG, INFO, WARN, ERROR
        self.PUBLISH_VISUALIZATION = True
        self.SAVE_TRAJECTORY_DATA = False

    def get_initial_state(self):
        """Get initial state vector"""
        return np.zeros(self.STATE_SIZE)
    
    def get_hover_control(self):
        """Get hover control input"""
        hover_thrust = self.DRONE_MASS * self.GRAVITY
        return np.array([hover_thrust, 0.0, 0.0, 0.0])
    
    def is_state_valid(self, state):
        """Check if state is within constraints"""
        return np.all(state >= self.STATE_MIN) and np.all(state <= self.STATE_MAX)
    
    def is_control_valid(self, control):
        """Check if control is within constraints"""
        return np.all(control >= self.CONTROL_MIN) and np.all(control <= self.CONTROL_MAX)
    
    def clip_state(self, state):
        """Clip state to constraints"""
        return np.clip(state, self.STATE_MIN, self.STATE_MAX)
    
    def clip_control(self, control):
        """Clip control to constraints"""
        return np.clip(control, self.CONTROL_MIN, self.CONTROL_MAX)

# Global configuration instance
nmpc_config = NMPCConfig()
