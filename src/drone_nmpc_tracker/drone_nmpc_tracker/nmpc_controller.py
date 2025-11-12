#!/usr/bin/env python3
"""
Nonlinear Model Predictive Controller for Drone Person Tracking
Based on blimp_nmpc_wrapper_node.py from AirshipMPC
Adapted for ROS2 Jazzy, Python 3.12, and quadrotor dynamics
"""

import math
import numpy as np
from typing import Tuple, List, Optional
import time

from .config import nmpc_config

class State:
    """State vector class for NMPC operations"""
    
    def __init__(self, data=None):
        if data is None:
            self.data = np.zeros(nmpc_config.STATE_SIZE)
        else:
            self.data = np.array(data, dtype=np.float64)
    
    def __getitem__(self, index):
        return self.data[index]
    
    def __setitem__(self, index, value):
        self.data[index] = value
    
    def __add__(self, other):
        if isinstance(other, State):
            return State(self.data + other.data)
        return State(self.data + other)
    
    def __sub__(self, other):
        if isinstance(other, State):
            return State(self.data - other.data)
        return State(self.data - other)
    
    def __mul__(self, scalar):
        return State(self.data * scalar)
    
    def __rmul__(self, scalar):
        return State(self.data * scalar)
    
    def __truediv__(self, scalar):
        return State(self.data / scalar)
    
    def copy(self):
        return State(self.data.copy())
    
    def norm(self):
        return np.linalg.norm(self.data)

class DroneNMPCController:
    """Nonlinear Model Predictive Controller for drone person tracking"""
    
    def __init__(self):
        self.config = nmpc_config
        
        # Current state
        self.current_state = State()
        self.target_position = np.array([0.0, 0.0, 2.0])
        self.target_attitude = np.zeros(3)
        
        # Person tracking state
        self.person_position = np.array([0.0, 0.0, 0.0])
        self.person_velocity = np.array([0.0, 0.0, 0.0])
        self.person_detected = False
        self.last_detection_time: float = 0.0
        self.tracking_height_offset = nmpc_config.TRACKING_HEIGHT_OFFSET
        self._last_horizontal_direction = np.array([1.0, 0.0], dtype=np.float64)
        self._last_target_attitude = None

        # Control history for warm start
        self.control_history = []
        for _ in range(self.config.N):
            self.control_history.append(self.config.get_hover_control())
        
        # Optimization state
        self.optimization_time = 0.0
        self.iterations_used = 0
        self.cost_value = 0.0

        # Internal smoothing state
        self._last_target_position = None
        self._planned_waypoints: list[np.ndarray] = []
        self._plan_sequence_id = 0
        self._last_plan_signature = None

        # Initialize solver parameters
        self._init_solver_parameters()
    
    def _init_solver_parameters(self):
        """Initialize solver parameters"""
        self.dt = self.config.TIMESTEP
        self.N = self.config.N
        
        # Gradient descent parameters
        self.alpha = self.config.STEP_SIZE
        self.max_iter = self.config.MAX_ITERATIONS
        self.tolerance = self.config.CONVERGENCE_TOLERANCE
        self.regularization = self.config.REGULARIZATION
    
    def set_drone_state(self, position: np.ndarray, velocity: np.ndarray, 
                       orientation: np.ndarray, angular_velocity: np.ndarray):
        """Set current drone state"""
        self.current_state[self.config.STATE_X:self.config.STATE_Z+1] = position
        self.current_state[self.config.STATE_VX:self.config.STATE_VZ+1] = velocity
        self.current_state[self.config.STATE_ROLL:self.config.STATE_YAW+1] = orientation
        self.current_state[self.config.STATE_WX:self.config.STATE_WZ+1] = angular_velocity
    
    def set_person_detection(
        self,
        position: np.ndarray,
        velocity: np.ndarray = None,
        detection_time: Optional[float] = None,
        allow_phase_change: bool = True,
    ):
        """Set detected person position and velocity"""
        self.person_position = position.copy()
        if velocity is not None:
            self.person_velocity = velocity.copy()
        else:
            self.person_velocity = np.array([0.0, 0.0, 0.0])

        self.person_detected = True
        current_time = detection_time if detection_time is not None else time.time()
        self.last_detection_time = current_time

        # Update tracking target using the most recent observation
        self.advance_tracking_target(
            current_time,
            allow_phase_change=allow_phase_change,
        )

    def advance_tracking_target(
        self,
        current_time: Optional[float] = None,
        *,
        allow_phase_change: bool = True,
    ):
        """Advance desired tracking target - now simply enforces fixed horizontal spacing."""
        if not self.person_detected:
            return
        # keep signature for compatibility with callers; allow_phase_change is unused
        _ = allow_phase_change
        self._update_tracking_target()

    def _update_tracking_target(self):
        """Update target position to maintain a fixed horizontal distance while facing the person."""
        if not self.person_detected:
            return

        desired_distance = self.config.DESIRED_TRACKING_DISTANCE_XY
        person_pos = self.person_position
        drone_pos = self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1]

        horizontal_delta = drone_pos[:2] - person_pos[:2]
        horizontal_distance = np.linalg.norm(horizontal_delta)

        if horizontal_distance > 1e-3:
            direction = horizontal_delta / horizontal_distance
            self._last_horizontal_direction = direction
        else:
            direction = self._last_horizontal_direction

        target_xy = person_pos[:2] + desired_distance * direction
        target_z = self.config.TRACKING_FIXED_ALTITUDE

        new_target = np.array([target_xy[0], target_xy[1], target_z])
        alpha = self.config.TARGET_POSITION_SMOOTHING
        if self._last_target_position is not None:
            new_target = alpha * new_target + (1.0 - alpha) * self._last_target_position
        else:
            alpha = 1.0
        self.target_position = new_target
        self._last_target_position = new_target.copy()

        to_person = person_pos - np.array([new_target[0], new_target[1], target_z])
        desired_yaw = math.atan2(to_person[1], to_person[0])
        self._desired_yaw = self._wrap_angle(desired_yaw)

        if not hasattr(self, '_last_debug_log_time'):
            self._last_debug_log_time = 0.0
        now = time.time()
        if now - self._last_debug_log_time > 1.0:
            print(f"[NMPC] desired_xy={desired_distance:.2f}m, "
                  f"current_xy={horizontal_distance:.2f}m, "
                  f"target_pos=[{new_target[0]:.2f}, {new_target[1]:.2f}, {target_z:.2f}]")
            self._last_debug_log_time = now

    def clear_detection(self):
        self.person_detected = False
        self._last_target_position = None
        self.person_velocity = np.zeros(3)
        self._last_target_attitude = None
        self._last_horizontal_direction = np.array([1.0, 0.0], dtype=np.float64)

    def set_tracking_height_offset(self, offset: float):
        self.tracking_height_offset = offset
    
    def set_fixed_altitude(self, altitude: float):
        self.config.TRACKING_FIXED_ALTITUDE = altitude
    
    def drone_dynamics(self, state: State, control: np.ndarray) -> State:
        """Quadrotor dynamics model"""
        # Extract state variables
        pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
        vel = state.data[self.config.STATE_VX:self.config.STATE_VZ+1]
        att = state.data[self.config.STATE_ROLL:self.config.STATE_YAW+1]
        omega = state.data[self.config.STATE_WX:self.config.STATE_WZ+1]
        
        # Extract control inputs
        thrust = control[0]
        roll_cmd = control[1]
        pitch_cmd = control[2]
        yaw_rate_cmd = control[3]
        
        # State derivative
        state_dot = State()
        
        # Position dynamics: dx/dt = v
        state_dot[self.config.STATE_X:self.config.STATE_Z+1] = vel
        
        # Velocity dynamics: dv/dt = a
        # Rotation matrix from body to world frame
        R = self._rotation_matrix(att[0], att[1], att[2])
        
        # Thrust vector in body frame
        thrust_body = np.array([0.0, 0.0, thrust])
        
        # Gravity in world frame
        gravity = np.array([0.0, 0.0, -self.config.GRAVITY * self.config.DRONE_MASS])
        
        # Total force in world frame
        force_world = R @ thrust_body + gravity
        
        # Acceleration = Force / Mass
        acc = force_world / self.config.DRONE_MASS
        
        state_dot[self.config.STATE_VX:self.config.STATE_VZ+1] = acc
        
        # Attitude dynamics (using proper rotational kinematics)
        # Convert body rates to Euler angle rates
        phi, theta, psi = att[0], att[1], att[2]
        
        # Avoid singularity at 90 degrees pitch
        cos_theta = math.cos(theta)
        tan_theta = math.tan(theta) if abs(cos_theta) > 1e-6 else 0.0
        
        p, q, r = omega[0], omega[1], omega[2]
        
        # Euler angle rates
        phi_dot = p + q * math.sin(phi) * tan_theta + r * math.cos(phi) * tan_theta
        theta_dot = q * math.cos(phi) - r * math.sin(phi)
        psi_dot = q * math.sin(phi) / cos_theta + r * math.cos(phi) / cos_theta
        
        state_dot[self.config.STATE_ROLL] = phi_dot
        state_dot[self.config.STATE_PITCH] = theta_dot
        state_dot[self.config.STATE_YAW] = psi_dot
        
        # Angular velocity dynamics (PD control for angular rates)
        # Simplified model with damping
        Ixx = self.config.DRONE_INERTIA_XX  # kg*m^2 (approximate moment of inertia)
        Iyy = self.config.DRONE_INERTIA_YY
        Izz = self.config.DRONE_INERTIA_ZZ
        
        # Control inputs to moments (simplified)
        tau_roll = 0.1 * (roll_cmd - att[0]) - 0.05 * omega[0]   # PD control
        tau_pitch = 0.1 * (pitch_cmd - att[1]) - 0.05 * omega[1]  # PD control
        tau_yaw = 0.1 * yaw_rate_cmd - 0.02 * omega[2]  # PD control for yaw rate
        
        # Euler's rotation equations
        omega_dot = np.array([
            (Iyy - Izz) / Ixx * omega[1] * omega[2] + tau_roll / Ixx,
            (Izz - Ixx) / Iyy * omega[0] * omega[2] + tau_pitch / Iyy,
            (Ixx - Iyy) / Izz * omega[0] * omega[1] + tau_yaw / Izz
        ])
        
        state_dot[self.config.STATE_WX:self.config.STATE_WZ+1] = omega_dot
        
        return state_dot
    
    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Calculate rotation matrix from Euler angles"""
        # Rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        # Combined rotation matrix
        return Rz @ Ry @ Rx
    
    def predict_trajectory(self, initial_state: State, controls: List[np.ndarray]) -> List[State]:
        """Predict future trajectory using Euler integration"""
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        
        for i in range(self.N):
            # Get control input
            control = controls[i] if i < len(controls) else self.config.get_hover_control()
            
            # Integrate dynamics using Euler method
            state_dot = self.drone_dynamics(current_state, control)
            current_state = current_state + state_dot * self.dt
            
            # Clip state to constraints
            current_state.data = self.config.clip_state(current_state.data)
            
            trajectory.append(current_state.copy())
        
        return trajectory
    
    def compute_stage_cost(self, state: State, control: np.ndarray, time_step: int) -> float:
        """Compute stage cost for a given state and control"""
        cost = 0.0
        
        # Position tracking cost
        pos_error = state.data[self.config.STATE_X:self.config.STATE_Z+1] - self.target_position
        cost += np.sum(self.config.W_POSITION * pos_error**2)

        # Velocity moderation (penalise aggressive motion)
        vel = state.data[self.config.STATE_VX:self.config.STATE_VZ+1]
        cost += np.sum(self.config.W_VELOCITY * vel**2)
        
        # Person tracking specific costs
        if self.person_detected:
            # Distance to person cost - with adaptive weight based on error magnitude
            drone_pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
            delta = drone_pos - self.person_position
            horizontal_distance = np.linalg.norm(delta[:2])
            distance_error = abs(horizontal_distance - self.config.DESIRED_TRACKING_DISTANCE_XY)

            # Adaptive weight: increase penalty when far from optimal distance
            # This helps prevent person from leaving camera view
            base_weight = self.config.W_TRACKING_DISTANCE
            if distance_error > 1.0:
                # Double the weight if more than 1m off
                adaptive_weight = base_weight * 2.0
            elif distance_error > 0.5:
                # 1.5x weight if more than 0.5m off
                adaptive_weight = base_weight * 1.5
            else:
                adaptive_weight = base_weight

            cost += adaptive_weight * distance_error**2

            # Yaw alignment: penalise deviation between predicted yaw and desired yaw
            desired_yaw = getattr(self, '_desired_yaw', None)
            if desired_yaw is not None:
                yaw = state.data[self.config.STATE_YAW]
                yaw_error = self._wrap_angle(yaw - desired_yaw)
                cost += self.config.W_YAW_ALIGNMENT * yaw_error * yaw_error

            # Smooth tracking cost - penalize large changes in target position
            # But reduce weight when distance error is large (prioritize catching up)
            if self._last_target_position is not None:
                target_change = np.linalg.norm(self.target_position - self._last_target_position)
                smooth_weight = self.config.W_SMOOTH_TRACKING
                if distance_error > 0.8:
                    # Reduce smoothing penalty when we need to catch up
                    smooth_weight *= 0.5
                cost += smooth_weight * target_change**2
        
        return cost
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def compute_total_cost(self, trajectory: List[State], controls: List[np.ndarray]) -> float:
        """Compute total cost over trajectory"""
        total_cost = 0.0
        
        for i in range(len(controls)):
            stage_cost = self.compute_stage_cost(trajectory[i], controls[i], i)
            total_cost += stage_cost

            if i + 1 < len(trajectory):
                vel_curr = trajectory[i].data[self.config.STATE_VX:self.config.STATE_VZ+1]
                vel_next = trajectory[i + 1].data[self.config.STATE_VX:self.config.STATE_VZ+1]
                accel = (vel_next - vel_curr) / self.dt
                total_cost += np.sum(self.config.W_ACCELERATION * accel**2)

                # Camera tilt penalty - keep person steady in frame
                # Lateral acceleration causes pitch/roll which moves person up/down in camera view
                lateral_acc = np.linalg.norm(accel[:2])  # XY acceleration magnitude
                tilt = math.atan2(lateral_acc, self.config.GRAVITY)  # Resulting tilt angle
                total_cost += self.config.W_CAMERA_TILT * tilt**2
        
        return total_cost
    
    def compute_gradient(self, controls: List[np.ndarray]) -> List[np.ndarray]:
        """Compute gradient of cost function with respect to controls"""
        gradient = []
        
        # Predict current trajectory
        trajectory = self.predict_trajectory(self.current_state, controls)
        current_cost = self.compute_total_cost(trajectory, controls)
        
        # Compute gradient using finite differences
        for i in range(len(controls)):
            control_grad = np.zeros_like(controls[i])
            
            for j in range(len(controls[i])):
                # Perturb control
                perturbed_controls = [ctrl.copy() for ctrl in controls]
                perturbed_controls[i][j] += self.regularization
                
                # Predict perturbed trajectory
                perturbed_trajectory = self.predict_trajectory(self.current_state, perturbed_controls)
                perturbed_cost = self.compute_total_cost(perturbed_trajectory, perturbed_controls)
                
                # Compute gradient component
                control_grad[j] = (perturbed_cost - current_cost) / self.regularization
            
            gradient.append(control_grad)
        
        return gradient
    
    def optimize(self) -> Tuple[np.ndarray, dict]:
        """Optimize control inputs using gradient descent"""
        start_time = time.time()
        
        # Initialize controls (use previous solution as warm start)
        controls = [ctrl.copy() for ctrl in self.control_history]
        
        # Gradient descent optimization
        for iteration in range(self.max_iter):
            # Compute gradient
            gradient = self.compute_gradient(controls)
            
            # Update controls
            for i in range(len(controls)):
                controls[i] -= self.alpha * gradient[i]
                controls[i] = self.config.clip_control(controls[i])
            
            # Check for convergence
            grad_norm = np.linalg.norm(np.concatenate(gradient))
            if grad_norm < self.tolerance:
                break
        
        # Store solution for warm start
        self.control_history = controls
        
        # Extract first control input as optimal control
        optimal_control = controls[0] if controls else self.config.get_hover_control()
        
        # Predict trajectory for visualization
        trajectory = self.predict_trajectory(self.current_state, controls)
        
        # Store optimization info
        self.optimization_time = time.time() - start_time
        self.iterations_used = iteration + 1
        self.cost_value = self.compute_total_cost(trajectory, controls)
        
        info = {
            'trajectory': [s.data for s in trajectory],
            'iterations': self.iterations_used,
            'optimization_time': self.optimization_time,
            'cost': self.cost_value
        }

        self._update_planned_waypoints_from_trajectory(trajectory)

        # Extract desired attitude from predicted trajectory
        if len(trajectory) > 1:
            desired_attitude = np.array(trajectory[1].data[self.config.STATE_ROLL:self.config.STATE_YAW+1])
        else:
            desired_attitude = np.array(trajectory[0].data[self.config.STATE_ROLL:self.config.STATE_YAW+1])

        desired_attitude[2] = self._wrap_angle(desired_attitude[2])
        smoothing = float(np.clip(self.config.TARGET_ATTITUDE_SMOOTHING, 0.0, 1.0))
        if self._last_target_attitude is not None:
            desired_attitude[:2] = smoothing * self._last_target_attitude[:2] + (1.0 - smoothing) * desired_attitude[:2]

        self.target_attitude = desired_attitude
        self._last_target_attitude = desired_attitude.copy()
        
        return optimal_control, info

    def get_planned_waypoints(self) -> List[np.ndarray]:
        return [wp.copy() for wp in self._planned_waypoints]

    def get_plan_sequence_id(self) -> int:
        return self._plan_sequence_id

    def _update_planned_waypoints_from_trajectory(self, trajectory: List[State]) -> None:
        if len(trajectory) < 2:
            return
        waypoints: List[np.ndarray] = []
        last_point = None
        for state in trajectory[1:]:
            waypoint = np.array(state.data[self.config.STATE_X:self.config.STATE_Z+1])
            if last_point is not None:
                if np.linalg.norm(waypoint - last_point) < self.config.MIN_WAYPOINT_SPACING:
                    continue
            waypoints.append(waypoint)
            last_point = waypoint

        if not waypoints:
            return

        signature = (
            tuple(np.round(waypoints[0], 3)),
            tuple(np.round(waypoints[-1], 3)),
            len(waypoints),
        )

        changed = (
            self._last_plan_signature is None or
            np.linalg.norm(np.array(signature[0]) - np.array(self._last_plan_signature[0])) > self.config.WAYPOINT_PLAN_CHANGE_THRESHOLD or
            np.linalg.norm(np.array(signature[1]) - np.array(self._last_plan_signature[1])) > self.config.WAYPOINT_PLAN_CHANGE_THRESHOLD or
            signature[2] != self._last_plan_signature[2]
        )

        self._planned_waypoints = waypoints
        if changed:
            self._plan_sequence_id += 1
            self._last_plan_signature = signature
    
    def get_status(self) -> dict:
        """Get controller status"""
        if self.person_detected:
            delta = self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1] - self.person_position
            horizontal_distance = float(np.linalg.norm(delta[:2]))
        else:
            horizontal_distance = 0.0

        status = {
            'person_detected': self.person_detected,
            'current_tracking_distance': horizontal_distance,
            'tracking_distance': horizontal_distance,  # Legacy alias for downstream nodes
            'desired_tracking_distance': self.config.DESIRED_TRACKING_DISTANCE_XY,
            'optimization_time': self.optimization_time,
            'iterations_used': float(self.iterations_used),
            'cost_value': self.cost_value
        }
        
        return status
