#!/usr/bin/env python3
"""
Drone NMPC Tracker Package
Nonlinear Model Predictive Control for drone person tracking

This package provides:
- NMPC controller for quadrotor dynamics
- ROS2 node for person tracking
- Configuration management
- Test utilities

Compatible with:
- ROS2 Jazzy
- Python 3.12
- Ubuntu 24.04
"""

__version__ = "1.0.0"
__author__ = "Yang Zuo"
__email__ = "zuoyang0601@gmail.com"

# Package imports
from .config import nmpc_config, NMPCConfig
from .nmpc_controller import DroneNMPCController, State
from .nmpc_node import NMPCTrackerNode

__all__ = [
    'nmpc_config',
    'NMPCConfig', 
    'DroneNMPCController',
    'State',
    'NMPCTrackerNode'
]

