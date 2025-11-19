#!/usr/bin/env python3
"""
Safety Validator for Mobile ALOHA Robot
Provides workspace bounds checking, velocity validation, and risk assessment
"""

import os
import math
import json
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
from dataclasses import dataclass, asdict

class RiskLevel(Enum):
    """Risk assessment levels for robot movements"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    NEEDS_CONFIRMATION = "needs_confirmation"
    BLOCKED = "blocked"

@dataclass
class WorkspaceBounds:
    """Workspace boundary configuration for the robot

    NOTE: These bounds MUST match the WORKSPACE constants in arm_controller.py
    and the VX300S model to ensure consistent safety validation across the system.
    """
    x_min: float = 0.15   # meters - forward reach only, cannot reach behind base
    x_max: float = 0.50   # meters - maximum forward reach
    y_min: float = -0.30  # meters - left reach
    y_max: float = 0.30   # meters - right reach
    z_min: float = 0.05   # meters - minimum height above table
    z_max: float = 0.40   # meters - maximum vertical reach
    
    @classmethod
    def from_env(cls):
        """Load bounds from environment variables if available"""
        bounds = cls()
        
        # Override with environment variables if set
        if os.getenv('BOUNDS_X'):
            x_bounds = os.getenv('BOUNDS_X').split(',')
            bounds.x_min, bounds.x_max = float(x_bounds[0]), float(x_bounds[1])
        if os.getenv('BOUNDS_Y'):
            y_bounds = os.getenv('BOUNDS_Y').split(',')
            bounds.y_min, bounds.y_max = float(y_bounds[0]), float(y_bounds[1])
        if os.getenv('BOUNDS_Z'):
            z_bounds = os.getenv('BOUNDS_Z').split(',')
            bounds.z_min, bounds.z_max = float(z_bounds[0]), float(z_bounds[1])
            
        return bounds
    
    def to_dict(self):
        return asdict(self)

@dataclass
class SafetyConfig:
    """Safety configuration parameters"""
    max_linear_velocity: float = 0.25   # m/s
    max_angular_velocity: float = 0.8   # rad/s
    max_linear_acceleration: float = 0.5  # m/s^2
    min_moving_time: float = 0.5  # seconds
    max_moving_time: float = 10.0  # seconds
    boundary_margin: float = 0.02  # meters - margin from boundaries
    
    @classmethod
    def from_env(cls):
        """Load config from environment variables if available"""
        config = cls()
        
        if os.getenv('SPEED_CAP_LIN'):
            config.max_linear_velocity = float(os.getenv('SPEED_CAP_LIN'))
        if os.getenv('SPEED_CAP_ANG'):
            config.max_angular_velocity = float(os.getenv('SPEED_CAP_ANG'))
            
        return config
    
    def to_dict(self):
        return asdict(self)

@dataclass
class ValidationResult:
    """Result of safety validation"""
    valid: bool
    risk_level: RiskLevel
    message: str
    details: Dict[str, Any]
    
    def to_dict(self):
        return {
            "valid": self.valid,
            "risk_level": self.risk_level.value,
            "message": self.message,
            "details": self.details
        }

class SafetyValidator:
    """
    Safety validator for robot movements
    Checks workspace bounds, velocities, and assesses risk
    """
    
    def __init__(self, 
                 bounds: Optional[WorkspaceBounds] = None,
                 config: Optional[SafetyConfig] = None,
                 safety_profile: str = "strict"):
        """
        Initialize safety validator
        
        Args:
            bounds: Workspace bounds configuration
            config: Safety configuration
            safety_profile: "strict", "soft", or "custom"
        """
        self.bounds = bounds or WorkspaceBounds.from_env()
        self.config = config or SafetyConfig.from_env()
        self.safety_profile = os.getenv('SAFETY_PROFILE', safety_profile)
        
        # Adjust parameters based on safety profile
        if self.safety_profile == "soft":
            self.config.boundary_margin = 0.01
            self.config.max_linear_velocity = 0.3
        elif self.safety_profile == "strict":
            self.config.boundary_margin = 0.03
            self.config.max_linear_velocity = 0.2
    
    def validate_position(self, x: float, y: float, z: float) -> ValidationResult:
        """
        Validate a single position against workspace bounds
        
        Args:
            x, y, z: Position in meters
            
        Returns:
            ValidationResult with validity and risk assessment
        """
        details = {
            "position": {"x": x, "y": y, "z": z},
            "bounds": self.bounds.to_dict()
        }
        
        # Check hard bounds
        if x < self.bounds.x_min or x > self.bounds.x_max:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"X position {x:.3f}m outside bounds [{self.bounds.x_min:.3f}, {self.bounds.x_max:.3f}]",
                details=details
            )
        
        if y < self.bounds.y_min or y > self.bounds.y_max:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"Y position {y:.3f}m outside bounds [{self.bounds.y_min:.3f}, {self.bounds.y_max:.3f}]",
                details=details
            )
        
        if z < self.bounds.z_min or z > self.bounds.z_max:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"Z position {z:.3f}m outside bounds [{self.bounds.z_min:.3f}, {self.bounds.z_max:.3f}]",
                details=details
            )
        
        # Check proximity to boundaries
        margin = self.config.boundary_margin
        near_boundary = False
        warnings = []
        
        if x < self.bounds.x_min + margin:
            warnings.append(f"X near minimum bound ({x:.3f}m)")
            near_boundary = True
        elif x > self.bounds.x_max - margin:
            warnings.append(f"X near maximum bound ({x:.3f}m)")
            near_boundary = True
            
        if y < self.bounds.y_min + margin:
            warnings.append(f"Y near minimum bound ({y:.3f}m)")
            near_boundary = True
        elif y > self.bounds.y_max - margin:
            warnings.append(f"Y near maximum bound ({y:.3f}m)")
            near_boundary = True
            
        if z < self.bounds.z_min + margin:
            warnings.append(f"Z near minimum bound ({z:.3f}m)")
            near_boundary = True
        elif z > self.bounds.z_max - margin:
            warnings.append(f"Z near maximum bound ({z:.3f}m)")
            near_boundary = True
        
        if near_boundary:
            details["warnings"] = warnings
            return ValidationResult(
                valid=True,
                risk_level=RiskLevel.NEEDS_CONFIRMATION,
                message=f"Position near workspace boundary: {', '.join(warnings)}",
                details=details
            )
        
        return ValidationResult(
            valid=True,
            risk_level=RiskLevel.LOW,
            message="Position within safe workspace bounds",
            details=details
        )
    
    def validate_trajectory(self, 
                          start_pos: List[float], 
                          end_pos: List[float],
                          moving_time: float,
                          waypoints: Optional[List[List[float]]] = None) -> ValidationResult:
        """
        Validate a trajectory for safety
        
        Args:
            start_pos: Starting position [x, y, z]
            end_pos: End position [x, y, z]
            moving_time: Time to complete movement
            waypoints: Optional intermediate waypoints
            
        Returns:
            ValidationResult with trajectory analysis
        """
        # Validate individual positions
        positions_to_check = [start_pos, end_pos]
        if waypoints:
            positions_to_check.extend(waypoints)
        
        for i, pos in enumerate(positions_to_check):
            result = self.validate_position(pos[0], pos[1], pos[2])
            if not result.valid:
                return ValidationResult(
                    valid=False,
                    risk_level=result.risk_level,
                    message=f"Waypoint {i} invalid: {result.message}",
                    details={"waypoint_index": i, "position": pos, "error": result.message}
                )
        
        # Calculate distance and velocity
        distance = math.sqrt(
            (end_pos[0] - start_pos[0])**2 +
            (end_pos[1] - start_pos[1])**2 +
            (end_pos[2] - start_pos[2])**2
        )
        
        velocity = distance / moving_time if moving_time > 0 else float('inf')
        
        details = {
            "start": start_pos,
            "end": end_pos,
            "distance": distance,
            "moving_time": moving_time,
            "velocity": velocity,
            "max_velocity": self.config.max_linear_velocity
        }
        
        # Check velocity limits
        if velocity > self.config.max_linear_velocity:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"Velocity {velocity:.3f}m/s exceeds limit {self.config.max_linear_velocity:.3f}m/s",
                details=details
            )
        
        # Check time constraints
        if moving_time < self.config.min_moving_time:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"Moving time {moving_time:.2f}s below minimum {self.config.min_moving_time:.2f}s",
                details=details
            )
        
        if moving_time > self.config.max_moving_time:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"Moving time {moving_time:.2f}s exceeds maximum {self.config.max_moving_time:.2f}s",
                details=details
            )
        
        # Assess overall risk
        if velocity > self.config.max_linear_velocity * 0.8:
            return ValidationResult(
                valid=True,
                risk_level=RiskLevel.MEDIUM,
                message=f"High velocity movement ({velocity:.3f}m/s)",
                details=details
            )
        
        if distance > 0.2:  # Large movement
            return ValidationResult(
                valid=True,
                risk_level=RiskLevel.MEDIUM,
                message=f"Large movement distance ({distance:.3f}m)",
                details=details
            )
        
        return ValidationResult(
            valid=True,
            risk_level=RiskLevel.LOW,
            message="Trajectory within safe parameters",
            details=details
        )
    
    def validate_joint_angles(self, angles: List[float], unit: str = "radians") -> ValidationResult:
        """
        Validate joint angles (basic range checking)
        
        Args:
            angles: List of 6 joint angles
            unit: "radians" or "degrees"
            
        Returns:
            ValidationResult
        """
        if len(angles) != 6:
            return ValidationResult(
                valid=False,
                risk_level=RiskLevel.BLOCKED,
                message=f"Expected 6 joint angles, got {len(angles)}",
                details={"angles": angles}
            )
        
        # Convert to radians if needed
        if unit == "degrees":
            angles = [math.radians(a) for a in angles]
        
        # Basic joint limits for ViperX 300s (approximate)
        joint_limits = [
            (-3.14, 3.14),    # Waist
            (-1.85, 1.85),    # Shoulder
            (-1.85, 1.85),    # Elbow
            (-3.14, 3.14),    # Wrist angle
            (-1.85, 1.85),    # Wrist rotate
            (-3.14, 3.14),    # Gripper rotate
        ]
        
        for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, joint_limits)):
            if angle < min_limit or angle > max_limit:
                return ValidationResult(
                    valid=False,
                    risk_level=RiskLevel.BLOCKED,
                    message=f"Joint {i} angle {angle:.3f}rad outside limits [{min_limit:.3f}, {max_limit:.3f}]",
                    details={"joint": i, "angle": angle, "limits": (min_limit, max_limit)}
                )
        
        return ValidationResult(
            valid=True,
            risk_level=RiskLevel.LOW,
            message="Joint angles within limits",
            details={"angles": angles}
        )
    
    def get_status(self) -> Dict:
        """Get current validator status and configuration"""
        return {
            "safety_profile": self.safety_profile,
            "workspace_bounds": self.bounds.to_dict(),
            "safety_config": self.config.to_dict(),
            "operational": True
        }


# Convenience functions
def create_validator(profile: str = "strict") -> SafetyValidator:
    """Create a safety validator with the specified profile"""
    return SafetyValidator(safety_profile=profile)

def validate_position(x: float, y: float, z: float, 
                     validator: Optional[SafetyValidator] = None) -> ValidationResult:
    """Quick position validation"""
    if validator is None:
        validator = SafetyValidator()
    return validator.validate_position(x, y, z)

def validate_trajectory(start: List[float], end: List[float], time: float,
                       validator: Optional[SafetyValidator] = None) -> ValidationResult:
    """Quick trajectory validation"""
    if validator is None:
        validator = SafetyValidator()
    return validator.validate_trajectory(start, end, time)


if __name__ == "__main__":
    # Test the validator
    validator = SafetyValidator(safety_profile="strict")
    
    print("Safety Validator Test")
    print("=" * 50)
    print(f"Configuration: {json.dumps(validator.get_status(), indent=2)}")
    print()
    
    # Test some positions
    test_positions = [
        (0.25, 0.0, 0.20, "Center position"),
        (0.10, 0.0, 0.12, "Minimum x and z"),
        (0.35, 0.0, 0.40, "Maximum x and z"),
        (0.40, 0.0, 0.20, "X too far"),
        (0.25, 0.0, 0.10, "Z too low"),
        (0.25, 0.0, 0.50, "Z too high"),
    ]
    
    print("Position Tests:")
    for x, y, z, label in test_positions:
        result = validator.validate_position(x, y, z)
        status = "✓" if result.valid else "✗"
        print(f"{status} {label}: ({x:.2f}, {y:.2f}, {z:.2f}) - {result.risk_level.value}: {result.message}")
    
    print()
    print("Trajectory Tests:")
    
    # Test trajectories
    result = validator.validate_trajectory([0.25, 0, 0.20], [0.30, 0, 0.25], 2.0)
    print(f"Small movement (2s): {result.risk_level.value} - {result.message}")
    
    result = validator.validate_trajectory([0.15, 0, 0.20], [0.35, 0, 0.35], 1.0)
    print(f"Large fast movement (1s): {result.risk_level.value} - {result.message}")
    
    result = validator.validate_trajectory([0.25, 0, 0.20], [0.30, 0, 0.45], 3.0)
    print(f"Movement to high position: {result.risk_level.value} - {result.message}")