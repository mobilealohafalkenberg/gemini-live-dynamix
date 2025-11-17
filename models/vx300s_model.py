#!/usr/bin/env python3
"""
VX300S Robot Kinematics Model
Product of Exponentials (PoE) representation for Modern Robotics library
"""

import numpy as np


class VX300S:
    """
    VX300S robot arm kinematics using Product of Exponentials formulation.

    This model defines:
    - Slist: Screw axes in space frame (6x6 matrix)
    - M: Home configuration matrix (4x4 transformation)
    - Joint limits for all 6 joints
    - Workspace bounds
    """

    # =========================================================================
    # Product of Exponentials Parameters
    # =========================================================================

    # Screw axes in space frame
    # Each column represents a joint's screw axis: [ω; v] where:
    #   ω = angular velocity (3x1)
    #   v = linear velocity or -ω × q (3x1) where q is a point on the axis
    Slist = np.array([
        [0.0, 0.0, 1.0,  0.0,     0.0,     0.0],      # Joint 1: Waist (revolute about Z)
        [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],      # Joint 2: Shoulder (revolute about Y)
        [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],  # Joint 3: Elbow (revolute about Y)
        [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],      # Joint 4: Forearm roll (revolute about X)
        [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],  # Joint 5: Wrist angle (revolute about Y)
        [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]       # Joint 6: Wrist rotate (revolute about X)
    ]).T  # Transpose to get 6x6 matrix

    # Home configuration (end-effector pose when all joints are at 0 radians)
    # This is a 4x4 homogeneous transformation matrix
    M = np.array([
        [1.0, 0.0, 0.0, 0.536494],  # End-effector X position at home
        [0.0, 1.0, 0.0, 0.0],       # End-effector Y position at home
        [0.0, 0.0, 1.0, 0.42705],   # End-effector Z position at home
        [0.0, 0.0, 0.0, 1.0]        # Homogeneous coordinate
    ])

    # =========================================================================
    # Joint Limits (radians)
    # =========================================================================

    joint_limits = [
        (-np.pi, np.pi),           # Joint 1: Waist (full rotation)
        (-1.97, 1.75),             # Joint 2: Shoulder
        (-1.52, 1.80),             # Joint 3: Elbow
        (-np.pi, np.pi),           # Joint 4: Forearm roll (full rotation)
        (-1.74, 2.23),             # Joint 5: Wrist angle
        (-np.pi, np.pi)            # Joint 6: Wrist rotate (full rotation)
    ]

    # =========================================================================
    # Workspace Limits (meters)
    # =========================================================================

    workspace_limits = {
        'x': (0.15, 0.50),   # Forward reach
        'y': (-0.30, 0.30),  # Left-right reach
        'z': (0.05, 0.40)    # Vertical reach (above table)
    }

    # =========================================================================
    # Physical Parameters
    # =========================================================================

    # Link lengths (meters) - for reference/visualization
    link_lengths = {
        'base_height': 0.12705,     # Base to shoulder pivot
        'shoulder_to_elbow': 0.30,  # Upper arm length
        'elbow_to_wrist': 0.30,     # Forearm length
        'wrist_to_ee': 0.10649      # Wrist to end-effector
    }

    # =========================================================================
    # Motor Configuration
    # =========================================================================

    # Dynamixel position range (12-bit resolution)
    dynamixel_range = (0, 4095)
    dynamixel_center = 2048

    # Conversion factor: Dynamixel units per radian
    # For Dynamixel X-series: 4096 steps = 360 degrees = 2π radians
    units_per_radian = 4096 / (2 * np.pi)  # ~651.74 units/radian

    @classmethod
    def radians_to_dynamixel(cls, radians: float, center: int = 2048) -> int:
        """
        Convert joint angle in radians to Dynamixel position units.

        Args:
            radians: Joint angle in radians
            center: Center position in Dynamixel units (default 2048)

        Returns:
            Dynamixel position (0-4095)
        """
        units = center + int(radians * cls.units_per_radian)
        return max(0, min(4095, units))

    @classmethod
    def dynamixel_to_radians(cls, position: int, center: int = 2048) -> float:
        """
        Convert Dynamixel position units to radians.

        Args:
            position: Dynamixel position (0-4095)
            center: Center position in Dynamixel units (default 2048)

        Returns:
            Joint angle in radians
        """
        return (position - center) / cls.units_per_radian

    @classmethod
    def validate_joint_angles(cls, joint_angles: np.ndarray) -> tuple[bool, str]:
        """
        Validate if joint angles are within limits.

        Args:
            joint_angles: Array of 6 joint angles in radians

        Returns:
            Tuple of (is_valid, error_message)
        """
        if len(joint_angles) != 6:
            return False, f"Expected 6 joint angles, got {len(joint_angles)}"

        for i, (angle, (min_limit, max_limit)) in enumerate(zip(joint_angles, cls.joint_limits)):
            if angle < min_limit or angle > max_limit:
                return False, (f"Joint {i+1} angle {np.degrees(angle):.1f}° "
                             f"outside limits [{np.degrees(min_limit):.1f}°, "
                             f"{np.degrees(max_limit):.1f}°]")

        return True, ""

    @classmethod
    def validate_workspace_position(cls, x: float, y: float, z: float) -> tuple[bool, str]:
        """
        Validate if end-effector position is within workspace.

        Args:
            x, y, z: End-effector position in meters

        Returns:
            Tuple of (is_valid, error_message)
        """
        limits = cls.workspace_limits

        if x < limits['x'][0] or x > limits['x'][1]:
            return False, f"X={x:.3f}m outside workspace [{limits['x'][0]}, {limits['x'][1]}]"

        if y < limits['y'][0] or y > limits['y'][1]:
            return False, f"Y={y:.3f}m outside workspace [{limits['y'][0]}, {limits['y'][1]}]"

        if z < limits['z'][0] or z > limits['z'][1]:
            return False, f"Z={z:.3f}m outside workspace [{limits['z'][0]}, {limits['z'][1]}]"

        return True, ""

    @classmethod
    def get_info(cls) -> dict:
        """
        Get robot model information.

        Returns:
            Dictionary with robot specifications
        """
        return {
            'name': 'VX300S',
            'manufacturer': 'Trossen Robotics',
            'dof': 6,
            'joint_limits_deg': [[np.degrees(l[0]), np.degrees(l[1])]
                                 for l in cls.joint_limits],
            'workspace_limits': cls.workspace_limits,
            'link_lengths': cls.link_lengths,
            'total_reach': sum(cls.link_lengths.values()),
            'dynamixel_motors': {
                '1-2-6-7-8': 'XM430-W350',
                '3-4-5': 'XM540-W270 (with shadow motors)'
            }
        }


if __name__ == "__main__":
    # Test and display robot model information
    print("=" * 80)
    print("VX300S Robot Model")
    print("=" * 80)

    robot = VX300S()
    info = robot.get_info()

    print(f"\nRobot: {info['name']} ({info['manufacturer']})")
    print(f"Degrees of Freedom: {info['dof']}")

    print("\nJoint Limits (degrees):")
    joint_names = ['Waist', 'Shoulder', 'Elbow', 'Forearm Roll', 'Wrist Angle', 'Wrist Rotate']
    for i, (name, limits) in enumerate(zip(joint_names, info['joint_limits_deg'])):
        print(f"  Joint {i+1} ({name}): [{limits[0]:.1f}°, {limits[1]:.1f}°]")

    print("\nWorkspace Limits (meters):")
    for axis, limits in info['workspace_limits'].items():
        print(f"  {axis.upper()}: [{limits[0]:.3f}, {limits[1]:.3f}]")

    print(f"\nTotal Reach: ~{info['total_reach']:.3f}m")

    print("\n" + "=" * 80)
    print("Screw Axes Matrix (Slist):")
    print("=" * 80)
    print(robot.Slist)

    print("\n" + "=" * 80)
    print("Home Configuration Matrix (M):")
    print("=" * 80)
    print(robot.M)

    print("\n" + "=" * 80)
    print("Conversion Tests:")
    print("=" * 80)

    # Test conversions
    test_angles = [0, np.pi/4, -np.pi/4, np.pi/2]
    for angle in test_angles:
        dxl_pos = robot.radians_to_dynamixel(angle)
        back = robot.dynamixel_to_radians(dxl_pos)
        print(f"  {np.degrees(angle):6.1f}° → {dxl_pos:4d} units → {np.degrees(back):6.1f}°")

    # Test validation
    print("\n" + "=" * 80)
    print("Validation Tests:")
    print("=" * 80)

    test_joints = np.array([0.0, -0.3, 0.6, 0.0, -0.3, 0.0])
    valid, msg = robot.validate_joint_angles(test_joints)
    print(f"  Home pose valid: {valid}")

    valid, msg = robot.validate_workspace_position(0.3, 0.0, 0.2)
    print(f"  Position (0.3, 0.0, 0.2) valid: {valid}")

    valid, msg = robot.validate_workspace_position(1.0, 0.0, 0.2)
    print(f"  Position (1.0, 0.0, 0.2) valid: {valid}")
    if not valid:
        print(f"    Error: {msg}")

    print("\n✓ Model loaded successfully")
