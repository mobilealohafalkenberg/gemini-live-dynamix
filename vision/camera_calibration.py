#!/usr/bin/env python3
"""
Camera Calibration - Transform points from camera frame to robot base frame

This module manages transformation matrices that convert 3D points from
camera coordinate frames to the robot's base coordinate frame.

CRITICAL: The default transformations are ESTIMATES and must be calibrated
using AprilTags or manual measurement for accurate manipulation.
"""

import numpy as np
import logging
import yaml
import os
from typing import Dict, Optional, Tuple
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


class CameraCalibration:
    """
    Manages camera-to-base frame transformations for ALOHA robot.

    Coordinate Frames:
    - Camera Frame: +X right, +Y down, +Z forward (RealSense standard)
    - Robot Base Frame: +X forward, +Y left, +Z up (robotics standard)

    Transformation matrices are 4x4 homogeneous transforms:
    T = [R | t]
        [0 | 1]
    where R is 3x3 rotation, t is 3x1 translation
    """

    # Default camera names
    CAMERA_NAMES = ['top_cam', 'gripper_cam']

    def __init__(self, calibration_file: Optional[str] = None):
        """
        Initialize camera calibration.

        Args:
            calibration_file: Path to YAML file with calibration data
                            If None, uses default (uncalibrated) transforms
        """
        self.transforms = {}
        self.calibration_file = calibration_file

        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
            logger.info(f"[CameraCalibration] Loaded calibration from {calibration_file}")
        else:
            self._initialize_default_transforms()
            logger.warning("[CameraCalibration] Using DEFAULT (uncalibrated) transforms! "
                          "Calibrate cameras for accurate positioning.")

    def _initialize_default_transforms(self):
        """
        Initialize default transformation matrices.

        WARNING: These are ESTIMATES based on typical ALOHA setup.
        Actual values depend on:
        - Camera mounting positions
        - Camera orientations
        - Robot base position

        MUST be calibrated for real-world use!
        """

        # Top camera (overhead view)
        # Assumed to be:
        # - 60cm above robot base (z = 0.60)
        # - Centered above workspace (x = 0.30, y = 0.00)
        # - Pointing down (180° rotation around X-axis)
        self.transforms['top_cam'] = self._create_transform(
            translation=[0.30, 0.00, 0.60],  # [x, y, z] in meters
            rotation_euler=[np.pi, 0, 0],     # [roll, pitch, yaw] in radians
            description="Top overhead camera (UNCALIBRATED - ESTIMATE ONLY)"
        )

        # Gripper camera (wrist-mounted)
        # Assumed to be:
        # - Mounted on end effector, looking forward
        # - Offset from gripper center: 5cm forward, 0cm lateral, 3cm up
        # - Rotated to align with gripper orientation
        # NOTE: This transform is relative to END EFFECTOR, not base!
        # For base frame transform, must combine with FK from current joint angles
        self.transforms['gripper_cam'] = self._create_transform(
            translation=[0.05, 0.00, 0.03],  # Relative to end effector
            rotation_euler=[0, np.pi/2, 0],  # Camera pointing along gripper axis
            description="Gripper wrist camera (UNCALIBRATED - ESTIMATE ONLY)"
        )

        logger.warning("[CameraCalibration] Initialized with DEFAULT transforms")
        logger.warning("  Top cam: [0.30, 0.00, 0.60]m above base")
        logger.warning("  Gripper cam: Relative to end effector (requires FK)")

    def _create_transform(
        self,
        translation: list,
        rotation_euler: list,
        description: str = ""
    ) -> Dict:
        """
        Create a transformation dictionary.

        Args:
            translation: [x, y, z] in meters
            rotation_euler: [roll, pitch, yaw] in radians
            description: Human-readable description

        Returns:
            Dictionary with transformation matrix and metadata
        """
        # Create rotation matrix from Euler angles (XYZ convention)
        R = Rotation.from_euler('xyz', rotation_euler).as_matrix()

        # Create homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation

        return {
            'matrix': T,
            'translation': np.array(translation),
            'rotation_euler': np.array(rotation_euler),
            'rotation_matrix': R,
            'description': description,
            'calibrated': False  # Mark as uncalibrated by default
        }

    def transform_point(
        self,
        point_camera: np.ndarray,
        camera_name: str,
        end_effector_pose: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Transform a 3D point from camera frame to robot base frame.

        Args:
            point_camera: 3D point in camera frame [x, y, z]
            camera_name: Name of camera ('top_cam' or 'gripper_cam')
            end_effector_pose: 4x4 transform of end effector (required for gripper_cam)

        Returns:
            3D point in robot base frame [x, y, z]
        """
        if camera_name not in self.transforms:
            raise ValueError(f"Unknown camera: {camera_name}. "
                           f"Available: {list(self.transforms.keys())}")

        # Convert point to homogeneous coordinates
        point_homogeneous = np.append(point_camera, 1.0)

        # Get camera-to-base transform
        if camera_name == 'gripper_cam' and end_effector_pose is not None:
            # For gripper camera: Transform = Base_T_EE * EE_T_Camera
            T_cam_to_base = end_effector_pose @ self.transforms[camera_name]['matrix']
        else:
            # For fixed cameras (top_cam): Direct transform
            T_cam_to_base = self.transforms[camera_name]['matrix']

        # Apply transformation
        point_base = T_cam_to_base @ point_homogeneous

        return point_base[:3]

    def transform_points(
        self,
        points_camera: np.ndarray,
        camera_name: str,
        end_effector_pose: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Transform multiple 3D points from camera frame to base frame.

        Args:
            points_camera: Array of shape (N, 3) with points in camera frame
            camera_name: Name of camera
            end_effector_pose: 4x4 transform of end effector (for gripper_cam)

        Returns:
            Array of shape (N, 3) with points in base frame
        """
        points_base = []

        for point in points_camera:
            point_base = self.transform_point(point, camera_name, end_effector_pose)
            points_base.append(point_base)

        return np.array(points_base)

    def get_transform_matrix(self, camera_name: str) -> np.ndarray:
        """Get the 4x4 transformation matrix for a camera."""
        if camera_name not in self.transforms:
            raise ValueError(f"Unknown camera: {camera_name}")

        return self.transforms[camera_name]['matrix']

    def is_calibrated(self, camera_name: str) -> bool:
        """Check if a camera has been properly calibrated."""
        if camera_name not in self.transforms:
            return False

        return self.transforms[camera_name].get('calibrated', False)

    def set_transform(
        self,
        camera_name: str,
        translation: np.ndarray,
        rotation_matrix: np.ndarray,
        calibrated: bool = True
    ):
        """
        Manually set transformation for a camera.

        Args:
            camera_name: Name of camera
            translation: Translation vector [x, y, z]
            rotation_matrix: 3x3 rotation matrix
            calibrated: Whether this is a calibrated (accurate) transform
        """
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation

        rotation_euler = Rotation.from_matrix(rotation_matrix).as_euler('xyz')

        self.transforms[camera_name] = {
            'matrix': T,
            'translation': translation,
            'rotation_euler': rotation_euler,
            'rotation_matrix': rotation_matrix,
            'calibrated': calibrated,
            'description': f"{camera_name} - {'Calibrated' if calibrated else 'Manual'}"
        }

        logger.info(f"[CameraCalibration] Set transform for {camera_name}: "
                   f"t={translation}, calibrated={calibrated}")

    def load_calibration(self, filepath: str):
        """
        Load calibration from YAML file.

        Expected format:
        cameras:
          top_cam:
            translation: [x, y, z]
            rotation_euler: [roll, pitch, yaw]
            calibrated: true
          gripper_cam:
            translation: [x, y, z]
            rotation_euler: [roll, pitch, yaw]
            calibrated: true
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            if 'cameras' not in data:
                raise ValueError("Invalid calibration file: missing 'cameras' key")

            for cam_name, cam_data in data['cameras'].items():
                translation = np.array(cam_data['translation'])
                rotation_euler = np.array(cam_data['rotation_euler'])
                calibrated = cam_data.get('calibrated', False)

                R = Rotation.from_euler('xyz', rotation_euler).as_matrix()

                self.set_transform(
                    cam_name,
                    translation,
                    R,
                    calibrated=calibrated
                )

            logger.info(f"[CameraCalibration] Loaded {len(data['cameras'])} camera calibrations")

        except Exception as e:
            logger.error(f"[CameraCalibration] Failed to load calibration: {e}")
            raise

    def save_calibration(self, filepath: str):
        """
        Save current calibration to YAML file.

        Args:
            filepath: Path to save calibration file
        """
        data = {'cameras': {}}

        for cam_name, transform in self.transforms.items():
            data['cameras'][cam_name] = {
                'translation': transform['translation'].tolist(),
                'rotation_euler': transform['rotation_euler'].tolist(),
                'calibrated': transform.get('calibrated', False),
                'description': transform.get('description', '')
            }

        try:
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            logger.info(f"[CameraCalibration] Saved calibration to {filepath}")

        except Exception as e:
            logger.error(f"[CameraCalibration] Failed to save calibration: {e}")
            raise

    def visualize_transforms(self):
        """Print transformation information for debugging."""
        print("\n" + "="*60)
        print("Camera Calibration Status")
        print("="*60)

        for cam_name, transform in self.transforms.items():
            calibrated = transform.get('calibrated', False)
            status = "✓ CALIBRATED" if calibrated else "✗ UNCALIBRATED"

            print(f"\n{cam_name}: {status}")
            print(f"  Description: {transform.get('description', 'N/A')}")
            print(f"  Translation: {transform['translation']}")
            print(f"  Rotation (euler): {np.degrees(transform['rotation_euler'])} degrees")
            print(f"  Transform Matrix:")
            print(f"    {transform['matrix'][:3, :].tolist()}")

        print("="*60 + "\n")


# Example usage and testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # Initialize calibration
    calib = CameraCalibration()

    # Visualize default transforms
    calib.visualize_transforms()

    # Test point transformation
    point_camera = np.array([0.10, 0.05, 0.50])  # Point in camera frame
    point_base = calib.transform_point(point_camera, 'top_cam')

    print(f"Point in camera frame: {point_camera}")
    print(f"Point in base frame: {point_base}")

    # Test multiple points
    points_camera = np.array([
        [0.10, 0.05, 0.50],
        [0.15, -0.03, 0.48],
        [0.08, 0.10, 0.52]
    ])
    points_base = calib.transform_points(points_camera, 'top_cam')
    print(f"\nTransformed {len(points_base)} points")

    # Test save/load
    calib.save_calibration('/tmp/test_calibration.yaml')
    calib2 = CameraCalibration('/tmp/test_calibration.yaml')
    print("\nCalibration save/load successful")
