#!/usr/bin/env python3
"""
Object Localizer - Complete object detection and 3D localization pipeline

This module combines:
1. GeminiVisionDetector - Object detection in images
2. DepthProjector - 2D to 3D conversion using depth
3. CameraCalibration - Camera to robot base frame transformation

Provides high-level API for detecting objects and getting their 3D positions
in the robot's base coordinate frame.
"""

import numpy as np
import logging
from typing import List, Dict, Optional, Tuple
import cv2

from .gemini_vision_detector import GeminiVisionDetector
from .depth_projector import DepthProjector
from .camera_calibration import CameraCalibration

logger = logging.getLogger(__name__)


class ObjectLocalizer:
    """
    Complete object localization system for ALOHA robot.

    Workflow:
    1. Detect objects in RGB image (Gemini Vision)
    2. Extract depth at object locations
    3. Project to 3D in camera frame
    4. Transform to robot base frame
    """

    def __init__(
        self,
        detector: GeminiVisionDetector,
        depth_projector: DepthProjector,
        calibration: CameraCalibration
    ):
        """
        Initialize object localizer.

        Args:
            detector: Gemini vision detector instance
            depth_projector: Depth projector instance
            calibration: Camera calibration instance
        """
        self.detector = detector
        self.projector = depth_projector
        self.calibration = calibration

        logger.info("[ObjectLocalizer] Initialized with Gemini detector + depth projection")

    def locate_object(
        self,
        rgb_frame: np.ndarray,
        depth_frame: np.ndarray,
        camera_name: str,
        object_name: str,
        end_effector_pose: Optional[np.ndarray] = None
    ) -> Optional[Dict]:
        """
        Locate a specific object in 3D space (robot base frame).

        Args:
            rgb_frame: RGB image (H, W, 3)
            depth_frame: Depth image (H, W) - raw depth values
            camera_name: Camera identifier ('top_cam' or 'gripper_cam')
            object_name: Name/description of object to find
            end_effector_pose: 4x4 transform of end effector (for gripper_cam)

        Returns:
            Dictionary with object information:
            {
                'name': str,
                'position_3d_base': [x, y, z],  # In robot base frame
                'position_3d_camera': [x, y, z],  # In camera frame
                'bbox': [x1, y1, x2, y2],
                'confidence': float,
                'depth_mean': float,
                'depth_std': float
            }
            Returns None if object not found
        """
        logger.info(f"[ObjectLocalizer] Locating '{object_name}' in {camera_name}")

        # Step 1: Detect object in image
        detection = self.detector.detect_specific_object(rgb_frame, object_name)

        if detection is None or not detection.get('found'):
            logger.warning(f"[ObjectLocalizer] '{object_name}' not detected in image")
            return None

        # Step 2: Get depth information at object location
        bbox = detection['bbox']
        center_x, center_y = detection['center']

        # Get average depth in ROI around object center
        depth_info = self.projector.get_roi_average_depth(
            depth_frame,
            center_x,
            center_y,
            roi_size=10
        )

        if depth_info is None:
            logger.warning(f"[ObjectLocalizer] No valid depth for '{object_name}'")
            return None

        depth_mean, depth_std = depth_info

        # Step 3: Project to 3D in camera frame
        point_3d_camera = self.projector.pixel_to_point(
            center_x,
            center_y,
            depth_mean / 0.001,  # Convert back to raw depth units
            depth_scale=0.001
        )

        if point_3d_camera is None:
            logger.warning(f"[ObjectLocalizer] Failed to project '{object_name}' to 3D")
            return None

        # Step 4: Transform to robot base frame
        point_3d_base = self.calibration.transform_point(
            point_3d_camera,
            camera_name,
            end_effector_pose
        )

        logger.info(f"[ObjectLocalizer] '{object_name}' located at {point_3d_base} "
                   f"(base frame)")

        return {
            'name': detection['name'],
            'position_3d_base': point_3d_base,
            'position_3d_camera': point_3d_camera,
            'bbox': bbox,
            'confidence': detection.get('confidence', 0.0),
            'depth_mean': depth_mean,
            'depth_std': depth_std,
            'camera_name': camera_name
        }

    def locate_all_objects(
        self,
        rgb_frame: np.ndarray,
        depth_frame: np.ndarray,
        camera_name: str,
        end_effector_pose: Optional[np.ndarray] = None
    ) -> List[Dict]:
        """
        Locate all graspable objects in the scene.

        Args:
            rgb_frame: RGB image
            depth_frame: Depth image
            camera_name: Camera identifier
            end_effector_pose: End effector pose (for gripper_cam)

        Returns:
            List of object dictionaries (same format as locate_object)
        """
        logger.info(f"[ObjectLocalizer] Detecting all objects in {camera_name}")

        # Step 1: Detect all objects
        detections = self.detector.detect_objects(rgb_frame)

        if not detections:
            logger.info("[ObjectLocalizer] No objects detected")
            return []

        # Step 2: Localize each detected object
        localized_objects = []

        for detection in detections:
            if 'center' not in detection or 'bbox' not in detection:
                continue

            center_x, center_y = detection['center']
            bbox = detection['bbox']

            # Get depth
            depth_info = self.projector.get_roi_average_depth(
                depth_frame, center_x, center_y, roi_size=10
            )

            if depth_info is None:
                logger.debug(f"[ObjectLocalizer] Skipping {detection['name']} - no depth")
                continue

            depth_mean, depth_std = depth_info

            # Project to 3D camera frame
            point_3d_camera = self.projector.pixel_to_point(
                center_x, center_y, depth_mean / 0.001, depth_scale=0.001
            )

            if point_3d_camera is None:
                continue

            # Transform to base frame
            point_3d_base = self.calibration.transform_point(
                point_3d_camera, camera_name, end_effector_pose
            )

            localized_objects.append({
                'name': detection['name'],
                'position_3d_base': point_3d_base,
                'position_3d_camera': point_3d_camera,
                'bbox': bbox,
                'confidence': detection.get('confidence', 0.0),
                'depth_mean': depth_mean,
                'depth_std': depth_std,
                'camera_name': camera_name
            })

        logger.info(f"[ObjectLocalizer] Localized {len(localized_objects)} objects")
        return localized_objects

    def analyze_scene_3d(
        self,
        rgb_frame: np.ndarray,
        depth_frame: np.ndarray,
        camera_name: str,
        end_effector_pose: Optional[np.ndarray] = None
    ) -> Dict:
        """
        Comprehensive 3D scene analysis.

        Args:
            rgb_frame: RGB image
            depth_frame: Depth image
            camera_name: Camera identifier
            end_effector_pose: End effector pose

        Returns:
            Scene analysis dictionary:
            {
                'objects': List of localized objects,
                'spatial_relations': List of spatial relationships,
                'description': Natural language description,
                'workspace_occupied': bool
            }
        """
        logger.info(f"[ObjectLocalizer] Analyzing 3D scene from {camera_name}")

        # Get 2D scene analysis from Gemini
        scene_2d = self.detector.analyze_scene(rgb_frame)

        # Localize all detected objects
        objects_3d = self.locate_all_objects(
            rgb_frame, depth_frame, camera_name, end_effector_pose
        )

        # Check if workspace is occupied (any object in reach)
        workspace_occupied = any(
            self._is_in_workspace(obj['position_3d_base'])
            for obj in objects_3d
        )

        return {
            'objects': objects_3d,
            'spatial_relations': scene_2d.get('spatial_relations', []),
            'description': scene_2d.get('description', ''),
            'workspace_occupied': workspace_occupied,
            'num_objects': len(objects_3d)
        }

    def find_closest_object(
        self,
        rgb_frame: np.ndarray,
        depth_frame: np.ndarray,
        camera_name: str,
        reference_point: Optional[np.ndarray] = None,
        end_effector_pose: Optional[np.ndarray] = None
    ) -> Optional[Dict]:
        """
        Find the object closest to a reference point (or robot base).

        Args:
            rgb_frame: RGB image
            depth_frame: Depth image
            camera_name: Camera identifier
            reference_point: Reference point [x,y,z] in base frame (default: [0,0,0])
            end_effector_pose: End effector pose

        Returns:
            Closest object dict or None
        """
        if reference_point is None:
            reference_point = np.array([0.3, 0.0, 0.0])  # Default: in front of base

        objects = self.locate_all_objects(
            rgb_frame, depth_frame, camera_name, end_effector_pose
        )

        if not objects:
            return None

        # Calculate distances
        closest_obj = None
        min_distance = float('inf')

        for obj in objects:
            distance = np.linalg.norm(obj['position_3d_base'] - reference_point)
            if distance < min_distance:
                min_distance = distance
                closest_obj = obj

        if closest_obj:
            closest_obj['distance_to_reference'] = min_distance
            logger.info(f"[ObjectLocalizer] Closest object: {closest_obj['name']} "
                       f"at {min_distance:.3f}m")

        return closest_obj

    def _is_in_workspace(self, position: np.ndarray) -> bool:
        """
        Check if a position is within robot's workspace.

        Args:
            position: [x, y, z] in base frame

        Returns:
            True if position is reachable
        """
        x, y, z = position

        # ALOHA workspace limits (from CLAUDE.md)
        in_workspace = (
            0.15 <= x <= 0.50 and
            -0.30 <= y <= 0.30 and
            0.05 <= z <= 0.40
        )

        return in_workspace

    def visualize_localization(
        self,
        rgb_frame: np.ndarray,
        objects: List[Dict],
        save_path: Optional[str] = None
    ) -> np.ndarray:
        """
        Visualize detected objects with 3D positions overlaid.

        Args:
            rgb_frame: RGB image
            objects: List of localized objects
            save_path: Optional path to save visualization

        Returns:
            Annotated image
        """
        vis_image = rgb_frame.copy()

        for obj in objects:
            bbox = obj['bbox']
            name = obj['name']
            pos_3d = obj['position_3d_base']
            confidence = obj.get('confidence', 0.0)

            # Draw bounding box
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Create label with 3D position
            label = f"{name} ({confidence:.2f})"
            pos_label = f"[{pos_3d[0]:.2f}, {pos_3d[1]:.2f}, {pos_3d[2]:.2f}]"

            # Draw labels
            cv2.putText(vis_image, label, (x1, y1 - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(vis_image, pos_label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # Draw center point
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(vis_image, (cx, cy), 5, (255, 0, 0), -1)

        if save_path:
            cv2.imwrite(save_path, cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR))
            logger.info(f"[ObjectLocalizer] Saved visualization to {save_path}")

        return vis_image


# Example usage
if __name__ == "__main__":
    import os

    logging.basicConfig(level=logging.INFO)

    # Initialize components
    api_key = os.getenv('GOOGLE_API_KEY')
    detector = GeminiVisionDetector(api_key=api_key)
    projector = DepthProjector()
    calibration = CameraCalibration()

    # Create localizer
    localizer = ObjectLocalizer(detector, projector, calibration)

    # Create test data
    rgb_test = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    depth_test = np.random.randint(500, 1500, (480, 640), dtype=np.uint16)

    # Test object localization
    print("\n=== Testing Object Localization ===")
    banana = localizer.locate_object(
        rgb_test, depth_test, 'top_cam', 'banana'
    )

    if banana:
        print(f"Banana found at: {banana['position_3d_base']}")
        print(f"Confidence: {banana['confidence']:.2f}")
        print(f"Depth: {banana['depth_mean']:.3f}m ± {banana['depth_std']:.3f}m")

    # Test scene analysis
    print("\n=== Testing Scene Analysis ===")
    scene = localizer.analyze_scene_3d(rgb_test, depth_test, 'top_cam')
    print(f"Objects detected: {scene['num_objects']}")
    print(f"Workspace occupied: {scene['workspace_occupied']}")
    print(f"Description: {scene['description']}")

    # Test find closest object
    print("\n=== Testing Closest Object ===")
    closest = localizer.find_closest_object(rgb_test, depth_test, 'top_cam')
    if closest:
        print(f"Closest: {closest['name']} at {closest['distance_to_reference']:.3f}m")
