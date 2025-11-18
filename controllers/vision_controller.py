#!/usr/bin/env python3
"""
Vision Controller - High-level interface for robot vision system

Integrates all vision components (detector, projector, calibration, localizer)
and provides simple API for the bridge server.
"""

import logging
import numpy as np
import os
from typing import Dict, List, Optional, Tuple

# Import vision system components
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from vision.gemini_vision_detector import GeminiVisionDetector
from vision.depth_projector import DepthProjector
from vision.camera_calibration import CameraCalibration
from vision.object_localizer import ObjectLocalizer

logger = logging.getLogger(__name__)


class VisionController:
    """
    High-level vision controller for ALOHA robot.

    Provides simple interface for:
    - Object detection and localization
    - Scene analysis
    - Pick-and-place target identification
    """

    def __init__(
        self,
        camera_controller,
        gemini_api_key: Optional[str] = None,
        calibration_file: Optional[str] = None
    ):
        """
        Initialize vision controller.

        Args:
            camera_controller: CameraController instance
            gemini_api_key: Google API key for Gemini (or None to use env var)
            calibration_file: Path to camera calibration YAML (or None for defaults)
        """
        self.camera = camera_controller
        self.initialized = False

        # Initialize vision components
        try:
            logger.info("[VisionController] Initializing vision system...")

            # 1. Gemini Vision Detector
            self.detector = GeminiVisionDetector(api_key=gemini_api_key)

            # 2. Depth Projector (will get intrinsics from camera)
            self.projector = DepthProjector()

            # 3. Camera Calibration
            if calibration_file and os.path.exists(calibration_file):
                self.calibration = CameraCalibration(calibration_file)
                logger.info(f"[VisionController] Loaded calibration from {calibration_file}")
            else:
                self.calibration = CameraCalibration()
                logger.warning("[VisionController] Using default (uncalibrated) transforms!")

            # 4. Object Localizer
            self.localizer = ObjectLocalizer(
                self.detector,
                self.projector,
                self.calibration
            )

            self.initialized = True
            logger.info("[VisionController] ✓ Vision system initialized successfully")

        except Exception as e:
            logger.error(f"[VisionController] Failed to initialize: {e}")
            self.initialized = False
            raise

    def find_object(
        self,
        object_name: str,
        camera_name: str = 'top_cam'
    ) -> Optional[Dict]:
        """
        Find a specific object in the scene.

        Args:
            object_name: Name or description of object
            camera_name: Camera to use ('top_cam' or 'gripper_cam')

        Returns:
            Object info dict with 'position_3d_base', 'bbox', 'confidence', etc.
            or None if not found
        """
        if not self.initialized:
            logger.error("[VisionController] Not initialized")
            return None

        try:
            # Get current camera frames
            rgb, depth = self.camera.get_rgbd_frames(camera_name)

            if rgb is None or depth is None:
                logger.error(f"[VisionController] Failed to get frames from {camera_name}")
                return None

            # Localize object
            obj_info = self.localizer.locate_object(
                rgb, depth, camera_name, object_name
            )

            if obj_info:
                logger.info(f"[VisionController] Found '{object_name}' at "
                           f"{obj_info['position_3d_base']}")
            else:
                logger.warning(f"[VisionController] '{object_name}' not found")

            return obj_info

        except Exception as e:
            logger.error(f"[VisionController] Error finding object: {e}")
            return None

    def find_all_objects(self, camera_name: str = 'top_cam') -> List[Dict]:
        """
        Find all graspable objects in the scene.

        Args:
            camera_name: Camera to use

        Returns:
            List of object info dicts
        """
        if not self.initialized:
            logger.error("[VisionController] Not initialized")
            return []

        try:
            rgb, depth = self.camera.get_rgbd_frames(camera_name)

            if rgb is None or depth is None:
                logger.error(f"[VisionController] Failed to get frames from {camera_name}")
                return []

            objects = self.localizer.locate_all_objects(rgb, depth, camera_name)

            logger.info(f"[VisionController] Found {len(objects)} objects")
            return objects

        except Exception as e:
            logger.error(f"[VisionController] Error finding objects: {e}")
            return []

    def analyze_scene(self, camera_name: str = 'top_cam') -> Dict:
        """
        Comprehensive scene analysis with 3D positions.

        Args:
            camera_name: Camera to use

        Returns:
            Scene analysis dict with objects, spatial relations, description
        """
        if not self.initialized:
            logger.error("[VisionController] Not initialized")
            return {'objects': [], 'description': ''}

        try:
            rgb, depth = self.camera.get_rgbd_frames(camera_name)

            if rgb is None or depth is None:
                return {'objects': [], 'description': 'Camera not available'}

            scene = self.localizer.analyze_scene_3d(rgb, depth, camera_name)

            logger.info(f"[VisionController] Scene analysis: {scene['num_objects']} objects, "
                       f"workspace_occupied={scene['workspace_occupied']}")

            return scene

        except Exception as e:
            logger.error(f"[VisionController] Error analyzing scene: {e}")
            return {'objects': [], 'description': f'Error: {e}'}

    def get_pick_and_place_targets(
        self,
        object_name: str,
        target_name: str,
        camera_name: str = 'top_cam'
    ) -> Optional[Dict]:
        """
        Get 3D positions for pick-and-place operation.

        Args:
            object_name: Object to pick up
            target_name: Target location (e.g., "bowl", "plate")
            camera_name: Camera to use

        Returns:
            Dict with 'object' and 'target' info, or None if either not found
        """
        if not self.initialized:
            return None

        try:
            logger.info(f"[VisionController] Finding targets: '{object_name}' → '{target_name}'")

            # Find object to pick
            obj_info = self.find_object(object_name, camera_name)
            if not obj_info:
                logger.error(f"[VisionController] Could not locate '{object_name}'")
                return None

            # Find target location
            target_info = self.find_object(target_name, camera_name)
            if not target_info:
                logger.error(f"[VisionController] Could not locate '{target_name}'")
                return None

            result = {
                'object': obj_info,
                'target': target_info,
                'success': True
            }

            logger.info(f"[VisionController] ✓ Found both targets:")
            logger.info(f"  {object_name}: {obj_info['position_3d_base']}")
            logger.info(f"  {target_name}: {target_info['position_3d_base']}")

            return result

        except Exception as e:
            logger.error(f"[VisionController] Error in get_pick_and_place_targets: {e}")
            return None

    def visualize_current_view(
        self,
        camera_name: str = 'top_cam',
        save_path: Optional[str] = None
    ) -> Optional[np.ndarray]:
        """
        Get annotated view with detected objects.

        Args:
            camera_name: Camera to use
            save_path: Optional path to save image

        Returns:
            Annotated RGB image or None
        """
        if not self.initialized:
            return None

        try:
            rgb, depth = self.camera.get_rgbd_frames(camera_name)
            if rgb is None:
                return None

            # Detect and localize objects
            objects = self.localizer.locate_all_objects(rgb, depth, camera_name)

            # Visualize
            vis_image = self.localizer.visualize_localization(rgb, objects, save_path)

            return vis_image

        except Exception as e:
            logger.error(f"[VisionController] Error in visualize: {e}")
            return None

    def get_status(self) -> Dict:
        """
        Get vision system status.

        Returns:
            Status dict with initialization state and calibration info
        """
        status = {
            'initialized': self.initialized,
            'detector_available': hasattr(self, 'detector'),
            'cameras_calibrated': {}
        }

        if self.initialized:
            for cam_name in ['top_cam', 'gripper_cam']:
                status['cameras_calibrated'][cam_name] = \
                    self.calibration.is_calibrated(cam_name)

        return status


# Example usage and testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("\n" + "="*60)
    print("Vision Controller - Standalone Test")
    print("="*60 + "\n")

    # This would normally be initialized with actual camera controller
    print("Note: This is a standalone test. In production, use with CameraController.")
    print("\nVision Controller API:")
    print("  - find_object(name, camera)")
    print("  - find_all_objects(camera)")
    print("  - analyze_scene(camera)")
    print("  - get_pick_and_place_targets(object, target, camera)")
    print("  - visualize_current_view(camera, save_path)")
    print("  - get_status()")
    print("\n" + "="*60 + "\n")
