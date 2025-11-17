#!/usr/bin/env python3
"""
Camera Calibration Tool using AprilTags

This tool helps calibrate the transformation from camera frames to robot base frame
using AprilTag fiducial markers.

Usage:
1. Print AprilTag markers (tag36h11 family, IDs 0-9)
2. Place AprilTag at known position in robot base frame
3. Run this script to capture images and compute transformations
4. Save calibration to YAML file

Requirements:
    pip install pupil-apriltags opencv-python
"""

import numpy as np
import cv2
import logging
import argparse
import os
from typing import List, Dict, Optional, Tuple
from scipy.spatial.transform import Rotation
import yaml

try:
    from pupil_apriltags import Detector
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    logging.warning("pupil-apriltags not installed. Install with: pip install pupil-apriltags")

from .camera_calibration import CameraCalibration
from .depth_projector import DepthProjector

logger = logging.getLogger(__name__)


class AprilTagCalibrator:
    """
    Calibrate camera-to-base transformations using AprilTag markers.

    Workflow:
    1. Place AprilTag at known position in robot base frame
    2. Detect AprilTag in camera image
    3. Compute camera pose relative to tag
    4. Compute transformation from camera to robot base
    """

    # AprilTag size in meters (measured from outer black border)
    DEFAULT_TAG_SIZE = 0.05  # 5cm tags (common size)

    # Known tag positions in robot base frame
    # These MUST be measured accurately!
    TAG_POSITIONS_BASE = {
        # Tag ID 0: On table in front of robot
        0: {
            'position': np.array([0.30, 0.00, 0.00]),  # [x, y, z] in base frame
            'orientation': np.array([0, 0, 0]),         # [roll, pitch, yaw] tag lying flat
            'description': 'Table center, tag lying flat facing up'
        },
        # Tag ID 1: On robot base (for gripper camera calibration)
        1: {
            'position': np.array([0.10, 0.00, 0.05]),
            'orientation': np.array([np.pi/2, 0, 0]),  # Tag standing upright
            'description': 'Robot base, tag standing facing forward'
        },
    }

    def __init__(self, tag_size: float = DEFAULT_TAG_SIZE):
        """
        Initialize AprilTag calibrator.

        Args:
            tag_size: Physical size of AprilTag in meters
        """
        if not APRILTAG_AVAILABLE:
            raise ImportError("pupil-apriltags not installed. "
                            "Install with: pip install pupil-apriltags")

        self.tag_size = tag_size
        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        logger.info(f"[AprilTagCalibrator] Initialized with tag size: {tag_size}m")

    def detect_tags(
        self,
        gray_image: np.ndarray,
        camera_params: Tuple[float, float, float, float]
    ) -> List[Dict]:
        """
        Detect AprilTags in grayscale image.

        Args:
            gray_image: Grayscale image
            camera_params: (fx, fy, cx, cy) camera intrinsics

        Returns:
            List of detected tags with pose information
        """
        fx, fy, cx, cy = camera_params

        detections = self.detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],
            tag_size=self.tag_size
        )

        logger.info(f"[AprilTagCalibrator] Detected {len(detections)} tags")

        results = []
        for det in detections:
            if det.pose_err is None or det.pose_err > 1e-5:
                logger.warning(f"Tag {det.tag_id}: High pose error {det.pose_err}")

            results.append({
                'tag_id': det.tag_id,
                'corners': det.corners,
                'center': det.center,
                'pose_R': det.pose_R,  # 3x3 rotation matrix
                'pose_t': det.pose_t,  # 3x1 translation vector (tag center in camera frame)
                'pose_err': det.pose_err
            })

        return results

    def compute_camera_to_base_transform(
        self,
        tag_detection: Dict,
        tag_id: int
    ) -> Optional[np.ndarray]:
        """
        Compute camera-to-base transformation using detected tag.

        Args:
            tag_detection: Detected tag information
            tag_id: Tag ID to use for calibration

        Returns:
            4x4 transformation matrix from camera to base frame
        """
        if tag_id not in self.TAG_POSITIONS_BASE:
            logger.error(f"Unknown tag ID: {tag_id}. "
                        f"Available: {list(self.TAG_POSITIONS_BASE.keys())}")
            return None

        # Get known tag pose in base frame
        tag_base_info = self.TAG_POSITIONS_BASE[tag_id]
        t_base = tag_base_info['position']
        R_base = Rotation.from_euler('xyz', tag_base_info['orientation']).as_matrix()

        # Create homogeneous transform: Base_T_Tag
        T_base_tag = np.eye(4)
        T_base_tag[:3, :3] = R_base
        T_base_tag[:3, 3] = t_base

        # Get detected tag pose in camera frame: Camera_T_Tag
        T_cam_tag = np.eye(4)
        T_cam_tag[:3, :3] = tag_detection['pose_R']
        T_cam_tag[:3, 3] = tag_detection['pose_t'].flatten()

        # Compute camera-to-base transform:
        # Base_T_Camera = Base_T_Tag * Tag_T_Camera
        # Tag_T_Camera = inv(Camera_T_Tag)
        T_tag_cam = np.linalg.inv(T_cam_tag)
        T_base_cam = T_base_tag @ T_tag_cam

        logger.info(f"[AprilTagCalibrator] Computed camera-to-base transform using tag {tag_id}")
        logger.debug(f"  Translation: {T_base_cam[:3, 3]}")

        return T_base_cam

    def calibrate_camera(
        self,
        rgb_image: np.ndarray,
        camera_params: Tuple[float, float, float, float],
        expected_tag_id: int = 0
    ) -> Optional[Dict]:
        """
        Calibrate a camera using AprilTag detection.

        Args:
            rgb_image: RGB camera image
            camera_params: (fx, fy, cx, cy) intrinsics
            expected_tag_id: Expected tag ID to use

        Returns:
            Calibration result dictionary with transformation
        """
        # Convert to grayscale
        if len(rgb_image.shape) == 3:
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        else:
            gray = rgb_image

        # Detect tags
        detections = self.detect_tags(gray, camera_params)

        if not detections:
            logger.error("[AprilTagCalibrator] No AprilTags detected!")
            return None

        # Find expected tag
        target_detection = None
        for det in detections:
            if det['tag_id'] == expected_tag_id:
                target_detection = det
                break

        if target_detection is None:
            logger.error(f"[AprilTagCalibrator] Expected tag {expected_tag_id} not found. "
                        f"Detected: {[d['tag_id'] for d in detections]}")
            return None

        # Compute transformation
        T_base_cam = self.compute_camera_to_base_transform(
            target_detection, expected_tag_id
        )

        if T_base_cam is None:
            return None

        # Extract translation and rotation
        translation = T_base_cam[:3, 3]
        rotation_matrix = T_base_cam[:3, :3]
        rotation_euler = Rotation.from_matrix(rotation_matrix).as_euler('xyz')

        logger.info(f"[AprilTagCalibrator] Calibration successful!")
        logger.info(f"  Translation: {translation}")
        logger.info(f"  Rotation (deg): {np.degrees(rotation_euler)}")

        return {
            'transform_matrix': T_base_cam,
            'translation': translation,
            'rotation_matrix': rotation_matrix,
            'rotation_euler': rotation_euler,
            'tag_id': expected_tag_id,
            'pose_error': target_detection['pose_err']
        }

    def visualize_detection(
        self,
        image: np.ndarray,
        detections: List[Dict],
        save_path: Optional[str] = None
    ) -> np.ndarray:
        """
        Draw detected AprilTags on image.

        Args:
            image: RGB image
            detections: List of tag detections
            save_path: Optional path to save visualization

        Returns:
            Annotated image
        """
        vis_image = image.copy()

        for det in detections:
            # Draw corners
            corners = det['corners'].astype(int)
            for i in range(4):
                cv2.line(vis_image,
                        tuple(corners[i]),
                        tuple(corners[(i + 1) % 4]),
                        (0, 255, 0), 2)

            # Draw center
            center = det['center'].astype(int)
            cv2.circle(vis_image, tuple(center), 5, (0, 0, 255), -1)

            # Draw tag ID
            cv2.putText(vis_image,
                       f"ID: {det['tag_id']}",
                       tuple(center + [10, -10]),
                       cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (255, 255, 0), 2)

        if save_path:
            cv2.imwrite(save_path, cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR))
            logger.info(f"[AprilTagCalibrator] Saved visualization to {save_path}")

        return vis_image


def calibrate_from_realsense(
    camera_name: str,
    tag_id: int = 0,
    output_file: str = "camera_calibration.yaml"
):
    """
    Interactive calibration using live RealSense camera feed.

    Args:
        camera_name: 'top_cam' or 'gripper_cam'
        tag_id: AprilTag ID to use for calibration
        output_file: Path to save calibration YAML
    """
    import pyrealsense2 as rs

    print(f"\n{'='*60}")
    print(f"AprilTag Camera Calibration - {camera_name}")
    print(f"{'='*60}\n")

    print("Instructions:")
    print(f"1. Place AprilTag ID {tag_id} at the known position (see TAG_POSITIONS_BASE)")
    print("2. Ensure tag is fully visible in camera view")
    print("3. Press 's' to capture and calibrate, 'q' to quit\n")

    # Initialize RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    pipeline.start(config)

    # Get camera intrinsics
    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    intrinsics = color_profile.get_intrinsics()
    camera_params = (intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

    print(f"Camera intrinsics: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}, "
          f"cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}\n")

    # Initialize calibrator
    calibrator = AprilTagCalibrator()

    try:
        while True:
            # Get frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # Convert to numpy
            color_image = np.asanyarray(color_frame.get_data())

            # Detect tags for visualization
            gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            detections = calibrator.detect_tags(gray, camera_params)

            # Visualize
            vis_image = calibrator.visualize_detection(color_image, detections)
            cv2.imshow(f'Calibration - {camera_name}', cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR))

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("Calibration cancelled.")
                break

            elif key == ord('s'):
                print("\nCapturing calibration image...")

                result = calibrator.calibrate_camera(
                    color_image, camera_params, tag_id
                )

                if result:
                    print("\n✓ Calibration successful!")
                    print(f"  Translation: {result['translation']}")
                    print(f"  Rotation (deg): {np.degrees(result['rotation_euler'])}")

                    # Save to file
                    calib = CameraCalibration()
                    calib.set_transform(
                        camera_name,
                        result['translation'],
                        result['rotation_matrix'],
                        calibrated=True
                    )
                    calib.save_calibration(output_file)

                    print(f"\n✓ Saved calibration to {output_file}")
                    break
                else:
                    print("\n✗ Calibration failed. Try again.")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


# Command-line interface
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(
        description="Calibrate camera-to-base transformation using AprilTags"
    )
    parser.add_argument(
        '--camera',
        choices=['top_cam', 'gripper_cam'],
        required=True,
        help='Camera to calibrate'
    )
    parser.add_argument(
        '--tag-id',
        type=int,
        default=0,
        help='AprilTag ID to use (default: 0)'
    )
    parser.add_argument(
        '--output',
        default='camera_calibration.yaml',
        help='Output calibration file (default: camera_calibration.yaml)'
    )

    args = parser.parse_args()

    if not APRILTAG_AVAILABLE:
        print("\nError: pupil-apriltags not installed")
        print("Install with: pip install pupil-apriltags")
        exit(1)

    calibrate_from_realsense(args.camera, args.tag_id, args.output)
