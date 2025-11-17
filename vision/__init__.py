"""
Vision system for ALOHA robot manipulation.

Components:
- GeminiVisionDetector: Object detection using Gemini Vision API
- DepthProjector: Convert 2D pixels + depth to 3D coordinates
- CameraCalibration: Camera-to-robot base transformations
- ObjectLocalizer: Combined detection and 3D localization
"""

from .gemini_vision_detector import GeminiVisionDetector
from .depth_projector import DepthProjector
from .camera_calibration import CameraCalibration
from .object_localizer import ObjectLocalizer

__all__ = [
    'GeminiVisionDetector',
    'DepthProjector',
    'CameraCalibration',
    'ObjectLocalizer'
]
