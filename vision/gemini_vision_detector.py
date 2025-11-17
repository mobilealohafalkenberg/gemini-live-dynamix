#!/usr/bin/env python3
"""
Gemini Vision Detector - Object detection using Gemini Vision API

This module uses Google's Gemini 2.5 Flash multimodal capabilities to detect
and localize objects in camera images. It returns structured detection data
including bounding boxes and confidence scores.
"""

import base64
import json
import logging
import numpy as np
import cv2
from typing import List, Dict, Optional, Tuple
import google.generativeai as genai
from PIL import Image
import io

logger = logging.getLogger(__name__)


class GeminiVisionDetector:
    """
    Object detection using Gemini Vision API.

    Uses Gemini's multimodal capabilities to:
    - Detect objects in images
    - Provide bounding boxes
    - Identify object names and categories
    - Estimate confidence scores
    """

    def __init__(self, api_key: Optional[str] = None, model_name: str = "gemini-2.0-flash-exp"):
        """
        Initialize Gemini Vision Detector.

        Args:
            api_key: Google API key (if None, uses GOOGLE_API_KEY env var)
            model_name: Gemini model to use
        """
        if api_key:
            genai.configure(api_key=api_key)

        self.model_name = model_name
        self.model = genai.GenerativeModel(model_name)

        logger.info(f"[GeminiVisionDetector] Initialized with model: {model_name}")

    def detect_objects(
        self,
        image: np.ndarray,
        prompt: Optional[str] = None,
        return_format: str = "json"
    ) -> List[Dict]:
        """
        Detect objects in an image using Gemini Vision API.

        Args:
            image: RGB image as numpy array (H, W, 3)
            prompt: Custom prompt (if None, uses default detection prompt)
            return_format: "json" for structured data, "text" for description

        Returns:
            List of detections, each containing:
            - name: Object name/label
            - bbox: Bounding box [x1, y1, x2, y2] in pixels
            - confidence: Confidence score (0-1)
            - center: Center point [x, y] in pixels
        """
        if prompt is None:
            prompt = self._get_default_detection_prompt(image.shape)

        try:
            # Convert numpy array to PIL Image
            if image.dtype == np.uint8:
                pil_image = Image.fromarray(image)
            else:
                # Normalize if float
                image_normalized = (image * 255).astype(np.uint8)
                pil_image = Image.fromarray(image_normalized)

            # Generate content with Gemini
            response = self.model.generate_content([prompt, pil_image])

            # Parse response
            if return_format == "json":
                detections = self._parse_json_response(response.text, image.shape)
            else:
                detections = self._parse_text_response(response.text, image.shape)

            logger.info(f"[GeminiVisionDetector] Detected {len(detections)} objects")
            return detections

        except Exception as e:
            logger.error(f"[GeminiVisionDetector] Detection failed: {e}")
            return []

    def detect_specific_object(
        self,
        image: np.ndarray,
        object_name: str
    ) -> Optional[Dict]:
        """
        Detect a specific object by name.

        Args:
            image: RGB image as numpy array
            object_name: Name or description of object to find

        Returns:
            Single detection dict or None if not found
        """
        prompt = f"""Find the {object_name} in this image.

Return ONLY a JSON object with this exact format:
{{
    "found": true/false,
    "name": "object name",
    "bbox": [x1, y1, x2, y2],
    "confidence": 0.0-1.0,
    "description": "brief description"
}}

Image dimensions: {image.shape[1]}x{image.shape[0]} pixels
Bounding box coordinates should be in pixels relative to top-left corner (0,0).
If object is not found, set "found": false."""

        try:
            pil_image = Image.fromarray(image)
            response = self.model.generate_content([prompt, pil_image])

            # Parse JSON response
            detection = self._extract_json_from_response(response.text)

            if detection and detection.get('found'):
                # Add center point
                bbox = detection['bbox']
                detection['center'] = [
                    (bbox[0] + bbox[2]) / 2,
                    (bbox[1] + bbox[3]) / 2
                ]
                logger.info(f"[GeminiVisionDetector] Found {object_name} at {detection['center']}")
                return detection
            else:
                logger.warning(f"[GeminiVisionDetector] Object '{object_name}' not found")
                return None

        except Exception as e:
            logger.error(f"[GeminiVisionDetector] Failed to detect {object_name}: {e}")
            return None

    def analyze_scene(self, image: np.ndarray) -> Dict:
        """
        Analyze entire scene for spatial understanding.

        Args:
            image: RGB image as numpy array

        Returns:
            Scene analysis containing:
            - objects: List of all detected objects
            - spatial_relations: Relationships between objects
            - description: Natural language scene description
        """
        prompt = f"""Analyze this workspace scene and provide a detailed JSON response.

Return ONLY a JSON object with this format:
{{
    "objects": [
        {{"name": "object1", "bbox": [x1,y1,x2,y2], "confidence": 0.95}},
        {{"name": "object2", "bbox": [x1,y1,x2,y2], "confidence": 0.87}}
    ],
    "spatial_relations": [
        {{"object1": "banana", "relation": "on", "object2": "table"}},
        {{"object1": "bowl", "relation": "left_of", "object2": "plate"}}
    ],
    "description": "A table with a banana, bowl, and plate arranged..."
}}

Image dimensions: {image.shape[1]}x{image.shape[0]} pixels"""

        try:
            pil_image = Image.fromarray(image)
            response = self.model.generate_content([prompt, pil_image])

            analysis = self._extract_json_from_response(response.text)

            # Add center points to objects
            if analysis and 'objects' in analysis:
                for obj in analysis['objects']:
                    if 'bbox' in obj:
                        bbox = obj['bbox']
                        obj['center'] = [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2]

            logger.info(f"[GeminiVisionDetector] Scene analysis: {len(analysis.get('objects', []))} objects")
            return analysis

        except Exception as e:
            logger.error(f"[GeminiVisionDetector] Scene analysis failed: {e}")
            return {'objects': [], 'spatial_relations': [], 'description': ''}

    def _get_default_detection_prompt(self, image_shape: Tuple[int, int, int]) -> str:
        """Generate default object detection prompt."""
        h, w, _ = image_shape
        return f"""Detect all graspable objects in this robotic workspace image.

Return ONLY a JSON array of objects with this exact format:
[
    {{"name": "object1", "bbox": [x1, y1, x2, y2], "confidence": 0.95}},
    {{"name": "object2", "bbox": [x1, y1, x2, y2], "confidence": 0.87}}
]

Image dimensions: {w}x{h} pixels
Bounding box format: [x1, y1, x2, y2] where (x1,y1) is top-left, (x2,y2) is bottom-right
Only include objects that could be grasped by a robot gripper.
Return empty array [] if no graspable objects found."""

    def _extract_json_from_response(self, response_text: str) -> Optional[Dict]:
        """Extract JSON from Gemini response text."""
        try:
            # Try to find JSON in code blocks
            if "```json" in response_text:
                json_start = response_text.find("```json") + 7
                json_end = response_text.find("```", json_start)
                json_text = response_text[json_start:json_end].strip()
            elif "```" in response_text:
                json_start = response_text.find("```") + 3
                json_end = response_text.find("```", json_start)
                json_text = response_text[json_start:json_end].strip()
            else:
                # Try to parse entire response
                json_text = response_text.strip()

            # Parse JSON
            return json.loads(json_text)

        except json.JSONDecodeError as e:
            logger.error(f"[GeminiVisionDetector] JSON parse error: {e}")
            logger.debug(f"Response text: {response_text}")
            return None

    def _parse_json_response(self, response_text: str, image_shape: Tuple) -> List[Dict]:
        """Parse JSON response into detection list."""
        data = self._extract_json_from_response(response_text)

        if data is None:
            return []

        # Handle both array and object formats
        if isinstance(data, list):
            detections = data
        elif isinstance(data, dict) and 'objects' in data:
            detections = data['objects']
        else:
            return []

        # Add center points
        for det in detections:
            if 'bbox' in det:
                bbox = det['bbox']
                det['center'] = [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2]

        return detections

    def _parse_text_response(self, response_text: str, image_shape: Tuple) -> List[Dict]:
        """Parse free-form text response (fallback method)."""
        # This is a simplified parser for when JSON isn't returned
        # In practice, you'd want more robust parsing
        logger.warning("[GeminiVisionDetector] Using text parsing (JSON parsing failed)")
        return []

    def visualize_detections(
        self,
        image: np.ndarray,
        detections: List[Dict],
        save_path: Optional[str] = None
    ) -> np.ndarray:
        """
        Draw bounding boxes and labels on image.

        Args:
            image: RGB image
            detections: List of detection dicts
            save_path: Optional path to save visualization

        Returns:
            Image with visualizations drawn
        """
        vis_image = image.copy()

        for det in detections:
            if 'bbox' not in det:
                continue

            bbox = det['bbox']
            name = det.get('name', 'unknown')
            confidence = det.get('confidence', 0.0)

            # Draw bounding box
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            label = f"{name} ({confidence:.2f})"
            cv2.putText(vis_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw center point
            if 'center' in det:
                cx, cy = map(int, det['center'])
                cv2.circle(vis_image, (cx, cy), 5, (255, 0, 0), -1)

        if save_path:
            cv2.imwrite(save_path, cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR))
            logger.info(f"[GeminiVisionDetector] Saved visualization to {save_path}")

        return vis_image


# Example usage
if __name__ == "__main__":
    import os

    # Setup logging
    logging.basicConfig(level=logging.INFO)

    # Initialize detector
    api_key = os.getenv('GOOGLE_API_KEY')
    detector = GeminiVisionDetector(api_key=api_key)

    # Load test image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Detect objects
    detections = detector.detect_objects(test_image)
    print(f"Detected {len(detections)} objects:")
    for det in detections:
        print(f"  - {det['name']}: {det['bbox']}")

    # Detect specific object
    banana = detector.detect_specific_object(test_image, "banana")
    if banana:
        print(f"Found banana at {banana['center']}")

    # Analyze scene
    scene = detector.analyze_scene(test_image)
    print(f"Scene: {scene['description']}")
