#!/usr/bin/env python3
"""
Photo Quality Checker Module

Verifies photo quality after capture to ensure usable images.
Checks for blur, darkness, proper framing, and object visibility.

This module provides:
- Photo quality assessment
- Blur detection
- Brightness/contrast checks
- Object visibility verification
- Quality score calculation
"""

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    np = None

from pathlib import Path
from typing import Optional, Dict


class PhotoQualityChecker:
    """
    Checks photo quality to ensure usable images.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize photo quality checker.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.min_brightness = parameters.get('photo_min_brightness', 30)  # 0-255
        self.max_brightness = parameters.get('photo_max_brightness', 240)  # 0-255
        self.min_contrast = parameters.get('photo_min_contrast', 20)  # contrast threshold
        self.min_sharpness = parameters.get('photo_min_sharpness', 100)  # Laplacian variance threshold
        self.min_file_size_kb = parameters.get('photo_min_file_size_kb', 10)  # minimum file size
        
        self.logger.info(
            f"PhotoQualityChecker initialized: "
            f"brightness=[{self.min_brightness}, {self.max_brightness}], "
            f"min_contrast={self.min_contrast}, "
            f"min_sharpness={self.min_sharpness}"
        )
    
    def check_photo_quality(
        self,
        photo_path: str
    ) -> Dict:
        """
        Check photo quality.
        
        Args:
            photo_path: Path to photo file
            
        Returns:
            Dict with:
                - 'quality_passed': bool - Overall quality check passed
                - 'score': float - Quality score (0-100)
                - 'is_blurry': bool - Is photo blurry?
                - 'is_too_dark': bool - Is photo too dark?
                - 'is_too_bright': bool - Is photo too bright?
                - 'has_low_contrast': bool - Has low contrast?
                - 'file_valid': bool - Is file valid and readable?
                - 'file_size_kb': float - File size in KB
                - 'brightness': float - Average brightness (0-255)
                - 'contrast': float - Contrast measure
                - 'sharpness': float - Sharpness measure (Laplacian variance)
                - 'issues': List[str] - List of quality issues found
        """
        result = {
            'quality_passed': False,
            'score': 0.0,
            'is_blurry': False,
            'is_too_dark': False,
            'is_too_bright': False,
            'has_low_contrast': False,
            'file_valid': False,
            'file_size_kb': 0.0,
            'brightness': 0.0,
            'contrast': 0.0,
            'sharpness': 0.0,
            'issues': []
        }
        
        if not CV2_AVAILABLE:
            result['issues'].append("OpenCV not available - cannot check photo quality")
            result['quality_passed'] = True  # Pass by default if OpenCV unavailable
            self.logger.warn("OpenCV not available. Skipping photo quality check.")
            return result
        
        try:
            photo_file = Path(photo_path)
            
            # Check if file exists
            if not photo_file.exists():
                result['issues'].append(f"Photo file does not exist: {photo_path}")
                return result
            
            # Check file size
            file_size_kb = photo_file.stat().st_size / 1024.0
            result['file_size_kb'] = file_size_kb
            
            if file_size_kb < self.min_file_size_kb:
                result['issues'].append(
                    f"File too small: {file_size_kb:.1f}KB < {self.min_file_size_kb}KB"
                )
                return result
            
            # Read image
            image = cv2.imread(str(photo_path))
            if image is None:
                result['issues'].append(f"Failed to read image: {photo_path}")
                return result
            
            result['file_valid'] = True
            
            # Convert to grayscale for analysis
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Check brightness
            brightness = np.mean(gray)
            result['brightness'] = brightness
            
            if brightness < self.min_brightness:
                result['is_too_dark'] = True
                result['issues'].append(
                    f"Photo too dark: brightness={brightness:.1f} < {self.min_brightness}"
                )
            elif brightness > self.max_brightness:
                result['is_too_bright'] = True
                result['issues'].append(
                    f"Photo too bright: brightness={brightness:.1f} > {self.max_brightness}"
                )
            
            # Check contrast (standard deviation of pixel intensities)
            contrast = np.std(gray)
            result['contrast'] = contrast
            
            if contrast < self.min_contrast:
                result['has_low_contrast'] = True
                result['issues'].append(
                    f"Low contrast: {contrast:.1f} < {self.min_contrast}"
                )
            
            # Check sharpness (Laplacian variance)
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            result['sharpness'] = laplacian_var
            
            if laplacian_var < self.min_sharpness:
                result['is_blurry'] = True
                result['issues'].append(
                    f"Photo is blurry: sharpness={laplacian_var:.1f} < {self.min_sharpness}"
                )
            
            # Calculate quality score (0-100)
            score = 100.0
            
            # Deduct points for issues
            if result['is_too_dark'] or result['is_too_bright']:
                score -= 30
            if result['has_low_contrast']:
                score -= 20
            if result['is_blurry']:
                score -= 40
            if not result['file_valid']:
                score = 0
            
            # Ensure score is in valid range
            score = max(0.0, min(100.0, score))
            result['score'] = score
            
            # Overall quality check (pass if score >= 50)
            result['quality_passed'] = score >= 50.0
            
            if result['quality_passed']:
                self.logger.info(
                    f"✅ Photo quality check passed: "
                    f"score={score:.1f}/100, "
                    f"brightness={brightness:.1f}, "
                    f"contrast={contrast:.1f}, "
                    f"sharpness={laplacian_var:.1f}"
                )
            else:
                self.logger.warn(
                    f"⚠️ Photo quality check failed: "
                    f"score={score:.1f}/100, "
                    f"issues: {', '.join(result['issues'])}"
                )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error checking photo quality: {e}", exc_info=True)
            result['issues'].append(f"Error during quality check: {str(e)}")
            return result
    
    def is_photo_usable(self, photo_path: str) -> bool:
        """
        Quick check if photo is usable.
        
        Args:
            photo_path: Path to photo file
            
        Returns:
            True if photo is usable, False otherwise
        """
        quality_result = self.check_photo_quality(photo_path)
        return quality_result['quality_passed']
