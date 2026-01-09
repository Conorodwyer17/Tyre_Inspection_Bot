"""
Detection Modules

Contains vehicle tracking, license plate detection, and tyre re-detection handling.
"""

from tyre_inspection_mission.detection.vehicle_tracker import VehicleTracker
from tyre_inspection_mission.detection.license_plate_detector import LicensePlateDetector
from tyre_inspection_mission.detection.tyre_re_detection_handler import TyreReDetectionHandler
from tyre_inspection_mission.detection.tyre_identifier import TyreIdentifier
from tyre_inspection_mission.detection.tyre_position_validator import TyrePositionValidator
from tyre_inspection_mission.detection.tyre_completeness_verifier import TyreCompletenessVerifier
from tyre_inspection_mission.detection.tyre_side_identifier import TyreSideIdentifier

__all__ = [
    'VehicleTracker',
    'LicensePlateDetector',
    'TyreReDetectionHandler',
    'TyreIdentifier',
    'TyrePositionValidator',
    'TyreCompletenessVerifier',
    'TyreSideIdentifier',
]
