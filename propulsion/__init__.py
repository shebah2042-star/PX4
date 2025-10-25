"""
Propulsion module for rocket thrust modeling

This module provides thrust curve modeling, mass depletion calculations,
and multi-stage propulsion systems.
"""

from .thrust_model import ThrustModel, ThrustCurveType, MotorParameters, VehicleMassProperties
from .multi_stage_system import (
    MultiStageSystem,
    MultiStageConfiguration,
    StageConfiguration,
    StageState
)

__all__ = [
    'ThrustModel',
    'ThrustCurveType',
    'MotorParameters',
    'VehicleMassProperties',
    'MultiStageSystem',
    'MultiStageConfiguration',
    'StageConfiguration',
    'StageState'
]
