"""
Propulsion module for rocket thrust modeling

This module provides thrust curve modeling and mass depletion calculations.
"""

from .thrust_model import ThrustModel, ThrustCurveType

__all__ = ['ThrustModel', 'ThrustCurveType']
