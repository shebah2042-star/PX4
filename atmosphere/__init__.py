"""
Atmosphere module for atmospheric modeling

This module provides standard atmosphere models for high-altitude flight.
"""

from .standard_atmosphere import StandardAtmosphere, AtmosphereModel

__all__ = ['StandardAtmosphere', 'AtmosphereModel']
