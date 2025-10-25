"""
Standard Atmosphere Model (ISA/US Standard Atmosphere 1976)

This module provides comprehensive atmospheric modeling for high-altitude flight including:
- Temperature, pressure, density profiles up to 86 km
- Speed of sound calculations
- Dynamic pressure calculations
- Support for both ISA and simplified exponential models

Author: Rocket Control System
Date: 2025-10-25
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional
import yaml


class AtmosphereModel(Enum):
    """Atmosphere model types"""
    STANDARD = "standard"  # US Standard Atmosphere 1976
    SIMPLE = "simple"  # Simple exponential model


@dataclass
class AtmosphericConditions:
    """Atmospheric conditions at a given altitude"""
    altitude: float  # Geometric altitude (m)
    temperature: float  # Temperature (K)
    pressure: float  # Pressure (Pa)
    density: float  # Density (kg/m³)
    speed_of_sound: float  # Speed of sound (m/s)
    dynamic_viscosity: float  # Dynamic viscosity (Pa·s)
    kinematic_viscosity: float  # Kinematic viscosity (m²/s)


class StandardAtmosphere:
    """
    US Standard Atmosphere 1976 implementation
    
    Provides accurate atmospheric properties from sea level to 86 km altitude.
    Based on the US Standard Atmosphere 1976 model.
    
    Features:
    - Accurate temperature, pressure, density profiles
    - Speed of sound calculations
    - Dynamic and kinematic viscosity
    - Geopotential altitude corrections
    - Fallback to simple exponential model
    """
    
    R = 287.05287  # Specific gas constant for air (J/(kg·K))
    GAMMA = 1.4  # Ratio of specific heats for air
    G0 = 9.80665  # Standard gravity (m/s²)
    RE = 6356766.0  # Earth radius (m)
    
    T0 = 288.15  # Sea level temperature (K)
    P0 = 101325.0  # Sea level pressure (Pa)
    RHO0 = 1.225  # Sea level density (kg/m³)
    
    LAYERS = [
        (0.0, 288.15, -6.5),      # Troposphere
        (11.0, 216.65, 0.0),      # Tropopause
        (20.0, 216.65, 1.0),      # Stratosphere 1
        (32.0, 228.65, 2.8),      # Stratosphere 2
        (47.0, 270.65, 0.0),      # Stratopause
        (51.0, 270.65, -2.8),     # Mesosphere 1
        (71.0, 214.65, -2.0),     # Mesosphere 2
        (84.852, 186.946, 0.0),   # Mesopause
    ]
    
    def __init__(self, model_type: AtmosphereModel = AtmosphereModel.STANDARD):
        """
        Initialize atmosphere model
        
        Args:
            model_type: Type of atmosphere model to use
        """
        self.model_type = model_type
        
        self.simple_scale_height = 8500.0  # Scale height (m)
        self.simple_temp_lapse = 0.0065  # Temperature lapse rate (K/m)
    
    def get_conditions(self, altitude: float) -> AtmosphericConditions:
        """
        Get atmospheric conditions at given altitude
        
        Args:
            altitude: Geometric altitude (m)
        
        Returns:
            AtmosphericConditions object
        """
        if self.model_type == AtmosphereModel.STANDARD:
            return self._get_standard_conditions(altitude)
        else:
            return self._get_simple_conditions(altitude)
    
    def _get_standard_conditions(self, altitude: float) -> AtmosphericConditions:
        """Get conditions using US Standard Atmosphere 1976"""
        h_geopotential = self._geometric_to_geopotential(altitude)
        
        temperature = self._get_temperature(h_geopotential)
        pressure = self._get_pressure(h_geopotential, temperature)
        
        density = pressure / (self.R * temperature)
        
        speed_of_sound = np.sqrt(self.GAMMA * self.R * temperature)
        
        dynamic_viscosity = self._get_dynamic_viscosity(temperature)
        kinematic_viscosity = dynamic_viscosity / density if density > 0 else 0.0
        
        return AtmosphericConditions(
            altitude=altitude,
            temperature=temperature,
            pressure=pressure,
            density=density,
            speed_of_sound=speed_of_sound,
            dynamic_viscosity=dynamic_viscosity,
            kinematic_viscosity=kinematic_viscosity
        )
    
    def _get_simple_conditions(self, altitude: float) -> AtmosphericConditions:
        """Get conditions using simple exponential model"""
        temperature = self.T0 - self.simple_temp_lapse * altitude
        temperature = max(temperature, 216.65)  # Minimum temperature
        
        pressure = self.P0 * np.exp(-altitude / self.simple_scale_height)
        
        density = pressure / (self.R * temperature)
        
        speed_of_sound = np.sqrt(self.GAMMA * self.R * temperature)
        
        dynamic_viscosity = self._get_dynamic_viscosity(temperature)
        kinematic_viscosity = dynamic_viscosity / density if density > 0 else 0.0
        
        return AtmosphericConditions(
            altitude=altitude,
            temperature=temperature,
            pressure=pressure,
            density=density,
            speed_of_sound=speed_of_sound,
            dynamic_viscosity=dynamic_viscosity,
            kinematic_viscosity=kinematic_viscosity
        )
    
    def _geometric_to_geopotential(self, h_geometric: float) -> float:
        """
        Convert geometric altitude to geopotential altitude
        
        Args:
            h_geometric: Geometric altitude (m)
        
        Returns:
            Geopotential altitude (km)
        """
        h_geopotential = (self.RE * h_geometric) / (self.RE + h_geometric)
        return h_geopotential / 1000.0  # Convert to km
    
    def _get_temperature(self, h_geopotential: float) -> float:
        """
        Get temperature at geopotential altitude
        
        Args:
            h_geopotential: Geopotential altitude (km)
        
        Returns:
            Temperature (K)
        """
        for i in range(len(self.LAYERS) - 1):
            h_base, T_base, lapse_rate = self.LAYERS[i]
            h_next = self.LAYERS[i + 1][0]
            
            if h_geopotential <= h_next:
                dh = h_geopotential - h_base
                temperature = T_base + lapse_rate * dh
                return temperature
        
        return self.LAYERS[-1][1]
    
    def _get_pressure(self, h_geopotential: float, temperature: float) -> float:
        """
        Get pressure at geopotential altitude
        
        Args:
            h_geopotential: Geopotential altitude (km)
            temperature: Temperature at altitude (K)
        
        Returns:
            Pressure (Pa)
        """
        pressure = self.P0
        
        for i in range(len(self.LAYERS) - 1):
            h_base, T_base, lapse_rate = self.LAYERS[i]
            h_next = self.LAYERS[i + 1][0]
            
            if h_geopotential <= h_base:
                break
            
            dh = min(h_geopotential, h_next) - h_base
            
            if abs(lapse_rate) < 1e-6:
                pressure *= np.exp(-self.G0 * dh * 1000.0 / (self.R * T_base))
            else:
                T_top = T_base + lapse_rate * dh
                pressure *= (T_top / T_base) ** (-self.G0 / (lapse_rate * self.R / 1000.0))
            
            if h_geopotential <= h_next:
                break
        
        return pressure
    
    def _get_dynamic_viscosity(self, temperature: float) -> float:
        """
        Calculate dynamic viscosity using Sutherland's formula
        
        Args:
            temperature: Temperature (K)
        
        Returns:
            Dynamic viscosity (Pa·s)
        """
        T_ref = 288.15  # Reference temperature (K)
        mu_ref = 1.7894e-5  # Reference viscosity (Pa·s)
        S = 110.4  # Sutherland's constant (K)
        
        mu = mu_ref * (temperature / T_ref) ** 1.5 * (T_ref + S) / (temperature + S)
        
        return mu
    
    def get_density(self, altitude: float) -> float:
        """Get density at altitude"""
        conditions = self.get_conditions(altitude)
        return conditions.density
    
    def get_pressure(self, altitude: float) -> float:
        """Get pressure at altitude"""
        conditions = self.get_conditions(altitude)
        return conditions.pressure
    
    def get_temperature(self, altitude: float) -> float:
        """Get temperature at altitude"""
        conditions = self.get_conditions(altitude)
        return conditions.temperature
    
    def get_speed_of_sound(self, altitude: float) -> float:
        """Get speed of sound at altitude"""
        conditions = self.get_conditions(altitude)
        return conditions.speed_of_sound
    
    def get_mach_number(self, velocity: float, altitude: float) -> float:
        """
        Calculate Mach number
        
        Args:
            velocity: Velocity (m/s)
            altitude: Altitude (m)
        
        Returns:
            Mach number
        """
        speed_of_sound = self.get_speed_of_sound(altitude)
        return velocity / speed_of_sound if speed_of_sound > 0 else 0.0
    
    def get_dynamic_pressure(self, velocity: float, altitude: float) -> float:
        """
        Calculate dynamic pressure
        
        Args:
            velocity: Velocity (m/s)
            altitude: Altitude (m)
        
        Returns:
            Dynamic pressure (Pa)
        """
        density = self.get_density(altitude)
        return 0.5 * density * velocity ** 2
    
    def get_reynolds_number(self, velocity: float, altitude: float, 
                           characteristic_length: float) -> float:
        """
        Calculate Reynolds number
        
        Args:
            velocity: Velocity (m/s)
            altitude: Altitude (m)
            characteristic_length: Characteristic length (m)
        
        Returns:
            Reynolds number
        """
        conditions = self.get_conditions(altitude)
        
        if conditions.kinematic_viscosity > 0:
            return velocity * characteristic_length / conditions.kinematic_viscosity
        return 0.0
    
    def get_status(self, altitude: float, velocity: float) -> dict:
        """Get comprehensive atmospheric status"""
        conditions = self.get_conditions(altitude)
        
        return {
            'model_type': self.model_type.value,
            'altitude': altitude,
            'temperature': conditions.temperature,
            'pressure': conditions.pressure,
            'density': conditions.density,
            'speed_of_sound': conditions.speed_of_sound,
            'mach_number': self.get_mach_number(velocity, altitude),
            'dynamic_pressure': self.get_dynamic_pressure(velocity, altitude),
            'dynamic_viscosity': conditions.dynamic_viscosity,
            'kinematic_viscosity': conditions.kinematic_viscosity,
        }
