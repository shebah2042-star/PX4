"""
Thrust Curve Model with Mass Depletion

This module provides comprehensive thrust modeling including:
- Multiple thrust curve formats (.eng, CSV, YAML)
- Time-based and pressure-based thrust curves
- Mass depletion and CG shift calculations
- Propellant consumption tracking
- Support for multi-stage rockets

Author: Rocket Control System
Date: 2025-10-25
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import yaml
import csv


class ThrustCurveType(Enum):
    """Thrust curve data format types"""
    TIME_BASED = "time_based"  # Thrust vs time
    PRESSURE_BASED = "pressure_based"  # Thrust vs chamber pressure
    CONSTANT = "constant"  # Constant thrust


@dataclass
class ThrustCurvePoint:
    """Single point on thrust curve"""
    time: float  # Time from ignition (seconds)
    thrust: float  # Thrust force (Newtons)


@dataclass
class MotorParameters:
    """Motor/engine parameters"""
    name: str = "Generic Motor"
    manufacturer: str = "Unknown"
    total_impulse: float = 0.0  # Total impulse (N·s)
    burn_time: float = 0.0  # Burn time (seconds)
    average_thrust: float = 0.0  # Average thrust (N)
    max_thrust: float = 0.0  # Maximum thrust (N)
    propellant_mass: float = 0.0  # Propellant mass (kg)
    motor_mass: float = 0.0  # Empty motor mass (kg)
    diameter: float = 0.0  # Motor diameter (meters)
    length: float = 0.0  # Motor length (meters)


@dataclass
class VehicleMassProperties:
    """Vehicle mass and CG properties"""
    dry_mass: float  # Mass without propellant (kg)
    propellant_mass: float  # Initial propellant mass (kg)
    current_mass: float  # Current total mass (kg)
    cg_dry: float  # CG position when dry (m from nose)
    cg_wet: float  # CG position when fully fueled (m from nose)
    current_cg: float  # Current CG position (m from nose)
    cp_position: float  # Center of pressure position (m from nose)


class ThrustModel:
    """
    Comprehensive thrust curve model with mass depletion
    
    Features:
    - Load thrust curves from multiple formats
    - Interpolate thrust at any time
    - Track propellant consumption
    - Calculate mass and CG changes
    - Support for multi-stage rockets
    - Pressure-based thrust curves
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize thrust model
        
        Args:
            config_file: Path to YAML configuration file
        """
        self.thrust_curve: List[ThrustCurvePoint] = []
        self.motor_params: MotorParameters = MotorParameters()
        self.mass_properties: Optional[VehicleMassProperties] = None
        self.curve_type: ThrustCurveType = ThrustCurveType.TIME_BASED
        
        self.ignition_time: Optional[float] = None
        self.current_time: float = 0.0
        self.propellant_consumed: float = 0.0
        self.is_burning: bool = False
        
        self.stages: List[Dict] = []
        self.current_stage: int = 0
        
        if config_file:
            self.load_configuration(config_file)
    
    def load_configuration(self, config_file: str):
        """Load configuration from YAML file"""
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        thrust_config = config.get('thrust_model', {})
        
        self.curve_type = ThrustCurveType(thrust_config.get('curve_type', 'time_based'))
        
        if 'curve_file' in thrust_config:
            curve_file = thrust_config['curve_file']
            if curve_file.endswith('.eng'):
                self.load_eng_file(curve_file)
            elif curve_file.endswith('.csv'):
                self.load_csv_file(curve_file)
            elif curve_file.endswith('.yaml') or curve_file.endswith('.yml'):
                self.load_yaml_curve(curve_file)
        elif 'curve_points' in thrust_config:
            self.thrust_curve = [
                ThrustCurvePoint(p['time'], p['thrust'])
                for p in thrust_config['curve_points']
            ]
        
        motor_config = thrust_config.get('motor_parameters', {})
        self.motor_params = MotorParameters(
            name=motor_config.get('name', 'Generic Motor'),
            manufacturer=motor_config.get('manufacturer', 'Unknown'),
            total_impulse=motor_config.get('total_impulse', 0.0),
            burn_time=motor_config.get('burn_time', 0.0),
            average_thrust=motor_config.get('average_thrust', 0.0),
            max_thrust=motor_config.get('max_thrust', 0.0),
            propellant_mass=motor_config.get('propellant_mass', 0.0),
            motor_mass=motor_config.get('motor_mass', 0.0),
            diameter=motor_config.get('diameter', 0.0),
            length=motor_config.get('length', 0.0),
        )
        
        if self.thrust_curve and self.motor_params.total_impulse == 0.0:
            self._calculate_motor_parameters()
        
        mass_config = thrust_config.get('mass_properties', {})
        if mass_config:
            self.mass_properties = VehicleMassProperties(
                dry_mass=mass_config.get('dry_mass', 0.0),
                propellant_mass=mass_config.get('propellant_mass', 
                                               self.motor_params.propellant_mass),
                current_mass=mass_config.get('dry_mass', 0.0) + 
                            mass_config.get('propellant_mass', 
                                          self.motor_params.propellant_mass),
                cg_dry=mass_config.get('cg_dry', 0.0),
                cg_wet=mass_config.get('cg_wet', 0.0),
                current_cg=mass_config.get('cg_wet', 0.0),
                cp_position=mass_config.get('cp_position', 0.0),
            )
    
    def load_eng_file(self, filename: str):
        """
        Load RASP .eng format thrust curve
        
        Format:
        ; comments
        <name> <diameter> <length> <delays> <propellant_mass> <total_mass> <manufacturer>
        <time1> <thrust1>
        <time2> <thrust2>
        ...
        """
        self.thrust_curve = []
        
        with open(filename, 'r') as f:
            lines = f.readlines()
        
        for line in lines:
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            
            parts = line.split()
            if len(parts) >= 7:
                self.motor_params.name = parts[0]
                self.motor_params.diameter = float(parts[1]) / 1000.0  # mm to m
                self.motor_params.length = float(parts[2]) / 1000.0  # mm to m
                self.motor_params.propellant_mass = float(parts[4])
                total_mass = float(parts[5])
                self.motor_params.motor_mass = total_mass - self.motor_params.propellant_mass
                self.motor_params.manufacturer = parts[6]
                break
        
        in_data = False
        for line in lines:
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            
            parts = line.split()
            if len(parts) == 2:
                try:
                    time = float(parts[0])
                    thrust = float(parts[1])
                    self.thrust_curve.append(ThrustCurvePoint(time, thrust))
                    in_data = True
                except ValueError:
                    if in_data:
                        break
        
        self._calculate_motor_parameters()
    
    def load_csv_file(self, filename: str):
        """
        Load CSV format thrust curve
        
        Format:
        time,thrust
        0.0,0.0
        0.1,100.0
        ...
        """
        self.thrust_curve = []
        
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                time = float(row['time'])
                thrust = float(row['thrust'])
                self.thrust_curve.append(ThrustCurvePoint(time, thrust))
        
        self._calculate_motor_parameters()
    
    def load_yaml_curve(self, filename: str):
        """Load YAML format thrust curve"""
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
        
        if 'motor' in data:
            motor = data['motor']
            self.motor_params = MotorParameters(
                name=motor.get('name', 'Generic Motor'),
                manufacturer=motor.get('manufacturer', 'Unknown'),
                propellant_mass=motor.get('propellant_mass', 0.0),
                motor_mass=motor.get('motor_mass', 0.0),
                diameter=motor.get('diameter', 0.0),
                length=motor.get('length', 0.0),
            )
        
        if 'thrust_curve' in data:
            self.thrust_curve = [
                ThrustCurvePoint(p['time'], p['thrust'])
                for p in data['thrust_curve']
            ]
        
        self._calculate_motor_parameters()
    
    def _calculate_motor_parameters(self):
        """Calculate motor parameters from thrust curve"""
        if not self.thrust_curve:
            return
        
        self.thrust_curve.sort(key=lambda p: p.time)
        
        total_impulse = 0.0
        for i in range(len(self.thrust_curve) - 1):
            dt = self.thrust_curve[i + 1].time - self.thrust_curve[i].time
            avg_thrust = (self.thrust_curve[i].thrust + self.thrust_curve[i + 1].thrust) / 2.0
            total_impulse += avg_thrust * dt
        
        self.motor_params.total_impulse = total_impulse
        self.motor_params.burn_time = self.thrust_curve[-1].time
        self.motor_params.average_thrust = total_impulse / self.motor_params.burn_time if self.motor_params.burn_time > 0 else 0.0
        self.motor_params.max_thrust = max(p.thrust for p in self.thrust_curve)
    
    def ignite(self, current_time: float):
        """
        Ignite the motor
        
        Args:
            current_time: Current simulation time (seconds)
        """
        self.ignition_time = current_time
        self.is_burning = True
        self.propellant_consumed = 0.0
    
    def update(self, current_time: float, dt: float) -> float:
        """
        Update thrust model and return current thrust
        
        Args:
            current_time: Current simulation time (seconds)
            dt: Time step (seconds)
        
        Returns:
            Current thrust (Newtons)
        """
        self.current_time = current_time
        
        if not self.is_burning or self.ignition_time is None:
            return 0.0
        
        time_since_ignition = current_time - self.ignition_time
        
        thrust = self.get_thrust(time_since_ignition)
        
        if thrust > 0 and self.motor_params.total_impulse > 0:
            impulse_rate = thrust  # N = N·s/s
            mass_flow_rate = (self.motor_params.propellant_mass / 
                            self.motor_params.total_impulse) * impulse_rate
            
            self.propellant_consumed += mass_flow_rate * dt
            self.propellant_consumed = min(self.propellant_consumed, 
                                          self.motor_params.propellant_mass)
            
            if self.mass_properties:
                self._update_mass_properties()
        
        if time_since_ignition >= self.motor_params.burn_time:
            self.is_burning = False
        
        return thrust
    
    def get_thrust(self, time_since_ignition: float) -> float:
        """
        Get thrust at specific time since ignition
        
        Args:
            time_since_ignition: Time since motor ignition (seconds)
        
        Returns:
            Thrust (Newtons)
        """
        if not self.thrust_curve:
            return 0.0
        
        if time_since_ignition < 0:
            return 0.0
        
        if time_since_ignition > self.motor_params.burn_time:
            return 0.0
        
        times = [p.time for p in self.thrust_curve]
        thrusts = [p.thrust for p in self.thrust_curve]
        
        thrust = np.interp(time_since_ignition, times, thrusts)
        
        return max(0.0, thrust)
    
    def _update_mass_properties(self):
        """Update vehicle mass and CG based on propellant consumption"""
        if not self.mass_properties:
            return
        
        remaining_propellant = (self.motor_params.propellant_mass - 
                               self.propellant_consumed)
        
        self.mass_properties.current_mass = (self.mass_properties.dry_mass + 
                                            remaining_propellant)
        
        if self.motor_params.propellant_mass > 0:
            propellant_fraction = remaining_propellant / self.motor_params.propellant_mass
            self.mass_properties.current_cg = (
                self.mass_properties.cg_dry + 
                propellant_fraction * (self.mass_properties.cg_wet - 
                                      self.mass_properties.cg_dry)
            )
    
    def get_mass(self) -> float:
        """Get current vehicle mass"""
        if self.mass_properties:
            return self.mass_properties.current_mass
        return 0.0
    
    def get_cg(self) -> float:
        """Get current center of gravity position"""
        if self.mass_properties:
            return self.mass_properties.current_cg
        return 0.0
    
    def get_static_margin(self) -> float:
        """
        Calculate static margin (stability margin)
        
        Returns:
            Static margin as fraction of body diameter
        """
        if not self.mass_properties or self.motor_params.diameter == 0:
            return 0.0
        
        margin = ((self.mass_properties.cp_position - 
                  self.mass_properties.current_cg) / 
                 self.motor_params.diameter)
        
        return margin
    
    def get_propellant_fraction(self) -> float:
        """Get remaining propellant as fraction of initial"""
        if self.motor_params.propellant_mass == 0:
            return 0.0
        
        remaining = self.motor_params.propellant_mass - self.propellant_consumed
        return remaining / self.motor_params.propellant_mass
    
    def get_burn_fraction(self) -> float:
        """Get burn progress as fraction (0 to 1)"""
        if not self.is_burning or self.ignition_time is None:
            return 0.0 if self.propellant_consumed == 0 else 1.0
        
        time_since_ignition = self.current_time - self.ignition_time
        
        if self.motor_params.burn_time == 0:
            return 1.0
        
        return min(1.0, time_since_ignition / self.motor_params.burn_time)
    
    def create_constant_thrust_curve(self, thrust: float, burn_time: float,
                                     propellant_mass: float):
        """
        Create a constant thrust curve
        
        Args:
            thrust: Constant thrust (Newtons)
            burn_time: Burn duration (seconds)
            propellant_mass: Propellant mass (kg)
        """
        self.thrust_curve = [
            ThrustCurvePoint(0.0, 0.0),
            ThrustCurvePoint(0.01, thrust),
            ThrustCurvePoint(burn_time - 0.01, thrust),
            ThrustCurvePoint(burn_time, 0.0),
        ]
        
        self.motor_params.propellant_mass = propellant_mass
        self._calculate_motor_parameters()
        self.curve_type = ThrustCurveType.CONSTANT
    
    def create_typical_curve(self, peak_thrust: float, burn_time: float,
                            propellant_mass: float, curve_shape: str = "progressive"):
        """
        Create a typical thrust curve shape
        
        Args:
            peak_thrust: Peak thrust (Newtons)
            burn_time: Burn duration (seconds)
            propellant_mass: Propellant mass (kg)
            curve_shape: "progressive", "regressive", or "neutral"
        """
        num_points = 50
        times = np.linspace(0, burn_time, num_points)
        
        if curve_shape == "progressive":
            thrusts = peak_thrust * (times / burn_time) ** 0.5
        elif curve_shape == "regressive":
            thrusts = peak_thrust * (1 - (times / burn_time) ** 0.5)
        else:  # neutral
            thrusts = np.full(num_points, peak_thrust * 0.8)
        
        thrusts[0] = 0.0
        thrusts[-1] = 0.0
        
        self.thrust_curve = [
            ThrustCurvePoint(t, thrust)
            for t, thrust in zip(times, thrusts)
        ]
        
        self.motor_params.propellant_mass = propellant_mass
        self._calculate_motor_parameters()
    
    def get_status(self) -> dict:
        """Get current thrust model status"""
        status = {
            'is_burning': self.is_burning,
            'current_thrust': self.get_thrust(
                self.current_time - self.ignition_time if self.ignition_time else 0.0
            ),
            'burn_fraction': self.get_burn_fraction(),
            'propellant_fraction': self.get_propellant_fraction(),
            'propellant_consumed': self.propellant_consumed,
            'motor_name': self.motor_params.name,
            'total_impulse': self.motor_params.total_impulse,
            'burn_time': self.motor_params.burn_time,
            'average_thrust': self.motor_params.average_thrust,
            'max_thrust': self.motor_params.max_thrust,
        }
        
        if self.mass_properties:
            status.update({
                'current_mass': self.mass_properties.current_mass,
                'current_cg': self.mass_properties.current_cg,
                'static_margin': self.get_static_margin(),
            })
        
        return status
