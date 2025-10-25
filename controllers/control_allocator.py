"""
Control Allocator for Rocket Control Surfaces and TVC

This module provides a comprehensive control allocation system that supports:
- Canard-only configuration
- Tail fin-only configuration
- Combined canard + tail configuration
- Thrust Vector Control (TVC) with enable/disable option
- Control surface effectiveness modeling based on dynamic pressure, Mach, and AoA
- Proper mixing and distribution of control commands

Author: Rocket Control System
Date: 2025-10-25
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import yaml


class SurfaceType(Enum):
    """Control surface types"""
    CANARD = "canard"
    TAIL = "tail"


class SurfaceConfiguration(Enum):
    """Overall control surface configuration"""
    CANARD_ONLY = "canard_only"
    TAIL_ONLY = "tail_only"
    CANARD_AND_TAIL = "canard_and_tail"


@dataclass
class ControlSurface:
    """Individual control surface definition"""
    surface_type: SurfaceType
    index: int  # 0-3 for quad configuration
    channel: int  # PWM channel number
    angle_offset: float  # Mounting angle offset (degrees)
    max_deflection: float  # Maximum deflection angle (degrees)
    min_deflection: float  # Minimum deflection angle (degrees)
    effectiveness_factor: float = 1.0  # Base effectiveness multiplier
    position_x: float = 0.0  # X position from CG (meters, positive forward)
    position_y: float = 0.0  # Y position from CG (meters)
    position_z: float = 0.0  # Z position from CG (meters)


@dataclass
class TVCConfiguration:
    """Thrust Vector Control configuration"""
    enabled: bool = False
    pitch_channel: int = 9  # PWM channel for pitch gimbal
    yaw_channel: int = 10  # PWM channel for yaw gimbal
    max_pitch_angle: float = 10.0  # Maximum pitch deflection (degrees)
    max_yaw_angle: float = 10.0  # Maximum yaw deflection (degrees)
    min_pitch_angle: float = -10.0  # Minimum pitch deflection (degrees)
    min_yaw_angle: float = -10.0  # Minimum yaw deflection (degrees)
    pitch_pwm_min: int = 1000  # PWM microseconds
    pitch_pwm_max: int = 2000
    yaw_pwm_min: int = 1000
    yaw_pwm_max: int = 2000
    pitch_pwm_center: int = 1500
    yaw_pwm_center: int = 1500
    effectiveness_factor: float = 1.0  # Base effectiveness multiplier
    position_x: float = -1.0  # X position from CG (meters, typically behind CG)


@dataclass
class ControlEffectiveness:
    """Control effectiveness parameters based on flight conditions"""
    mach_effectiveness: float = 1.0  # Effectiveness multiplier based on Mach
    dynamic_pressure_effectiveness: float = 1.0  # Effectiveness based on q
    aoa_effectiveness: float = 1.0  # Effectiveness based on angle of attack
    combined_effectiveness: float = 1.0  # Combined effectiveness


class ControlAllocator:
    """
    Control allocator that distributes control commands to surfaces and TVC
    
    Features:
    - Supports multiple surface configurations (canards, tail, both)
    - Optional TVC with configurable parameters
    - Control surface effectiveness modeling
    - Rate limiting and saturation protection
    - Proper moment arm calculations
    - Anti-windup protection
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize control allocator
        
        Args:
            config_file: Path to YAML configuration file
        """
        self.surfaces: List[ControlSurface] = []
        self.tvc_config: Optional[TVCConfiguration] = None
        self.configuration: SurfaceConfiguration = SurfaceConfiguration.TAIL_ONLY
        
        self.mach_effectiveness_curve: List[Tuple[float, float]] = [
            (0.0, 1.0),    # Subsonic: full effectiveness
            (0.8, 1.0),    # High subsonic
            (1.0, 0.7),    # Transonic: reduced effectiveness
            (1.2, 0.8),    # Low supersonic
            (2.0, 0.9),    # Supersonic
            (5.0, 0.85),   # Hypersonic
        ]
        
        self.q_effectiveness_curve: List[Tuple[float, float]] = [
            (0.0, 0.1),      # Very low q: minimal effectiveness
            (500.0, 0.5),    # Low q
            (2000.0, 1.0),   # Nominal q: full effectiveness
            (10000.0, 1.0),  # High q: full effectiveness
            (50000.0, 0.9),  # Very high q: slight reduction due to flutter
        ]
        
        self.aoa_effectiveness_curve: List[Tuple[float, float]] = [
            (0.0, 1.0),      # Zero AoA: full effectiveness
            (5.0, 1.0),      # Small AoA
            (10.0, 0.95),    # Moderate AoA
            (15.0, 0.85),    # High AoA
            (20.0, 0.7),     # Very high AoA: reduced effectiveness
            (30.0, 0.5),     # Stall region
        ]
        
        self.max_deflection_rate: float = 200.0  # degrees/second
        self.max_tvc_rate: float = 100.0  # degrees/second
        
        self.previous_surface_commands: Dict[int, float] = {}
        self.previous_tvc_pitch: float = 0.0
        self.previous_tvc_yaw: float = 0.0
        
        self.canard_weight: float = 0.5  # Weight for canards when both present
        self.tail_weight: float = 0.5    # Weight for tail when both present
        self.tvc_weight: float = 0.3     # Weight for TVC when enabled
        
        if config_file:
            self.load_configuration(config_file)
    
    def load_configuration(self, config_file: str):
        """Load configuration from YAML file"""
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        allocator_config = config.get('control_allocator', {})
        
        config_type = allocator_config.get('configuration', 'tail_only')
        self.configuration = SurfaceConfiguration(config_type)
        
        self.surfaces = []
        for surface_config in allocator_config.get('surfaces', []):
            surface = ControlSurface(
                surface_type=SurfaceType(surface_config['type']),
                index=surface_config['index'],
                channel=surface_config['channel'],
                angle_offset=surface_config.get('angle_offset', 0.0),
                max_deflection=surface_config.get('max_deflection', 30.0),
                min_deflection=surface_config.get('min_deflection', -30.0),
                effectiveness_factor=surface_config.get('effectiveness_factor', 1.0),
                position_x=surface_config.get('position_x', 0.0),
                position_y=surface_config.get('position_y', 0.0),
                position_z=surface_config.get('position_z', 0.0),
            )
            self.surfaces.append(surface)
        
        tvc_config = allocator_config.get('tvc', {})
        if tvc_config.get('enabled', False):
            self.tvc_config = TVCConfiguration(
                enabled=True,
                pitch_channel=tvc_config.get('pitch_channel', 9),
                yaw_channel=tvc_config.get('yaw_channel', 10),
                max_pitch_angle=tvc_config.get('max_pitch_angle', 10.0),
                max_yaw_angle=tvc_config.get('max_yaw_angle', 10.0),
                min_pitch_angle=tvc_config.get('min_pitch_angle', -10.0),
                min_yaw_angle=tvc_config.get('min_yaw_angle', -10.0),
                pitch_pwm_min=tvc_config.get('pitch_pwm_min', 1000),
                pitch_pwm_max=tvc_config.get('pitch_pwm_max', 2000),
                yaw_pwm_min=tvc_config.get('yaw_pwm_min', 1000),
                yaw_pwm_max=tvc_config.get('yaw_pwm_max', 2000),
                pitch_pwm_center=tvc_config.get('pitch_pwm_center', 1500),
                yaw_pwm_center=tvc_config.get('yaw_pwm_center', 1500),
                effectiveness_factor=tvc_config.get('effectiveness_factor', 1.0),
                position_x=tvc_config.get('position_x', -1.0),
            )
        
        mixing = allocator_config.get('mixing_weights', {})
        self.canard_weight = mixing.get('canard', 0.5)
        self.tail_weight = mixing.get('tail', 0.5)
        self.tvc_weight = mixing.get('tvc', 0.3)
        
        limits = allocator_config.get('rate_limits', {})
        self.max_deflection_rate = limits.get('surface_rate', 200.0)
        self.max_tvc_rate = limits.get('tvc_rate', 100.0)
        
        if 'effectiveness_curves' in allocator_config:
            curves = allocator_config['effectiveness_curves']
            if 'mach' in curves:
                self.mach_effectiveness_curve = [(p['mach'], p['effectiveness']) 
                                                  for p in curves['mach']]
            if 'dynamic_pressure' in curves:
                self.q_effectiveness_curve = [(p['q'], p['effectiveness']) 
                                               for p in curves['dynamic_pressure']]
            if 'angle_of_attack' in curves:
                self.aoa_effectiveness_curve = [(p['aoa'], p['effectiveness']) 
                                                 for p in curves['angle_of_attack']]
    
    def calculate_effectiveness(self, mach: float, dynamic_pressure: float, 
                                angle_of_attack: float) -> ControlEffectiveness:
        """
        Calculate control effectiveness based on flight conditions
        
        Args:
            mach: Mach number
            dynamic_pressure: Dynamic pressure (Pa)
            angle_of_attack: Angle of attack (degrees)
        
        Returns:
            ControlEffectiveness object with all effectiveness factors
        """
        mach_eff = np.interp(mach, 
                             [p[0] for p in self.mach_effectiveness_curve],
                             [p[1] for p in self.mach_effectiveness_curve])
        
        q_eff = np.interp(dynamic_pressure,
                          [p[0] for p in self.q_effectiveness_curve],
                          [p[1] for p in self.q_effectiveness_curve])
        
        aoa_eff = np.interp(abs(angle_of_attack),
                            [p[0] for p in self.aoa_effectiveness_curve],
                            [p[1] for p in self.aoa_effectiveness_curve])
        
        combined = mach_eff * q_eff * aoa_eff
        
        return ControlEffectiveness(
            mach_effectiveness=mach_eff,
            dynamic_pressure_effectiveness=q_eff,
            aoa_effectiveness=aoa_eff,
            combined_effectiveness=combined
        )
    
    def allocate_control(self, roll_command: float, pitch_command: float, 
                        yaw_command: float, mach: float, dynamic_pressure: float,
                        angle_of_attack: float, dt: float) -> Dict[int, float]:
        """
        Allocate control commands to surfaces and TVC
        
        Args:
            roll_command: Desired roll moment (normalized -1 to 1)
            pitch_command: Desired pitch moment (normalized -1 to 1)
            yaw_command: Desired yaw moment (normalized -1 to 1)
            mach: Current Mach number
            dynamic_pressure: Current dynamic pressure (Pa)
            angle_of_attack: Current angle of attack (degrees)
            dt: Time step (seconds)
        
        Returns:
            Dictionary mapping channel numbers to deflection angles (degrees)
        """
        effectiveness = self.calculate_effectiveness(mach, dynamic_pressure, angle_of_attack)
        
        actuator_commands = {}
        
        if self.configuration == SurfaceConfiguration.CANARD_ONLY:
            surface_commands = self._allocate_to_surfaces(
                roll_command, pitch_command, yaw_command,
                SurfaceType.CANARD, effectiveness.combined_effectiveness
            )
            actuator_commands.update(surface_commands)
        
        elif self.configuration == SurfaceConfiguration.TAIL_ONLY:
            surface_commands = self._allocate_to_surfaces(
                roll_command, pitch_command, yaw_command,
                SurfaceType.TAIL, effectiveness.combined_effectiveness
            )
            actuator_commands.update(surface_commands)
        
        elif self.configuration == SurfaceConfiguration.CANARD_AND_TAIL:
            canard_commands = self._allocate_to_surfaces(
                roll_command * self.canard_weight,
                pitch_command * self.canard_weight,
                yaw_command * self.canard_weight,
                SurfaceType.CANARD, effectiveness.combined_effectiveness
            )
            tail_commands = self._allocate_to_surfaces(
                roll_command * self.tail_weight,
                pitch_command * self.tail_weight,
                yaw_command * self.tail_weight,
                SurfaceType.TAIL, effectiveness.combined_effectiveness
            )
            actuator_commands.update(canard_commands)
            actuator_commands.update(tail_commands)
        
        if self.tvc_config and self.tvc_config.enabled:
            tvc_commands = self._allocate_to_tvc(
                pitch_command * self.tvc_weight,
                yaw_command * self.tvc_weight,
                effectiveness.combined_effectiveness
            )
            actuator_commands.update(tvc_commands)
        
        actuator_commands = self._apply_rate_limiting(actuator_commands, dt)
        
        return actuator_commands
    
    def _allocate_to_surfaces(self, roll_cmd: float, pitch_cmd: float, 
                             yaw_cmd: float, surface_type: SurfaceType,
                             effectiveness: float) -> Dict[int, float]:
        """Allocate commands to a specific surface type (canard or tail)"""
        commands = {}
        
        surfaces = [s for s in self.surfaces if s.surface_type == surface_type]
        
        if len(surfaces) == 4:
            
            deflections = [
                pitch_cmd - roll_cmd - yaw_cmd,  # Surface 0
                -pitch_cmd - roll_cmd + yaw_cmd,  # Surface 1
                pitch_cmd + roll_cmd + yaw_cmd,   # Surface 2
                -pitch_cmd + roll_cmd - yaw_cmd,  # Surface 3
            ]
            
            for i, surface in enumerate(surfaces):
                deflection = deflections[i] * effectiveness * surface.effectiveness_factor
                
                max_deflection = max(abs(surface.max_deflection), abs(surface.min_deflection))
                deflection_deg = deflection * max_deflection
                
                deflection_deg = np.clip(deflection_deg, 
                                        surface.min_deflection, 
                                        surface.max_deflection)
                
                commands[surface.channel] = deflection_deg
        
        return commands
    
    def _allocate_to_tvc(self, pitch_cmd: float, yaw_cmd: float,
                        effectiveness: float) -> Dict[int, float]:
        """Allocate commands to TVC"""
        if not self.tvc_config:
            return {}
        
        commands = {}
        
        pitch_angle = pitch_cmd * effectiveness * self.tvc_config.effectiveness_factor
        yaw_angle = yaw_cmd * effectiveness * self.tvc_config.effectiveness_factor
        
        pitch_deg = pitch_angle * max(abs(self.tvc_config.max_pitch_angle),
                                      abs(self.tvc_config.min_pitch_angle))
        yaw_deg = yaw_angle * max(abs(self.tvc_config.max_yaw_angle),
                                  abs(self.tvc_config.min_yaw_angle))
        
        pitch_deg = np.clip(pitch_deg, 
                           self.tvc_config.min_pitch_angle,
                           self.tvc_config.max_pitch_angle)
        yaw_deg = np.clip(yaw_deg,
                         self.tvc_config.min_yaw_angle,
                         self.tvc_config.max_yaw_angle)
        
        commands[self.tvc_config.pitch_channel] = pitch_deg
        commands[self.tvc_config.yaw_channel] = yaw_deg
        
        return commands
    
    def _apply_rate_limiting(self, commands: Dict[int, float], dt: float) -> Dict[int, float]:
        """Apply rate limiting to prevent excessive actuator rates"""
        limited_commands = {}
        
        for channel, command in commands.items():
            previous = self.previous_surface_commands.get(channel, 0.0)
            
            if self.tvc_config and channel in [self.tvc_config.pitch_channel, 
                                               self.tvc_config.yaw_channel]:
                max_rate = self.max_tvc_rate
            else:
                max_rate = self.max_deflection_rate
            
            max_change = max_rate * dt
            
            change = command - previous
            if abs(change) > max_change:
                limited_command = previous + np.sign(change) * max_change
            else:
                limited_command = command
            
            limited_commands[channel] = limited_command
            self.previous_surface_commands[channel] = limited_command
        
        return limited_commands
    
    def angles_to_pwm(self, angle_commands: Dict[int, float]) -> Dict[int, int]:
        """
        Convert angle commands to PWM values
        
        Args:
            angle_commands: Dictionary mapping channels to angles (degrees)
        
        Returns:
            Dictionary mapping channels to PWM values (microseconds)
        """
        pwm_commands = {}
        
        for channel, angle in angle_commands.items():
            if self.tvc_config and channel == self.tvc_config.pitch_channel:
                pwm = self._angle_to_pwm(
                    angle,
                    self.tvc_config.min_pitch_angle,
                    self.tvc_config.max_pitch_angle,
                    self.tvc_config.pitch_pwm_min,
                    self.tvc_config.pitch_pwm_max,
                    self.tvc_config.pitch_pwm_center
                )
                pwm_commands[channel] = pwm
            
            elif self.tvc_config and channel == self.tvc_config.yaw_channel:
                pwm = self._angle_to_pwm(
                    angle,
                    self.tvc_config.min_yaw_angle,
                    self.tvc_config.max_yaw_angle,
                    self.tvc_config.yaw_pwm_min,
                    self.tvc_config.yaw_pwm_max,
                    self.tvc_config.yaw_pwm_center
                )
                pwm_commands[channel] = pwm
            
            else:
                surface = next((s for s in self.surfaces if s.channel == channel), None)
                if surface:
                    pwm = self._angle_to_pwm(
                        angle,
                        surface.min_deflection,
                        surface.max_deflection,
                        1000, 2000, 1500
                    )
                    pwm_commands[channel] = pwm
        
        return pwm_commands
    
    def _angle_to_pwm(self, angle: float, min_angle: float, max_angle: float,
                     pwm_min: int, pwm_max: int, pwm_center: int) -> int:
        """Convert angle to PWM value"""
        if angle >= 0:
            normalized = angle / max_angle if max_angle != 0 else 0
            pwm = pwm_center + normalized * (pwm_max - pwm_center)
        else:
            normalized = angle / min_angle if min_angle != 0 else 0
            pwm = pwm_center - normalized * (pwm_center - pwm_min)
        
        return int(np.clip(pwm, pwm_min, pwm_max))
    
    def get_status(self) -> dict:
        """Get current allocator status"""
        return {
            'configuration': self.configuration.value,
            'num_surfaces': len(self.surfaces),
            'tvc_enabled': self.tvc_config.enabled if self.tvc_config else False,
            'canard_weight': self.canard_weight,
            'tail_weight': self.tail_weight,
            'tvc_weight': self.tvc_weight,
            'max_deflection_rate': self.max_deflection_rate,
            'max_tvc_rate': self.max_tvc_rate,
        }
