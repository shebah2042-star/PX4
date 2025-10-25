"""
Warhead Separation System

This module provides comprehensive warhead separation and terminal attack modes:
- Warhead separation with configurable timing
- Multiple separation triggers (time, altitude, distance to target)
- Terminal attack modes (glide, fragmentation, dive)
- Mass and aerodynamic updates after separation
- Enable/disable warhead separation functionality

Author: Devin
Date: 2025-10-25
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple
from enum import Enum
import yaml
import logging


class SeparationTrigger(Enum):
    """Warhead separation trigger types"""
    TIME = "time"                      # Separate at specific time
    ALTITUDE = "altitude"              # Separate at specific altitude
    DISTANCE = "distance"              # Separate at distance from target
    APOGEE = "apogee"                  # Separate at apogee
    MANUAL = "manual"                  # Manual separation command


class TerminalMode(Enum):
    """Terminal attack modes after separation"""
    NONE = "none"                      # No special mode, ballistic
    GLIDE = "glide"                    # Glide toward target with control
    DIVE = "dive"                      # Vertical dive on target
    FRAGMENTATION = "fragmentation"    # Fragment into multiple submunitions
    MANEUVER = "maneuver"              # Maneuvering reentry


class WarheadState(Enum):
    """Warhead states"""
    ATTACHED = "attached"              # Warhead attached to vehicle
    SEPARATING = "separating"          # Separation in progress
    SEPARATED = "separated"            # Warhead separated
    TERMINAL = "terminal"              # In terminal attack phase
    IMPACT = "impact"                  # Warhead impacted


@dataclass
class WarheadConfiguration:
    """Warhead configuration"""
    mass: float = 10.0                 # Warhead mass (kg)
    length: float = 0.5                # Warhead length (m)
    diameter: float = 0.15             # Warhead diameter (m)
    
    cg_position: np.ndarray = None     # CG position [x, y, z] (m)
    
    drag_coefficient: float = 0.3      # Drag coefficient
    lift_coefficient: float = 0.0      # Lift coefficient (for glide mode)
    reference_area: float = 0.0177     # Reference area (m²)
    
    def __post_init__(self):
        if self.cg_position is None:
            self.cg_position = np.array([self.length / 2, 0.0, 0.0])
        if self.reference_area == 0.0177:  # Default value
            self.reference_area = np.pi * (self.diameter / 2) ** 2


@dataclass
class SeparationConfiguration:
    """Separation configuration"""
    enabled: bool = False              # Enable/disable separation
    
    trigger_type: SeparationTrigger = SeparationTrigger.TIME
    trigger_value: float = 60.0        # Trigger value (s, m, or m depending on type)
    
    separation_impulse: float = 100.0  # Separation impulse (N·s)
    separation_direction: np.ndarray = None  # Separation direction (unit vector)
    
    terminal_mode: TerminalMode = TerminalMode.NONE
    
    glide_ratio: float = 3.0           # L/D ratio for glide
    glide_angle: float = -30.0         # Glide angle (degrees, negative = down)
    
    dive_angle: float = -90.0          # Dive angle (degrees, -90 = vertical)
    dive_start_altitude: float = 1000.0  # Altitude to start dive (m)
    
    num_fragments: int = 10            # Number of fragments
    fragment_spread_angle: float = 30.0  # Spread angle (degrees)
    fragment_velocity: float = 50.0    # Fragment ejection velocity (m/s)
    
    def __post_init__(self):
        if self.separation_direction is None:
            self.separation_direction = np.array([1.0, 0.0, 0.0])


@dataclass
class Fragment:
    """Individual fragment after fragmentation"""
    id: int                            # Fragment ID
    mass: float                        # Fragment mass (kg)
    position: np.ndarray               # Position [x, y, z] (m)
    velocity: np.ndarray               # Velocity [vx, vy, vz] (m/s)
    drag_coefficient: float = 0.5      # Drag coefficient
    reference_area: float = 0.01       # Reference area (m²)


class WarheadSeparationSystem:
    """
    Warhead separation and terminal attack system
    
    Handles:
    - Warhead separation with multiple trigger types
    - Terminal attack modes (glide, dive, fragmentation)
    - Mass and aerodynamic updates after separation
    - Fragment tracking for fragmentation mode
    """
    
    def __init__(self, 
                 warhead_config: Optional[WarheadConfiguration] = None,
                 separation_config: Optional[SeparationConfiguration] = None):
        """
        Initialize warhead separation system
        
        Args:
            warhead_config: Warhead configuration
            separation_config: Separation configuration
        """
        self.logger = logging.getLogger(__name__)
        
        self.warhead_config = warhead_config or WarheadConfiguration()
        self.separation_config = separation_config or SeparationConfiguration()
        
        self.state = WarheadState.ATTACHED
        self.mission_time = 0.0
        self.separation_time: Optional[float] = None
        self.terminal_start_time: Optional[float] = None
        
        self.warhead_position = np.array([0.0, 0.0, 0.0])
        self.warhead_velocity = np.array([0.0, 0.0, 0.0])
        
        self.fragments: List[Fragment] = []
        
        self.target_position: Optional[np.ndarray] = None
        
        self.logger.info(f"Warhead separation system initialized: enabled={self.separation_config.enabled}, "
                        f"trigger={self.separation_config.trigger_type.value}")
    
    @classmethod
    def from_yaml(cls, config_file: str) -> 'WarheadSeparationSystem':
        """
        Load configuration from YAML file
        
        Args:
            config_file: Path to YAML configuration file
            
        Returns:
            WarheadSeparationSystem instance
        """
        with open(config_file, 'r') as f:
            config_dict = yaml.safe_load(f)
        
        warhead_dict = config_dict.get('warhead', {})
        warhead_config = WarheadConfiguration(
            mass=warhead_dict.get('mass', 10.0),
            length=warhead_dict.get('length', 0.5),
            diameter=warhead_dict.get('diameter', 0.15),
            drag_coefficient=warhead_dict.get('drag_coefficient', 0.3),
            lift_coefficient=warhead_dict.get('lift_coefficient', 0.0),
            reference_area=warhead_dict.get('reference_area', 0.0177)
        )
        
        sep_dict = config_dict.get('separation', {})
        separation_config = SeparationConfiguration(
            enabled=sep_dict.get('enabled', False),
            trigger_type=SeparationTrigger(sep_dict.get('trigger_type', 'time')),
            trigger_value=sep_dict.get('trigger_value', 60.0),
            separation_impulse=sep_dict.get('separation_impulse', 100.0),
            terminal_mode=TerminalMode(sep_dict.get('terminal_mode', 'none')),
            glide_ratio=sep_dict.get('glide_ratio', 3.0),
            glide_angle=sep_dict.get('glide_angle', -30.0),
            dive_angle=sep_dict.get('dive_angle', -90.0),
            dive_start_altitude=sep_dict.get('dive_start_altitude', 1000.0),
            num_fragments=sep_dict.get('num_fragments', 10),
            fragment_spread_angle=sep_dict.get('fragment_spread_angle', 30.0),
            fragment_velocity=sep_dict.get('fragment_velocity', 50.0)
        )
        
        return cls(warhead_config, separation_config)
    
    def update(self, dt: float, 
               vehicle_position: np.ndarray,
               vehicle_velocity: np.ndarray,
               altitude: float,
               target_position: Optional[np.ndarray] = None) -> Dict[str, Any]:
        """
        Update warhead separation system
        
        Args:
            dt: Time step (s)
            vehicle_position: Vehicle position [x, y, z] (m)
            vehicle_velocity: Vehicle velocity [vx, vy, vz] (m/s)
            altitude: Current altitude (m)
            target_position: Target position [x, y, z] (m), optional
            
        Returns:
            Dictionary with system status
        """
        self.mission_time += dt
        
        if not self.separation_config.enabled:
            return self._get_status()
        
        if target_position is not None:
            self.target_position = target_position
        
        if self.state == WarheadState.ATTACHED:
            if self._check_separation_trigger(altitude, vehicle_position):
                self._separate_warhead(vehicle_position, vehicle_velocity)
        
        if self.state in [WarheadState.SEPARATED, WarheadState.TERMINAL]:
            self._update_warhead_dynamics(dt, altitude)
        
        if self.state == WarheadState.SEPARATED:
            if self._check_terminal_start(altitude):
                self._start_terminal_phase()
        
        if self.separation_config.terminal_mode == TerminalMode.FRAGMENTATION:
            self._update_fragments(dt, altitude)
        
        return self._get_status()
    
    def _check_separation_trigger(self, altitude: float, position: np.ndarray) -> bool:
        """Check if separation should be triggered"""
        trigger = self.separation_config.trigger_type
        value = self.separation_config.trigger_value
        
        if trigger == SeparationTrigger.TIME:
            return self.mission_time >= value
        
        elif trigger == SeparationTrigger.ALTITUDE:
            return altitude >= value
        
        elif trigger == SeparationTrigger.DISTANCE:
            if self.target_position is not None:
                distance = np.linalg.norm(position - self.target_position)
                return distance <= value
            return False
        
        elif trigger == SeparationTrigger.APOGEE:
            return False
        
        return False
    
    def _separate_warhead(self, position: np.ndarray, velocity: np.ndarray):
        """Separate warhead from vehicle"""
        self.state = WarheadState.SEPARATING
        self.separation_time = self.mission_time
        
        self.warhead_position = position.copy()
        self.warhead_velocity = velocity.copy()
        
        separation_dv = (self.separation_config.separation_impulse / 
                        self.warhead_config.mass * 
                        self.separation_config.separation_direction)
        self.warhead_velocity += separation_dv
        
        self.state = WarheadState.SEPARATED
        
        self.logger.info(f"Warhead separated at t={self.mission_time:.2f}s, "
                        f"altitude={position[2]:.1f}m")
    
    def _check_terminal_start(self, altitude: float) -> bool:
        """Check if terminal phase should start"""
        mode = self.separation_config.terminal_mode
        
        if mode == TerminalMode.DIVE:
            return altitude <= self.separation_config.dive_start_altitude
        
        elif mode == TerminalMode.FRAGMENTATION:
            return True
        
        elif mode == TerminalMode.GLIDE:
            return True
        
        return False
    
    def _start_terminal_phase(self):
        """Start terminal attack phase"""
        self.state = WarheadState.TERMINAL
        self.terminal_start_time = self.mission_time
        
        mode = self.separation_config.terminal_mode
        
        if mode == TerminalMode.FRAGMENTATION:
            self._create_fragments()
        
        self.logger.info(f"Terminal phase started: mode={mode.value}, "
                        f"t={self.mission_time:.2f}s")
    
    def _create_fragments(self):
        """Create fragments for fragmentation mode"""
        num_fragments = self.separation_config.num_fragments
        spread_angle = np.radians(self.separation_config.fragment_spread_angle)
        fragment_velocity = self.separation_config.fragment_velocity
        
        fragment_mass = self.warhead_config.mass / num_fragments
        
        for i in range(num_fragments):
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, spread_angle)
            
            direction = np.array([
                np.cos(phi),
                np.sin(phi) * np.cos(theta),
                np.sin(phi) * np.sin(theta)
            ])
            
            frag_velocity = self.warhead_velocity + direction * fragment_velocity
            
            fragment = Fragment(
                id=i,
                mass=fragment_mass,
                position=self.warhead_position.copy(),
                velocity=frag_velocity,
                drag_coefficient=0.5,
                reference_area=0.01
            )
            
            self.fragments.append(fragment)
        
        self.logger.info(f"Created {num_fragments} fragments")
    
    def _update_warhead_dynamics(self, dt: float, altitude: float):
        """Update warhead position and velocity"""
        
        g = 9.81
        gravity_force = np.array([0.0, 0.0, -g * self.warhead_config.mass])
        
        velocity_magnitude = np.linalg.norm(self.warhead_velocity)
        if velocity_magnitude > 0:
            rho = 1.225 * np.exp(-altitude / 8500.0)
            
            drag_force = -0.5 * rho * velocity_magnitude**2 * \
                        self.warhead_config.drag_coefficient * \
                        self.warhead_config.reference_area * \
                        (self.warhead_velocity / velocity_magnitude)
        else:
            drag_force = np.array([0.0, 0.0, 0.0])
        
        if self.state == WarheadState.TERMINAL:
            if self.separation_config.terminal_mode == TerminalMode.GLIDE:
                if velocity_magnitude > 0:
                    lift_direction = np.array([0.0, 0.0, 1.0])
                    lift_force = 0.5 * rho * velocity_magnitude**2 * \
                                self.warhead_config.lift_coefficient * \
                                self.warhead_config.reference_area * lift_direction
                    drag_force += lift_force
            
            elif self.separation_config.terminal_mode == TerminalMode.DIVE:
                dive_velocity = np.array([0.0, 0.0, -velocity_magnitude])
                self.warhead_velocity = 0.9 * self.warhead_velocity + 0.1 * dive_velocity
        
        total_force = gravity_force + drag_force
        
        acceleration = total_force / self.warhead_config.mass
        self.warhead_velocity += acceleration * dt
        self.warhead_position += self.warhead_velocity * dt
    
    def _update_fragments(self, dt: float, altitude: float):
        """Update fragment positions"""
        g = 9.81
        rho = 1.225 * np.exp(-altitude / 8500.0)
        
        for fragment in self.fragments:
            gravity_force = np.array([0.0, 0.0, -g * fragment.mass])
            
            velocity_magnitude = np.linalg.norm(fragment.velocity)
            if velocity_magnitude > 0:
                drag_force = -0.5 * rho * velocity_magnitude**2 * \
                            fragment.drag_coefficient * \
                            fragment.reference_area * \
                            (fragment.velocity / velocity_magnitude)
            else:
                drag_force = np.array([0.0, 0.0, 0.0])
            
            total_force = gravity_force + drag_force
            acceleration = total_force / fragment.mass
            fragment.velocity += acceleration * dt
            fragment.position += fragment.velocity * dt
    
    def trigger_separation_manual(self, position: np.ndarray, velocity: np.ndarray):
        """Manually trigger warhead separation"""
        if self.state == WarheadState.ATTACHED:
            self._separate_warhead(position, velocity)
    
    def get_warhead_mass(self) -> float:
        """Get warhead mass (0 if separated)"""
        if self.state == WarheadState.ATTACHED:
            return self.warhead_config.mass
        return 0.0
    
    def is_separated(self) -> bool:
        """Check if warhead is separated"""
        return self.state != WarheadState.ATTACHED
    
    def _get_status(self) -> Dict[str, Any]:
        """Get system status"""
        status = {
            'enabled': self.separation_config.enabled,
            'state': self.state.value,
            'mission_time': self.mission_time,
            'separation_time': self.separation_time,
            'terminal_start_time': self.terminal_start_time,
            'terminal_mode': self.separation_config.terminal_mode.value,
            'warhead_mass': self.get_warhead_mass(),
            'warhead_position': self.warhead_position.tolist() if self.is_separated() else None,
            'warhead_velocity': self.warhead_velocity.tolist() if self.is_separated() else None,
            'num_fragments': len(self.fragments)
        }
        
        if self.fragments:
            status['fragments'] = [
                {
                    'id': f.id,
                    'position': f.position.tolist(),
                    'velocity': f.velocity.tolist()
                }
                for f in self.fragments[:5]  # Only first 5 for brevity
            ]
        
        return status


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    print("Warhead Separation System Example")
    print("=" * 50)
    
    print("\nExample 1: Glide Mode")
    warhead_config = WarheadConfiguration(
        mass=10.0,
        length=0.5,
        diameter=0.15,
        lift_coefficient=0.5
    )
    
    separation_config = SeparationConfiguration(
        enabled=True,
        trigger_type=SeparationTrigger.TIME,
        trigger_value=60.0,
        terminal_mode=TerminalMode.GLIDE,
        glide_ratio=3.0
    )
    
    system = WarheadSeparationSystem(warhead_config, separation_config)
    
    position = np.array([0.0, 0.0, 5000.0])
    velocity = np.array([200.0, 0.0, 0.0])
    
    for i in range(700):
        t = i * 0.1
        status = system.update(0.1, position, velocity, position[2])
        
        if i % 100 == 0:
            print(f"t={t:.1f}s: state={status['state']}, "
                  f"mass={status['warhead_mass']:.1f}kg")
    
    print("\nExample 2: Fragmentation Mode")
    separation_config2 = SeparationConfiguration(
        enabled=True,
        trigger_type=SeparationTrigger.ALTITUDE,
        trigger_value=3000.0,
        terminal_mode=TerminalMode.FRAGMENTATION,
        num_fragments=10,
        fragment_spread_angle=30.0
    )
    
    system2 = WarheadSeparationSystem(warhead_config, separation_config2)
    
    position = np.array([0.0, 0.0, 3500.0])
    velocity = np.array([200.0, 0.0, -50.0])
    
    for i in range(100):
        t = i * 0.1
        status = system2.update(0.1, position, velocity, position[2])
        position[2] -= 5.0  # Simulate descent
        
        if i % 20 == 0:
            print(f"t={t:.1f}s: state={status['state']}, "
                  f"fragments={status['num_fragments']}")
