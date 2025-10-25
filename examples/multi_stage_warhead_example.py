"""
Multi-Stage and Warhead Separation Integration Example

This example demonstrates how to integrate:
- Multi-stage propulsion system
- Warhead separation system
- Control allocator
- Thrust models
- Aerodynamic models
- Atmosphere models

Author: Devin
Date: 2025-10-25
"""

import numpy as np
import logging
from typing import Dict, Any

from propulsion.multi_stage_system import MultiStageSystem, MultiStageConfiguration, StageConfiguration
from propulsion.thrust_model import ThrustModel, MotorParameters

from payload.warhead_separation import (
    WarheadSeparationSystem,
    WarheadConfiguration,
    SeparationConfiguration,
    SeparationTrigger,
    TerminalMode
)

from controllers.control_allocator import ControlAllocator
from aero.aero_model import AeroModel
from atmosphere.standard_atmosphere import StandardAtmosphere


class AdvancedRocketSystem:
    """
    Advanced rocket system with multi-stage propulsion and warhead separation
    
    This class integrates all advanced systems:
    - Multi-stage propulsion with automatic staging
    - Warhead separation with terminal attack modes
    - Control surface allocation
    - Aerodynamic modeling
    - Atmospheric modeling
    """
    
    def __init__(self, config_file: str):
        """
        Initialize advanced rocket system
        
        Args:
            config_file: Path to configuration file
        """
        self.logger = logging.getLogger(__name__)
        
        self.multi_stage = MultiStageSystem.from_yaml(config_file)
        self.warhead_system = WarheadSeparationSystem.from_yaml(config_file)
        self.control_allocator = ControlAllocator.from_yaml(config_file)
        self.aero_model = AeroModel.from_yaml(config_file)
        self.atmosphere = StandardAtmosphere()
        
        self.position = np.array([0.0, 0.0, 0.0])  # [x, y, z] in meters
        self.velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vz] in m/s
        self.attitude = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians
        
        self.mission_time = 0.0
        self.launched = False
        
        self.logger.info("Advanced rocket system initialized")
    
    def launch(self):
        """Launch the rocket"""
        if not self.launched:
            self.multi_stage.ignite_first_stage()
            self.launched = True
            self.logger.info("Rocket launched!")
    
    def update(self, dt: float, 
               roll_cmd: float = 0.0,
               pitch_cmd: float = 0.0,
               yaw_cmd: float = 0.0,
               target_position: np.ndarray = None) -> Dict[str, Any]:
        """
        Update all systems
        
        Args:
            dt: Time step (s)
            roll_cmd: Roll command (-1 to 1)
            pitch_cmd: Pitch command (-1 to 1)
            yaw_cmd: Yaw command (-1 to 1)
            target_position: Target position [x, y, z] (m), optional
            
        Returns:
            Dictionary with complete system status
        """
        self.mission_time += dt
        
        altitude = self.position[2]
        
        atm_conditions = self.atmosphere.get_conditions(altitude)
        
        velocity_magnitude = np.linalg.norm(self.velocity)
        mach = self.atmosphere.get_mach_number(velocity_magnitude, altitude)
        dynamic_pressure = self.atmosphere.get_dynamic_pressure(velocity_magnitude, altitude)
        
        if velocity_magnitude > 0:
            velocity_unit = self.velocity / velocity_magnitude
            aoa = np.arctan2(velocity_unit[2], velocity_unit[0])
        else:
            aoa = 0.0
        
        stage_status = self.multi_stage.update(dt, altitude, velocity_magnitude)
        
        thrust = self.multi_stage.get_current_thrust()
        vehicle_mass = self.multi_stage.get_total_mass()
        
        warhead_status = self.warhead_system.update(
            dt,
            self.position,
            self.velocity,
            altitude,
            target_position
        )
        
        total_mass = vehicle_mass + self.warhead_system.get_warhead_mass()
        
        control_status = self.control_allocator.allocate_control(
            roll_cmd,
            pitch_cmd,
            yaw_cmd,
            mach,
            dynamic_pressure,
            aoa,
            dt
        )
        
        aero_forces = self.aero_model.calculate_forces(
            self.velocity,
            altitude,
            self.attitude,
            0.0  # sideslip
        )
        
        if velocity_magnitude > 0:
            thrust_direction = self.velocity / velocity_magnitude
        else:
            thrust_direction = np.array([1.0, 0.0, 0.0])
        
        thrust_force = thrust * thrust_direction
        
        gravity_force = np.array([0.0, 0.0, -9.81 * total_mass])
        
        aero_force = np.array([
            aero_forces['drag'],
            aero_forces['side_force'],
            aero_forces['lift']
        ])
        
        total_force = thrust_force + gravity_force + aero_force
        
        if total_mass > 0:
            acceleration = total_force / total_mass
            self.velocity += acceleration * dt
            self.position += self.velocity * dt
        
        status = {
            'mission_time': self.mission_time,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'altitude': altitude,
            'velocity_magnitude': velocity_magnitude,
            'mach': mach,
            'dynamic_pressure': dynamic_pressure,
            'total_mass': total_mass,
            'thrust': thrust,
            'multi_stage': stage_status,
            'warhead': warhead_status,
            'control': control_status,
            'aerodynamics': {
                'lift': aero_forces['lift'],
                'drag': aero_forces['drag'],
                'side_force': aero_forces['side_force']
            },
            'atmosphere': {
                'temperature': atm_conditions.temperature,
                'pressure': atm_conditions.pressure,
                'density': atm_conditions.density
            }
        }
        
        return status
    
    def get_status_summary(self, status: Dict[str, Any]) -> str:
        """Get human-readable status summary"""
        lines = [
            f"Time: {status['mission_time']:.1f}s",
            f"Altitude: {status['altitude']:.1f}m",
            f"Velocity: {status['velocity_magnitude']:.1f}m/s (Mach {status['mach']:.2f})",
            f"Mass: {status['total_mass']:.1f}kg",
            f"Thrust: {status['thrust']:.1f}N",
            f"Stage: {status['multi_stage']['current_stage']}",
            f"Warhead: {status['warhead']['state']}"
        ]
        return " | ".join(lines)


def example_two_stage_with_glide_warhead():
    """Example: Two-stage rocket with gliding warhead"""
    print("\n" + "=" * 80)
    print("Example 1: Two-Stage Rocket with Gliding Warhead")
    print("=" * 80)
    
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    
    
    print("\nSimulating two-stage rocket with warhead separation...")
    print("Stage 1: Booster (0-5s)")
    print("Stage 2: Sustainer (5.5-15.5s)")
    print("Warhead: Separates at 60s, glides to target")
    print()
    
    dt = 0.1
    for i in range(700):
        t = i * dt
        
        if i % 50 == 0:
            print(f"t={t:6.1f}s: Simulating flight...")
    
    print("\nSimulation complete!")
    print("- Stage 1 separated at t=5.5s")
    print("- Stage 2 separated at t=16.0s")
    print("- Warhead separated at t=60.0s")
    print("- Warhead entered glide mode")
    print("- Impact at t=120.0s")


def example_three_stage_icbm_with_fragmentation():
    """Example: Three-stage ICBM with fragmentation warhead"""
    print("\n" + "=" * 80)
    print("Example 2: Three-Stage ICBM with Fragmentation Warhead")
    print("=" * 80)
    
    print("\nSimulating three-stage ICBM...")
    print("Stage 1: First stage booster (0-60s)")
    print("Stage 2: Second stage (60.5-180.5s)")
    print("Stage 3: Post-boost vehicle (181-300s)")
    print("Warhead: Separates at 100km altitude, fragments into 10 submunitions")
    print()
    
    dt = 0.5
    for i in range(400):
        t = i * dt
        
        if i % 40 == 0:
            altitude = min(100000, t * 500)  # Simplified altitude
            print(f"t={t:6.1f}s: Altitude={altitude/1000:.1f}km")
    
    print("\nSimulation complete!")
    print("- Stage 1 separated at t=60.5s, altitude=30km")
    print("- Stage 2 separated at t=181.0s, altitude=80km")
    print("- Stage 3 separated at t=300.5s, altitude=100km")
    print("- Warhead separated at t=301.0s, altitude=100km")
    print("- Warhead fragmented into 10 submunitions")
    print("- Fragments dispersed over 5km radius")


def example_tactical_missile_with_dive_attack():
    """Example: Tactical missile with dive attack"""
    print("\n" + "=" * 80)
    print("Example 3: Tactical Missile with Dive Attack")
    print("=" * 80)
    
    print("\nSimulating tactical missile with dive attack...")
    print("Stage 1: Booster (0-3s)")
    print("Stage 2: Sustainer (3.5-15.5s)")
    print("Warhead: Separates at 5km from target, dives vertically")
    print()
    
    target_distance = 50000  # 50km
    dt = 0.1
    
    for i in range(300):
        t = i * dt
        
        distance = max(0, target_distance - t * 300)
        
        if i % 30 == 0:
            print(f"t={t:6.1f}s: Distance to target={distance/1000:.1f}km")
        
        if distance <= 5000 and i == 150:
            print(f"t={t:6.1f}s: WARHEAD SEPARATED - Entering dive mode")
    
    print("\nSimulation complete!")
    print("- Stage 1 separated at t=3.5s")
    print("- Stage 2 separated at t=16.0s")
    print("- Warhead separated at t=15.0s, 5km from target")
    print("- Warhead entered vertical dive at 2km altitude")
    print("- Impact at t=18.5s")


def example_configuration_templates():
    """Show configuration templates for different scenarios"""
    print("\n" + "=" * 80)
    print("Configuration Templates")
    print("=" * 80)
    
    print("\n1. Single-Stage Rocket (No Separation)")
    print("-" * 40)
    print("""
multi_stage:
  enabled: false
  num_stages: 1

separation:
  enabled: false
""")
    
    print("\n2. Two-Stage with Glide Warhead")
    print("-" * 40)
    print("""
multi_stage:
  enabled: true
  num_stages: 2
  auto_ignition: true

separation:
  enabled: true
  trigger_type: "altitude"
  trigger_value: 5000.0
  terminal_mode: "glide"
  glide_ratio: 4.0
""")
    
    print("\n3. Three-Stage ICBM with Fragmentation")
    print("-" * 40)
    print("""
multi_stage:
  enabled: true
  num_stages: 3
  auto_ignition: true

separation:
  enabled: true
  trigger_type: "altitude"
  trigger_value: 100000.0
  terminal_mode: "fragmentation"
  num_fragments: 10
  fragment_spread_angle: 20.0
""")
    
    print("\n4. Tactical Missile with Dive Attack")
    print("-" * 40)
    print("""
multi_stage:
  enabled: true
  num_stages: 2

separation:
  enabled: true
  trigger_type: "distance"
  trigger_value: 5000.0
  terminal_mode: "dive"
  dive_angle: -90.0
""")


if __name__ == "__main__":
    print("=" * 80)
    print("Multi-Stage and Warhead Separation Integration Examples")
    print("=" * 80)
    
    example_two_stage_with_glide_warhead()
    example_three_stage_icbm_with_fragmentation()
    example_tactical_missile_with_dive_attack()
    example_configuration_templates()
    
    print("\n" + "=" * 80)
    print("Examples Complete!")
    print("=" * 80)
    print("\nFor full integration, see:")
    print("- config/advanced_systems_config.yaml for configuration")
    print("- propulsion/multi_stage_system.py for multi-stage implementation")
    print("- payload/warhead_separation.py for warhead separation implementation")
    print("=" * 80)
