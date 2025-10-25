"""
Advanced Integration Example

This example demonstrates how to integrate all the advanced control systems:
- Control Allocator (canards/tail/TVC)
- Thrust Model with mass depletion
- Aerodynamic Model
- Standard Atmosphere Model

This shows the complete integration for a comprehensive rocket control system.

Author: Rocket Control System
Date: 2025-10-25
"""

import asyncio
import numpy as np
from typing import Tuple

from controllers.control_allocator import ControlAllocator
from propulsion.thrust_model import ThrustModel
from aero.aero_model import AeroModel
from atmosphere.standard_atmosphere import StandardAtmosphere, AtmosphereModel


class AdvancedRocketController:
    """
    Advanced rocket controller integrating all systems
    
    This class shows how to integrate:
    - Control allocation with TVC
    - Thrust modeling with mass changes
    - Aerodynamic forces and moments
    - Atmospheric conditions
    """
    
    def __init__(self, config_file: str = "config/control_systems_config.yaml"):
        """Initialize all subsystems"""
        
        self.control_allocator = ControlAllocator(config_file)
        self.thrust_model = ThrustModel(config_file)
        self.aero_model = AeroModel(config_file)
        self.atmosphere = StandardAtmosphere(AtmosphereModel.STANDARD)
        
        self.altitude = 0.0
        self.velocity = 0.0
        self.angle_of_attack = 0.0
        self.sideslip_angle = 0.0
        
        self.roll_command = 0.0
        self.pitch_command = 0.0
        self.yaw_command = 0.0
        
        self.time = 0.0
        self.dt = 0.01  # 100 Hz
    
    def update(self, altitude: float, velocity: float, 
               angle_of_attack: float, sideslip_angle: float,
               roll_cmd: float, pitch_cmd: float, yaw_cmd: float) -> dict:
        """
        Update all systems and calculate control outputs
        
        Args:
            altitude: Current altitude (m)
            velocity: Current velocity (m/s)
            angle_of_attack: Angle of attack (degrees)
            sideslip_angle: Sideslip angle (degrees)
            roll_cmd: Roll command (-1 to 1)
            pitch_cmd: Pitch command (-1 to 1)
            yaw_cmd: Yaw command (-1 to 1)
        
        Returns:
            Dictionary with all system outputs
        """
        self.altitude = altitude
        self.velocity = velocity
        self.angle_of_attack = angle_of_attack
        self.sideslip_angle = sideslip_angle
        self.roll_command = roll_cmd
        self.pitch_command = pitch_cmd
        self.yaw_command = yaw_cmd
        
        atm_conditions = self.atmosphere.get_conditions(altitude)
        mach = self.atmosphere.get_mach_number(velocity, altitude)
        dynamic_pressure = self.atmosphere.get_dynamic_pressure(velocity, altitude)
        
        thrust = self.thrust_model.update(self.time, self.dt)
        current_mass = self.thrust_model.get_mass()
        current_cg = self.thrust_model.get_cg()
        
        aero_coeffs = self.aero_model.get_coefficients(
            mach, angle_of_attack, sideslip_angle
        )
        
        aero_forces = self.aero_model.calculate_forces(
            aero_coeffs, dynamic_pressure, velocity,
            angle_of_attack, sideslip_angle
        )
        
        actuator_commands = self.control_allocator.allocate_control(
            roll_cmd, pitch_cmd, yaw_cmd,
            mach, dynamic_pressure, angle_of_attack,
            self.dt
        )
        
        pwm_commands = self.control_allocator.angles_to_pwm(actuator_commands)
        
        self.time += self.dt
        
        return {
            'time': self.time,
            'altitude': altitude,
            'velocity': velocity,
            'mach': mach,
            'dynamic_pressure': dynamic_pressure,
            'temperature': atm_conditions.temperature,
            'pressure': atm_conditions.pressure,
            'density': atm_conditions.density,
            'thrust': thrust,
            'mass': current_mass,
            'cg': current_cg,
            'CL': aero_coeffs.CL,
            'CD': aero_coeffs.CD,
            'Cm': aero_coeffs.Cm,
            'aero_force_x': aero_forces.force_x,
            'aero_force_z': aero_forces.force_z,
            'aero_moment_y': aero_forces.moment_y,
            'actuator_commands': actuator_commands,
            'pwm_commands': pwm_commands,
        }
    
    def ignite_motor(self):
        """Ignite the rocket motor"""
        self.thrust_model.ignite(self.time)
    
    def get_status(self) -> dict:
        """Get comprehensive system status"""
        return {
            'control_allocator': self.control_allocator.get_status(),
            'thrust_model': self.thrust_model.get_status(),
            'aero_model': self.aero_model.get_status(
                self.atmosphere.get_mach_number(self.velocity, self.altitude),
                self.angle_of_attack,
                self.sideslip_angle
            ),
            'atmosphere': self.atmosphere.get_status(self.altitude, self.velocity),
        }


async def example_flight_simulation():
    """
    Example flight simulation showing integration of all systems
    """
    print("=" * 80)
    print("Advanced Rocket Control System - Integration Example")
    print("=" * 80)
    
    controller = AdvancedRocketController()
    
    print("\nInitial System Configuration:")
    status = controller.get_status()
    print(f"  Control Configuration: {status['control_allocator']['configuration']}")
    print(f"  TVC Enabled: {status['control_allocator']['tvc_enabled']}")
    print(f"  Motor: {status['thrust_model']['motor_name']}")
    print(f"  Total Impulse: {status['thrust_model']['total_impulse']:.1f} N·s")
    print(f"  Atmosphere Model: {status['atmosphere']['model_type']}")
    
    print("\n" + "=" * 80)
    print("Flight Simulation")
    print("=" * 80)
    
    await asyncio.sleep(0.1)
    controller.ignite_motor()
    print("\n🚀 Motor Ignited!")
    
    altitude = 0.0
    velocity = 0.0
    angle_of_attack = 2.0  # degrees
    sideslip_angle = 0.0
    
    roll_cmd = 0.0
    pitch_cmd = 0.1  # Small pitch command
    yaw_cmd = 0.0
    
    simulation_time = 10.0
    dt = 0.1  # 10 Hz for display
    
    print("\nTime(s) | Alt(m) | Vel(m/s) | Mach | Thrust(N) | Mass(kg) | CL    | CD    | Fin1(°)")
    print("-" * 90)
    
    for t in np.arange(0, simulation_time, dt):
        result = controller.update(
            altitude, velocity, angle_of_attack, sideslip_angle,
            roll_cmd, pitch_cmd, yaw_cmd
        )
        
        if result['thrust'] > 0:
            drag_force = 0.5 * result['density'] * velocity**2 * result['CD'] * 0.00785
            net_force = result['thrust'] - drag_force - result['mass'] * 9.81
            acceleration = net_force / result['mass'] if result['mass'] > 0 else 0
            
            velocity += acceleration * dt
            altitude += velocity * dt
        
        if abs(t - round(t)) < dt/2:
            fin1_angle = result['actuator_commands'].get(1, 0.0)
            print(f"{t:7.1f} | {altitude:6.1f} | {velocity:8.2f} | {result['mach']:4.2f} | "
                  f"{result['thrust']:9.1f} | {result['mass']:8.2f} | "
                  f"{result['CL']:5.3f} | {result['CD']:5.3f} | {fin1_angle:6.2f}")
        
        await asyncio.sleep(0.001)  # Small delay for async
    
    print("\n" + "=" * 80)
    print("Simulation Complete")
    print("=" * 80)
    
    final_status = controller.get_status()
    print(f"\nFinal Altitude: {altitude:.1f} m")
    print(f"Final Velocity: {velocity:.1f} m/s")
    print(f"Propellant Consumed: {final_status['thrust_model']['propellant_consumed']:.2f} kg")
    print(f"Burn Fraction: {final_status['thrust_model']['burn_fraction']*100:.1f}%")


async def example_tvc_configuration():
    """
    Example showing TVC configuration
    """
    print("\n" + "=" * 80)
    print("TVC Configuration Example")
    print("=" * 80)
    
    controller = AdvancedRocketController()
    
    status = controller.get_status()
    
    print(f"\nTVC Status: {status['control_allocator']['tvc_enabled']}")
    print(f"Control Configuration: {status['control_allocator']['configuration']}")
    print(f"Mixing Weights:")
    print(f"  Canard: {status['control_allocator']['canard_weight']}")
    print(f"  Tail: {status['control_allocator']['tail_weight']}")
    print(f"  TVC: {status['control_allocator']['tvc_weight']}")
    
    print("\nControl Allocation Test:")
    print("  Command: Roll=0.5, Pitch=0.3, Yaw=0.2")
    
    actuator_commands = controller.control_allocator.allocate_control(
        roll_command=0.5,
        pitch_command=0.3,
        yaw_command=0.2,
        mach=0.5,
        dynamic_pressure=5000.0,
        angle_of_attack=5.0,
        dt=0.01
    )
    
    print("\nActuator Commands:")
    for channel, angle in actuator_commands.items():
        print(f"  Channel {channel}: {angle:6.2f}°")


async def example_high_speed_flight():
    """
    Example showing high-speed flight (supersonic/hypersonic)
    """
    print("\n" + "=" * 80)
    print("High-Speed Flight Example")
    print("=" * 80)
    
    controller = AdvancedRocketController()
    
    test_conditions = [
        (0.5, 5000, "Subsonic"),
        (0.9, 10000, "High Subsonic"),
        (1.1, 15000, "Supersonic"),
        (2.0, 20000, "Supersonic"),
        (3.0, 25000, "Supersonic"),
        (5.0, 30000, "Hypersonic"),
    ]
    
    print("\nMach | Alt(m) | Regime      | Temp(K) | Density | Speed of Sound | CL    | CD   ")
    print("-" * 85)
    
    for mach, altitude, regime in test_conditions:
        speed_of_sound = controller.atmosphere.get_speed_of_sound(altitude)
        velocity = mach * speed_of_sound
        
        atm = controller.atmosphere.get_conditions(altitude)
        aero_coeffs = controller.aero_model.get_coefficients(mach, 5.0, 0.0)
        
        print(f"{mach:4.1f} | {altitude:6.0f} | {regime:11s} | {atm.temperature:7.2f} | "
              f"{atm.density:7.4f} | {atm.speed_of_sound:14.2f} | "
              f"{aero_coeffs.CL:5.3f} | {aero_coeffs.CD:5.3f}")


async def main():
    """Run all examples"""
    await example_flight_simulation()
    
    await example_tvc_configuration()
    
    await example_high_speed_flight()
    
    print("\n" + "=" * 80)
    print("All Examples Complete!")
    print("=" * 80)


if __name__ == "__main__":
    asyncio.run(main())
