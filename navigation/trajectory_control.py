"""
نظام التحكم في المسار والمدى
Trajectory Control and Range Management System
"""

import math
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass
from enum import Enum


class TrajectoryType(Enum):
    """نوع المسار"""
    BALLISTIC = "ballistic"
    GUIDED = "guided"
    LOFTED = "lofted"
    DEPRESSED = "depressed"
    OPTIMAL = "optimal"


@dataclass
class TrajectoryParameters:
    """معاملات المسار"""
    launch_angle: float  # degrees
    azimuth: float  # degrees
    initial_velocity: float  # m/s
    burn_time: float  # seconds
    coast_time: float  # seconds
    target_range: float  # meters
    target_altitude: float  # meters


@dataclass
class TrajectoryPoint:
    """نقطة على المسار"""
    time: float
    position: Tuple[float, float, float]  # x, y, z in meters
    velocity: Tuple[float, float, float]  # vx, vy, vz in m/s
    acceleration: Tuple[float, float, float]  # ax, ay, az in m/s^2


class TrajectoryControl:
    """
    نظام التحكم في المسار
    Trajectory Control System
    """
    
    def __init__(self):
        """تهيئة نظام التحكم في المسار"""
        self.trajectory_type = TrajectoryType.GUIDED
        self.trajectory_params: Optional[TrajectoryParameters] = None
        self.reference_trajectory: List[TrajectoryPoint] = []
        self.current_trajectory_index = 0
        
        self.g = 9.81  # m/s^2
        self.air_density_sea_level = 1.225  # kg/m^3
        self.scale_height = 8500.0  # meters
        
        self.rocket_mass = 10.0  # kg
        self.rocket_diameter = 0.1  # meters
        self.drag_coefficient = 0.5
        self.thrust = 200.0  # Newtons
        self.specific_impulse = 180.0  # seconds
        self.propellant_mass = 2.0  # kg
        
        self.max_angle_of_attack = 15.0  # degrees
        self.max_lateral_acceleration = 50.0  # m/s^2 (5g)
        self.trajectory_update_rate = 10.0  # Hz
    
    def set_rocket_parameters(self, mass: float, diameter: float, 
                            drag_coeff: float, thrust: float,
                            specific_impulse: float, propellant_mass: float):
        """
        تعيين معاملات الصاروخ
        Set rocket parameters
        """
        self.rocket_mass = mass
        self.rocket_diameter = diameter
        self.drag_coefficient = drag_coeff
        self.thrust = thrust
        self.specific_impulse = specific_impulse
        self.propellant_mass = propellant_mass
    
    def calculate_optimal_launch_angle(self, target_range: float, 
                                      initial_velocity: float,
                                      target_altitude: float = 0.0) -> float:
        """
        حساب زاوية الإطلاق المثلى
        Calculate optimal launch angle for given range
        
        Uses simplified ballistic equations (no drag)
        """
        
        if initial_velocity <= 0:
            return 45.0
        
        effective_range = target_range
        
        sin_2theta = (effective_range * self.g) / (initial_velocity ** 2)
        
        if sin_2theta > 1.0:
            return 45.0
        elif sin_2theta < -1.0:
            return 45.0
        
        angle_low = 0.5 * math.degrees(math.asin(sin_2theta))
        angle_high = 90.0 - angle_low
        
        if self.trajectory_type == TrajectoryType.LOFTED:
            return angle_high
        else:
            return angle_low
    
    def calculate_required_velocity(self, target_range: float, 
                                   launch_angle: float,
                                   target_altitude: float = 0.0) -> float:
        """
        حساب السرعة المطلوبة
        Calculate required velocity for given range and angle
        """
        angle_rad = math.radians(launch_angle)
        sin_2theta = math.sin(2 * angle_rad)
        
        if sin_2theta <= 0:
            return 0.0
        
        required_velocity = math.sqrt((target_range * self.g) / sin_2theta)
        return required_velocity
    
    def generate_reference_trajectory(self, launch_angle: float, azimuth: float,
                                     initial_velocity: float, burn_time: float,
                                     dt: float = 0.1) -> List[TrajectoryPoint]:
        """
        توليد مسار مرجعي
        Generate reference trajectory with atmospheric drag
        
        Args:
            launch_angle: Launch angle in degrees (from horizontal)
            azimuth: Azimuth angle in degrees (0 = North, 90 = East)
            initial_velocity: Initial velocity in m/s
            burn_time: Motor burn time in seconds
            dt: Time step in seconds
            
        Returns:
            List of trajectory points
        """
        trajectory = []
        
        t = 0.0
        x, y, z = 0.0, 0.0, 0.0  # Position (North, East, Up)
        
        angle_rad = math.radians(launch_angle)
        azimuth_rad = math.radians(azimuth)
        
        vx = initial_velocity * math.cos(angle_rad) * math.cos(azimuth_rad)
        vy = initial_velocity * math.cos(angle_rad) * math.sin(azimuth_rad)
        vz = initial_velocity * math.sin(angle_rad)
        
        mass = self.rocket_mass
        
        max_time = 300.0  # Maximum 5 minutes
        
        while t < max_time and z >= 0:
            rho = self.air_density_sea_level * math.exp(-z / self.scale_height)
            
            v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
            
            if v_mag > 0:
                cross_section = math.pi * (self.rocket_diameter / 2) ** 2
                drag_force = 0.5 * rho * v_mag**2 * self.drag_coefficient * cross_section
                
                drag_ax = -(drag_force / mass) * (vx / v_mag)
                drag_ay = -(drag_force / mass) * (vy / v_mag)
                drag_az = -(drag_force / mass) * (vz / v_mag)
            else:
                drag_ax = drag_ay = drag_az = 0.0
            
            if t < burn_time:
                if v_mag > 0:
                    thrust_ax = (self.thrust / mass) * (vx / v_mag)
                    thrust_ay = (self.thrust / mass) * (vy / v_mag)
                    thrust_az = (self.thrust / mass) * (vz / v_mag)
                else:
                    thrust_ax = (self.thrust / mass) * math.cos(angle_rad) * math.cos(azimuth_rad)
                    thrust_ay = (self.thrust / mass) * math.cos(angle_rad) * math.sin(azimuth_rad)
                    thrust_az = (self.thrust / mass) * math.sin(angle_rad)
                
                mass_flow_rate = self.propellant_mass / burn_time
                mass -= mass_flow_rate * dt
                mass = max(mass, self.rocket_mass - self.propellant_mass)
            else:
                thrust_ax = thrust_ay = thrust_az = 0.0
            
            ax = thrust_ax + drag_ax
            ay = thrust_ay + drag_ay
            az = thrust_az + drag_az - self.g
            
            trajectory.append(TrajectoryPoint(
                time=t,
                position=(x, y, z),
                velocity=(vx, vy, vz),
                acceleration=(ax, ay, az)
            ))
            
            vx += ax * dt
            vy += ay * dt
            vz += az * dt
            
            x += vx * dt
            y += vy * dt
            z += vz * dt
            
            t += dt
            
            if z < 0:
                break
        
        self.reference_trajectory = trajectory
        return trajectory
    
    def get_trajectory_deviation(self, current_position: Tuple[float, float, float],
                                current_time: float) -> Tuple[float, float, float]:
        """
        حساب انحراف المسار
        Calculate deviation from reference trajectory
        
        Returns:
            (cross_track_error, altitude_error, along_track_error) in meters
        """
        if not self.reference_trajectory:
            return 0.0, 0.0, 0.0
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, point in enumerate(self.reference_trajectory):
            if abs(point.time - current_time) < min_distance:
                min_distance = abs(point.time - current_time)
                closest_index = i
        
        if closest_index >= len(self.reference_trajectory):
            return 0.0, 0.0, 0.0
        
        ref_point = self.reference_trajectory[closest_index]
        
        cross_track = math.sqrt(
            (current_position[0] - ref_point.position[0])**2 +
            (current_position[1] - ref_point.position[1])**2
        )
        altitude_error = current_position[2] - ref_point.position[2]
        
        along_track = 0.0
        if closest_index > 0:
            prev_point = self.reference_trajectory[closest_index - 1]
            segment_length = math.sqrt(
                (ref_point.position[0] - prev_point.position[0])**2 +
                (ref_point.position[1] - prev_point.position[1])**2
            )
            along_track = segment_length * (current_time - ref_point.time) / \
                         (ref_point.time - prev_point.time) if segment_length > 0 else 0.0
        
        return cross_track, altitude_error, along_track
    
    def calculate_guidance_command(self, current_position: Tuple[float, float, float],
                                  current_velocity: Tuple[float, float, float],
                                  current_time: float) -> Tuple[float, float, float]:
        """
        حساب أمر التوجيه
        Calculate guidance command to follow reference trajectory
        
        Returns:
            (pitch_command, yaw_command, thrust_command)
        """
        if not self.reference_trajectory:
            return 0.0, 0.0, 0.0
        
        cross_track, altitude_error, along_track = self.get_trajectory_deviation(
            current_position, current_time
        )
        
        ref_index = min(self.current_trajectory_index, len(self.reference_trajectory) - 1)
        ref_point = self.reference_trajectory[ref_index]
        
        desired_vx = ref_point.velocity[0]
        desired_vy = ref_point.velocity[1]
        desired_vz = ref_point.velocity[2]
        
        vx, vy, vz = current_velocity
        v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
        
        if v_mag < 0.1:
            return 0.0, 0.0, 1.0
        
        velocity_error_x = desired_vx - vx
        velocity_error_y = desired_vy - vy
        velocity_error_z = desired_vz - vz
        
        kp_cross = 0.5
        kp_altitude = 0.3
        kp_velocity = 0.2
        
        pitch_error = math.atan2(vz, math.sqrt(vx**2 + vy**2))
        desired_pitch = math.atan2(desired_vz, math.sqrt(desired_vx**2 + desired_vy**2))
        pitch_command = desired_pitch - pitch_error
        pitch_command += kp_altitude * altitude_error / max(v_mag, 1.0)
        pitch_command += kp_velocity * velocity_error_z / max(v_mag, 1.0)
        
        yaw_error = math.atan2(vy, vx)
        desired_yaw = math.atan2(desired_vy, desired_vx)
        yaw_command = desired_yaw - yaw_error
        yaw_command += kp_cross * cross_track / max(v_mag, 1.0)
        
        max_command = math.radians(self.max_angle_of_attack)
        pitch_command = max(-max_command, min(max_command, pitch_command))
        yaw_command = max(-max_command, min(max_command, yaw_command))
        
        thrust_command = 1.0 if current_time < ref_point.time else 0.0
        
        return math.degrees(pitch_command), math.degrees(yaw_command), thrust_command
    
    def calculate_impact_point(self, current_position: Tuple[float, float, float],
                              current_velocity: Tuple[float, float, float]) -> Tuple[float, float]:
        """
        حساب نقطة الارتطام المتوقعة
        Calculate predicted impact point
        
        Returns:
            (impact_x, impact_y) in meters from launch point
        """
        x, y, z = current_position
        vx, vy, vz = current_velocity
        
        if z <= 0:
            return x, y
        
        discriminant = vz**2 + 2 * self.g * z
        if discriminant < 0:
            return x, y
        
        time_to_impact = (vz + math.sqrt(discriminant)) / self.g
        
        impact_x = x + vx * time_to_impact
        impact_y = y + vy * time_to_impact
        
        return impact_x, impact_y
    
    def calculate_range(self, current_position: Tuple[float, float, float]) -> float:
        """
        حساب المدى من نقطة الإطلاق
        Calculate range from launch point
        """
        x, y, z = current_position
        return math.sqrt(x**2 + y**2)
    
    def calculate_maximum_range(self, initial_velocity: float) -> float:
        """
        حساب المدى الأقصى
        Calculate maximum range for given initial velocity (vacuum)
        """
        return (initial_velocity ** 2) / self.g
    
    def optimize_trajectory_for_range(self, target_range: float,
                                     available_delta_v: float) -> TrajectoryParameters:
        """
        تحسين المسار للمدى المطلوب
        Optimize trajectory to achieve target range
        
        Returns:
            Optimal trajectory parameters
        """
        launch_angle = self.calculate_optimal_launch_angle(target_range, available_delta_v)
        
        mass_ratio = math.exp(available_delta_v / (self.specific_impulse * self.g))
        propellant_fraction = 1.0 - (1.0 / mass_ratio)
        required_propellant = self.rocket_mass * propellant_fraction
        burn_time = (required_propellant * self.specific_impulse * self.g) / self.thrust
        
        return TrajectoryParameters(
            launch_angle=launch_angle,
            azimuth=0.0,  # North
            initial_velocity=available_delta_v,
            burn_time=burn_time,
            coast_time=0.0,
            target_range=target_range,
            target_altitude=0.0
        )
    
    def get_status(self) -> dict:
        """
        الحصول على حالة التحكم في المسار
        Get trajectory control status
        """
        return {
            'trajectory_type': self.trajectory_type.value,
            'has_reference_trajectory': len(self.reference_trajectory) > 0,
            'trajectory_points': len(self.reference_trajectory),
            'current_index': self.current_trajectory_index,
            'rocket_parameters': {
                'mass': self.rocket_mass,
                'diameter': self.rocket_diameter,
                'drag_coefficient': self.drag_coefficient,
                'thrust': self.thrust,
            }
        }
