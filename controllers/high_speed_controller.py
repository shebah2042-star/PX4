"""
نظام التحكم في السرعات العالية
High-Speed Flight Control System with Adaptive Control
"""

import math
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class FlightRegime(Enum):
    """نظام الطيران"""
    SUBSONIC = "subsonic"  # Mach < 0.8
    TRANSONIC = "transonic"  # 0.8 <= Mach < 1.2
    SUPERSONIC = "supersonic"  # 1.2 <= Mach < 5.0
    HYPERSONIC = "hypersonic"  # Mach >= 5.0


@dataclass
class AerodynamicCoefficients:
    """معاملات الديناميكا الهوائية"""
    cd: float  # Drag coefficient
    cl: float  # Lift coefficient
    cm: float  # Moment coefficient
    cn: float  # Normal force coefficient


class HighSpeedController:
    """
    نظام التحكم في السرعات العالية
    High-Speed Flight Controller with Adaptive Control
    """
    
    def __init__(self):
        """تهيئة نظام التحكم في السرعات العالية"""
        self.current_regime = FlightRegime.SUBSONIC
        self.mach_number = 0.0
        self.dynamic_pressure = 0.0
        
        self.speed_of_sound_sea_level = 340.3  # m/s
        self.air_density_sea_level = 1.225  # kg/m^3
        self.temperature_sea_level = 288.15  # Kelvin
        self.temperature_lapse_rate = 0.0065  # K/m
        self.scale_height = 8500.0  # meters
        
        self.adaptive_gain = 1.0
        self.adaptation_rate = 0.1
        self.reference_model_damping = 0.7
        self.reference_model_frequency = 2.0  # rad/s
        
        self.base_kp_roll = 2.0
        self.base_ki_roll = 0.1
        self.base_kd_roll = 0.5
        
        self.base_kp_pitch = 2.0
        self.base_ki_pitch = 0.1
        self.base_kd_pitch = 0.5
        
        self.base_kp_yaw = 1.5
        self.base_ki_yaw = 0.05
        self.base_kd_yaw = 0.3
        
        self.regime_multipliers = {
            FlightRegime.SUBSONIC: 1.0,
            FlightRegime.TRANSONIC: 1.5,  # Increased gains for transonic instability
            FlightRegime.SUPERSONIC: 1.2,
            FlightRegime.HYPERSONIC: 0.8  # Reduced gains for high-speed stability
        }
        
        self.enable_aero_compensation = True
        self.center_of_pressure_offset = 0.0  # meters from CG
        
        self.actuator_delay = 0.02  # seconds
        self.sensor_delay = 0.01  # seconds
        self.total_delay = self.actuator_delay + self.sensor_delay
        
        self.state_history = []
        self.max_history_length = 100
        
        self.estimated_parameters = np.zeros(6)  # Aerodynamic parameter estimates
        self.parameter_covariance = np.eye(6) * 10.0
    
    def update_flight_conditions(self, velocity: float, altitude: float):
        """
        تحديث ظروف الطيران
        Update flight conditions and determine flight regime
        
        Args:
            velocity: Current velocity in m/s
            altitude: Current altitude in meters
        """
        if altitude < 11000:  # Troposphere
            temperature = self.temperature_sea_level - self.temperature_lapse_rate * altitude
        else:  # Stratosphere (simplified)
            temperature = 216.65  # K
        
        speed_of_sound = math.sqrt(1.4 * 287.05 * temperature)
        
        self.mach_number = velocity / speed_of_sound if speed_of_sound > 0 else 0.0
        
        air_density = self.air_density_sea_level * math.exp(-altitude / self.scale_height)
        
        self.dynamic_pressure = 0.5 * air_density * velocity**2
        
        if self.mach_number < 0.8:
            self.current_regime = FlightRegime.SUBSONIC
        elif self.mach_number < 1.2:
            self.current_regime = FlightRegime.TRANSONIC
        elif self.mach_number < 5.0:
            self.current_regime = FlightRegime.SUPERSONIC
        else:
            self.current_regime = FlightRegime.HYPERSONIC
    
    def get_adaptive_gains(self) -> Tuple[Tuple[float, float, float], 
                                         Tuple[float, float, float],
                                         Tuple[float, float, float]]:
        """
        الحصول على معاملات التحكم التكيفية
        Get adaptive PID gains based on flight regime
        
        Returns:
            ((kp_roll, ki_roll, kd_roll), (kp_pitch, ki_pitch, kd_pitch), (kp_yaw, ki_yaw, kd_yaw))
        """
        multiplier = self.regime_multipliers[self.current_regime]
        
        total_gain = multiplier * self.adaptive_gain
        
        roll_gains = (
            self.base_kp_roll * total_gain,
            self.base_ki_roll * total_gain,
            self.base_kd_roll * total_gain
        )
        
        pitch_gains = (
            self.base_kp_pitch * total_gain,
            self.base_ki_pitch * total_gain,
            self.base_kd_pitch * total_gain
        )
        
        yaw_gains = (
            self.base_kp_yaw * total_gain,
            self.base_ki_yaw * total_gain,
            self.base_kd_yaw * total_gain
        )
        
        return roll_gains, pitch_gains, yaw_gains
    
    def compensate_aerodynamic_effects(self, angle_of_attack: float, 
                                      sideslip_angle: float,
                                      roll_rate: float, pitch_rate: float, 
                                      yaw_rate: float) -> Tuple[float, float, float]:
        """
        تعويض التأثيرات الديناميكية الهوائية
        Compensate for aerodynamic effects at high speeds
        
        Returns:
            (roll_compensation, pitch_compensation, yaw_compensation) in degrees
        """
        if not self.enable_aero_compensation:
            return 0.0, 0.0, 0.0
        
        aero_coeffs = self._calculate_aerodynamic_coefficients(
            angle_of_attack, sideslip_angle
        )
        
        cp_compensation = self._compensate_center_of_pressure_shift()
        
        damping_compensation = self._compensate_damping_effects(
            roll_rate, pitch_rate, yaw_rate
        )
        
        roll_comp = cp_compensation[0] + damping_compensation[0]
        pitch_comp = cp_compensation[1] + damping_compensation[1]
        yaw_comp = cp_compensation[2] + damping_compensation[2]
        
        return roll_comp, pitch_comp, yaw_comp
    
    def _calculate_aerodynamic_coefficients(self, angle_of_attack: float,
                                           sideslip_angle: float) -> AerodynamicCoefficients:
        """
        حساب معاملات الديناميكا الهوائية
        Calculate aerodynamic coefficients based on Mach number
        """
        
        cd_base = 0.5
        cl_alpha = 0.1  # per degree
        cm_alpha = -0.05  # per degree (stable)
        cn_beta = -0.08  # per degree (stable)
        
        if self.current_regime == FlightRegime.TRANSONIC:
            cd_base *= (1.0 + 2.0 * (self.mach_number - 0.8))
            cl_alpha *= 0.7
        elif self.current_regime == FlightRegime.SUPERSONIC:
            cd_base = 0.3 + 0.2 * self.mach_number
            cl_alpha = 0.08
        elif self.current_regime == FlightRegime.HYPERSONIC:
            cd_base = 0.2 + 0.1 * self.mach_number
            cl_alpha = 0.05
        
        cd = cd_base + cl_alpha * abs(angle_of_attack) * 0.1
        cl = cl_alpha * angle_of_attack
        cm = cm_alpha * angle_of_attack
        cn = cn_beta * sideslip_angle
        
        return AerodynamicCoefficients(cd=cd, cl=cl, cm=cm, cn=cn)
    
    def _compensate_center_of_pressure_shift(self) -> Tuple[float, float, float]:
        """
        تعويض تحول مركز الضغط
        Compensate for center of pressure shift at high speeds
        """
        if self.current_regime == FlightRegime.SUPERSONIC:
            cp_shift = 0.1 * (self.mach_number - 1.0)  # meters
        elif self.current_regime == FlightRegime.HYPERSONIC:
            cp_shift = 0.2 * (self.mach_number - 1.0)
        else:
            cp_shift = 0.0
        
        pitch_compensation = cp_shift * 5.0  # degrees (simplified)
        
        return 0.0, pitch_compensation, 0.0
    
    def _compensate_damping_effects(self, roll_rate: float, pitch_rate: float,
                                   yaw_rate: float) -> Tuple[float, float, float]:
        """
        تعويض تأثيرات التخميد
        Compensate for aerodynamic damping effects
        """
        damping_factor = self.dynamic_pressure / 10000.0  # Normalized
        
        roll_comp = -roll_rate * damping_factor * 0.1
        
        pitch_comp = -pitch_rate * damping_factor * 0.1
        
        yaw_comp = -yaw_rate * damping_factor * 0.1
        
        return roll_comp, pitch_comp, yaw_comp
    
    def predict_state_with_delay_compensation(self, current_state: np.ndarray,
                                             control_input: np.ndarray) -> np.ndarray:
        """
        التنبؤ بالحالة مع تعويض التأخير
        Predict future state to compensate for actuator/sensor delays
        
        Args:
            current_state: [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
            control_input: [roll_cmd, pitch_cmd, yaw_cmd]
            
        Returns:
            Predicted state after delay compensation
        """
        dt = self.total_delay
        
        predicted_state = current_state.copy()
        
        predicted_state[0] += current_state[3] * dt  # roll
        predicted_state[1] += current_state[4] * dt  # pitch
        predicted_state[2] += current_state[5] * dt  # yaw
        
        predicted_state[3] += control_input[0] * dt * 10.0  # roll_rate
        predicted_state[4] += control_input[1] * dt * 10.0  # pitch_rate
        predicted_state[5] += control_input[2] * dt * 10.0  # yaw_rate
        
        return predicted_state
    
    def update_adaptive_parameters(self, state_error: np.ndarray, 
                                  control_input: np.ndarray):
        """
        تحديث المعاملات التكيفية
        Update adaptive parameters using Model Reference Adaptive Control (MRAC)
        
        Args:
            state_error: Error between actual and reference model
            control_input: Current control input
        """
        
        regressor = np.concatenate([state_error[:3], control_input])
        
        parameter_update = -self.adaptation_rate * regressor
        self.estimated_parameters += parameter_update * 0.01  # Small step
        
        error_magnitude = np.linalg.norm(state_error)
        if error_magnitude > 0.1:
            self.adaptive_gain *= (1.0 + self.adaptation_rate * 0.1)
        else:
            self.adaptive_gain *= (1.0 - self.adaptation_rate * 0.05)
        
        self.adaptive_gain = max(0.5, min(2.0, self.adaptive_gain))
    
    def calculate_reference_model_response(self, command: float, 
                                          current_value: float,
                                          dt: float) -> float:
        """
        حساب استجابة النموذج المرجعي
        Calculate reference model response (2nd order system)
        
        Args:
            command: Desired value
            current_value: Current value
            dt: Time step
            
        Returns:
            Reference model output
        """
        omega_n = self.reference_model_frequency
        zeta = self.reference_model_damping
        
        error = command - current_value
        response = current_value + omega_n**2 * error * dt
        
        return response
    
    def limit_control_rates(self, control_command: Tuple[float, float, float],
                           previous_command: Tuple[float, float, float],
                           dt: float) -> Tuple[float, float, float]:
        """
        تحديد معدلات التحكم
        Limit control command rates to prevent actuator saturation
        
        Returns:
            Rate-limited control commands
        """
        if self.current_regime == FlightRegime.HYPERSONIC:
            max_rate = 50.0  # Slower for high-speed stability
        elif self.current_regime == FlightRegime.SUPERSONIC:
            max_rate = 100.0
        else:
            max_rate = 200.0
        
        limited_command = []
        for i in range(3):
            rate = (control_command[i] - previous_command[i]) / dt if dt > 0 else 0
            if abs(rate) > max_rate:
                limited_value = previous_command[i] + np.sign(rate) * max_rate * dt
            else:
                limited_value = control_command[i]
            limited_command.append(limited_value)
        
        return tuple(limited_command)
    
    def calculate_max_angle_of_attack(self) -> float:
        """
        حساب زاوية الهجوم القصوى
        Calculate maximum safe angle of attack for current regime
        """
        if self.current_regime == FlightRegime.SUBSONIC:
            return 20.0  # degrees
        elif self.current_regime == FlightRegime.TRANSONIC:
            return 10.0  # Reduced for transonic stability
        elif self.current_regime == FlightRegime.SUPERSONIC:
            return 15.0
        else:  # HYPERSONIC
            return 8.0  # Very limited for hypersonic
    
    def calculate_max_lateral_acceleration(self) -> float:
        """
        حساب التسارع الجانبي الأقصى
        Calculate maximum safe lateral acceleration
        """
        max_q = 50000.0  # Pa (maximum dynamic pressure)
        
        if self.dynamic_pressure > max_q:
            reduction_factor = max_q / self.dynamic_pressure
        else:
            reduction_factor = 1.0
        
        base_max_accel = 50.0  # m/s^2 (about 5g)
        return base_max_accel * reduction_factor
    
    def get_flight_envelope_limits(self) -> dict:
        """
        الحصول على حدود غلاف الطيران
        Get current flight envelope limits
        """
        return {
            'max_angle_of_attack': self.calculate_max_angle_of_attack(),
            'max_lateral_acceleration': self.calculate_max_lateral_acceleration(),
            'max_dynamic_pressure': 50000.0,
            'current_dynamic_pressure': self.dynamic_pressure,
            'max_mach': 6.0,
            'current_mach': self.mach_number,
        }
    
    def get_status(self) -> dict:
        """
        الحصول على حالة التحكم في السرعات العالية
        Get high-speed controller status
        """
        roll_gains, pitch_gains, yaw_gains = self.get_adaptive_gains()
        
        return {
            'flight_regime': self.current_regime.value,
            'mach_number': self.mach_number,
            'dynamic_pressure': self.dynamic_pressure,
            'adaptive_gain': self.adaptive_gain,
            'gains': {
                'roll': {'kp': roll_gains[0], 'ki': roll_gains[1], 'kd': roll_gains[2]},
                'pitch': {'kp': pitch_gains[0], 'ki': pitch_gains[1], 'kd': pitch_gains[2]},
                'yaw': {'kp': yaw_gains[0], 'ki': yaw_gains[1], 'kd': yaw_gains[2]},
            },
            'envelope_limits': self.get_flight_envelope_limits(),
        }
