"""
نظام تتبع الأهداف الثابتة والمتحركة
Target Tracking System for Fixed and Moving Targets
"""

import math
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum


class TargetType(Enum):
    """نوع الهدف"""
    FIXED = "fixed"
    MOVING = "moving"
    PREDICTED = "predicted"


@dataclass
class Target:
    """معلومات الهدف"""
    latitude: float
    longitude: float
    altitude: float
    velocity_north: float = 0.0  # m/s
    velocity_east: float = 0.0   # m/s
    velocity_down: float = 0.0   # m/s
    target_type: TargetType = TargetType.FIXED
    last_update_time: float = 0.0


@dataclass
class InterceptPoint:
    """نقطة الاعتراض"""
    latitude: float
    longitude: float
    altitude: float
    time_to_intercept: float
    intercept_angle: float


class TargetTracking:
    """
    نظام تتبع الأهداف
    Target Tracking System
    """
    
    EARTH_RADIUS = 6371000.0  # meters
    
    def __init__(self):
        """تهيئة نظام التتبع"""
        self.current_target: Optional[Target] = None
        self.target_history: List[Target] = []
        self.max_history_size = 10
        
        self.prediction_time = 5.0  # seconds
        self.intercept_angle_preference = 90.0  # degrees (perpendicular)
        self.min_intercept_distance = 10.0  # meters
        
        self.position_variance = 1.0
        self.velocity_variance = 0.1
    
    def set_fixed_target(self, latitude: float, longitude: float, altitude: float):
        """
        تعيين هدف ثابت
        Set fixed target
        """
        self.current_target = Target(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            target_type=TargetType.FIXED,
            last_update_time=time.time()
        )
        self.target_history.clear()
    
    def update_moving_target(self, latitude: float, longitude: float, altitude: float,
                            velocity_north: float = 0.0, velocity_east: float = 0.0,
                            velocity_down: float = 0.0):
        """
        تحديث هدف متحرك
        Update moving target with position and velocity
        """
        current_time = time.time()
        
        new_target = Target(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            velocity_north=velocity_north,
            velocity_east=velocity_east,
            velocity_down=velocity_down,
            target_type=TargetType.MOVING,
            last_update_time=current_time
        )
        
        if self.current_target:
            self.target_history.append(self.current_target)
            if len(self.target_history) > self.max_history_size:
                self.target_history.pop(0)
        
        self.current_target = new_target
    
    def estimate_target_velocity(self) -> Tuple[float, float, float]:
        """
        تقدير سرعة الهدف من التاريخ
        Estimate target velocity from history
        
        Returns:
            (velocity_north, velocity_east, velocity_down) in m/s
        """
        if len(self.target_history) < 2:
            return 0.0, 0.0, 0.0
        
        prev_target = self.target_history[-1]
        curr_target = self.current_target
        
        if not curr_target:
            return 0.0, 0.0, 0.0
        
        dt = curr_target.last_update_time - prev_target.last_update_time
        if dt <= 0:
            return 0.0, 0.0, 0.0
        
        dlat = curr_target.latitude - prev_target.latitude
        dlon = curr_target.longitude - prev_target.longitude
        dalt = curr_target.altitude - prev_target.altitude
        
        lat_avg = (curr_target.latitude + prev_target.latitude) / 2
        meters_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat_avg))
        meters_per_deg_lon = 111412.84 * math.cos(math.radians(lat_avg))
        
        velocity_north = dlat * meters_per_deg_lat / dt
        velocity_east = dlon * meters_per_deg_lon / dt
        velocity_down = -dalt / dt  # negative because down is positive
        
        return velocity_north, velocity_east, velocity_down
    
    def predict_target_position(self, prediction_time: float) -> Optional[Target]:
        """
        التنبؤ بموقع الهدف
        Predict target position after given time
        
        Args:
            prediction_time: Time in seconds to predict ahead
            
        Returns:
            Predicted target position
        """
        if not self.current_target:
            return None
        
        if self.current_target.target_type == TargetType.FIXED:
            return self.current_target
        
        if (self.current_target.velocity_north == 0 and 
            self.current_target.velocity_east == 0):
            vn, ve, vd = self.estimate_target_velocity()
        else:
            vn = self.current_target.velocity_north
            ve = self.current_target.velocity_east
            vd = self.current_target.velocity_down
        
        lat_avg = self.current_target.latitude
        meters_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat_avg))
        meters_per_deg_lon = 111412.84 * math.cos(math.radians(lat_avg))
        
        predicted_lat = self.current_target.latitude + (vn * prediction_time) / meters_per_deg_lat
        predicted_lon = self.current_target.longitude + (ve * prediction_time) / meters_per_deg_lon
        predicted_alt = self.current_target.altitude - (vd * prediction_time)
        
        return Target(
            latitude=predicted_lat,
            longitude=predicted_lon,
            altitude=predicted_alt,
            velocity_north=vn,
            velocity_east=ve,
            velocity_down=vd,
            target_type=TargetType.PREDICTED,
            last_update_time=time.time()
        )
    
    def calculate_intercept_point(self, rocket_lat: float, rocket_lon: float,
                                 rocket_alt: float, rocket_speed: float) -> Optional[InterceptPoint]:
        """
        حساب نقطة الاعتراض
        Calculate intercept point for moving target
        
        Args:
            rocket_lat, rocket_lon, rocket_alt: Current rocket position
            rocket_speed: Rocket speed in m/s
            
        Returns:
            InterceptPoint with optimal intercept location and time
        """
        if not self.current_target:
            return None
        
        if self.current_target.target_type == TargetType.FIXED:
            distance = self._calculate_distance(rocket_lat, rocket_lon,
                                               self.current_target.latitude,
                                               self.current_target.longitude)
            time_to_intercept = distance / rocket_speed if rocket_speed > 0 else 0
            bearing = self._calculate_bearing(rocket_lat, rocket_lon,
                                             self.current_target.latitude,
                                             self.current_target.longitude)
            
            return InterceptPoint(
                latitude=self.current_target.latitude,
                longitude=self.current_target.longitude,
                altitude=self.current_target.altitude,
                time_to_intercept=time_to_intercept,
                intercept_angle=bearing
            )
        
        
        if (self.current_target.velocity_north == 0 and 
            self.current_target.velocity_east == 0):
            vn, ve, vd = self.estimate_target_velocity()
        else:
            vn = self.current_target.velocity_north
            ve = self.current_target.velocity_east
            vd = self.current_target.velocity_down
        
        target_speed = math.sqrt(vn**2 + ve**2 + vd**2)
        
        distance = self._calculate_distance(rocket_lat, rocket_lon,
                                           self.current_target.latitude,
                                           self.current_target.longitude)
        
        time_to_intercept = distance / (rocket_speed + 0.001)  # avoid division by zero
        
        for _ in range(10):  # Newton's method iterations
            predicted_target = self.predict_target_position(time_to_intercept)
            if not predicted_target:
                break
            
            new_distance = self._calculate_distance(rocket_lat, rocket_lon,
                                                    predicted_target.latitude,
                                                    predicted_target.longitude)
            
            new_time = new_distance / (rocket_speed + 0.001)
            
            if abs(new_time - time_to_intercept) < 0.01:
                time_to_intercept = new_time
                break
            
            time_to_intercept = new_time
        
        predicted_target = self.predict_target_position(time_to_intercept)
        if not predicted_target:
            return None
        
        intercept_bearing = self._calculate_bearing(rocket_lat, rocket_lon,
                                                    predicted_target.latitude,
                                                    predicted_target.longitude)
        
        return InterceptPoint(
            latitude=predicted_target.latitude,
            longitude=predicted_target.longitude,
            altitude=predicted_target.altitude,
            time_to_intercept=time_to_intercept,
            intercept_angle=intercept_bearing
        )
    
    def calculate_lead_angle(self, rocket_lat: float, rocket_lon: float,
                           rocket_speed: float, rocket_heading: float) -> float:
        """
        حساب زاوية التقدم
        Calculate lead angle for moving target interception
        
        Returns:
            Lead angle in degrees
        """
        if not self.current_target:
            return 0.0
        
        intercept = self.calculate_intercept_point(rocket_lat, rocket_lon,
                                                   0, rocket_speed)
        if not intercept:
            return 0.0
        
        lead_angle = intercept.intercept_angle - rocket_heading
        
        while lead_angle > 180:
            lead_angle -= 360
        while lead_angle < -180:
            lead_angle += 360
        
        return lead_angle
    
    def get_proportional_navigation_command(self, rocket_lat: float, rocket_lon: float,
                                           rocket_alt: float, rocket_speed: float,
                                           rocket_heading: float, 
                                           navigation_constant: float = 3.0) -> Tuple[float, float]:
        """
        حساب أمر الملاحة التناسبية
        Calculate proportional navigation command
        
        This is a classic guidance law used in missiles
        
        Args:
            navigation_constant: Typically 3-5 for missiles
            
        Returns:
            (lateral_acceleration, vertical_acceleration) in m/s^2
        """
        if not self.current_target:
            return 0.0, 0.0
        
        bearing = self._calculate_bearing(rocket_lat, rocket_lon,
                                         self.current_target.latitude,
                                         self.current_target.longitude)
        
        if len(self.target_history) > 0:
            prev_bearing = self._calculate_bearing(rocket_lat, rocket_lon,
                                                   self.target_history[-1].latitude,
                                                   self.target_history[-1].longitude)
            dt = self.current_target.last_update_time - self.target_history[-1].last_update_time
            if dt > 0:
                los_rate = (bearing - prev_bearing) / dt
            else:
                los_rate = 0.0
        else:
            los_rate = 0.0
        
        
        distance = self._calculate_distance(rocket_lat, rocket_lon,
                                           self.current_target.latitude,
                                           self.current_target.longitude)
        
        if len(self.target_history) > 0:
            prev_distance = self._calculate_distance(rocket_lat, rocket_lon,
                                                     self.target_history[-1].latitude,
                                                     self.target_history[-1].longitude)
            dt = self.current_target.last_update_time - self.target_history[-1].last_update_time
            if dt > 0:
                closing_velocity = (prev_distance - distance) / dt
            else:
                closing_velocity = rocket_speed
        else:
            closing_velocity = rocket_speed
        
        lateral_accel = navigation_constant * closing_velocity * math.radians(los_rate)
        
        altitude_error = self.current_target.altitude - rocket_alt
        vertical_accel = navigation_constant * altitude_error / max(distance, 1.0)
        
        max_accel = 50.0  # m/s^2 (about 5g)
        lateral_accel = max(-max_accel, min(max_accel, lateral_accel))
        vertical_accel = max(-max_accel, min(max_accel, vertical_accel))
        
        return lateral_accel, vertical_accel
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                           lat2: float, lon2: float) -> float:
        """حساب المسافة بين نقطتين"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return self.EARTH_RADIUS * c
    
    def _calculate_bearing(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
        """حساب الاتجاه بين نقطتين"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    def get_distance_to_target(self, rocket_lat: float, rocket_lon: float) -> float:
        """
        حساب المسافة للهدف
        Get distance to current target
        """
        if not self.current_target:
            return 0.0
        
        return self._calculate_distance(rocket_lat, rocket_lon,
                                       self.current_target.latitude,
                                       self.current_target.longitude)
    
    def get_bearing_to_target(self, rocket_lat: float, rocket_lon: float) -> float:
        """
        حساب الاتجاه للهدف
        Get bearing to current target
        """
        if not self.current_target:
            return 0.0
        
        return self._calculate_bearing(rocket_lat, rocket_lon,
                                      self.current_target.latitude,
                                      self.current_target.longitude)
    
    def is_target_in_range(self, rocket_lat: float, rocket_lon: float,
                          max_range: float) -> bool:
        """
        التحقق من وجود الهدف في المدى
        Check if target is within range
        """
        if not self.current_target:
            return False
        
        distance = self.get_distance_to_target(rocket_lat, rocket_lon)
        return distance <= max_range
    
    def get_status(self) -> dict:
        """
        الحصول على حالة التتبع
        Get tracking status
        """
        if not self.current_target:
            return {
                'has_target': False,
                'target_type': None,
            }
        
        return {
            'has_target': True,
            'target_type': self.current_target.target_type.value,
            'target_position': {
                'lat': self.current_target.latitude,
                'lon': self.current_target.longitude,
                'alt': self.current_target.altitude,
            },
            'target_velocity': {
                'north': self.current_target.velocity_north,
                'east': self.current_target.velocity_east,
                'down': self.current_target.velocity_down,
            },
            'history_size': len(self.target_history),
            'last_update': self.current_target.last_update_time,
        }
