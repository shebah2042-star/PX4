"""
نظام الملاحة بالإحداثيات GPS
GPS Navigation System for Waypoint Following and Target Guidance
"""

import math
import asyncio
from typing import Tuple, Optional, List
from dataclasses import dataclass
from enum import Enum


class NavigationMode(Enum):
    """أوضاع الملاحة"""
    IDLE = "idle"
    WAYPOINT = "waypoint"
    TARGET = "target"
    ORBIT = "orbit"
    RETURN_HOME = "return_home"


@dataclass
class Waypoint:
    """نقطة مسار"""
    latitude: float
    longitude: float
    altitude: float
    acceptance_radius: float = 10.0


@dataclass
class GPSPosition:
    """موقع GPS"""
    latitude: float
    longitude: float
    altitude: float
    heading: float = 0.0


class GPSNavigation:
    """
    نظام الملاحة بالإحداثيات GPS
    GPS Navigation System
    """
    
    EARTH_RADIUS = 6371000.0  # meters
    
    def __init__(self):
        """تهيئة نظام الملاحة"""
        self.mode = NavigationMode.IDLE
        self.home_position: Optional[GPSPosition] = None
        self.current_position: Optional[GPSPosition] = None
        self.target_waypoint: Optional[Waypoint] = None
        self.waypoint_list: List[Waypoint] = []
        self.current_waypoint_index = 0
        
        self.max_speed = 100.0  # m/s
        self.approach_speed = 20.0  # m/s
        self.orbit_radius = 50.0  # meters
        self.acceptance_radius = 10.0  # meters
    
    def set_home_position(self, latitude: float, longitude: float, altitude: float):
        """
        تعيين موقع المنزل
        Set home position
        """
        self.home_position = GPSPosition(latitude, longitude, altitude)
    
    def update_current_position(self, latitude: float, longitude: float, 
                               altitude: float, heading: float = 0.0):
        """
        تحديث الموقع الحالي
        Update current position
        """
        self.current_position = GPSPosition(latitude, longitude, altitude, heading)
    
    def add_waypoint(self, latitude: float, longitude: float, altitude: float,
                    acceptance_radius: float = 10.0):
        """
        إضافة نقطة مسار
        Add waypoint to list
        """
        waypoint = Waypoint(latitude, longitude, altitude, acceptance_radius)
        self.waypoint_list.append(waypoint)
    
    def set_target_waypoint(self, latitude: float, longitude: float, 
                           altitude: float, acceptance_radius: float = 10.0):
        """
        تعيين نقطة الهدف
        Set target waypoint
        """
        self.target_waypoint = Waypoint(latitude, longitude, altitude, acceptance_radius)
        self.mode = NavigationMode.WAYPOINT
    
    def calculate_distance(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
        """
        حساب المسافة بين نقطتين (Haversine formula)
        Calculate distance between two GPS coordinates
        
        Returns:
            Distance in meters
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        distance = self.EARTH_RADIUS * c
        return distance
    
    def calculate_bearing(self, lat1: float, lon1: float, 
                         lat2: float, lon2: float) -> float:
        """
        حساب الاتجاه بين نقطتين
        Calculate bearing from point 1 to point 2
        
        Returns:
            Bearing in degrees (0-360)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.degrees(math.atan2(y, x))
        bearing = (bearing + 360) % 360
        
        return bearing
    
    def calculate_cross_track_error(self, start_lat: float, start_lon: float,
                                   end_lat: float, end_lon: float,
                                   current_lat: float, current_lon: float) -> float:
        """
        حساب خطأ المسار العرضي
        Calculate cross-track error (distance from desired path)
        
        Returns:
            Cross-track error in meters (positive = right, negative = left)
        """
        d13 = self.calculate_distance(start_lat, start_lon, current_lat, current_lon)
        d13_rad = d13 / self.EARTH_RADIUS
        
        bearing13 = math.radians(self.calculate_bearing(start_lat, start_lon, 
                                                        current_lat, current_lon))
        
        bearing12 = math.radians(self.calculate_bearing(start_lat, start_lon, 
                                                        end_lat, end_lon))
        
        cross_track = math.asin(math.sin(d13_rad) * 
                               math.sin(bearing13 - bearing12)) * self.EARTH_RADIUS
        
        return cross_track
    
    def get_navigation_command(self) -> Tuple[float, float, float]:
        """
        حساب أوامر الملاحة
        Calculate navigation commands
        
        Returns:
            (desired_heading, desired_speed, desired_altitude)
        """
        if self.current_position is None:
            return 0.0, 0.0, 0.0
        
        if self.mode == NavigationMode.WAYPOINT and self.target_waypoint:
            return self._navigate_to_waypoint()
        
        elif self.mode == NavigationMode.RETURN_HOME and self.home_position:
            return self._navigate_to_home()
        
        elif self.mode == NavigationMode.ORBIT and self.target_waypoint:
            return self._orbit_waypoint()
        
        return 0.0, 0.0, self.current_position.altitude
    
    def _navigate_to_waypoint(self) -> Tuple[float, float, float]:
        """
        الملاحة نحو نقطة الهدف
        Navigate to target waypoint
        """
        if not self.target_waypoint or not self.current_position:
            return 0.0, 0.0, 0.0
        
        distance = self.calculate_distance(
            self.current_position.latitude,
            self.current_position.longitude,
            self.target_waypoint.latitude,
            self.target_waypoint.longitude
        )
        
        bearing = self.calculate_bearing(
            self.current_position.latitude,
            self.current_position.longitude,
            self.target_waypoint.latitude,
            self.target_waypoint.longitude
        )
        
        if distance > 100:
            desired_speed = self.max_speed
        elif distance > 50:
            desired_speed = self.max_speed * 0.7
        else:
            desired_speed = self.approach_speed
        
        if distance < self.target_waypoint.acceptance_radius:
            self._advance_to_next_waypoint()
        
        desired_altitude = self.target_waypoint.altitude
        
        return bearing, desired_speed, desired_altitude
    
    def _navigate_to_home(self) -> Tuple[float, float, float]:
        """
        العودة للمنزل
        Navigate back to home position
        """
        if not self.home_position or not self.current_position:
            return 0.0, 0.0, 0.0
        
        distance = self.calculate_distance(
            self.current_position.latitude,
            self.current_position.longitude,
            self.home_position.latitude,
            self.home_position.longitude
        )
        
        bearing = self.calculate_bearing(
            self.current_position.latitude,
            self.current_position.longitude,
            self.home_position.latitude,
            self.home_position.longitude
        )
        
        if distance > 100:
            desired_speed = self.max_speed * 0.8
        elif distance > 50:
            desired_speed = self.approach_speed
        else:
            desired_speed = self.approach_speed * 0.5
        
        desired_altitude = self.home_position.altitude
        
        return bearing, desired_speed, desired_altitude
    
    def _orbit_waypoint(self) -> Tuple[float, float, float]:
        """
        الدوران حول نقطة
        Orbit around waypoint
        """
        if not self.target_waypoint or not self.current_position:
            return 0.0, 0.0, 0.0
        
        bearing_to_center = self.calculate_bearing(
            self.current_position.latitude,
            self.current_position.longitude,
            self.target_waypoint.latitude,
            self.target_waypoint.longitude
        )
        
        distance = self.calculate_distance(
            self.current_position.latitude,
            self.current_position.longitude,
            self.target_waypoint.latitude,
            self.target_waypoint.longitude
        )
        
        orbit_heading = (bearing_to_center + 90) % 360
        
        radius_error = distance - self.orbit_radius
        speed_adjustment = radius_error / 10.0  # proportional control
        desired_speed = self.approach_speed + speed_adjustment
        desired_speed = max(10.0, min(self.max_speed, desired_speed))
        
        desired_altitude = self.target_waypoint.altitude
        
        return orbit_heading, desired_speed, desired_altitude
    
    def _advance_to_next_waypoint(self):
        """
        الانتقال للنقطة التالية
        Advance to next waypoint in list
        """
        if self.current_waypoint_index < len(self.waypoint_list) - 1:
            self.current_waypoint_index += 1
            self.target_waypoint = self.waypoint_list[self.current_waypoint_index]
        else:
            self.mode = NavigationMode.IDLE
            self.target_waypoint = None
    
    def start_waypoint_mission(self):
        """
        بدء مهمة نقاط المسار
        Start waypoint mission
        """
        if len(self.waypoint_list) > 0:
            self.current_waypoint_index = 0
            self.target_waypoint = self.waypoint_list[0]
            self.mode = NavigationMode.WAYPOINT
    
    def return_to_home(self):
        """
        العودة للمنزل
        Return to home position
        """
        if self.home_position:
            self.mode = NavigationMode.RETURN_HOME
    
    def start_orbit(self, latitude: float, longitude: float, 
                   altitude: float, radius: float = 50.0):
        """
        بدء الدوران حول نقطة
        Start orbiting around a point
        """
        self.target_waypoint = Waypoint(latitude, longitude, altitude)
        self.orbit_radius = radius
        self.mode = NavigationMode.ORBIT
    
    def get_distance_to_target(self) -> float:
        """
        حساب المسافة للهدف
        Get distance to current target
        """
        if not self.current_position or not self.target_waypoint:
            return 0.0
        
        return self.calculate_distance(
            self.current_position.latitude,
            self.current_position.longitude,
            self.target_waypoint.latitude,
            self.target_waypoint.longitude
        )
    
    def get_bearing_to_target(self) -> float:
        """
        حساب الاتجاه للهدف
        Get bearing to current target
        """
        if not self.current_position or not self.target_waypoint:
            return 0.0
        
        return self.calculate_bearing(
            self.current_position.latitude,
            self.current_position.longitude,
            self.target_waypoint.latitude,
            self.target_waypoint.longitude
        )
    
    def get_status(self) -> dict:
        """
        الحصول على حالة الملاحة
        Get navigation status
        """
        return {
            'mode': self.mode.value,
            'current_position': {
                'lat': self.current_position.latitude if self.current_position else 0,
                'lon': self.current_position.longitude if self.current_position else 0,
                'alt': self.current_position.altitude if self.current_position else 0,
            } if self.current_position else None,
            'target_waypoint': {
                'lat': self.target_waypoint.latitude if self.target_waypoint else 0,
                'lon': self.target_waypoint.longitude if self.target_waypoint else 0,
                'alt': self.target_waypoint.altitude if self.target_waypoint else 0,
            } if self.target_waypoint else None,
            'distance_to_target': self.get_distance_to_target(),
            'bearing_to_target': self.get_bearing_to_target(),
            'waypoint_count': len(self.waypoint_list),
            'current_waypoint_index': self.current_waypoint_index,
        }
