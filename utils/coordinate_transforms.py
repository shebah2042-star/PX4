"""
أدوات تحويل الإحداثيات
Coordinate Transformation Utilities
"""

import math
import numpy as np
from typing import Tuple
from dataclasses import dataclass


@dataclass
class LLA:
    """إحداثيات جغرافية - Latitude, Longitude, Altitude"""
    latitude: float  # degrees
    longitude: float  # degrees
    altitude: float  # meters


@dataclass
class ECEF:
    """إحداثيات مركزية أرضية - Earth-Centered Earth-Fixed"""
    x: float  # meters
    y: float  # meters
    z: float  # meters


@dataclass
class NED:
    """إحداثيات محلية - North, East, Down"""
    north: float  # meters
    east: float  # meters
    down: float  # meters


@dataclass
class ENU:
    """إحداثيات محلية - East, North, Up"""
    east: float  # meters
    north: float  # meters
    up: float  # meters


class CoordinateTransforms:
    """
    أدوات تحويل الإحداثيات
    Coordinate Transformation Utilities
    """
    
    WGS84_A = 6378137.0  # Semi-major axis (meters)
    WGS84_B = 6356752.314245  # Semi-minor axis (meters)
    WGS84_E2 = 0.00669437999014  # First eccentricity squared
    
    @staticmethod
    def lla_to_ecef(lla: LLA) -> ECEF:
        """
        تحويل من LLA إلى ECEF
        Convert from Latitude/Longitude/Altitude to ECEF
        """
        lat_rad = math.radians(lla.latitude)
        lon_rad = math.radians(lla.longitude)
        
        N = CoordinateTransforms.WGS84_A / math.sqrt(
            1 - CoordinateTransforms.WGS84_E2 * math.sin(lat_rad)**2
        )
        
        x = (N + lla.altitude) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + lla.altitude) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - CoordinateTransforms.WGS84_E2) + lla.altitude) * math.sin(lat_rad)
        
        return ECEF(x, y, z)
    
    @staticmethod
    def ecef_to_lla(ecef: ECEF) -> LLA:
        """
        تحويل من ECEF إلى LLA
        Convert from ECEF to Latitude/Longitude/Altitude
        """
        p = math.sqrt(ecef.x**2 + ecef.y**2)
        
        lat = math.atan2(ecef.z, p * (1 - CoordinateTransforms.WGS84_E2))
        
        for _ in range(5):
            N = CoordinateTransforms.WGS84_A / math.sqrt(
                1 - CoordinateTransforms.WGS84_E2 * math.sin(lat)**2
            )
            lat = math.atan2(
                ecef.z + CoordinateTransforms.WGS84_E2 * N * math.sin(lat),
                p
            )
        
        lon = math.atan2(ecef.y, ecef.x)
        
        N = CoordinateTransforms.WGS84_A / math.sqrt(
            1 - CoordinateTransforms.WGS84_E2 * math.sin(lat)**2
        )
        alt = p / math.cos(lat) - N
        
        return LLA(
            latitude=math.degrees(lat),
            longitude=math.degrees(lon),
            altitude=alt
        )
    
    @staticmethod
    def lla_to_ned(lla: LLA, reference_lla: LLA) -> NED:
        """
        تحويل من LLA إلى NED
        Convert from LLA to NED (relative to reference point)
        """
        ecef = CoordinateTransforms.lla_to_ecef(lla)
        ref_ecef = CoordinateTransforms.lla_to_ecef(reference_lla)
        
        dx = ecef.x - ref_ecef.x
        dy = ecef.y - ref_ecef.y
        dz = ecef.z - ref_ecef.z
        
        lat_rad = math.radians(reference_lla.latitude)
        lon_rad = math.radians(reference_lla.longitude)
        
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)
        
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        east = -sin_lon * dx + cos_lon * dy
        down = -cos_lat * cos_lon * dx - cos_lat * sin_lon * dy - sin_lat * dz
        
        return NED(north, east, down)
    
    @staticmethod
    def ned_to_lla(ned: NED, reference_lla: LLA) -> LLA:
        """
        تحويل من NED إلى LLA
        Convert from NED to LLA (relative to reference point)
        """
        ref_ecef = CoordinateTransforms.lla_to_ecef(reference_lla)
        
        lat_rad = math.radians(reference_lla.latitude)
        lon_rad = math.radians(reference_lla.longitude)
        
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)
        
        dx = -sin_lat * cos_lon * ned.north - sin_lon * ned.east - cos_lat * cos_lon * ned.down
        dy = -sin_lat * sin_lon * ned.north + cos_lon * ned.east - cos_lat * sin_lon * ned.down
        dz = cos_lat * ned.north - sin_lat * ned.down
        
        ecef = ECEF(
            ref_ecef.x + dx,
            ref_ecef.y + dy,
            ref_ecef.z + dz
        )
        
        return CoordinateTransforms.ecef_to_lla(ecef)
    
    @staticmethod
    def ned_to_enu(ned: NED) -> ENU:
        """
        تحويل من NED إلى ENU
        Convert from NED to ENU
        """
        return ENU(
            east=ned.east,
            north=ned.north,
            up=-ned.down
        )
    
    @staticmethod
    def enu_to_ned(enu: ENU) -> NED:
        """
        تحويل من ENU إلى NED
        Convert from ENU to NED
        """
        return NED(
            north=enu.north,
            east=enu.east,
            down=-enu.up
        )
    
    @staticmethod
    def body_to_ned(body_vector: Tuple[float, float, float],
                   roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
        """
        تحويل من إطار الجسم إلى NED
        Convert from body frame to NED frame
        
        Args:
            body_vector: (x_body, y_body, z_body)
            roll, pitch, yaw: Euler angles in degrees
            
        Returns:
            (north, east, down)
        """
        phi = math.radians(roll)
        theta = math.radians(pitch)
        psi = math.radians(yaw)
        
        
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        
        r11 = cos_theta * cos_psi
        r12 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi
        r13 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi
        
        r21 = cos_theta * sin_psi
        r22 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi
        r23 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi
        
        r31 = -sin_theta
        r32 = sin_phi * cos_theta
        r33 = cos_phi * cos_theta
        
        north = r11 * body_vector[0] + r12 * body_vector[1] + r13 * body_vector[2]
        east = r21 * body_vector[0] + r22 * body_vector[1] + r23 * body_vector[2]
        down = r31 * body_vector[0] + r32 * body_vector[1] + r33 * body_vector[2]
        
        return north, east, down
    
    @staticmethod
    def ned_to_body(ned_vector: Tuple[float, float, float],
                   roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
        """
        تحويل من NED إلى إطار الجسم
        Convert from NED frame to body frame
        
        Args:
            ned_vector: (north, east, down)
            roll, pitch, yaw: Euler angles in degrees
            
        Returns:
            (x_body, y_body, z_body)
        """
        phi = math.radians(roll)
        theta = math.radians(pitch)
        psi = math.radians(yaw)
        
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        
        r11 = cos_theta * cos_psi
        r12 = cos_theta * sin_psi
        r13 = -sin_theta
        
        r21 = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi
        r22 = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi
        r23 = sin_phi * cos_theta
        
        r31 = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi
        r32 = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi
        r33 = cos_phi * cos_theta
        
        x_body = r11 * ned_vector[0] + r12 * ned_vector[1] + r13 * ned_vector[2]
        y_body = r21 * ned_vector[0] + r22 * ned_vector[1] + r23 * ned_vector[2]
        z_body = r31 * ned_vector[0] + r32 * ned_vector[1] + r33 * ned_vector[2]
        
        return x_body, y_body, z_body
    
    @staticmethod
    def quaternion_to_euler(q0: float, q1: float, q2: float, q3: float) -> Tuple[float, float, float]:
        """
        تحويل من كواترنيون إلى زوايا أويلر
        Convert from quaternion to Euler angles
        
        Args:
            q0, q1, q2, q3: Quaternion components (w, x, y, z)
            
        Returns:
            (roll, pitch, yaw) in degrees
        """
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        تحويل من زوايا أويلر إلى كواترنيون
        Convert from Euler angles to quaternion
        
        Args:
            roll, pitch, yaw: Euler angles in degrees
            
        Returns:
            (q0, q1, q2, q3) - quaternion components (w, x, y, z)
        """
        roll_rad = math.radians(roll) / 2
        pitch_rad = math.radians(pitch) / 2
        yaw_rad = math.radians(yaw) / 2
        
        cr = math.cos(roll_rad)
        sr = math.sin(roll_rad)
        cp = math.cos(pitch_rad)
        sp = math.sin(pitch_rad)
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        
        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy
        
        return q0, q1, q2, q3
    
    @staticmethod
    def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        حساب المسافة بين نقطتين
        Calculate distance between two GPS coordinates (Haversine formula)
        
        Returns:
            Distance in meters
        """
        EARTH_RADIUS = 6371000.0  # meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return EARTH_RADIUS * c
    
    @staticmethod
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
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
        return (bearing + 360) % 360
    
    @staticmethod
    def offset_position(lat: float, lon: float, bearing: float, distance: float) -> Tuple[float, float]:
        """
        حساب موقع جديد بناءً على الإزاحة
        Calculate new position given bearing and distance
        
        Args:
            lat, lon: Starting position in degrees
            bearing: Bearing in degrees
            distance: Distance in meters
            
        Returns:
            (new_lat, new_lon) in degrees
        """
        EARTH_RADIUS = 6371000.0  # meters
        
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)
        
        angular_distance = distance / EARTH_RADIUS
        
        new_lat = math.asin(
            math.sin(lat_rad) * math.cos(angular_distance) +
            math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
        )
        
        new_lon = lon_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
            math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat)
        )
        
        return math.degrees(new_lat), math.degrees(new_lon)
    
    @staticmethod
    def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        إنشاء مصفوفة دوران من زوايا أويلر
        Create rotation matrix from Euler angles
        
        Args:
            roll, pitch, yaw: Euler angles in degrees
            
        Returns:
            3x3 rotation matrix
        """
        phi = math.radians(roll)
        theta = math.radians(pitch)
        psi = math.radians(yaw)
        
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        
        R = np.array([
            [cos_theta * cos_psi, 
             sin_phi * sin_theta * cos_psi - cos_phi * sin_psi,
             cos_phi * sin_theta * cos_psi + sin_phi * sin_psi],
            [cos_theta * sin_psi,
             sin_phi * sin_theta * sin_psi + cos_phi * cos_psi,
             cos_phi * sin_theta * sin_psi - sin_phi * cos_psi],
            [-sin_theta,
             sin_phi * cos_theta,
             cos_phi * cos_theta]
        ])
        
        return R
