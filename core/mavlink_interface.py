"""
واجهة MAVLink
MAVLink Interface Module
"""

import asyncio
import yaml
from mavsdk import System
from typing import Optional, Dict
import logging


class MAVLinkInterface:
    """
    واجهة الاتصال مع PX4 عبر MAVLink
    """
    
    def __init__(self, config_path: str = "config/system_config.yaml"):
        """
        تهيئة واجهة MAVLink
        
        Args:
            config_path: مسار ملف الإعدادات
        """
        # تحميل الإعدادات
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        self.mavlink_config = config['mavlink']
        
        # إنشاء نظام MAVSDK
        self.drone = System()
        
        # حالة الاتصال
        self.connected = False
        
        # بيانات المستشعرات
        self.sensor_data = {
            'attitude': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'position': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'velocity': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'imu': {'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0,
                   'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0},
            'barometer': {'pressure': 0.0, 'temperature': 0.0, 'altitude': 0.0},
            'gps': {'num_satellites': 0, 'fix_type': 0},
            'battery': {'voltage': 0.0, 'current': 0.0, 'remaining': 0.0}
        }
        
        # السجل
        self.logger = logging.getLogger(__name__)
    
    async def connect(self) -> bool:
        """
        الاتصال بـ PX4
        
        Returns:
            True إذا نجح الاتصال
        """
        try:
            connection_string = self.mavlink_config['connection_string']
            self.logger.info(f"الاتصال بـ PX4 عبر {connection_string}")
            
            await self.drone.connect(system_address=connection_string)
            
            # انتظار الاتصال
            self.logger.info("انتظار الاتصال...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("تم الاتصال بنجاح!")
                    self.connected = True
                    break
            
            return self.connected
        except Exception as e:
            self.logger.error(f"خطأ في الاتصال: {e}")
            return False
    
    async def start_telemetry(self):
        """بدء استقبال البيانات"""
        # بدء مهام الاستقبال
        asyncio.create_task(self._receive_attitude())
        asyncio.create_task(self._receive_position())
        asyncio.create_task(self._receive_velocity())
        asyncio.create_task(self._receive_imu())
        asyncio.create_task(self._receive_barometer())
        asyncio.create_task(self._receive_gps())
        asyncio.create_task(self._receive_battery())
        
        self.logger.info("بدء استقبال البيانات")
    
    async def _receive_attitude(self):
        """استقبال بيانات الزوايا"""
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                self.sensor_data['attitude'] = {
                    'roll': attitude.roll_deg,
                    'pitch': attitude.pitch_deg,
                    'yaw': attitude.yaw_deg
                }
        except Exception as e:
            self.logger.error(f"خطأ في استقبال الزوايا: {e}")
    
    async def _receive_position(self):
        """استقبال بيانات الموقع"""
        try:
            async for position in self.drone.telemetry.position():
                self.sensor_data['position'] = {
                    'lat': position.latitude_deg,
                    'lon': position.longitude_deg,
                    'alt': position.absolute_altitude_m
                }
        except Exception as e:
            self.logger.error(f"خطأ في استقبال الموقع: {e}")
    
    async def _receive_velocity(self):
        """استقبال بيانات السرعة"""
        try:
            async for velocity in self.drone.telemetry.velocity_ned():
                self.sensor_data['velocity'] = {
                    'vx': velocity.north_m_s,
                    'vy': velocity.east_m_s,
                    'vz': velocity.down_m_s
                }
        except Exception as e:
            self.logger.error(f"خطأ في استقبال السرعة: {e}")
    
    async def _receive_imu(self):
        """استقبال بيانات IMU"""
        try:
            async for imu in self.drone.telemetry.imu():
                self.sensor_data['imu'] = {
                    'accel_x': imu.acceleration_frd.forward_m_s2,
                    'accel_y': imu.acceleration_frd.right_m_s2,
                    'accel_z': imu.acceleration_frd.down_m_s2,
                    'gyro_x': imu.angular_velocity_frd.forward_rad_s,
                    'gyro_y': imu.angular_velocity_frd.right_rad_s,
                    'gyro_z': imu.angular_velocity_frd.down_rad_s
                }
        except Exception as e:
            self.logger.error(f"خطأ في استقبال IMU: {e}")
    
    async def _receive_barometer(self):
        """استقبال بيانات البارومتر"""
        try:
            async for baro in self.drone.telemetry.raw_gps():
                # ملاحظة: MAVSDK قد لا يوفر بيانات البارومتر مباشرة
                # يمكن استخدام altitude من position
                self.sensor_data['barometer']['altitude'] = self.sensor_data['position']['alt']
        except Exception as e:
            self.logger.error(f"خطأ في استقبال البارومتر: {e}")
    
    async def _receive_gps(self):
        """استقبال بيانات GPS"""
        try:
            async for gps_info in self.drone.telemetry.gps_info():
                self.sensor_data['gps'] = {
                    'num_satellites': gps_info.num_satellites,
                    'fix_type': gps_info.fix_type
                }
        except Exception as e:
            self.logger.error(f"خطأ في استقبال GPS: {e}")
    
    async def _receive_battery(self):
        """استقبال بيانات البطارية"""
        try:
            async for battery in self.drone.telemetry.battery():
                self.sensor_data['battery'] = {
                    'voltage': battery.voltage_v,
                    'remaining': battery.remaining_percent
                }
        except Exception as e:
            self.logger.error(f"خطأ في استقبال البطارية: {e}")
    
    def get_attitude(self) -> Dict[str, float]:
        """
        الحصول على بيانات الزوايا
        
        Returns:
            قاموس يحتوي على roll, pitch, yaw
        """
        return self.sensor_data['attitude'].copy()
    
    def get_position(self) -> Dict[str, float]:
        """
        الحصول على بيانات الموقع
        
        Returns:
            قاموس يحتوي على lat, lon, alt
        """
        return self.sensor_data['position'].copy()
    
    def get_velocity(self) -> Dict[str, float]:
        """
        الحصول على بيانات السرعة
        
        Returns:
            قاموس يحتوي على vx, vy, vz
        """
        return self.sensor_data['velocity'].copy()
    
    def get_imu(self) -> Dict[str, float]:
        """
        الحصول على بيانات IMU
        
        Returns:
            قاموس يحتوي على التسارع والسرعة الزاوية
        """
        return self.sensor_data['imu'].copy()
    
    def get_altitude(self) -> float:
        """
        الحصول على الارتفاع
        
        Returns:
            الارتفاع بالأمتار
        """
        return self.sensor_data['position']['alt']
    
    def get_vertical_velocity(self) -> float:
        """
        الحصول على السرعة العمودية
        
        Returns:
            السرعة العمودية (م/ث)
        """
        return -self.sensor_data['velocity']['vz']  # سالب لأن down موجب في NED
    
    def get_all_sensor_data(self) -> Dict:
        """
        الحصول على جميع بيانات المستشعرات
        
        Returns:
            قاموس يحتوي على جميع البيانات
        """
        return {
            **self.sensor_data['attitude'],
            **self.sensor_data['position'],
            **self.sensor_data['velocity'],
            **self.sensor_data['imu'],
            **self.sensor_data['barometer'],
            **self.sensor_data['gps'],
            **self.sensor_data['battery']
        }
    
    def is_connected(self) -> bool:
        """
        التحقق من حالة الاتصال
        
        Returns:
            True إذا كان متصلاً
        """
        return self.connected
    
    async def disconnect(self):
        """قطع الاتصال"""
        self.connected = False
        self.logger.info("تم قطع الاتصال")

