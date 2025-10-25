"""
وحدة التحقق من صحة البيانات
Data Validation Module
"""

import numpy as np
from typing import Optional, Tuple
from utils.math_utils import is_valid_number


class DataValidator:
    """فئة للتحقق من صحة البيانات"""
    
    def __init__(self):
        """تهيئة المدقق"""
        self.previous_values = {}
        self.error_counts = {}
    
    def validate_sensor_data(self, sensor_name: str, value: float, 
                            min_val: Optional[float] = None,
                            max_val: Optional[float] = None,
                            max_change_rate: Optional[float] = None,
                            dt: float = 0.01) -> Tuple[bool, str]:
        """
        التحقق من صحة بيانات المستشعر
        
        Args:
            sensor_name: اسم المستشعر
            value: القيمة المراد فحصها
            min_val: الحد الأدنى المسموح
            max_val: الحد الأعلى المسموح
            max_change_rate: أقصى معدل تغيير مسموح
            dt: الفارق الزمني
        
        Returns:
            (is_valid, error_message)
        """
        # فحص القيمة الرقمية
        if not is_valid_number(value):
            return False, f"{sensor_name}: قيمة غير صحيحة (NaN أو Inf)"
        
        # فحص النطاق
        if min_val is not None and value < min_val:
            return False, f"{sensor_name}: قيمة أقل من الحد الأدنى ({value} < {min_val})"
        
        if max_val is not None and value > max_val:
            return False, f"{sensor_name}: قيمة أكبر من الحد الأعلى ({value} > {max_val})"
        
        # فحص معدل التغيير
        if max_change_rate is not None and sensor_name in self.previous_values:
            prev_value = self.previous_values[sensor_name]
            change_rate = abs(value - prev_value) / dt
            
            if change_rate > max_change_rate:
                return False, f"{sensor_name}: معدل تغيير مرتفع جداً ({change_rate:.2f} > {max_change_rate})"
        
        # تحديث القيمة السابقة
        self.previous_values[sensor_name] = value
        
        return True, ""
    
    def validate_gps_data(self, latitude: float, longitude: float, 
                         altitude: float, num_satellites: int,
                         hdop: float) -> Tuple[bool, str]:
        """
        التحقق من صحة بيانات GPS
        
        Args:
            latitude: خط العرض
            longitude: خط الطول
            altitude: الارتفاع
            num_satellites: عدد الأقمار الصناعية
            hdop: دقة الموقع الأفقية
        
        Returns:
            (is_valid, error_message)
        """
        # فحص خط العرض
        if not (-90 <= latitude <= 90):
            return False, f"GPS: خط عرض غير صحيح ({latitude})"
        
        # فحص خط الطول
        if not (-180 <= longitude <= 180):
            return False, f"GPS: خط طول غير صحيح ({longitude})"
        
        # فحص الارتفاع
        if not (-500 <= altitude <= 50000):
            return False, f"GPS: ارتفاع غير معقول ({altitude})"
        
        # فحص عدد الأقمار
        if num_satellites < 4:
            return False, f"GPS: عدد أقمار غير كافٍ ({num_satellites})"
        
        # فحص HDOP
        if hdop > 5.0:
            return False, f"GPS: دقة منخفضة جداً (HDOP={hdop})"
        
        return True, ""
    
    def validate_imu_data(self, accel_x: float, accel_y: float, accel_z: float,
                         gyro_x: float, gyro_y: float, gyro_z: float) -> Tuple[bool, str]:
        """
        التحقق من صحة بيانات IMU
        
        Args:
            accel_x, accel_y, accel_z: التسارع في المحاور الثلاثة (m/s²)
            gyro_x, gyro_y, gyro_z: السرعة الزاوية في المحاور الثلاثة (rad/s)
        
        Returns:
            (is_valid, error_message)
        """
        # فحص التسارع
        max_accel = 200.0  # m/s²
        for axis, value in [('X', accel_x), ('Y', accel_y), ('Z', accel_z)]:
            if not is_valid_number(value):
                return False, f"IMU: تسارع غير صحيح في المحور {axis}"
            if abs(value) > max_accel:
                return False, f"IMU: تسارع مرتفع جداً في المحور {axis} ({value})"
        
        # فحص السرعة الزاوية
        max_gyro = 10.0  # rad/s
        for axis, value in [('X', gyro_x), ('Y', gyro_y), ('Z', gyro_z)]:
            if not is_valid_number(value):
                return False, f"IMU: سرعة زاوية غير صحيحة في المحور {axis}"
            if abs(value) > max_gyro:
                return False, f"IMU: سرعة زاوية مرتفعة جداً في المحور {axis} ({value})"
        
        return True, ""
    
    def validate_attitude(self, roll: float, pitch: float, yaw: float) -> Tuple[bool, str]:
        """
        التحقق من صحة بيانات الزوايا
        
        Args:
            roll, pitch, yaw: الزوايا بالدرجات
        
        Returns:
            (is_valid, error_message)
        """
        # فحص Roll
        if not (-180 <= roll <= 180):
            return False, f"Attitude: زاوية Roll غير صحيحة ({roll})"
        
        # فحص Pitch
        if not (-90 <= pitch <= 90):
            return False, f"Attitude: زاوية Pitch غير صحيحة ({pitch})"
        
        # فحص Yaw
        if not (-180 <= yaw <= 180):
            return False, f"Attitude: زاوية Yaw غير صحيحة ({yaw})"
        
        return True, ""
    
    def validate_barometer(self, pressure: float, temperature: float, 
                          altitude: float) -> Tuple[bool, str]:
        """
        التحقق من صحة بيانات البارومتر
        
        Args:
            pressure: الضغط الجوي (hPa)
            temperature: درجة الحرارة (°C)
            altitude: الارتفاع المحسوب (m)
        
        Returns:
            (is_valid, error_message)
        """
        # فحص الضغط
        if not (300 <= pressure <= 1100):
            return False, f"Barometer: ضغط غير معقول ({pressure} hPa)"
        
        # فحص درجة الحرارة
        if not (-60 <= temperature <= 85):
            return False, f"Barometer: درجة حرارة غير معقولة ({temperature}°C)"
        
        # فحص الارتفاع
        if not (-500 <= altitude <= 50000):
            return False, f"Barometer: ارتفاع غير معقول ({altitude} m)"
        
        return True, ""
    
    def track_error(self, sensor_name: str, max_errors: int = 5) -> bool:
        """
        تتبع أخطاء المستشعر
        
        Args:
            sensor_name: اسم المستشعر
            max_errors: أقصى عدد أخطاء مسموح
        
        Returns:
            True إذا تجاوز عدد الأخطاء الحد المسموح
        """
        if sensor_name not in self.error_counts:
            self.error_counts[sensor_name] = 0
        
        self.error_counts[sensor_name] += 1
        
        return self.error_counts[sensor_name] >= max_errors
    
    def reset_error_count(self, sensor_name: str):
        """
        إعادة تعيين عداد الأخطاء
        
        Args:
            sensor_name: اسم المستشعر
        """
        if sensor_name in self.error_counts:
            self.error_counts[sensor_name] = 0
    
    def get_error_count(self, sensor_name: str) -> int:
        """
        الحصول على عدد الأخطاء
        
        Args:
            sensor_name: اسم المستشعر
        
        Returns:
            عدد الأخطاء
        """
        return self.error_counts.get(sensor_name, 0)

