"""
كاشف القمة (Apogee Detector)
Detects the apogee (highest point) of the rocket flight
"""

import time
import yaml
from typing import Optional
from collections import deque


class ApogeeDetector:
    """
    كاشف القمة للصاروخ
    يستخدم طرق متعددة لكشف نقطة القمة بدقة
    """
    
    def __init__(self, config_path: str = "config/system_config.yaml"):
        """
        تهيئة كاشف القمة
        
        Args:
            config_path: مسار ملف الإعدادات
        """
        # تحميل الإعدادات
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        apogee_config = config['apogee_detection']
        
        # العتبات
        self.velocity_threshold = apogee_config['velocity_threshold']
        self.acceleration_threshold = apogee_config['acceleration_threshold']
        self.confirmation_time = apogee_config['confirmation_time']
        self.barometer_weight = apogee_config['barometer_weight']
        self.gps_weight = apogee_config['gps_weight']
        
        # الحالة
        self.apogee_detected = False
        self.apogee_time = None
        self.apogee_altitude = None
        self.detection_start_time = None
        
        # سجل البيانات
        self.altitude_history = deque(maxlen=50)
        self.velocity_history = deque(maxlen=50)
        self.acceleration_history = deque(maxlen=50)
        
        # طرق الكشف
        self.velocity_method_triggered = False
        self.acceleration_method_triggered = False
        self.altitude_method_triggered = False
    
    def update(self, altitude_baro: float, altitude_gps: Optional[float],
               vertical_velocity: float, vertical_acceleration: float) -> bool:
        """
        تحديث كاشف القمة
        
        Args:
            altitude_baro: الارتفاع من البارومتر (متر)
            altitude_gps: الارتفاع من GPS (متر، اختياري)
            vertical_velocity: السرعة العمودية (م/ث)
            vertical_acceleration: التسارع العمودي (م/ث²)
        
        Returns:
            True إذا تم كشف القمة
        """
        if self.apogee_detected:
            return True
        
        # حساب الارتفاع المدمج
        if altitude_gps is not None:
            altitude = (self.barometer_weight * altitude_baro + 
                       self.gps_weight * altitude_gps)
        else:
            altitude = altitude_baro
        
        # إضافة البيانات إلى السجل
        self.altitude_history.append(altitude)
        self.velocity_history.append(vertical_velocity)
        self.acceleration_history.append(vertical_acceleration)
        
        # الطريقة 1: كشف بناءً على السرعة
        if abs(vertical_velocity) < self.velocity_threshold:
            if not self.velocity_method_triggered:
                self.velocity_method_triggered = True
        else:
            self.velocity_method_triggered = False
        
        # الطريقة 2: كشف بناءً على التسارع
        if vertical_acceleration < self.acceleration_threshold:
            if not self.acceleration_method_triggered:
                self.acceleration_method_triggered = True
        else:
            self.acceleration_method_triggered = False
        
        # الطريقة 3: كشف بناءً على الارتفاع (انخفاض الارتفاع)
        if len(self.altitude_history) >= 10:
            recent_altitudes = list(self.altitude_history)[-10:]
            if all(recent_altitudes[i] >= recent_altitudes[i+1] 
                   for i in range(len(recent_altitudes)-1)):
                if not self.altitude_method_triggered:
                    self.altitude_method_triggered = True
            else:
                self.altitude_method_triggered = False
        
        # التحقق من الكشف
        methods_triggered = sum([
            self.velocity_method_triggered,
            self.acceleration_method_triggered,
            self.altitude_method_triggered
        ])
        
        # يجب أن تتفق طريقتان على الأقل
        if methods_triggered >= 2:
            if self.detection_start_time is None:
                self.detection_start_time = time.time()
            
            # التأكيد بعد فترة زمنية
            if time.time() - self.detection_start_time >= self.confirmation_time:
                self._confirm_apogee(altitude)
                return True
        else:
            self.detection_start_time = None
        
        return False
    
    def _confirm_apogee(self, altitude: float):
        """
        تأكيد كشف القمة
        
        Args:
            altitude: الارتفاع عند القمة
        """
        self.apogee_detected = True
        self.apogee_time = time.time()
        self.apogee_altitude = altitude
    
    def is_apogee_detected(self) -> bool:
        """
        التحقق من كشف القمة
        
        Returns:
            True إذا تم كشف القمة
        """
        return self.apogee_detected
    
    def get_apogee_altitude(self) -> Optional[float]:
        """
        الحصول على ارتفاع القمة
        
        Returns:
            ارتفاع القمة أو None
        """
        return self.apogee_altitude
    
    def get_apogee_time(self) -> Optional[float]:
        """
        الحصول على وقت القمة
        
        Returns:
            وقت القمة أو None
        """
        return self.apogee_time
    
    def get_detection_methods_status(self) -> dict:
        """
        الحصول على حالة طرق الكشف
        
        Returns:
            قاموس يحتوي على حالة كل طريقة
        """
        return {
            'velocity_method': self.velocity_method_triggered,
            'acceleration_method': self.acceleration_method_triggered,
            'altitude_method': self.altitude_method_triggered,
            'methods_triggered': sum([
                self.velocity_method_triggered,
                self.acceleration_method_triggered,
                self.altitude_method_triggered
            ])
        }
    
    def reset(self):
        """إعادة تعيين الكاشف"""
        self.apogee_detected = False
        self.apogee_time = None
        self.apogee_altitude = None
        self.detection_start_time = None
        self.altitude_history.clear()
        self.velocity_history.clear()
        self.acceleration_history.clear()
        self.velocity_method_triggered = False
        self.acceleration_method_triggered = False
        self.altitude_method_triggered = False
    
    def get_statistics(self) -> dict:
        """
        الحصول على إحصائيات
        
        Returns:
            قاموس يحتوي على الإحصائيات
        """
        if len(self.altitude_history) == 0:
            return {}
        
        return {
            'max_altitude': max(self.altitude_history),
            'min_altitude': min(self.altitude_history),
            'current_altitude': self.altitude_history[-1] if self.altitude_history else 0,
            'max_velocity': max(self.velocity_history) if self.velocity_history else 0,
            'min_velocity': min(self.velocity_history) if self.velocity_history else 0,
            'current_velocity': self.velocity_history[-1] if self.velocity_history else 0,
            'current_acceleration': self.acceleration_history[-1] if self.acceleration_history else 0
        }

