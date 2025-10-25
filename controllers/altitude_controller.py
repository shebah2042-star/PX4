"""
متحكم الارتفاع (Altitude Controller)
Controls altitude and vertical velocity
"""

import yaml
from controllers.pid_controller import PIDController
from typing import Dict


class AltitudeController:
    """
    متحكم الارتفاع للصاروخ
    يتحكم في الارتفاع والسرعة العمودية
    """
    
    def __init__(self, config_path: str = "config/pid_config.yaml"):
        """
        تهيئة متحكم الارتفاع
        
        Args:
            config_path: مسار ملف الإعدادات
        """
        # تحميل الإعدادات
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # إنشاء متحكمات PID
        self.altitude_controller = self._create_pid_from_config(config['altitude'])
        self.velocity_controller = self._create_pid_from_config(config['vertical_velocity'])
        
        # حالة التفعيل
        self.enabled = True
        
        # وضع التحكم: 'altitude' أو 'velocity'
        self.control_mode = 'altitude'
    
    def _create_pid_from_config(self, config: Dict) -> PIDController:
        """
        إنشاء متحكم PID من الإعدادات
        
        Args:
            config: إعدادات المتحكم
        
        Returns:
            متحكم PID
        """
        return PIDController(
            kp=config['kp'],
            ki=config['ki'],
            kd=config['kd'],
            output_min=config['output_min'],
            output_max=config['output_max'],
            integral_min=config['integral_min'],
            integral_max=config['integral_max'],
            derivative_filter=config.get('derivative_filter', 0.0)
        )
    
    def update_altitude(self, target_altitude: float, current_altitude: float,
                       dt: float = 0.01) -> float:
        """
        تحديث التحكم في الارتفاع
        
        Args:
            target_altitude: الارتفاع المستهدف (متر)
            current_altitude: الارتفاع الحالي (متر)
            dt: الفارق الزمني (ثانية)
        
        Returns:
            أمر التحكم في السرعة العمودية
        """
        if not self.enabled:
            return 0.0
        
        return self.altitude_controller.update(target_altitude, current_altitude, dt)
    
    def update_velocity(self, target_velocity: float, current_velocity: float,
                       dt: float = 0.01) -> float:
        """
        تحديث التحكم في السرعة العمودية
        
        Args:
            target_velocity: السرعة العمودية المستهدفة (م/ث)
            current_velocity: السرعة العمودية الحالية (م/ث)
            dt: الفارق الزمني (ثانية)
        
        Returns:
            أمر التحكم في التسارع
        """
        if not self.enabled:
            return 0.0
        
        return self.velocity_controller.update(target_velocity, current_velocity, dt)
    
    def update_cascade(self, target_altitude: float, current_altitude: float,
                      current_velocity: float, dt: float = 0.01) -> float:
        """
        تحديث التحكم المتسلسل (Cascade Control)
        الارتفاع -> السرعة العمودية
        
        Args:
            target_altitude: الارتفاع المستهدف (متر)
            current_altitude: الارتفاع الحالي (متر)
            current_velocity: السرعة العمودية الحالية (م/ث)
            dt: الفارق الزمني (ثانية)
        
        Returns:
            أمر التحكم النهائي
        """
        if not self.enabled:
            return 0.0
        
        # الحلقة الخارجية: التحكم في الارتفاع
        target_velocity = self.altitude_controller.update(
            target_altitude, current_altitude, dt
        )
        
        # الحلقة الداخلية: التحكم في السرعة
        output = self.velocity_controller.update(
            target_velocity, current_velocity, dt
        )
        
        return output
    
    def reset(self):
        """إعادة تعيين جميع المتحكمات"""
        self.altitude_controller.reset()
        self.velocity_controller.reset()
    
    def enable(self):
        """تفعيل المتحكم"""
        self.enabled = True
    
    def disable(self):
        """تعطيل المتحكم"""
        self.enabled = False
        self.reset()
    
    def is_enabled(self) -> bool:
        """
        التحقق من حالة التفعيل
        
        Returns:
            True إذا كان المتحكم مفعلاً
        """
        return self.enabled
    
    def set_control_mode(self, mode: str):
        """
        تعيين وضع التحكم
        
        Args:
            mode: 'altitude' أو 'velocity'
        """
        if mode not in ['altitude', 'velocity']:
            raise ValueError(f"وضع تحكم غير صحيح: {mode}")
        self.control_mode = mode
    
    def get_control_mode(self) -> str:
        """
        الحصول على وضع التحكم الحالي
        
        Returns:
            وضع التحكم
        """
        return self.control_mode
    
    def get_controller_states(self) -> Dict:
        """
        الحصول على حالات المتحكمات
        
        Returns:
            قاموس يحتوي على حالات المتحكمات
        """
        return {
            'altitude': {
                'p': self.altitude_controller.last_p_term,
                'i': self.altitude_controller.last_i_term,
                'd': self.altitude_controller.last_d_term,
                'output': self.altitude_controller.last_output,
                'integral': self.altitude_controller.get_integral()
            },
            'velocity': {
                'p': self.velocity_controller.last_p_term,
                'i': self.velocity_controller.last_i_term,
                'd': self.velocity_controller.last_d_term,
                'output': self.velocity_controller.last_output,
                'integral': self.velocity_controller.get_integral()
            }
        }

