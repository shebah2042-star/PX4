"""
متحكم الزوايا (Attitude Controller)
Controls Roll, Pitch, and Yaw
"""

import yaml
from controllers.pid_controller import PIDController
from utils.math_utils import normalize_angle
from typing import Tuple, Dict


class AttitudeController:
    """
    متحكم الزوايا للصاروخ
    يتحكم في Roll, Pitch, Yaw
    """
    
    def __init__(self, config_path: str = "config/pid_config.yaml"):
        """
        تهيئة متحكم الزوايا
        
        Args:
            config_path: مسار ملف الإعدادات
        """
        # تحميل الإعدادات
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # إنشاء متحكمات PID للزوايا الثلاثة
        self.roll_controller = self._create_pid_from_config(config['roll'])
        self.pitch_controller = self._create_pid_from_config(config['pitch'])
        self.yaw_controller = self._create_pid_from_config(config['yaw'])
        
        # حالة التفعيل
        self.enabled = True
    
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
    
    def update(self, target_roll: float, target_pitch: float, target_yaw: float,
               current_roll: float, current_pitch: float, current_yaw: float,
               dt: float = 0.01) -> Tuple[float, float, float]:
        """
        تحديث متحكم الزوايا
        
        Args:
            target_roll: زاوية Roll المستهدفة (درجات)
            target_pitch: زاوية Pitch المستهدفة (درجات)
            target_yaw: زاوية Yaw المستهدفة (درجات)
            current_roll: زاوية Roll الحالية (درجات)
            current_pitch: زاوية Pitch الحالية (درجات)
            current_yaw: زاوية Yaw الحالية (درجات)
            dt: الفارق الزمني (ثانية)
        
        Returns:
            (roll_output, pitch_output, yaw_output) - أوامر التحكم
        """
        if not self.enabled:
            return 0.0, 0.0, 0.0
        
        # تطبيع الزوايا
        target_roll = normalize_angle(target_roll)
        target_pitch = normalize_angle(target_pitch)
        target_yaw = normalize_angle(target_yaw)
        current_roll = normalize_angle(current_roll)
        current_pitch = normalize_angle(current_pitch)
        current_yaw = normalize_angle(current_yaw)
        
        # حساب أوامر التحكم
        roll_output = self.roll_controller.update(target_roll, current_roll, dt)
        pitch_output = self.pitch_controller.update(target_pitch, current_pitch, dt)
        yaw_output = self.yaw_controller.update(target_yaw, current_yaw, dt)
        
        return roll_output, pitch_output, yaw_output
    
    def reset(self):
        """إعادة تعيين جميع المتحكمات"""
        self.roll_controller.reset()
        self.pitch_controller.reset()
        self.yaw_controller.reset()
    
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
    
    def get_controller_states(self) -> Dict:
        """
        الحصول على حالات المتحكمات
        
        Returns:
            قاموس يحتوي على حالات المتحكمات
        """
        return {
            'roll': {
                'p': self.roll_controller.last_p_term,
                'i': self.roll_controller.last_i_term,
                'd': self.roll_controller.last_d_term,
                'output': self.roll_controller.last_output,
                'integral': self.roll_controller.get_integral()
            },
            'pitch': {
                'p': self.pitch_controller.last_p_term,
                'i': self.pitch_controller.last_i_term,
                'd': self.pitch_controller.last_d_term,
                'output': self.pitch_controller.last_output,
                'integral': self.pitch_controller.get_integral()
            },
            'yaw': {
                'p': self.yaw_controller.last_p_term,
                'i': self.yaw_controller.last_i_term,
                'd': self.yaw_controller.last_d_term,
                'output': self.yaw_controller.last_output,
                'integral': self.yaw_controller.get_integral()
            }
        }
    
    def set_gains(self, axis: str, kp: float = None, ki: float = None, kd: float = None):
        """
        تعيين معاملات PID لمحور معين
        
        Args:
            axis: المحور ('roll', 'pitch', 'yaw')
            kp: معامل التناسب
            ki: معامل التكامل
            kd: معامل المشتقة
        """
        if axis.lower() == 'roll':
            self.roll_controller.set_gains(kp, ki, kd)
        elif axis.lower() == 'pitch':
            self.pitch_controller.set_gains(kp, ki, kd)
        elif axis.lower() == 'yaw':
            self.yaw_controller.set_gains(kp, ki, kd)
        else:
            raise ValueError(f"محور غير صحيح: {axis}")
    
    def get_gains(self, axis: str) -> Tuple[float, float, float]:
        """
        الحصول على معاملات PID لمحور معين
        
        Args:
            axis: المحور ('roll', 'pitch', 'yaw')
        
        Returns:
            (kp, ki, kd)
        """
        if axis.lower() == 'roll':
            return self.roll_controller.get_gains()
        elif axis.lower() == 'pitch':
            return self.pitch_controller.get_gains()
        elif axis.lower() == 'yaw':
            return self.yaw_controller.get_gains()
        else:
            raise ValueError(f"محور غير صحيح: {axis}")

