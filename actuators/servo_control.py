"""
وحدة التحكم في السيرفو
Servo Control Module
"""

import asyncio
import yaml
from typing import Optional, Dict
from mavsdk import System
from mavsdk.action import ActionError
from utils.math_utils import constrain, map_range
import logging


class ServoController:
    """
    متحكم السيرفوهات للصاروخ
    """
    
    def __init__(self, drone: System, config_path: str = "config/system_config.yaml"):
        """
        تهيئة متحكم السيرفو
        
        Args:
            drone: نظام MAVSDK
            config_path: مسار ملف الإعدادات
        """
        self.drone = drone
        
        # تحميل الإعدادات
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        self.servo_config = config['servos']
        
        # حالة السيرفوهات
        self.servo_positions = {}
        for servo_name in self.servo_config.keys():
            self.servo_positions[servo_name] = self.servo_config[servo_name]['center_pwm']
        
        # السجل
        self.logger = logging.getLogger(__name__)
    
    async def set_servo_pwm(self, servo_name: str, pwm_value: int) -> bool:
        """
        تعيين قيمة PWM للسيرفو
        
        Args:
            servo_name: اسم السيرفو
            pwm_value: قيمة PWM (1000-2000)
        
        Returns:
            True إذا نجحت العملية
        """
        if servo_name not in self.servo_config:
            self.logger.error(f"سيرفو غير معروف: {servo_name}")
            return False
        
        config = self.servo_config[servo_name]
        channel = config['channel']
        
        # تقييد القيمة
        pwm_value = int(constrain(pwm_value, config['min_pwm'], config['max_pwm']))
        
        # عكس الاتجاه إذا لزم الأمر
        if config.get('reverse', False):
            center = config['center_pwm']
            pwm_value = center - (pwm_value - center)
        
        try:
            # إرسال أمر التحكم المباشر
            await self.drone.action.set_actuator(channel, pwm_value / 2000.0)
            self.servo_positions[servo_name] = pwm_value
            return True
        except ActionError as e:
            self.logger.error(f"خطأ في تعيين السيرفو {servo_name}: {e}")
            return False
    
    async def set_servo_angle(self, servo_name: str, angle: float) -> bool:
        """
        تعيين زاوية السيرفو
        
        Args:
            servo_name: اسم السيرفو
            angle: الزاوية (-100 إلى +100)
        
        Returns:
            True إذا نجحت العملية
        """
        if servo_name not in self.servo_config:
            self.logger.error(f"سيرفو غير معروف: {servo_name}")
            return False
        
        config = self.servo_config[servo_name]
        
        # تحويل الزاوية إلى PWM
        pwm_value = map_range(
            angle,
            -100, 100,
            config['min_pwm'], config['max_pwm']
        )
        
        return await self.set_servo_pwm(servo_name, int(pwm_value))
    
    async def center_servo(self, servo_name: str) -> bool:
        """
        وضع السيرفو في المنتصف
        
        Args:
            servo_name: اسم السيرفو
        
        Returns:
            True إذا نجحت العملية
        """
        if servo_name not in self.servo_config:
            self.logger.error(f"سيرفو غير معروف: {servo_name}")
            return False
        
        center_pwm = self.servo_config[servo_name]['center_pwm']
        return await self.set_servo_pwm(servo_name, center_pwm)
    
    async def center_all_servos(self) -> bool:
        """
        وضع جميع السيرفوهات في المنتصف
        
        Returns:
            True إذا نجحت جميع العمليات
        """
        results = []
        for servo_name in self.servo_config.keys():
            result = await self.center_servo(servo_name)
            results.append(result)
        
        return all(results)
    
    async def set_fin_deflections(self, roll_cmd: float, pitch_cmd: float, 
                                  yaw_cmd: float) -> bool:
        """
        تعيين انحرافات الزعانف بناءً على أوامر التحكم
        
        Args:
            roll_cmd: أمر Roll (-100 إلى +100)
            pitch_cmd: أمر Pitch (-100 إلى +100)
            yaw_cmd: أمر Yaw (-100 إلى +100)
        
        Returns:
            True إذا نجحت العملية
        """
        # خلط الأوامر (Mixing)
        # هذا مثال بسيط، يمكن تعديله حسب تصميم الصاروخ
        
        # زعنفة 1 (أمامية)
        fin1_angle = pitch_cmd
        
        # زعنفة 2 (يمين)
        fin2_angle = roll_cmd + yaw_cmd
        
        # زعنفة 3 (خلفية)
        fin3_angle = -pitch_cmd
        
        # زعنفة 4 (يسار)
        fin4_angle = -roll_cmd + yaw_cmd
        
        # تطبيق الأوامر
        results = []
        if 'fin_1' in self.servo_config:
            results.append(await self.set_servo_angle('fin_1', fin1_angle))
        if 'fin_2' in self.servo_config:
            results.append(await self.set_servo_angle('fin_2', fin2_angle))
        if 'fin_3' in self.servo_config:
            results.append(await self.set_servo_angle('fin_3', fin3_angle))
        if 'fin_4' in self.servo_config:
            results.append(await self.set_servo_angle('fin_4', fin4_angle))
        
        return all(results)
    
    def get_servo_position(self, servo_name: str) -> Optional[int]:
        """
        الحصول على موضع السيرفو الحالي
        
        Args:
            servo_name: اسم السيرفو
        
        Returns:
            قيمة PWM الحالية أو None
        """
        return self.servo_positions.get(servo_name)
    
    def get_all_positions(self) -> Dict[str, int]:
        """
        الحصول على مواضع جميع السيرفوهات
        
        Returns:
            قاموس بمواضع السيرفوهات
        """
        return self.servo_positions.copy()

