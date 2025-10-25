"""
وحدة التحكم المباشر في PX4
PX4 Direct Control Module - Full control via MAVLink
"""

import asyncio
import logging
from mavsdk import System
from mavsdk.offboard import (OffboardError, ActuatorControl, 
                              ActuatorControlGroup, Offboard)
from typing import Optional


class PX4DirectControl:
    """
    التحكم المباشر الكامل في PX4 عبر MAVLink
    يوفر تحكم كامل في المحركات والسيرفوهات
    """
    
    def __init__(self, drone: System):
        """
        تهيئة وحدة التحكم المباشر
        
        Args:
            drone: نظام MAVSDK
        """
        self.drone = drone
        self.logger = logging.getLogger(__name__)
        self.offboard_mode_active = False
        
    async def start_offboard_mode(self) -> bool:
        """
        بدء وضع التحكم المباشر (Offboard Mode)
        
        Returns:
            True إذا نجح التفعيل
        """
        try:
            await self.set_actuator_controls([0.0] * 8)
            
            await self.drone.offboard.start()
            self.offboard_mode_active = True
            self.logger.info("تم تفعيل وضع التحكم المباشر (Offboard)")
            return True
        except OffboardError as e:
            self.logger.error(f"خطأ في تفعيل وضع Offboard: {e}")
            return False
    
    async def stop_offboard_mode(self) -> bool:
        """
        إيقاف وضع التحكم المباشر
        
        Returns:
            True إذا نجح الإيقاف
        """
        try:
            await self.drone.offboard.stop()
            self.offboard_mode_active = False
            self.logger.info("تم إيقاف وضع التحكم المباشر")
            return True
        except OffboardError as e:
            self.logger.error(f"خطأ في إيقاف وضع Offboard: {e}")
            return False
    
    async def set_actuator_controls(self, controls: list) -> bool:
        """
        تعيين قيم التحكم المباشرة للمحركات
        
        Args:
            controls: قائمة بقيم التحكم (8 قيم من -1.0 إلى 1.0)
        
        Returns:
            True إذا نجحت العملية
        """
        if len(controls) != 8:
            self.logger.error("يجب أن تحتوي قائمة التحكم على 8 قيم")
            return False
        
        try:
            actuator_control = ActuatorControl(
                groups=[
                    ActuatorControlGroup(
                        controls=controls
                    )
                ]
            )
            
            await self.drone.offboard.set_actuator_control(actuator_control)
            return True
        except Exception as e:
            self.logger.error(f"خطأ في تعيين قيم التحكم: {e}")
            return False
    
    async def set_servo_positions(self, servo_values: dict) -> bool:
        """
        تعيين مواضع السيرفوهات
        
        Args:
            servo_values: قاموس بأرقام السيرفوهات وقيمها
                         مثال: {0: 0.5, 1: -0.3, 2: 0.0, 3: 0.2}
        
        Returns:
            True إذا نجحت العملية
        """
        controls = [0.0] * 8
        
        for channel, value in servo_values.items():
            if 0 <= channel < 8:
                controls[channel] = max(-1.0, min(1.0, value))
        
        return await self.set_actuator_controls(controls)
    
    async def set_fin_deflections(self, roll: float, pitch: float, yaw: float) -> bool:
        """
        تعيين انحرافات الزعانف بناءً على أوامر التحكم
        
        Args:
            roll: أمر Roll (-1.0 إلى 1.0)
            pitch: أمر Pitch (-1.0 إلى 1.0)
            yaw: أمر Yaw (-1.0 إلى 1.0)
        
        Returns:
            True إذا نجحت العملية
        """
        
        fin1 = pitch  # أمامية
        fin2 = roll + yaw  # يمين
        fin3 = -pitch  # خلفية
        fin4 = -roll + yaw  # يسار
        
        servo_values = {
            0: fin1,
            1: fin2,
            2: fin3,
            3: fin4
        }
        
        return await self.set_servo_positions(servo_values)
    
    async def arm(self) -> bool:
        """
        تسليح النظام
        
        Returns:
            True إذا نجح التسليح
        """
        try:
            await self.drone.action.arm()
            self.logger.info("تم تسليح النظام")
            return True
        except Exception as e:
            self.logger.error(f"خطأ في التسليح: {e}")
            return False
    
    async def disarm(self) -> bool:
        """
        نزع سلاح النظام
        
        Returns:
            True إذا نجح نزع السلاح
        """
        try:
            await self.drone.action.disarm()
            self.logger.info("تم نزع سلاح النظام")
            return True
        except Exception as e:
            self.logger.error(f"خطأ في نزع السلاح: {e}")
            return False
    
    def is_offboard_active(self) -> bool:
        """
        التحقق من حالة وضع Offboard
        
        Returns:
            True إذا كان وضع Offboard نشطاً
        """
        return self.offboard_mode_active
