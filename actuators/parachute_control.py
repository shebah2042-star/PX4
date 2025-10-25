"""
وحدة التحكم في المظلة
Parachute Control Module
"""

import asyncio
import yaml
from mavsdk import System
from mavsdk.action import ActionError
import logging


class ParachuteController:
    """
    متحكم المظلة للصاروخ
    """
    
    def __init__(self, drone: System, config_path: str = "config/system_config.yaml"):
        """
        تهيئة متحكم المظلة
        
        Args:
            drone: نظام MAVSDK
            config_path: مسار ملف الإعدادات
        """
        self.drone = drone
        
        # تحميل الإعدادات
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        self.parachute_config = config['parachute']
        
        # حالة المظلات
        self.drogue_deployed = False
        self.main_deployed = False
        
        # السجل
        self.logger = logging.getLogger(__name__)
    
    async def deploy_drogue(self) -> bool:
        """
        نشر المظلة الأولية (Drogue)
        
        Returns:
            True إذا نجحت العملية
        """
        if self.drogue_deployed:
            self.logger.warning("المظلة الأولية منشورة بالفعل")
            return True
        
        try:
            channel = self.parachute_config['drogue_channel']
            pwm = self.parachute_config['drogue_deploy_pwm']
            
            await self.drone.action.set_actuator(channel, pwm / 2000.0)
            self.drogue_deployed = True
            self.logger.info("تم نشر المظلة الأولية")
            return True
        except ActionError as e:
            self.logger.error(f"خطأ في نشر المظلة الأولية: {e}")
            return False
    
    async def deploy_main(self) -> bool:
        """
        نشر المظلة الرئيسية (Main)
        
        Returns:
            True إذا نجحت العملية
        """
        if self.main_deployed:
            self.logger.warning("المظلة الرئيسية منشورة بالفعل")
            return True
        
        try:
            channel = self.parachute_config['main_channel']
            pwm = self.parachute_config['main_deploy_pwm']
            
            await self.drone.action.set_actuator(channel, pwm / 2000.0)
            self.main_deployed = True
            self.logger.info("تم نشر المظلة الرئيسية")
            return True
        except ActionError as e:
            self.logger.error(f"خطأ في نشر المظلة الرئيسية: {e}")
            return False
    
    async def check_and_deploy(self, altitude: float, phase: str) -> bool:
        """
        فحص ونشر المظلات تلقائياً حسب الارتفاع
        
        Args:
            altitude: الارتفاع الحالي (متر)
            phase: المرحلة الحالية
        
        Returns:
            True إذا تم نشر مظلة
        """
        deployed = False
        
        # نشر المظلة الأولية عند القمة أو عند الارتفاع المحدد
        if not self.drogue_deployed and phase in ['apogee', 'descent']:
            if altitude <= self.parachute_config['drogue_altitude'] or phase == 'apogee':
                await self.deploy_drogue()
                deployed = True
        
        # نشر المظلة الرئيسية عند الارتفاع المحدد
        if not self.main_deployed and self.drogue_deployed:
            if altitude <= self.parachute_config['main_altitude']:
                await self.deploy_main()
                deployed = True
        
        return deployed
    
    def is_drogue_deployed(self) -> bool:
        """
        التحقق من نشر المظلة الأولية
        
        Returns:
            True إذا كانت منشورة
        """
        return self.drogue_deployed
    
    def is_main_deployed(self) -> bool:
        """
        التحقق من نشر المظلة الرئيسية
        
        Returns:
            True إذا كانت منشورة
        """
        return self.main_deployed
    
    def get_status(self) -> dict:
        """
        الحصول على حالة المظلات
        
        Returns:
            قاموس يحتوي على حالة المظلات
        """
        return {
            'drogue_deployed': self.drogue_deployed,
            'main_deployed': self.main_deployed,
            'drogue_altitude': self.parachute_config['drogue_altitude'],
            'main_altitude': self.parachute_config['main_altitude']
        }
    
    def reset(self):
        """إعادة تعيين حالة المظلات"""
        self.drogue_deployed = False
        self.main_deployed = False
        self.logger.info("إعادة تعيين حالة المظلات")

