"""
البرنامج الرئيسي المحسّن لنظام التحكم في الصاروخ
Enhanced Rocket Control System - Main Program with Full PX4 Control
"""

import asyncio
import logging
import signal
import sys
from datetime import datetime

# استيراد الوحدات
from core.mavlink_interface import MAVLinkInterface
from core.px4_direct_control import PX4DirectControl
from core.state_machine import StateMachine, FlightPhase
from core.data_logger import DataLogger
from controllers.attitude_controller import AttitudeController
from controllers.altitude_controller import AltitudeController
from algorithms.kalman_filter import AltitudeKalmanFilter
from algorithms.apogee_detector import ApogeeDetector
from actuators.servo_control import ServoController
from actuators.parachute_control import ParachuteController


class EnhancedRocketControlSystem:
    """
    نظام التحكم المحسّن للصاروخ مع تحكم كامل عبر MAVLink
    """
    
    def __init__(self):
        """تهيئة النظام"""
        # إعداد السجل
        self.setup_logging()
        self.logger = logging.getLogger(__name__)
        self.logger.info("=" * 80)
        self.logger.info("نظام التحكم المحسّن في الصاروخ - Enhanced Rocket Control System")
        self.logger.info("التحكم الكامل عبر الحاسوب المرفق - Full Companion Computer Control")
        self.logger.info("=" * 80)
        
        # واجهة MAVLink
        self.mavlink = MAVLinkInterface()
        
        self.px4_control = None
        
        # آلة الحالة
        self.state_machine = StateMachine()
        
        # مسجل البيانات
        self.data_logger = DataLogger()
        
        # المتحكمات
        self.attitude_controller = AttitudeController()
        self.altitude_controller = AltitudeController()
        
        # الخوارزميات
        self.altitude_filter = AltitudeKalmanFilter()
        self.apogee_detector = ApogeeDetector()
        
        # المحركات (سيتم تهيئتها بعد الاتصال)
        self.servo_controller = None
        self.parachute_controller = None
        
        # حالة النظام
        self.running = False
        self.control_enabled = False
        self.offboard_mode_active = False
        
        # معدلات التحديث
        self.control_loop_dt = 0.01  # 100 Hz
        
        # تسجيل دوال رد الفعل
        self.register_state_callbacks()
    
    def setup_logging(self):
        """إعداد نظام السجلات"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(f'logs/enhanced_rocket_system_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
    
    def register_state_callbacks(self):
        """تسجيل دوال رد الفعل لآلة الحالة"""
        # عند الدخول إلى مرحلة الإطلاق
        self.state_machine.register_on_enter(
            FlightPhase.LAUNCH,
            lambda prev, curr: self.on_launch()
        )
        
        self.state_machine.register_on_enter(
            FlightPhase.ASCENT,
            lambda prev, curr: self.on_ascent()
        )
        
        # عند الدخول إلى مرحلة القمة
        self.state_machine.register_on_enter(
            FlightPhase.APOGEE,
            lambda prev, curr: self.on_apogee()
        )
        
        # عند الدخول إلى مرحلة الهبوط
        self.state_machine.register_on_enter(
            FlightPhase.DESCENT,
            lambda prev, curr: self.on_descent()
        )
    
    async def initialize(self) -> bool:
        """
        تهيئة النظام
        
        Returns:
            True إذا نجحت التهيئة
        """
        self.logger.info("بدء تهيئة النظام...")
        
        # الاتصال بـ PX4
        if not await self.mavlink.connect():
            self.logger.error("فشل الاتصال بـ PX4")
            return False
        
        self.px4_control = PX4DirectControl(self.mavlink.drone)
        
        # تهيئة المحركات
        self.servo_controller = ServoController(self.mavlink.drone)
        self.parachute_controller = ParachuteController(self.mavlink.drone)
        
        # بدء استقبال البيانات
        await self.mavlink.start_telemetry()
        
        # انتظار استقرار البيانات
        await asyncio.sleep(2)
        
        self.logger.info("تسليح النظام...")
        if not await self.px4_control.arm():
            self.logger.error("فشل تسليح النظام")
            return False
        
        self.logger.info("تفعيل وضع التحكم المباشر...")
        if not await self.px4_control.start_offboard_mode():
            self.logger.error("فشل تفعيل وضع Offboard")
            return False
        
        self.offboard_mode_active = True
        
        # وضع السيرفوهات في المنتصف
        await self.servo_controller.center_all_servos()
        
        # بدء التسجيل
        self.data_logger.start_logging()
        
        self.logger.info("تم تهيئة النظام بنجاح - التحكم الكامل نشط")
        return True
    
    async def run(self):
        """تشغيل النظام الرئيسي"""
        self.running = True
        self.logger.info("بدء التشغيل - التحكم الكامل عبر الحاسوب المرفق...")
        
        # حلقة التحكم الرئيسية
        while self.running:
            try:
                await self.control_loop()
                await asyncio.sleep(self.control_loop_dt)
            except Exception as e:
                self.logger.error(f"خطأ في حلقة التحكم: {e}")
                await asyncio.sleep(0.1)
    
    async def control_loop(self):
        """حلقة التحكم الرئيسية"""
        # قراءة بيانات المستشعرات
        attitude = self.mavlink.get_attitude()
        position = self.mavlink.get_position()
        velocity = self.mavlink.get_velocity()
        imu = self.mavlink.get_imu()
        
        # تصفية الارتفاع
        altitude_filtered, velocity_filtered = self.altitude_filter.update(
            position['alt'], self.control_loop_dt
        )
        
        # كشف القمة
        apogee_detected = self.apogee_detector.update(
            altitude_filtered,
            position['alt'],
            velocity_filtered,
            imu['accel_z']
        )
        
        # تحديث آلة الحالة
        sensor_data = {
            'altitude': altitude_filtered,
            'vertical_velocity': velocity_filtered,
            'accel_z': imu['accel_z'],
            'apogee_detected': apogee_detected,
            'drogue_deployed': self.parachute_controller.is_drogue_deployed(),
            'main_deployed': self.parachute_controller.is_main_deployed()
        }
        
        current_phase = self.state_machine.update(sensor_data)
        
        # التحكم حسب المرحلة
        if current_phase == FlightPhase.PRE_LAUNCH:
            await self.handle_pre_launch()
        elif current_phase == FlightPhase.ASCENT:
            await self.handle_ascent(attitude, altitude_filtered, velocity_filtered)
        elif current_phase == FlightPhase.APOGEE:
            await self.handle_apogee_phase(altitude_filtered)
        elif current_phase == FlightPhase.DESCENT:
            await self.handle_descent(altitude_filtered)
        
        # تسجيل البيانات
        self.log_data(attitude, position, velocity, imu, current_phase)
    
    async def handle_pre_launch(self):
        """معالجة مرحلة ما قبل الإطلاق"""
        # الحفاظ على السيرفوهات في المنتصف
        if not self.control_enabled and self.offboard_mode_active:
            await self.px4_control.set_fin_deflections(0.0, 0.0, 0.0)
    
    async def handle_ascent(self, attitude: dict, altitude: float, velocity: float):
        """معالجة مرحلة الصعود - التحكم الكامل"""
        # تفعيل التحكم
        if not self.control_enabled:
            self.control_enabled = True
            self.logger.info("تفعيل التحكم الكامل في الصاروخ")
        
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw = 0.0
        
        # حساب أوامر التحكم
        roll_cmd, pitch_cmd, yaw_cmd = self.attitude_controller.update(
            target_roll, target_pitch, target_yaw,
            attitude['roll'], attitude['pitch'], attitude['yaw'],
            self.control_loop_dt
        )
        
        if self.offboard_mode_active:
            await self.px4_control.set_fin_deflections(roll_cmd, pitch_cmd, yaw_cmd)
    
    async def handle_apogee_phase(self, altitude: float):
        """معالجة مرحلة القمة"""
        # نشر المظلة الأولية
        await self.parachute_controller.deploy_drogue()
        
        if self.control_enabled:
            self.control_enabled = False
            await self.px4_control.set_fin_deflections(0.0, 0.0, 0.0)
    
    async def handle_descent(self, altitude: float):
        """معالجة مرحلة الهبوط"""
        # فحص ونشر المظلات
        await self.parachute_controller.check_and_deploy(
            altitude,
            self.state_machine.get_current_phase().value
        )
    
    def on_launch(self):
        """عند الإطلاق"""
        self.logger.info("🚀 الإطلاق!")
        self.data_logger.log_event("LAUNCH", "تم إطلاق الصاروخ")
    
    def on_ascent(self):
        """عند بدء الصعود"""
        self.logger.info("⬆️  بدء مرحلة الصعود - التحكم الكامل نشط")
        self.data_logger.log_event("ASCENT", "بدء مرحلة الصعود")
    
    def on_apogee(self):
        """عند الوصول للقمة"""
        apogee_alt = self.apogee_detector.get_apogee_altitude()
        self.logger.info(f"🎯 الوصول للقمة - الارتفاع: {apogee_alt:.2f} متر")
        self.data_logger.log_event("APOGEE", f"الوصول للقمة", {'altitude': apogee_alt})
    
    def on_descent(self):
        """عند بدء الهبوط"""
        self.logger.info("🪂 بدء الهبوط")
        self.data_logger.log_event("DESCENT", "بدء مرحلة الهبوط")
    
    def log_data(self, attitude: dict, position: dict, velocity: dict, 
                 imu: dict, phase: FlightPhase):
        """تسجيل البيانات"""
        # تسجيل بيانات المستشعرات
        sensor_data = {
            **attitude,
            **position,
            **velocity,
            **imu,
            'pressure': 0,
            'temperature': 0,
            'baro_alt': position['alt'],
            'num_satellites': 0,
            'battery_voltage': 0
        }
        self.data_logger.log_sensor_data(sensor_data)
        
        # تسجيل بيانات التحكم
        control_data = {
            'phase': phase.value,
            'control_enabled': self.control_enabled,
            'offboard_active': self.offboard_mode_active,
            'target_roll': 0,
            'target_pitch': 0,
            'target_yaw': 0,
            'roll_output': 0,
            'pitch_output': 0,
            'yaw_output': 0,
            'fin1': 0,
            'fin2': 0,
            'fin3': 0,
            'fin4': 0,
            'drogue_deployed': self.parachute_controller.is_drogue_deployed(),
            'main_deployed': self.parachute_controller.is_main_deployed()
        }
        self.data_logger.log_control_data(control_data)
    
    async def shutdown(self):
        """إيقاف النظام"""
        self.logger.info("إيقاف النظام...")
        self.running = False
        
        if self.offboard_mode_active:
            await self.px4_control.set_fin_deflections(0.0, 0.0, 0.0)
            await self.px4_control.stop_offboard_mode()
        
        # وضع السيرفوهات في المنتصف
        if self.servo_controller:
            await self.servo_controller.center_all_servos()
        
        await self.px4_control.disarm()
        
        # إيقاف التسجيل
        self.data_logger.stop_logging()
        
        # حفظ الملخص
        summary = {
            'session': self.data_logger.get_session_name(),
            'max_altitude': self.apogee_detector.get_apogee_altitude(),
            'flight_time': self.state_machine.get_flight_time(),
            'final_phase': self.state_machine.get_current_phase().value
        }
        self.data_logger.save_summary(summary)
        
        # قطع الاتصال
        await self.mavlink.disconnect()
        
        self.logger.info("تم إيقاف النظام بنجاح")


async def main():
    """الدالة الرئيسية"""
    # إنشاء النظام
    system = EnhancedRocketControlSystem()
    
    # معالج الإشارات للإيقاف الآمن
    def signal_handler(sig, frame):
        print("\n\nإيقاف النظام...")
        asyncio.create_task(system.shutdown())
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # تهيئة النظام
    if not await system.initialize():
        print("فشل تهيئة النظام")
        return
    
    # تشغيل النظام
    try:
        await system.run()
    except KeyboardInterrupt:
        await system.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
