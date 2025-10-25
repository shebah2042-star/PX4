"""
آلة الحالة (State Machine)
Manages flight phases and transitions
"""

import time
from enum import Enum
from typing import Optional, Callable, Dict
import logging


class FlightPhase(Enum):
    """مراحل الطيران"""
    PRE_LAUNCH = "pre_launch"
    LAUNCH = "launch"
    ASCENT = "ascent"
    APOGEE = "apogee"
    DESCENT = "descent"
    RECOVERY = "recovery"
    LANDED = "landed"
    ABORT = "abort"


class StateMachine:
    """
    آلة الحالة لإدارة مراحل طيران الصاروخ
    """
    
    def __init__(self):
        """تهيئة آلة الحالة"""
        self.current_phase = FlightPhase.PRE_LAUNCH
        self.previous_phase = None
        self.phase_start_time = time.time()
        self.flight_start_time = None
        
        # معايير الانتقال
        self.transition_criteria = {
            FlightPhase.PRE_LAUNCH: self._check_launch_criteria,
            FlightPhase.LAUNCH: self._check_ascent_criteria,
            FlightPhase.ASCENT: self._check_apogee_criteria,
            FlightPhase.APOGEE: self._check_descent_criteria,
            FlightPhase.DESCENT: self._check_recovery_criteria,
            FlightPhase.RECOVERY: self._check_landed_criteria,
        }
        
        # دوال رد الفعل عند الدخول إلى مرحلة
        self.on_enter_callbacks: Dict[FlightPhase, list] = {phase: [] for phase in FlightPhase}
        
        # دوال رد الفعل عند الخروج من مرحلة
        self.on_exit_callbacks: Dict[FlightPhase, list] = {phase: [] for phase in FlightPhase}
        
        # البيانات المشتركة
        self.shared_data = {}
        
        # السجل
        self.logger = logging.getLogger(__name__)
    
    def update(self, sensor_data: dict) -> FlightPhase:
        """
        تحديث آلة الحالة
        
        Args:
            sensor_data: بيانات المستشعرات
        
        Returns:
            المرحلة الحالية
        """
        # تحديث البيانات المشتركة
        self.shared_data.update(sensor_data)
        
        # التحقق من معايير الانتقال
        if self.current_phase in self.transition_criteria:
            should_transition, next_phase = self.transition_criteria[self.current_phase](sensor_data)
            
            if should_transition:
                self.transition_to(next_phase)
        
        return self.current_phase
    
    def transition_to(self, new_phase: FlightPhase):
        """
        الانتقال إلى مرحلة جديدة
        
        Args:
            new_phase: المرحلة الجديدة
        """
        if new_phase == self.current_phase:
            return
        
        self.logger.info(f"الانتقال من {self.current_phase.value} إلى {new_phase.value}")
        
        # تنفيذ دوال الخروج
        for callback in self.on_exit_callbacks[self.current_phase]:
            callback(self.current_phase, new_phase)
        
        # تحديث الحالة
        self.previous_phase = self.current_phase
        self.current_phase = new_phase
        self.phase_start_time = time.time()
        
        # تسجيل وقت بدء الطيران
        if new_phase == FlightPhase.LAUNCH and self.flight_start_time is None:
            self.flight_start_time = time.time()
        
        # تنفيذ دوال الدخول
        for callback in self.on_enter_callbacks[new_phase]:
            callback(self.previous_phase, new_phase)
    
    def _check_launch_criteria(self, data: dict) -> tuple:
        """
        التحقق من معايير الإطلاق
        
        Args:
            data: بيانات المستشعرات
        
        Returns:
            (should_transition, next_phase)
        """
        # كشف الإطلاق بناءً على التسارع
        accel_z = data.get('accel_z', 0)
        
        # إذا كان التسارع أكبر من 2g
        if accel_z > 20.0:
            return True, FlightPhase.LAUNCH
        
        return False, None
    
    def _check_ascent_criteria(self, data: dict) -> tuple:
        """
        التحقق من معايير الصعود
        
        Args:
            data: بيانات المستشعرات
        
        Returns:
            (should_transition, next_phase)
        """
        # الانتقال إلى مرحلة الصعود بعد ثانية واحدة من الإطلاق
        time_in_phase = time.time() - self.phase_start_time
        
        if time_in_phase > 1.0:
            return True, FlightPhase.ASCENT
        
        return False, None
    
    def _check_apogee_criteria(self, data: dict) -> tuple:
        """
        التحقق من معايير القمة
        
        Args:
            data: بيانات المستشعرات
        
        Returns:
            (should_transition, next_phase)
        """
        # كشف القمة
        apogee_detected = data.get('apogee_detected', False)
        
        if apogee_detected:
            return True, FlightPhase.APOGEE
        
        return False, None
    
    def _check_descent_criteria(self, data: dict) -> tuple:
        """
        التحقق من معايير الهبوط
        
        Args:
            data: بيانات المستشعرات
        
        Returns:
            (should_transition, next_phase)
        """
        # الانتقال إلى مرحلة الهبوط بعد نشر المظلة
        drogue_deployed = data.get('drogue_deployed', False)
        
        if drogue_deployed:
            return True, FlightPhase.DESCENT
        
        return False, None
    
    def _check_recovery_criteria(self, data: dict) -> tuple:
        """
        التحقق من معايير الاسترجاع
        
        Args:
            data: بيانات المستشعرات
        
        Returns:
            (should_transition, next_phase)
        """
        # الانتقال إلى مرحلة الاسترجاع بعد نشر المظلة الرئيسية
        main_deployed = data.get('main_deployed', False)
        
        if main_deployed:
            return True, FlightPhase.RECOVERY
        
        return False, None
    
    def _check_landed_criteria(self, data: dict) -> tuple:
        """
        التحقق من معايير الهبوط
        
        Args:
            data: بيانات المستشعرات
        
        Returns:
            (should_transition, next_phase)
        """
        # كشف الهبوط بناءً على السرعة العمودية والارتفاع
        vertical_velocity = data.get('vertical_velocity', 0)
        altitude = data.get('altitude', 1000)
        
        if abs(vertical_velocity) < 0.5 and altitude < 10:
            return True, FlightPhase.LANDED
        
        return False, None
    
    def register_on_enter(self, phase: FlightPhase, callback: Callable):
        """
        تسجيل دالة رد فعل عند الدخول إلى مرحلة
        
        Args:
            phase: المرحلة
            callback: دالة رد الفعل
        """
        self.on_enter_callbacks[phase].append(callback)
    
    def register_on_exit(self, phase: FlightPhase, callback: Callable):
        """
        تسجيل دالة رد فعل عند الخروج من مرحلة
        
        Args:
            phase: المرحلة
            callback: دالة رد الفعل
        """
        self.on_exit_callbacks[phase].append(callback)
    
    def get_current_phase(self) -> FlightPhase:
        """
        الحصول على المرحلة الحالية
        
        Returns:
            المرحلة الحالية
        """
        return self.current_phase
    
    def get_time_in_phase(self) -> float:
        """
        الحصول على الوقت المنقضي في المرحلة الحالية
        
        Returns:
            الوقت بالثواني
        """
        return time.time() - self.phase_start_time
    
    def get_flight_time(self) -> Optional[float]:
        """
        الحصول على وقت الطيران الكلي
        
        Returns:
            الوقت بالثواني أو None
        """
        if self.flight_start_time is None:
            return None
        return time.time() - self.flight_start_time
    
    def force_transition(self, phase: FlightPhase):
        """
        إجبار الانتقال إلى مرحلة معينة
        
        Args:
            phase: المرحلة المستهدفة
        """
        self.logger.warning(f"انتقال قسري إلى {phase.value}")
        self.transition_to(phase)
    
    def abort_flight(self):
        """إلغاء الطيران"""
        self.logger.critical("إلغاء الطيران!")
        self.transition_to(FlightPhase.ABORT)
    
    def reset(self):
        """إعادة تعيين آلة الحالة"""
        self.current_phase = FlightPhase.PRE_LAUNCH
        self.previous_phase = None
        self.phase_start_time = time.time()
        self.flight_start_time = None
        self.shared_data.clear()
        self.logger.info("إعادة تعيين آلة الحالة")

