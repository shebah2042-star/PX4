"""
متحكم PID
PID Controller Module
"""

import time
from typing import Optional
from utils.math_utils import constrain, is_valid_number


class PIDController:
    """
    متحكم PID (Proportional-Integral-Derivative)
    """
    
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -100.0, output_max: float = 100.0,
                 integral_min: float = -10.0, integral_max: float = 10.0,
                 derivative_filter: float = 0.0):
        """
        تهيئة متحكم PID
        
        Args:
            kp: معامل التناسب (Proportional gain)
            ki: معامل التكامل (Integral gain)
            kd: معامل المشتقة (Derivative gain)
            output_min: الحد الأدنى للمخرج
            output_max: الحد الأعلى للمخرج
            integral_min: الحد الأدنى للتكامل
            integral_max: الحد الأعلى للتكامل
            derivative_filter: معامل تصفية المشتقة (0-1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.derivative_filter = derivative_filter
        
        # المتغيرات الداخلية
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_derivative = 0.0
        self.previous_time = None
        
        # للإحصائيات
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        self.last_output = 0.0
    
    def update(self, setpoint: float, measured_value: float, 
               dt: Optional[float] = None) -> float:
        """
        تحديث متحكم PID
        
        Args:
            setpoint: القيمة المستهدفة
            measured_value: القيمة المقاسة
            dt: الفارق الزمني (إذا كان None، سيتم حسابه تلقائياً)
        
        Returns:
            قيمة المخرج
        """
        # حساب الفارق الزمني
        if dt is None:
            current_time = time.time()
            if self.previous_time is None:
                self.previous_time = current_time
                return 0.0
            dt = current_time - self.previous_time
            self.previous_time = current_time
        
        # التأكد من صحة dt
        if dt <= 0 or dt > 1.0:
            dt = 0.01
        
        # حساب الخطأ
        error = setpoint - measured_value
        
        # التحقق من صحة القيم
        if not is_valid_number(error):
            return self.last_output
        
        # حساب المكون التناسبي (P)
        p_term = self.kp * error
        
        # حساب المكون التكاملي (I)
        self.integral += error * dt
        self.integral = constrain(self.integral, self.integral_min, self.integral_max)
        i_term = self.ki * self.integral
        
        # حساب المكون التفاضلي (D)
        derivative = (error - self.previous_error) / dt
        
        # تصفية المشتقة
        if self.derivative_filter > 0:
            derivative = (self.derivative_filter * derivative + 
                         (1 - self.derivative_filter) * self.previous_derivative)
        
        d_term = self.kd * derivative
        
        # حساب المخرج الكلي
        output = p_term + i_term + d_term
        
        # تقييد المخرج
        output = constrain(output, self.output_min, self.output_max)
        
        # حفظ القيم للدورة القادمة
        self.previous_error = error
        self.previous_derivative = derivative
        
        # حفظ القيم للإحصائيات
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term
        self.last_output = output
        
        return output
    
    def reset(self):
        """إعادة تعيين المتحكم"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_derivative = 0.0
        self.previous_time = None
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        self.last_output = 0.0
    
    def set_gains(self, kp: Optional[float] = None, 
                  ki: Optional[float] = None, 
                  kd: Optional[float] = None):
        """
        تعيين معاملات PID
        
        Args:
            kp: معامل التناسب
            ki: معامل التكامل
            kd: معامل المشتقة
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def get_gains(self) -> tuple:
        """
        الحصول على معاملات PID
        
        Returns:
            (kp, ki, kd)
        """
        return self.kp, self.ki, self.kd
    
    def get_terms(self) -> tuple:
        """
        الحصول على مكونات PID الأخيرة
        
        Returns:
            (p_term, i_term, d_term)
        """
        return self.last_p_term, self.last_i_term, self.last_d_term
    
    def get_integral(self) -> float:
        """
        الحصول على قيمة التكامل الحالية
        
        Returns:
            قيمة التكامل
        """
        return self.integral
    
    def set_integral(self, value: float):
        """
        تعيين قيمة التكامل
        
        Args:
            value: قيمة التكامل الجديدة
        """
        self.integral = constrain(value, self.integral_min, self.integral_max)
    
    def set_output_limits(self, min_val: float, max_val: float):
        """
        تعيين حدود المخرج
        
        Args:
            min_val: الحد الأدنى
            max_val: الحد الأعلى
        """
        self.output_min = min_val
        self.output_max = max_val
    
    def set_integral_limits(self, min_val: float, max_val: float):
        """
        تعيين حدود التكامل
        
        Args:
            min_val: الحد الأدنى
            max_val: الحد الأعلى
        """
        self.integral_min = min_val
        self.integral_max = max_val
        # إعادة تقييد التكامل الحالي
        self.integral = constrain(self.integral, min_val, max_val)

