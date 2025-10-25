"""
اختبارات المتحكمات
Controllers Tests
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from controllers.pid_controller import PIDController
from controllers.attitude_controller import AttitudeController
from controllers.altitude_controller import AltitudeController


def test_pid_controller():
    """اختبار متحكم PID"""
    print("اختبار متحكم PID...")
    
    # إنشاء متحكم
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    
    # اختبار التحديث
    output = pid.update(setpoint=10.0, measured_value=5.0, dt=0.01)
    assert output != 0, "يجب أن يكون المخرج غير صفر"
    
    # اختبار إعادة التعيين
    pid.reset()
    assert pid.get_integral() == 0, "يجب أن يكون التكامل صفراً بعد إعادة التعيين"
    
    print("✓ نجح اختبار متحكم PID")


def test_pid_limits():
    """اختبار حدود PID"""
    print("اختبار حدود PID...")
    
    pid = PIDController(kp=10.0, ki=1.0, kd=1.0, output_min=-10, output_max=10)
    
    # اختبار التقييد
    output = pid.update(setpoint=100.0, measured_value=0.0, dt=0.01)
    assert -10 <= output <= 10, "يجب أن يكون المخرج ضمن الحدود"
    
    print("✓ نجح اختبار حدود PID")


def test_attitude_controller():
    """اختبار متحكم الزوايا"""
    print("اختبار متحكم الزوايا...")
    
    # إنشاء متحكم (سيستخدم الإعدادات الافتراضية)
    try:
        controller = AttitudeController()
        
        # اختبار التحديث
        roll_out, pitch_out, yaw_out = controller.update(
            target_roll=0.0, target_pitch=0.0, target_yaw=0.0,
            current_roll=5.0, current_pitch=3.0, current_yaw=2.0,
            dt=0.01
        )
        
        assert roll_out != 0 or pitch_out != 0 or yaw_out != 0, "يجب أن يكون هناك مخرج"
        
        # اختبار التفعيل/التعطيل
        controller.disable()
        assert not controller.is_enabled(), "يجب أن يكون المتحكم معطلاً"
        
        controller.enable()
        assert controller.is_enabled(), "يجب أن يكون المتحكم مفعلاً"
        
        print("✓ نجح اختبار متحكم الزوايا")
    except FileNotFoundError:
        print("⚠ تخطي اختبار متحكم الزوايا (ملف الإعدادات غير موجود)")


def test_altitude_controller():
    """اختبار متحكم الارتفاع"""
    print("اختبار متحكم الارتفاع...")
    
    try:
        controller = AltitudeController()
        
        # اختبار التحكم في الارتفاع
        output = controller.update_altitude(
            target_altitude=100.0,
            current_altitude=50.0,
            dt=0.01
        )
        
        assert output != 0, "يجب أن يكون المخرج غير صفر"
        
        # اختبار التحكم المتسلسل
        output = controller.update_cascade(
            target_altitude=100.0,
            current_altitude=50.0,
            current_velocity=5.0,
            dt=0.01
        )
        
        assert isinstance(output, (int, float)), "يجب أن يكون المخرج رقماً"
        
        print("✓ نجح اختبار متحكم الارتفاع")
    except FileNotFoundError:
        print("⚠ تخطي اختبار متحكم الارتفاع (ملف الإعدادات غير موجود)")


def run_all_tests():
    """تشغيل جميع الاختبارات"""
    print("=" * 60)
    print("بدء اختبارات المتحكمات")
    print("=" * 60)
    
    test_pid_controller()
    test_pid_limits()
    test_attitude_controller()
    test_altitude_controller()
    
    print("=" * 60)
    print("✓ نجحت جميع الاختبارات!")
    print("=" * 60)


if __name__ == "__main__":
    run_all_tests()

