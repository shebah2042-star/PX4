"""
اختبارات الأدوات المساعدة
Utils Tests
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from utils.math_utils import *
from utils.validation import DataValidator


def test_constrain():
    """اختبار دالة التقييد"""
    print("اختبار دالة constrain...")
    
    assert constrain(5, 0, 10) == 5
    assert constrain(-5, 0, 10) == 0
    assert constrain(15, 0, 10) == 10
    
    print("✓ نجح اختبار constrain")


def test_map_range():
    """اختبار دالة تحويل النطاق"""
    print("اختبار دالة map_range...")
    
    result = map_range(5, 0, 10, 0, 100)
    assert abs(result - 50) < 0.01
    
    result = map_range(0, 0, 10, 0, 100)
    assert abs(result - 0) < 0.01
    
    result = map_range(10, 0, 10, 0, 100)
    assert abs(result - 100) < 0.01
    
    print("✓ نجح اختبار map_range")


def test_normalize_angle():
    """اختبار دالة تطبيع الزاوية"""
    print("اختبار دالة normalize_angle...")
    
    assert abs(normalize_angle(0) - 0) < 0.01
    assert abs(normalize_angle(180) - 180) < 0.01
    assert abs(normalize_angle(-180) - -180) < 0.01
    assert abs(normalize_angle(360) - 0) < 0.01
    assert abs(normalize_angle(270) - -90) < 0.01
    
    print("✓ نجح اختبار normalize_angle")


def test_quaternion_euler_conversion():
    """اختبار تحويل Quaternion و Euler"""
    print("اختبار تحويل Quaternion و Euler...")
    
    # تحويل من Euler إلى Quaternion ثم العودة
    roll, pitch, yaw = 10.0, 20.0, 30.0
    w, x, y, z = euler_to_quaternion(roll, pitch, yaw)
    roll2, pitch2, yaw2 = quaternion_to_euler(w, x, y, z)
    
    assert abs(roll - roll2) < 0.01
    assert abs(pitch - pitch2) < 0.01
    assert abs(yaw - yaw2) < 0.01
    
    print("✓ نجح اختبار تحويل Quaternion و Euler")


def test_calculate_distance():
    """اختبار حساب المسافة"""
    print("اختبار حساب المسافة...")
    
    # نفس النقطة
    distance = calculate_distance(0, 0, 0, 0)
    assert abs(distance) < 0.01
    
    # نقطتان مختلفتان
    distance = calculate_distance(0, 0, 0, 1)
    assert distance > 0
    
    print("✓ نجح اختبار calculate_distance")


def test_low_pass_filter():
    """اختبار مرشح التمرير المنخفض"""
    print("اختبار مرشح التمرير المنخفض...")
    
    current = 0.0
    new = 10.0
    alpha = 0.5
    
    result = low_pass_filter(current, new, alpha)
    assert 0 < result < 10
    
    print("✓ نجح اختبار low_pass_filter")


def test_is_valid_number():
    """اختبار التحقق من صحة الرقم"""
    print("اختبار is_valid_number...")
    
    assert is_valid_number(5.0) == True
    assert is_valid_number(0.0) == True
    assert is_valid_number(-5.0) == True
    assert is_valid_number(np.nan) == False
    assert is_valid_number(np.inf) == False
    
    print("✓ نجح اختبار is_valid_number")


def test_data_validator():
    """اختبار مدقق البيانات"""
    print("اختبار DataValidator...")
    
    validator = DataValidator()
    
    # اختبار التحقق من بيانات المستشعر
    is_valid, msg = validator.validate_sensor_data("test", 5.0, min_val=0, max_val=10)
    assert is_valid == True
    
    is_valid, msg = validator.validate_sensor_data("test", 15.0, min_val=0, max_val=10)
    assert is_valid == False
    
    # اختبار التحقق من GPS
    is_valid, msg = validator.validate_gps_data(25.0, 45.0, 100.0, 8, 1.5)
    assert is_valid == True
    
    is_valid, msg = validator.validate_gps_data(95.0, 45.0, 100.0, 8, 1.5)
    assert is_valid == False
    
    # اختبار التحقق من IMU
    is_valid, msg = validator.validate_imu_data(1.0, 2.0, 9.8, 0.1, 0.2, 0.3)
    assert is_valid == True
    
    print("✓ نجح اختبار DataValidator")


def run_all_tests():
    """تشغيل جميع الاختبارات"""
    print("=" * 60)
    print("بدء اختبارات الأدوات المساعدة")
    print("=" * 60)
    
    test_constrain()
    test_map_range()
    test_normalize_angle()
    test_quaternion_euler_conversion()
    test_calculate_distance()
    test_low_pass_filter()
    test_is_valid_number()
    test_data_validator()
    
    print("=" * 60)
    print("✓ نجحت جميع الاختبارات!")
    print("=" * 60)


if __name__ == "__main__":
    run_all_tests()

