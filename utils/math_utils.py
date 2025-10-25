"""
وحدة الدوال الرياضية المساعدة
Mathematical Utilities Module
"""

import numpy as np
from typing import Tuple, List


def constrain(value: float, min_val: float, max_val: float) -> float:
    """
    تقييد قيمة ضمن نطاق محدد
    
    Args:
        value: القيمة المراد تقييدها
        min_val: الحد الأدنى
        max_val: الحد الأعلى
    
    Returns:
        القيمة المقيدة
    """
    return max(min_val, min(max_val, value))


def map_range(value: float, in_min: float, in_max: float, 
              out_min: float, out_max: float) -> float:
    """
    تحويل قيمة من نطاق إلى نطاق آخر
    
    Args:
        value: القيمة المراد تحويلها
        in_min: الحد الأدنى للنطاق الأصلي
        in_max: الحد الأعلى للنطاق الأصلي
        out_min: الحد الأدنى للنطاق الجديد
        out_max: الحد الأعلى للنطاق الجديد
    
    Returns:
        القيمة المحولة
    """
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def normalize_angle(angle: float) -> float:
    """
    تطبيع زاوية لتكون ضمن النطاق [-180, 180]
    
    Args:
        angle: الزاوية بالدرجات
    
    Returns:
        الزاوية المطبعة
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def quaternion_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    تحويل من Quaternion إلى زوايا أويلر
    
    Args:
        w, x, y, z: مكونات Quaternion
    
    Returns:
        (roll, pitch, yaw) بالدرجات
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    تحويل من زوايا أويلر إلى Quaternion
    
    Args:
        roll, pitch, yaw: الزوايا بالدرجات
    
    Returns:
        (w, x, y, z) مكونات Quaternion
    """
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    
    cy = np.cos(yaw_rad * 0.5)
    sy = np.sin(yaw_rad * 0.5)
    cp = np.cos(pitch_rad * 0.5)
    sp = np.sin(pitch_rad * 0.5)
    cr = np.cos(roll_rad * 0.5)
    sr = np.sin(roll_rad * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return w, x, y, z


def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    حساب المسافة بين نقطتين GPS باستخدام صيغة Haversine
    
    Args:
        lat1, lon1: إحداثيات النقطة الأولى
        lat2, lon2: إحداثيات النقطة الثانية
    
    Returns:
        المسافة بالأمتار
    """
    R = 6371000  # نصف قطر الأرض بالأمتار
    
    lat1_rad = np.radians(lat1)
    lat2_rad = np.radians(lat2)
    delta_lat = np.radians(lat2 - lat1)
    delta_lon = np.radians(lon2 - lon1)
    
    a = np.sin(delta_lat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(delta_lon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    حساب الاتجاه بين نقطتين GPS
    
    Args:
        lat1, lon1: إحداثيات النقطة الأولى
        lat2, lon2: إحداثيات النقطة الثانية
    
    Returns:
        الاتجاه بالدرجات (0-360)
    """
    lat1_rad = np.radians(lat1)
    lat2_rad = np.radians(lat2)
    delta_lon = np.radians(lon2 - lon1)
    
    x = np.sin(delta_lon) * np.cos(lat2_rad)
    y = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(delta_lon)
    
    bearing = np.degrees(np.arctan2(x, y))
    return (bearing + 360) % 360


def low_pass_filter(current_value: float, new_value: float, alpha: float) -> float:
    """
    مرشح تمرير منخفض (Low-pass filter)
    
    Args:
        current_value: القيمة الحالية المصفاة
        new_value: القيمة الجديدة
        alpha: معامل التصفية (0-1)، كلما كان أقل كان التصفية أقوى
    
    Returns:
        القيمة المصفاة الجديدة
    """
    return alpha * new_value + (1 - alpha) * current_value


def moving_average(data: List[float], window_size: int) -> float:
    """
    حساب المتوسط المتحرك
    
    Args:
        data: قائمة البيانات
        window_size: حجم النافذة
    
    Returns:
        المتوسط المتحرك
    """
    if len(data) < window_size:
        return np.mean(data)
    return np.mean(data[-window_size:])


def derivative(current_value: float, previous_value: float, dt: float) -> float:
    """
    حساب المشتقة
    
    Args:
        current_value: القيمة الحالية
        previous_value: القيمة السابقة
        dt: الفارق الزمني
    
    Returns:
        المشتقة
    """
    if dt <= 0:
        return 0.0
    return (current_value - previous_value) / dt


def integral(current_integral: float, value: float, dt: float) -> float:
    """
    حساب التكامل
    
    Args:
        current_integral: قيمة التكامل الحالية
        value: القيمة المراد تكاملها
        dt: الفارق الزمني
    
    Returns:
        التكامل الجديد
    """
    return current_integral + value * dt


def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    إنشاء مصفوفة الدوران من زوايا أويلر
    
    Args:
        roll, pitch, yaw: الزوايا بالدرجات
    
    Returns:
        مصفوفة الدوران 3x3
    """
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    
    # مصفوفة الدوران حول X (Roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    # مصفوفة الدوران حول Y (Pitch)
    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    
    # مصفوفة الدوران حول Z (Yaw)
    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    # مصفوفة الدوران الكلية
    return Rz @ Ry @ Rx


def is_valid_number(value: float) -> bool:
    """
    التحقق من صحة الرقم
    
    Args:
        value: القيمة المراد فحصها
    
    Returns:
        True إذا كانت القيمة صحيحة
    """
    return not (np.isnan(value) or np.isinf(value))


def saturate_rate(current_value: float, target_value: float, 
                  max_rate: float, dt: float) -> float:
    """
    تحديد معدل التغيير
    
    Args:
        current_value: القيمة الحالية
        target_value: القيمة المستهدفة
        max_rate: أقصى معدل تغيير
        dt: الفارق الزمني
    
    Returns:
        القيمة الجديدة مع معدل محدود
    """
    max_change = max_rate * dt
    error = target_value - current_value
    
    if abs(error) <= max_change:
        return target_value
    elif error > 0:
        return current_value + max_change
    else:
        return current_value - max_change

