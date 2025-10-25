"""
مرشح كالمان (Kalman Filter)
For sensor fusion and state estimation
"""

import numpy as np
from typing import Optional


class KalmanFilter:
    """
    مرشح كالمان لدمج بيانات المستشعرات وتقدير الحالة
    """
    
    def __init__(self, state_dim: int, measurement_dim: int):
        """
        تهيئة مرشح كالمان
        
        Args:
            state_dim: عدد أبعاد الحالة
            measurement_dim: عدد أبعاد القياس
        """
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # متجه الحالة
        self.x = np.zeros((state_dim, 1))
        
        # مصفوفة التباين
        self.P = np.eye(state_dim)
        
        # مصفوفة الانتقال
        self.F = np.eye(state_dim)
        
        # مصفوفة القياس
        self.H = np.zeros((measurement_dim, state_dim))
        
        # مصفوفة ضوضاء العملية
        self.Q = np.eye(state_dim) * 0.01
        
        # مصفوفة ضوضاء القياس
        self.R = np.eye(measurement_dim) * 0.1
        
        # مصفوفة التحكم
        self.B = None
        
        # مصفوفة كسب كالمان
        self.K = np.zeros((state_dim, measurement_dim))
    
    def predict(self, u: Optional[np.ndarray] = None):
        """
        مرحلة التنبؤ
        
        Args:
            u: متجه التحكم (اختياري)
        """
        # تنبؤ الحالة
        if u is not None and self.B is not None:
            self.x = self.F @ self.x + self.B @ u
        else:
            self.x = self.F @ self.x
        
        # تنبؤ التباين
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z: np.ndarray):
        """
        مرحلة التحديث
        
        Args:
            z: متجه القياس
        """
        # حساب الابتكار (Innovation)
        y = z - self.H @ self.x
        
        # حساب تباين الابتكار
        S = self.H @ self.P @ self.H.T + self.R
        
        # حساب كسب كالمان
        self.K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # تحديث الحالة
        self.x = self.x + self.K @ y
        
        # تحديث التباين
        I = np.eye(self.state_dim)
        self.P = (I - self.K @ self.H) @ self.P
    
    def get_state(self) -> np.ndarray:
        """
        الحصول على الحالة المقدرة
        
        Returns:
            متجه الحالة
        """
        return self.x.flatten()
    
    def get_covariance(self) -> np.ndarray:
        """
        الحصول على مصفوفة التباين
        
        Returns:
            مصفوفة التباين
        """
        return self.P
    
    def set_state(self, x: np.ndarray):
        """
        تعيين الحالة
        
        Args:
            x: متجه الحالة الجديد
        """
        self.x = x.reshape((self.state_dim, 1))
    
    def set_covariance(self, P: np.ndarray):
        """
        تعيين مصفوفة التباين
        
        Args:
            P: مصفوفة التباين الجديدة
        """
        self.P = P
    
    def set_transition_matrix(self, F: np.ndarray):
        """
        تعيين مصفوفة الانتقال
        
        Args:
            F: مصفوفة الانتقال
        """
        self.F = F
    
    def set_measurement_matrix(self, H: np.ndarray):
        """
        تعيين مصفوفة القياس
        
        Args:
            H: مصفوفة القياس
        """
        self.H = H
    
    def set_process_noise(self, Q: np.ndarray):
        """
        تعيين مصفوفة ضوضاء العملية
        
        Args:
            Q: مصفوفة ضوضاء العملية
        """
        self.Q = Q
    
    def set_measurement_noise(self, R: np.ndarray):
        """
        تعيين مصفوفة ضوضاء القياس
        
        Args:
            R: مصفوفة ضوضاء القياس
        """
        self.R = R
    
    def set_control_matrix(self, B: np.ndarray):
        """
        تعيين مصفوفة التحكم
        
        Args:
            B: مصفوفة التحكم
        """
        self.B = B


class AltitudeKalmanFilter:
    """
    مرشح كالمان مخصص للارتفاع
    يدمج بيانات البارومتر و GPS
    """
    
    def __init__(self):
        """تهيئة مرشح كالمان للارتفاع"""
        # الحالة: [altitude, velocity]
        self.kf = KalmanFilter(state_dim=2, measurement_dim=1)
        
        # إعداد المصفوفات
        self._setup_matrices()
    
    def _setup_matrices(self, dt: float = 0.01):
        """
        إعداد مصفوفات مرشح كالمان
        
        Args:
            dt: الفارق الزمني
        """
        # مصفوفة الانتقال
        F = np.array([
            [1, dt],
            [0, 1]
        ])
        self.kf.set_transition_matrix(F)
        
        # مصفوفة القياس (نقيس الارتفاع فقط)
        H = np.array([[1, 0]])
        self.kf.set_measurement_matrix(H)
        
        # ضوضاء العملية
        Q = np.array([
            [0.01, 0],
            [0, 0.1]
        ])
        self.kf.set_process_noise(Q)
        
        # ضوضاء القياس
        R = np.array([[1.0]])
        self.kf.set_measurement_noise(R)
    
    def update(self, altitude_measurement: float, dt: float = 0.01) -> tuple:
        """
        تحديث المرشح
        
        Args:
            altitude_measurement: قياس الارتفاع
            dt: الفارق الزمني
        
        Returns:
            (altitude_estimate, velocity_estimate)
        """
        # تحديث مصفوفة الانتقال بناءً على dt
        self._setup_matrices(dt)
        
        # التنبؤ
        self.kf.predict()
        
        # التحديث
        z = np.array([[altitude_measurement]])
        self.kf.update(z)
        
        # الحصول على الحالة المقدرة
        state = self.kf.get_state()
        return state[0], state[1]
    
    def get_altitude(self) -> float:
        """
        الحصول على الارتفاع المقدر
        
        Returns:
            الارتفاع
        """
        return self.kf.get_state()[0]
    
    def get_velocity(self) -> float:
        """
        الحصول على السرعة العمودية المقدرة
        
        Returns:
            السرعة العمودية
        """
        return self.kf.get_state()[1]
    
    def reset(self, initial_altitude: float = 0.0):
        """
        إعادة تعيين المرشح
        
        Args:
            initial_altitude: الارتفاع الأولي
        """
        self.kf.set_state(np.array([initial_altitude, 0.0]))
        self.kf.set_covariance(np.eye(2))

