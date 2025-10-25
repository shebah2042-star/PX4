"""
وحدة تسجيل البيانات
Data Logger Module
"""

import os
import csv
import json
import time
from datetime import datetime
from typing import Dict, List, Any
import logging


class DataLogger:
    """
    مسجل البيانات للصاروخ
    """
    
    def __init__(self, log_dir: str = "logs", data_dir: str = "data"):
        """
        تهيئة مسجل البيانات
        
        Args:
            log_dir: مجلد السجلات
            data_dir: مجلد البيانات
        """
        self.log_dir = log_dir
        self.data_dir = data_dir
        
        # إنشاء المجلدات
        os.makedirs(log_dir, exist_ok=True)
        os.makedirs(data_dir, exist_ok=True)
        
        # اسم الجلسة
        self.session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # ملفات البيانات
        self.sensor_data_file = None
        self.control_data_file = None
        self.events_file = None
        
        # كتّاب CSV
        self.sensor_writer = None
        self.control_writer = None
        
        # قائمة الأحداث
        self.events = []
        
        # وقت البداية
        self.start_time = time.time()
        
        # السجل
        self.logger = logging.getLogger(__name__)
    
    def start_logging(self):
        """بدء التسجيل"""
        # فتح ملفات البيانات
        sensor_file_path = os.path.join(self.data_dir, f"{self.session_name}_sensors.csv")
        control_file_path = os.path.join(self.data_dir, f"{self.session_name}_control.csv")
        
        self.sensor_data_file = open(sensor_file_path, 'w', newline='')
        self.control_data_file = open(control_file_path, 'w', newline='')
        
        # إنشاء كتّاب CSV
        sensor_fields = [
            'timestamp', 'time_elapsed',
            'roll', 'pitch', 'yaw',
            'lat', 'lon', 'alt',
            'vx', 'vy', 'vz',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'pressure', 'temperature', 'baro_alt',
            'num_satellites', 'battery_voltage'
        ]
        
        control_fields = [
            'timestamp', 'time_elapsed',
            'phase', 'target_roll', 'target_pitch', 'target_yaw',
            'roll_output', 'pitch_output', 'yaw_output',
            'fin1', 'fin2', 'fin3', 'fin4',
            'drogue_deployed', 'main_deployed'
        ]
        
        self.sensor_writer = csv.DictWriter(self.sensor_data_file, fieldnames=sensor_fields)
        self.control_writer = csv.DictWriter(self.control_data_file, fieldnames=control_fields)
        
        # كتابة الرؤوس
        self.sensor_writer.writeheader()
        self.control_writer.writeheader()
        
        self.logger.info(f"بدء التسجيل - الجلسة: {self.session_name}")
    
    def log_sensor_data(self, data: Dict[str, Any]):
        """
        تسجيل بيانات المستشعرات
        
        Args:
            data: قاموس بيانات المستشعرات
        """
        if self.sensor_writer is None:
            return
        
        timestamp = time.time()
        time_elapsed = timestamp - self.start_time
        
        row = {
            'timestamp': timestamp,
            'time_elapsed': time_elapsed,
            **data
        }
        
        self.sensor_writer.writerow(row)
    
    def log_control_data(self, data: Dict[str, Any]):
        """
        تسجيل بيانات التحكم
        
        Args:
            data: قاموس بيانات التحكم
        """
        if self.control_writer is None:
            return
        
        timestamp = time.time()
        time_elapsed = timestamp - self.start_time
        
        row = {
            'timestamp': timestamp,
            'time_elapsed': time_elapsed,
            **data
        }
        
        self.control_writer.writerow(row)
    
    def log_event(self, event_type: str, description: str, data: Dict = None):
        """
        تسجيل حدث
        
        Args:
            event_type: نوع الحدث
            description: وصف الحدث
            data: بيانات إضافية
        """
        timestamp = time.time()
        time_elapsed = timestamp - self.start_time
        
        event = {
            'timestamp': timestamp,
            'time_elapsed': time_elapsed,
            'type': event_type,
            'description': description,
            'data': data or {}
        }
        
        self.events.append(event)
        self.logger.info(f"حدث: {event_type} - {description}")
    
    def stop_logging(self):
        """إيقاف التسجيل"""
        # إغلاق ملفات البيانات
        if self.sensor_data_file:
            self.sensor_data_file.close()
        
        if self.control_data_file:
            self.control_data_file.close()
        
        # حفظ الأحداث
        events_file_path = os.path.join(self.data_dir, f"{self.session_name}_events.json")
        with open(events_file_path, 'w', encoding='utf-8') as f:
            json.dump(self.events, f, indent=2, ensure_ascii=False)
        
        self.logger.info("إيقاف التسجيل")
    
    def save_summary(self, summary: Dict[str, Any]):
        """
        حفظ ملخص الطيران
        
        Args:
            summary: قاموس الملخص
        """
        summary_file_path = os.path.join(self.data_dir, f"{self.session_name}_summary.json")
        
        with open(summary_file_path, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        
        self.logger.info("تم حفظ ملخص الطيران")
    
    def get_session_name(self) -> str:
        """
        الحصول على اسم الجلسة
        
        Returns:
            اسم الجلسة
        """
        return self.session_name

