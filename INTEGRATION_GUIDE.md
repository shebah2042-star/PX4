# دليل التكامل - PX4 Rocket Control System Integration Guide

## نظرة عامة - Overview

هذا النظام يوفر تحكماً كاملاً في الصواريخ عبر الحاسوب المرفق (Companion Computer) باستخدام Python و MAVLink للتواصل مع PX4.

This system provides full rocket control via a Companion Computer using Python and MAVLink to communicate with PX4.

---

## البنية المعمارية - Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Companion Computer                        │
│                  (Raspberry Pi / Jetson)                     │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │         Python Rocket Control System               │    │
│  │                                                     │    │
│  │  • State Machine (Flight Phases)                   │    │
│  │  • Apogee Detector                                 │    │
│  │  • Attitude Controller (PID)                       │    │
│  │  • Parachute Controller                            │    │
│  │  • Data Logger                                     │    │
│  │  • PX4 Direct Control (Offboard Mode)             │    │
│  └────────────────────────────────────────────────────┘    │
│                          │                                   │
│                          │ MAVLink                          │
│                          ▼                                   │
└──────────────────────────┼───────────────────────────────────┘
                           │
                           │ Serial/UDP
                           │
┌──────────────────────────┼───────────────────────────────────┐
│                          ▼                                   │
│                    PX4 Autopilot                            │
│                  (Pixhawk / Flight Controller)              │
│                                                              │
│  • Sensor Fusion (EKF2)                                     │
│  • IMU, Barometer, GPS                                      │
│  • Actuator Outputs (PWM)                                   │
│  • Safety Features                                          │
│                                                              │
│         │      │      │      │      │      │                │
└─────────┼──────┼──────┼──────┼──────┼──────┼────────────────┘
          │      │      │      │      │      │
          ▼      ▼      ▼      ▼      ▼      ▼
        Fin1   Fin2   Fin3   Fin4  Drogue  Main
       (Servo)(Servo)(Servo)(Servo)(Para.) (Para.)
```

---

## المتطلبات - Requirements

### الأجهزة - Hardware

1. **Flight Controller**: Pixhawk 4 / Pixhawk 6C / Cube Orange (أو متوافق مع PX4)
2. **Companion Computer**: Raspberry Pi 4 / Jetson Nano (أو مشابه)
3. **Servos**: 4x سيرفوهات رقمية للزعانف (Digital servos for fins)
4. **Parachute Deployment**: 2x آليات نشر المظلة (Parachute deployment mechanisms)
5. **Sensors**: IMU, Barometer, GPS (مدمجة في Pixhawk)
6. **Power**: بطارية LiPo 4S (14.8V) للطيران + 5V BEC للإلكترونيات

### البرمجيات - Software

1. **PX4 Autopilot**: v1.14 أو أحدث
2. **Python**: 3.8 أو أحدث
3. **MAVSDK-Python**: 2.0 أو أحدث
4. **Operating System**: Ubuntu 20.04+ / Raspberry Pi OS

---

## التثبيت - Installation

### 1. إعداد PX4 - PX4 Setup

#### أ. تحميل وبناء PX4 - Download and Build PX4

```bash
# Clone PX4 repository
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Build for your hardware (example: Pixhawk 4)
make px4_fmu-v5_default

# Flash to hardware
make px4_fmu-v5_default upload
```

#### ب. تكوين PX4 للصواريخ - Configure PX4 for Rockets

1. افتح QGroundControl
2. اذهب إلى Vehicle Setup > Airframe
3. اختر "Custom" أو قم بتحميل ملف التكوين من `px4_config/rocket_airframe.txt`
4. أعد تشغيل النظام

أو استخدم ملف التكوين مباشرة:

```bash
# Copy configuration to PX4
cp px4_config/rocket_airframe.txt ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/50000_rocket

# Rebuild and flash
cd ~/PX4-Autopilot
make px4_fmu-v5_default upload
```

### 2. إعداد الحاسوب المرفق - Companion Computer Setup

#### أ. تثبيت المتطلبات - Install Requirements

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install Python and pip
sudo apt install python3 python3-pip -y

# Install system dependencies
sudo apt install git -y

# Clone the rocket control system
cd ~/
git clone <repository-url> rocket_control_system
cd rocket_control_system

# Install Python dependencies
pip3 install -r requirements.txt
```

#### ب. تكوين الاتصال - Configure Connection

عدّل ملف `config/system_config.yaml`:

```yaml
mavlink:
  # For serial connection (UART)
  connection_string: "serial:///dev/ttyUSB0:921600"
  
  # OR for UDP connection (WiFi/Ethernet)
  # connection_string: "udp://:14540"
  
  timeout: 10
  heartbeat_rate: 1.0
```

### 3. توصيل الأجهزة - Hardware Connections

#### أ. توصيل Pixhawk بالحاسوب المرفق

**عبر UART/Serial:**
```
Pixhawk TELEM2 <---> USB-to-Serial <---> Companion Computer USB
```

**عبر Ethernet (إذا كان متاحاً):**
```
Pixhawk Ethernet <---> Ethernet Cable <---> Companion Computer
```

#### ب. توصيل السيرفوهات والمظلات

```
Pixhawk Main Out:
  Channel 1 (MAIN1) --> Fin 1 (Front)
  Channel 2 (MAIN2) --> Fin 2 (Right)
  Channel 3 (MAIN3) --> Fin 3 (Back)
  Channel 4 (MAIN4) --> Fin 4 (Left)
  Channel 5 (MAIN5) --> Drogue Parachute Deployment
  Channel 6 (MAIN6) --> Main Parachute Deployment
```

---

## التكوين - Configuration

### 1. إعدادات النظام - System Settings

عدّل `config/system_config.yaml`:

```yaml
# Servo configuration
servos:
  fin_1:
    channel: 0
    min_pwm: 1000
    max_pwm: 2000
    center_pwm: 1500
    reverse: false
  # ... (كرر للزعانف الأخرى)

# Parachute configuration
parachute:
  drogue_channel: 4
  main_channel: 5
  drogue_altitude: 300  # meters
  main_altitude: 150    # meters

# Safety limits
safety_limits:
  max_altitude: 3000      # meters
  max_velocity: 300       # m/s
  max_acceleration: 100   # m/s²
  max_angle: 45          # degrees
```

### 2. إعدادات PID - PID Settings

عدّل `config/pid_config.yaml`:

```yaml
roll:
  kp: 2.5
  ki: 0.1
  kd: 0.8
  output_min: -100
  output_max: 100
  integral_min: -50
  integral_max: 50

pitch:
  kp: 2.5
  ki: 0.1
  kd: 0.8
  # ... (نفس الإعدادات)

yaw:
  kp: 1.5
  ki: 0.05
  kd: 0.5
  # ... (نفس الإعدادات)
```

### 3. إعدادات الأمان - Safety Settings

عدّل `config/safety_config.yaml`:

```yaml
failsafe:
  enabled: true
  connection_timeout: 5.0  # seconds
  action: "deploy_parachute"

operational_limits:
  max_wind_speed: 15.0  # m/s
  min_battery_voltage: 10.5  # V
  max_temperature: 60.0  # °C

flight_termination:
  enabled: true
  max_flight_time: 600  # seconds
  emergency_altitude: 100  # meters
```

---

## التشغيل - Operation

### 1. الاختبار الأرضي - Ground Testing

```bash
# Test 1: Connection test
cd ~/rocket_control_system
python3 -c "from core.mavlink_interface import MAVLinkInterface; import asyncio; asyncio.run(MAVLinkInterface().connect())"

# Test 2: Servo test
python3 tests/test_servos.py

# Test 3: System check
python3 tests/test_system.py
```

### 2. التشغيل الكامل - Full Operation

```bash
# Run the enhanced system with full control
cd ~/rocket_control_system
python3 main_enhanced.py
```

### 3. مراحل الطيران - Flight Phases

النظام يدير المراحل التالية تلقائياً:

1. **PRE_LAUNCH**: ما قبل الإطلاق - فحص الأنظمة
2. **LAUNCH**: الإطلاق - كشف الإطلاق تلقائياً
3. **ASCENT**: الصعود - التحكم النشط في الاستقرار
4. **APOGEE**: القمة - نشر المظلة الأولية
5. **DESCENT**: الهبوط - نشر المظلة الرئيسية
6. **RECOVERY**: الاسترجاع - حفظ البيانات

---

## استكشاف الأخطاء - Troubleshooting

### مشكلة: فشل الاتصال بـ PX4
**الحل:**
```bash
# Check serial connection
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# Check MAVLink traffic
mavproxy.py --master=/dev/ttyUSB0 --baudrate=921600
```

### مشكلة: السيرفوهات لا تتحرك
**الحل:**
1. تحقق من توصيل الطاقة
2. تحقق من إعدادات PWM في QGroundControl
3. تحقق من تفعيل وضع Offboard

### مشكلة: فشل تفعيل وضع Offboard
**الحل:**
```bash
# Check PX4 parameters
param show COM_OF_LOSS_T
param set COM_OF_LOSS_T 1.0

# Check RC override
param show COM_RC_OVERRIDE
param set COM_RC_OVERRIDE 0
```

---

## السلامة - Safety

### ⚠️ تحذيرات هامة - Important Warnings

1. **اختبر دائماً على الأرض أولاً** - Always test on ground first
2. **استخدم منطقة آمنة للإطلاق** - Use safe launch area
3. **تحقق من جميع الأنظمة قبل الإطلاق** - Check all systems before launch
4. **احتفظ بمسافة آمنة** - Maintain safe distance
5. **اتبع القوانين المحلية** - Follow local regulations

### قائمة الفحص قبل الإطلاق - Pre-Launch Checklist

- [ ] فحص البطارية (Battery check)
- [ ] فحص الاتصال (Connection check)
- [ ] فحص السيرفوهات (Servo check)
- [ ] فحص المظلات (Parachute check)
- [ ] فحص المستشعرات (Sensor check)
- [ ] فحص GPS (GPS check)
- [ ] فحص منطقة الإطلاق (Launch area check)
- [ ] فحص الطقس (Weather check)

---

## البيانات والسجلات - Data and Logs

### موقع الملفات - File Locations

```
rocket_control_system/
├── logs/           # System logs
│   └── enhanced_rocket_system_YYYYMMDD_HHMMSS.log
├── data/           # Flight data
│   ├── {session}_sensors.csv
│   ├── {session}_control.csv
│   ├── {session}_events.json
│   └── {session}_summary.json
```

### تحليل البيانات - Data Analysis

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load sensor data
sensors = pd.read_csv('data/{session}_sensors.csv')

# Plot altitude
plt.plot(sensors['timestamp'], sensors['alt'])
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Rocket Altitude Profile')
plt.show()
```

---

## الدعم - Support

للمساعدة والدعم:
- GitHub Issues: <repository-url>/issues
- Documentation: <repository-url>/docs
- Email: support@example.com

---

## الترخيص - License

هذا المشروع مفتوح المصدر - This project is open source

---

**ملاحظة**: هذا النظام مخصص للأغراض التعليمية والبحثية. استخدمه بمسؤولية واتبع جميع القوانين واللوائح المحلية.

**Note**: This system is for educational and research purposes. Use responsibly and follow all local laws and regulations.
