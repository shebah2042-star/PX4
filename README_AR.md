# نظام التحكم في الصواريخ - PX4 Rocket Control System

## نظرة عامة

نظام متكامل للتحكم في الصواريخ باستخدام الحاسوب المرفق (Companion Computer) مع PX4 Autopilot. يوفر النظام تحكماً كاملاً عبر Python و MAVLink، مع إدارة ذكية لجميع مراحل الطيران.

### المميزات الرئيسية

- ✅ **تحكم كامل عبر Python**: جميع قرارات التحكم تتم في الحاسوب المرفق
- ✅ **اتصال MAVLink**: تواصل موثوق مع PX4 عبر بروتوكول MAVLink
- ✅ **إدارة مراحل الطيران**: آلة حالة متقدمة لإدارة جميع مراحل الطيران
- ✅ **كشف القمة الذكي**: خوارزمية متعددة الطرق لكشف القمة بدقة
- ✅ **تحكم في الاستقرار**: متحكمات PID للتحكم في الزعانف
- ✅ **نشر المظلات التلقائي**: نظام آمن لنشر المظلات
- ✅ **تسجيل البيانات**: تسجيل شامل لجميع بيانات الطيران
- ✅ **أمان متقدم**: أنظمة أمان متعددة المستويات

---

## البنية المعمارية

### نظرة عامة على المكونات

```
┌─────────────────────────────────────────────────────────┐
│              Companion Computer (Python)                 │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │           Core Components                        │  │
│  │  • MAVLink Interface                            │  │
│  │  • PX4 Direct Control (Offboard Mode)          │  │
│  │  • State Machine                                │  │
│  │  • Data Logger                                  │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │           Controllers                            │  │
│  │  • Attitude Controller (PID)                    │  │
│  │  • Altitude Controller                          │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │           Algorithms                             │  │
│  │  • Kalman Filter                                │  │
│  │  • Apogee Detector                              │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │           Actuators                              │  │
│  │  • Servo Controller                             │  │
│  │  • Parachute Controller                         │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
└────────────────────┬─────────────────────────────────────┘
                     │ MAVLink (Serial/UDP)
                     ▼
┌─────────────────────────────────────────────────────────┐
│                  PX4 Autopilot                          │
│              (Pixhawk Flight Controller)                │
├─────────────────────────────────────────────────────────┤
│  • Sensor Fusion (EKF2)                                │
│  • IMU / Barometer / GPS                               │
│  • PWM Outputs                                         │
└─────────────────────────────────────────────────────────┘
```

---

## هيكل المشروع

```
px4_rocket_integration/
│
├── core/                          # المكونات الأساسية
│   ├── mavlink_interface.py      # واجهة MAVLink
│   ├── px4_direct_control.py     # التحكم المباشر في PX4
│   ├── state_machine.py          # آلة الحالة
│   └── data_logger.py            # مسجل البيانات
│
├── controllers/                   # المتحكمات
│   ├── attitude_controller.py    # متحكم الاستقرار
│   └── altitude_controller.py    # متحكم الارتفاع
│
├── algorithms/                    # الخوارزميات
│   ├── kalman_filter.py          # مرشح كالمان
│   └── apogee_detector.py        # كاشف القمة
│
├── actuators/                     # المحركات
│   ├── servo_control.py          # التحكم في السيرفوهات
│   └── parachute_control.py      # التحكم في المظلات
│
├── config/                        # ملفات التكوين
│   ├── system_config.yaml        # إعدادات النظام
│   ├── pid_config.yaml           # إعدادات PID
│   └── safety_config.yaml        # إعدادات الأمان
│
├── px4_config/                    # تكوينات PX4
│   └── rocket_airframe.txt       # تكوين هيكل الصاروخ
│
├── tests/                         # الاختبارات
│   ├── test_servos.py            # اختبار السيرفوهات
│   └── test_system.py            # اختبار النظام
│
├── logs/                          # السجلات
├── data/                          # بيانات الطيران
│
├── main.py                        # البرنامج الرئيسي
├── main_enhanced.py               # البرنامج المحسّن
├── requirements.txt               # المتطلبات
├── README.md                      # التوثيق (إنجليزي)
├── README_AR.md                   # التوثيق (عربي)
└── INTEGRATION_GUIDE.md          # دليل التكامل
```

---

## مراحل الطيران

يدير النظام المراحل التالية تلقائياً:

### 1. PRE_LAUNCH (ما قبل الإطلاق)
- فحص جميع الأنظمة
- التحقق من الاتصالات
- معايرة المستشعرات
- الاستعداد للإطلاق

### 2. LAUNCH (الإطلاق)
- كشف الإطلاق تلقائياً (تسارع > 2g)
- بدء تسجيل البيانات
- تفعيل أنظمة الأمان

### 3. ASCENT (الصعود)
- **التحكم النشط في الاستقرار**
- استخدام متحكمات PID للتحكم في الزعانف
- الحفاظ على مسار الصاروخ
- مراقبة السرعة والارتفاع

### 4. APOGEE (القمة)
- كشف القمة باستخدام ثلاث طرق:
  - السرعة العمودية
  - التسارع العمودي
  - الارتفاع الأقصى
- نشر المظلة الأولية (Drogue)
- تعطيل التحكم النشط

### 5. DESCENT (الهبوط)
- نشر المظلة الرئيسية عند ارتفاع محدد
- مراقبة معدل الهبوط
- الاستعداد للهبوط

### 6. RECOVERY (الاسترجاع)
- حفظ جميع البيانات
- إنشاء ملخص الطيران
- إيقاف الأنظمة بأمان

### 7. ABORT (الإجهاض)
- تفعيل إجراءات الطوارئ
- نشر المظلات فوراً
- حفظ البيانات

---

## التحكم في الاستقرار

### نظام التحكم

يستخدم النظام متحكمات PID للتحكم في الزعانف الأربع:

```
Target Attitude (Roll, Pitch, Yaw)
         ↓
    PID Controllers
         ↓
  Control Commands
         ↓
    Fin Mixing (X-Configuration)
         ↓
    Fin 1, Fin 2, Fin 3, Fin 4
```

### تكوين الزعانف (X-Configuration)

```
        Fin 1 (Front)
            ↑
            |
Fin 4 ←----+----→ Fin 2
  (Left)   |    (Right)
            |
            ↓
        Fin 3 (Back)
```

### معادلات التحكم

```python
# Roll control
fin2 = roll_cmd + yaw_cmd
fin4 = -roll_cmd + yaw_cmd

# Pitch control
fin1 = pitch_cmd
fin3 = -pitch_cmd
```

---

## كشف القمة

### الطرق المستخدمة

#### 1. طريقة السرعة
```python
if vertical_velocity < velocity_threshold:
    velocity_method_triggered = True
```

#### 2. طريقة التسارع
```python
if vertical_acceleration < accel_threshold:
    accel_method_triggered = True
```

#### 3. طريقة الارتفاع
```python
if altitude < max_altitude:
    altitude_method_triggered = True
```

### التأكيد
يتطلب النظام موافقة طريقتين من أصل ثلاث لتأكيد الوصول للقمة، مع فترة تأكيد لمنع الإيجابيات الكاذبة.

---

## نشر المظلات

### المظلة الأولية (Drogue)
- **متى**: عند الوصول للقمة
- **الغرض**: تقليل السرعة الأولية
- **القناة**: PWM Channel 5

### المظلة الرئيسية (Main)
- **متى**: عند ارتفاع محدد (افتراضي: 150 متر)
- **الغرض**: هبوط آمن
- **القناة**: PWM Channel 6

### آلية النشر

```python
# Drogue deployment at apogee
if phase == APOGEE:
    deploy_drogue()

# Main deployment at altitude
if altitude < main_altitude and drogue_deployed:
    deploy_main()
```

---

## التثبيت والإعداد

### 1. المتطلبات

#### الأجهزة
- Pixhawk 4 / 6C أو متوافق
- Raspberry Pi 4 أو Jetson Nano
- 4x سيرفوهات رقمية
- 2x آليات نشر المظلة
- بطارية LiPo 4S

#### البرمجيات
```bash
# تثبيت Python
sudo apt install python3 python3-pip

# تثبيت المتطلبات
pip3 install -r requirements.txt
```

### 2. التكوين

#### أ. تكوين PX4
```bash
# نسخ ملف التكوين
cp px4_config/rocket_airframe.txt ~/PX4-Autopilot/ROMFS/...

# إعادة بناء PX4
cd ~/PX4-Autopilot
make px4_fmu-v5_default upload
```

#### ب. تكوين النظام
```bash
# تعديل ملف التكوين
nano config/system_config.yaml

# تعيين سلسلة الاتصال
mavlink:
  connection_string: "serial:///dev/ttyUSB0:921600"
```

### 3. الاختبار

```bash
# اختبار الاتصال
python3 tests/test_connection.py

# اختبار السيرفوهات
python3 tests/test_servos.py

# اختبار النظام الكامل
python3 tests/test_system.py
```

---

## التشغيل

### البرنامج الأساسي
```bash
python3 main.py
```

### البرنامج المحسّن (تحكم كامل)
```bash
python3 main_enhanced.py
```

### المراقبة
```bash
# مراقبة السجلات
tail -f logs/enhanced_rocket_system_*.log

# مراقبة البيانات
watch -n 1 'ls -lh data/'
```

---

## الأمان

### ⚠️ تحذيرات هامة

1. **اختبر دائماً على الأرض أولاً**
2. **استخدم منطقة آمنة ومفتوحة**
3. **احتفظ بمسافة آمنة (100+ متر)**
4. **تحقق من الطقس (رياح < 15 م/ث)**
5. **اتبع القوانين المحلية**

### قائمة الفحص

قبل كل إطلاق:

- [ ] فحص البطارية (> 14V)
- [ ] فحص الاتصال بـ PX4
- [ ] فحص GPS (> 8 أقمار)
- [ ] فحص السيرفوهات (جميع القنوات)
- [ ] فحص المظلات (آليات النشر)
- [ ] فحص المستشعرات (IMU, Baro)
- [ ] فحص منطقة الإطلاق (خالية)
- [ ] فحص الطقس (مناسب)
- [ ] إخطار السلطات (إن لزم)

### أنظمة الأمان

1. **Connection Timeout**: نشر المظلة عند فقدان الاتصال
2. **Altitude Limit**: إجهاض الطيران عند تجاوز الحد
3. **Battery Monitor**: تحذير عند انخفاض البطارية
4. **Angle Limit**: تقييد زوايا الانحراف
5. **Emergency Abort**: إمكانية الإجهاض اليدوي

---

## البيانات والتحليل

### الملفات المسجلة

```
data/
├── {session}_sensors.csv      # بيانات المستشعرات
├── {session}_control.csv      # بيانات التحكم
├── {session}_events.json      # الأحداث
└── {session}_summary.json     # الملخص
```

### تحليل البيانات

```python
import pandas as pd
import matplotlib.pyplot as plt

# قراءة البيانات
df = pd.read_csv('data/{session}_sensors.csv')

# رسم الارتفاع
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp'], df['alt'])
plt.xlabel('الوقت (ثانية)')
plt.ylabel('الارتفاع (متر)')
plt.title('منحنى الارتفاع')
plt.grid(True)
plt.show()

# رسم السرعة
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp'], df['vz'])
plt.xlabel('الوقت (ثانية)')
plt.ylabel('السرعة العمودية (م/ث)')
plt.title('منحنى السرعة')
plt.grid(True)
plt.show()
```

---

## استكشاف الأخطاء

### المشكلة: فشل الاتصال بـ PX4

**الأعراض**: `Connection timeout` أو `Failed to connect`

**الحلول**:
```bash
# 1. فحص المنفذ التسلسلي
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# 2. فحص معدل البود
# تأكد من أن PX4 و Python يستخدمان نفس المعدل (921600)

# 3. فحص حركة MAVLink
mavproxy.py --master=/dev/ttyUSB0 --baudrate=921600
```

### المشكلة: السيرفوهات لا تتحرك

**الأعراض**: السيرفوهات ثابتة أو لا تستجيب

**الحلول**:
```bash
# 1. فحص الطاقة
# تأكد من توصيل BEC 5V للسيرفوهات

# 2. فحص إعدادات PWM في QGroundControl
# Vehicle Setup > Actuators > PWM Outputs

# 3. فحص وضع Offboard
# تأكد من تفعيل وضع Offboard في الكود
```

### المشكلة: كشف القمة المبكر

**الأعراض**: نشر المظلة قبل الوصول للقمة الفعلية

**الحلول**:
```python
# تعديل عتبات الكشف في config/system_config.yaml
apogee_detection:
  velocity_threshold: -2.0    # زيادة القيمة المطلقة
  accel_threshold: -5.0       # زيادة القيمة المطلقة
  confirmation_time: 0.5      # زيادة وقت التأكيد
```

---

## الأداء المتوقع

### المواصفات النموذجية

- **معدل التحكم**: 100 Hz
- **معدل التسجيل**: 50 Hz
- **زمن الاستجابة**: < 10 ms
- **دقة الارتفاع**: ± 1 متر
- **دقة الاستقرار**: ± 2 درجة

### استهلاك الموارد

- **CPU**: 30-50% (Raspberry Pi 4)
- **RAM**: 200-300 MB
- **التخزين**: ~10 MB لكل دقيقة طيران

---

## التطوير المستقبلي

### المميزات المخططة

- [ ] دعم GPS للملاحة
- [ ] تحكم في المسار (Trajectory Control)
- [ ] تحسين الخوارزميات بالتعلم الآلي
- [ ] واجهة مستخدم رسومية
- [ ] دعم الطيران المتعدد
- [ ] تكامل مع محطة أرضية

---

## المساهمة

نرحب بالمساهمات! يرجى:

1. Fork المشروع
2. إنشاء فرع للميزة (`git checkout -b feature/AmazingFeature`)
3. Commit التغييرات (`git commit -m 'Add AmazingFeature'`)
4. Push للفرع (`git push origin feature/AmazingFeature`)
5. فتح Pull Request

---

## الترخيص

هذا المشروع مرخص تحت MIT License - انظر ملف LICENSE للتفاصيل.

---

## الدعم والتواصل

- **GitHub Issues**: للإبلاغ عن المشاكل
- **Discussions**: للأسئلة والنقاشات
- **Email**: للدعم المباشر

---

## شكر وتقدير

- فريق PX4 Autopilot
- مجتمع MAVSDK
- جميع المساهمين في المشروع

---

**تنويه**: هذا النظام مخصص للأغراض التعليمية والبحثية. استخدمه بمسؤولية واتبع جميع القوانين واللوائح المحلية المتعلقة بالطيران والصواريخ.

**آخر تحديث**: 2025-10-25
