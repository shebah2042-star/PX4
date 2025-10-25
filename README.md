# PX4 Rocket Control System

## Overview

A comprehensive rocket control system using a Companion Computer with PX4 Autopilot. The system provides full control via Python and MAVLink, with intelligent management of all flight phases.

### Key Features

- ✅ **Full Python Control**: All control decisions made in the companion computer
- ✅ **MAVLink Communication**: Reliable communication with PX4 via MAVLink protocol
- ✅ **Flight Phase Management**: Advanced state machine for managing all flight phases
- ✅ **Intelligent Apogee Detection**: Multi-method algorithm for accurate apogee detection
- ✅ **Stability Control**: PID controllers for fin control
- ✅ **Automatic Parachute Deployment**: Safe parachute deployment system
- ✅ **Data Logging**: Comprehensive logging of all flight data
- ✅ **Advanced Safety**: Multi-level safety systems

---

## Architecture

### Component Overview

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

## Project Structure

```
px4_rocket_integration/
│
├── core/                          # Core components
│   ├── mavlink_interface.py      # MAVLink interface
│   ├── px4_direct_control.py     # Direct PX4 control
│   ├── state_machine.py          # State machine
│   └── data_logger.py            # Data logger
│
├── controllers/                   # Controllers
│   ├── attitude_controller.py    # Attitude controller
│   └── altitude_controller.py    # Altitude controller
│
├── algorithms/                    # Algorithms
│   ├── kalman_filter.py          # Kalman filter
│   └── apogee_detector.py        # Apogee detector
│
├── actuators/                     # Actuators
│   ├── servo_control.py          # Servo control
│   └── parachute_control.py      # Parachute control
│
├── config/                        # Configuration files
│   ├── system_config.yaml        # System settings
│   ├── pid_config.yaml           # PID settings
│   └── safety_config.yaml        # Safety settings
│
├── px4_config/                    # PX4 configurations
│   └── rocket_airframe.txt       # Rocket airframe config
│
├── tests/                         # Tests
│   ├── test_servos.py            # Servo tests
│   └── test_system.py            # System tests
│
├── logs/                          # Logs
├── data/                          # Flight data
│
├── main.py                        # Main program
├── main_enhanced.py               # Enhanced program
├── requirements.txt               # Requirements
├── README.md                      # Documentation (English)
├── README_AR.md                   # Documentation (Arabic)
└── INTEGRATION_GUIDE.md          # Integration guide
```

---

## Flight Phases

The system automatically manages the following phases:

### 1. PRE_LAUNCH
- Check all systems
- Verify connections
- Calibrate sensors
- Prepare for launch

### 2. LAUNCH
- Automatic launch detection (acceleration > 2g)
- Start data logging
- Activate safety systems

### 3. ASCENT
- **Active stability control**
- Use PID controllers for fin control
- Maintain rocket trajectory
- Monitor velocity and altitude

### 4. APOGEE
- Detect apogee using three methods:
  - Vertical velocity
  - Vertical acceleration
  - Maximum altitude
- Deploy drogue parachute
- Disable active control

### 5. DESCENT
- Deploy main parachute at specified altitude
- Monitor descent rate
- Prepare for landing

### 6. RECOVERY
- Save all data
- Generate flight summary
- Safely shutdown systems

### 7. ABORT
- Activate emergency procedures
- Deploy parachutes immediately
- Save data

---

## Installation and Setup

For detailed installation instructions, see [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)

### Quick Start

```bash
# Install requirements
pip3 install -r requirements.txt

# Configure system
nano config/system_config.yaml

# Run tests
python3 tests/test_system.py

# Run enhanced system
python3 main_enhanced.py
```

---

## Safety

### ⚠️ Important Warnings

1. **Always test on ground first**
2. **Use safe and open area**
3. **Maintain safe distance (100+ meters)**
4. **Check weather (wind < 15 m/s)**
5. **Follow local regulations**

---

## Documentation

- **English**: [README.md](README.md) (this file)
- **Arabic**: [README_AR.md](README_AR.md)
- **Integration Guide**: [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)

---

## License

This project is licensed under the MIT License.

---

## Support

- **GitHub Issues**: For bug reports
- **Discussions**: For questions
- **Email**: For direct support

---

**Disclaimer**: This system is for educational and research purposes. Use responsibly and follow all local laws and regulations.

**Last Updated**: 2025-10-25
