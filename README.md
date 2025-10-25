# PX4 Rocket Control System

## Overview

A comprehensive rocket control system using a Companion Computer with PX4 Autopilot. The system provides full control via Python and MAVLink, with intelligent management of all flight phases.

### Key Features

#### Core Features
- ✅ **Full Python Control**: All control decisions made in the companion computer
- ✅ **MAVLink Communication**: Reliable communication with PX4 via MAVLink protocol
- ✅ **Flight Phase Management**: Advanced state machine for managing all flight phases
- ✅ **Intelligent Apogee Detection**: Multi-method algorithm for accurate apogee detection
- ✅ **Stability Control**: PID controllers for fin control
- ✅ **Automatic Parachute Deployment**: Safe parachute deployment system
- ✅ **Data Logging**: Comprehensive logging of all flight data
- ✅ **Advanced Safety**: Multi-level safety systems

#### Advanced Features (NEW)
- 🚀 **GPS Navigation**: Waypoint following, return-to-home, orbit mode
- 🎯 **Target Tracking**: Fixed and moving target tracking with proportional navigation
- 📐 **Trajectory Control**: Optimal trajectory planning and guidance
- ⚡ **High-Speed Flight**: Adaptive control for subsonic to hypersonic flight (Mach 0-6)
- 🌍 **Coordinate Transforms**: Comprehensive coordinate transformation utilities

#### Control Systems (NEW)
- 🎛️ **Control Allocator**: Support for canards, tail fins, or both configurations
- 🔥 **Thrust Vector Control (TVC)**: Optional 2-axis gimbal control with enable/disable
- 📊 **Thrust Curve Model**: Complete thrust modeling with mass depletion and CG shift
- ✈️ **Aerodynamic Model**: Full coefficient modeling (CL, CD, Cm, CY, Cl, Cn)
- 🌡️ **Standard Atmosphere**: US Standard Atmosphere 1976 for high-altitude flight

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
│   ├── altitude_controller.py    # Altitude controller
│   ├── high_speed_controller.py  # High-speed adaptive controller (NEW)
│   └── control_allocator.py     # Control allocator for surfaces/TVC (NEW)
│
├── algorithms/                    # Algorithms
│   ├── kalman_filter.py          # Kalman filter
│   └── apogee_detector.py        # Apogee detector
│
├── navigation/                    # Navigation systems (NEW)
│   ├── gps_navigation.py         # GPS waypoint navigation
│   ├── target_tracking.py        # Target tracking system
│   └── trajectory_control.py     # Trajectory planning and control
│
├── actuators/                     # Actuators
│   ├── servo_control.py          # Servo control
│   └── parachute_control.py      # Parachute control
│
├── utils/                         # Utilities (NEW)
│   └── coordinate_transforms.py  # Coordinate transformation utilities
│
├── propulsion/                    # Propulsion systems (NEW)
│   └── thrust_model.py           # Thrust curve model with mass depletion
│
├── aero/                          # Aerodynamics (NEW)
│   └── aero_model.py             # Aerodynamic coefficient modeling
│
├── atmosphere/                    # Atmosphere modeling (NEW)
│   └── standard_atmosphere.py    # US Standard Atmosphere 1976
│
├── config/                        # Configuration files
│   ├── system_config.yaml        # System settings
│   ├── pid_config.yaml           # PID settings
│   ├── safety_config.yaml        # Safety settings
│   ├── advanced_features_config.yaml  # Advanced features config (NEW)
│   └── control_systems_config.yaml    # Control systems config (NEW)
│
├── px4_config/                    # PX4 configurations
│   └── rocket_airframe.txt       # Rocket airframe config
│
├── docs/                          # Documentation (NEW)
│   ├── ADVANCED_FEATURES.md      # Advanced features documentation
│   └── CONTROL_SYSTEMS.md        # Control systems documentation (NEW)
│
├── examples/                      # Examples (NEW)
│   └── advanced_integration_example.py  # Integration example
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

## Advanced Features

The system now includes advanced features for all rocket types, ranges, speeds, and target systems:

### 🚀 GPS Navigation
- Waypoint following with automatic path planning
- Return-to-home functionality
- Orbit mode for circling targets
- Cross-track error correction

### 🎯 Target Tracking
- Fixed target tracking
- Moving target tracking with velocity estimation
- Intercept point calculation
- Proportional navigation guidance

### 📐 Trajectory Control
- Optimal launch angle calculation
- Reference trajectory generation with atmospheric drag
- Trajectory deviation tracking
- Guidance commands for trajectory following

### ⚡ High-Speed Flight Control
- Automatic flight regime detection (subsonic, transonic, supersonic, hypersonic)
- Adaptive gain scheduling based on Mach number
- Aerodynamic compensation for center of pressure shift
- Time delay compensation
- Flight envelope protection

### 🌍 Coordinate Transformations
- LLA ↔ ECEF transformations
- LLA ↔ NED transformations
- Body ↔ NED transformations
- Quaternion ↔ Euler angle conversions
- Distance and bearing calculations

For detailed documentation on advanced features, see [docs/ADVANCED_FEATURES.md](docs/ADVANCED_FEATURES.md)

---

## Documentation

- **English**: [README.md](README.md) (this file)
- **Arabic**: [README_AR.md](README_AR.md)
- **Integration Guide**: [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)
- **Advanced Features**: [docs/ADVANCED_FEATURES.md](docs/ADVANCED_FEATURES.md)
- **Control Systems**: [docs/CONTROL_SYSTEMS.md](docs/CONTROL_SYSTEMS.md) (NEW)

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
