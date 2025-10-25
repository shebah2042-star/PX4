# المميزات المتقدمة - Advanced Features Documentation

## نظرة عامة - Overview

This document describes the advanced features added to the rocket control system to support all rocket types, ranges, speeds, and target systems.

## المميزات الجديدة - New Features

### 1. نظام الملاحة بالإحداثيات GPS - GPS Navigation System

The GPS navigation system enables waypoint following and coordinate-based guidance.

#### Features:
- **Waypoint Following**: Navigate through a list of GPS waypoints
- **Return to Home**: Automatic return to launch site
- **Orbit Mode**: Circle around a specific point
- **Cross-Track Error Correction**: Stay on the desired path
- **Adaptive Speed Control**: Slow down when approaching waypoints

#### Usage:

```python
from navigation.gps_navigation import GPSNavigation, NavigationMode

# Initialize navigation system
nav = GPSNavigation()

# Set home position (launch site)
nav.set_home_position(latitude=40.7128, longitude=-74.0060, altitude=100.0)

# Add waypoints
nav.add_waypoint(latitude=40.7200, longitude=-74.0100, altitude=500.0)
nav.add_waypoint(latitude=40.7300, longitude=-74.0150, altitude=800.0)

# Start waypoint mission
nav.start_waypoint_mission()

# In control loop:
nav.update_current_position(current_lat, current_lon, current_alt, current_heading)
desired_heading, desired_speed, desired_altitude = nav.get_navigation_command()
```

#### Configuration:

Edit `config/advanced_features_config.yaml`:

```yaml
gps_navigation:
  enabled: true
  max_speed: 100.0  # m/s
  approach_speed: 20.0  # m/s
  acceptance_radius: 10.0  # meters
```

---

### 2. نظام تتبع الأهداف - Target Tracking System

The target tracking system enables tracking and intercepting fixed or moving targets.

#### Features:
- **Fixed Target Tracking**: Track stationary targets
- **Moving Target Tracking**: Track targets with velocity
- **Intercept Point Calculation**: Predict optimal intercept location
- **Proportional Navigation**: Classic missile guidance law
- **Lead Angle Calculation**: Compensate for target motion
- **Velocity Estimation**: Estimate target velocity from position history

#### Usage:

```python
from navigation.target_tracking import TargetTracking, TargetType

# Initialize tracking system
tracker = TargetTracking()

# Set fixed target
tracker.set_fixed_target(latitude=40.7500, longitude=-74.0200, altitude=0.0)

# OR update moving target
tracker.update_moving_target(
    latitude=40.7500, 
    longitude=-74.0200, 
    altitude=100.0,
    velocity_north=50.0,  # m/s
    velocity_east=20.0,   # m/s
    velocity_down=0.0
)

# Calculate intercept point
intercept = tracker.calculate_intercept_point(
    rocket_lat=current_lat,
    rocket_lon=current_lon,
    rocket_alt=current_alt,
    rocket_speed=current_speed
)

# Get proportional navigation command
lateral_accel, vertical_accel = tracker.get_proportional_navigation_command(
    rocket_lat=current_lat,
    rocket_lon=current_lon,
    rocket_alt=current_alt,
    rocket_speed=current_speed,
    rocket_heading=current_heading,
    navigation_constant=3.0
)
```

#### Configuration:

```yaml
target_tracking:
  enabled: true
  prediction_time: 5.0  # seconds
  navigation_constant: 3.0  # 3-5 typical for missiles
  max_target_range: 10000.0  # meters
```

---

### 3. نظام التحكم في المسار - Trajectory Control System

The trajectory control system enables precise trajectory planning and guidance.

#### Features:
- **Optimal Launch Angle Calculation**: Calculate best angle for target range
- **Reference Trajectory Generation**: Generate complete flight path with drag
- **Trajectory Deviation Tracking**: Monitor cross-track and altitude errors
- **Guidance Commands**: Generate pitch/yaw commands to follow trajectory
- **Impact Point Prediction**: Predict where rocket will land
- **Range Optimization**: Optimize trajectory for maximum or specific range

#### Usage:

```python
from navigation.trajectory_control import TrajectoryControl, TrajectoryType

# Initialize trajectory control
traj = TrajectoryControl()

# Set rocket parameters
traj.set_rocket_parameters(
    mass=10.0,  # kg
    diameter=0.1,  # meters
    drag_coeff=0.5,
    thrust=200.0,  # Newtons
    specific_impulse=180.0,  # seconds
    propellant_mass=2.0  # kg
)

# Calculate optimal launch angle for target range
target_range = 5000.0  # meters
initial_velocity = 150.0  # m/s
launch_angle = traj.calculate_optimal_launch_angle(target_range, initial_velocity)

# Generate reference trajectory
trajectory = traj.generate_reference_trajectory(
    launch_angle=launch_angle,
    azimuth=0.0,  # North
    initial_velocity=initial_velocity,
    burn_time=5.0,  # seconds
    dt=0.1
)

# In control loop:
pitch_cmd, yaw_cmd, thrust_cmd = traj.calculate_guidance_command(
    current_position=(x, y, z),
    current_velocity=(vx, vy, vz),
    current_time=t
)

# Predict impact point
impact_x, impact_y = traj.calculate_impact_point(
    current_position=(x, y, z),
    current_velocity=(vx, vy, vz)
)
```

#### Configuration:

```yaml
trajectory_control:
  enabled: true
  trajectory_type: "guided"  # ballistic, guided, lofted, depressed, optimal
  max_angle_of_attack: 15.0  # degrees
  max_lateral_acceleration: 50.0  # m/s^2
  
  rocket_parameters:
    mass: 10.0  # kg
    diameter: 0.1  # meters
    drag_coefficient: 0.5
    thrust: 200.0  # Newtons
    specific_impulse: 180.0  # seconds
    propellant_mass: 2.0  # kg
```

---

### 4. نظام التحكم في السرعات العالية - High-Speed Flight Control

The high-speed controller adapts control parameters based on flight regime (subsonic, transonic, supersonic, hypersonic).

#### Features:
- **Flight Regime Detection**: Automatic detection of subsonic/transonic/supersonic/hypersonic
- **Adaptive Gain Scheduling**: Adjust PID gains based on Mach number
- **Aerodynamic Compensation**: Compensate for center of pressure shift and damping
- **Time Delay Compensation**: Predict future state to compensate for actuator delays
- **Model Reference Adaptive Control (MRAC)**: Adapt to changing dynamics
- **Flight Envelope Protection**: Limit commands based on dynamic pressure and Mach

#### Features by Flight Regime:

| Regime | Mach Range | Characteristics | Control Adjustments |
|--------|-----------|-----------------|---------------------|
| **Subsonic** | < 0.8 | Stable, predictable | Standard gains |
| **Transonic** | 0.8 - 1.2 | Shock waves, instability | Increased gains (1.5x) |
| **Supersonic** | 1.2 - 5.0 | Supersonic drag, CP shift | Moderate gains (1.2x) |
| **Hypersonic** | ≥ 5.0 | Extreme heating, limited control | Reduced gains (0.8x) |

#### Usage:

```python
from controllers.high_speed_controller import HighSpeedController, FlightRegime

# Initialize high-speed controller
hs_controller = HighSpeedController()

# In control loop:
# Update flight conditions
hs_controller.update_flight_conditions(velocity=current_velocity, altitude=current_altitude)

# Get adaptive gains
roll_gains, pitch_gains, yaw_gains = hs_controller.get_adaptive_gains()

# Apply gains to PID controllers
attitude_controller.set_roll_gains(*roll_gains)
attitude_controller.set_pitch_gains(*pitch_gains)
attitude_controller.set_yaw_gains(*yaw_gains)

# Get aerodynamic compensation
roll_comp, pitch_comp, yaw_comp = hs_controller.compensate_aerodynamic_effects(
    angle_of_attack=aoa,
    sideslip_angle=beta,
    roll_rate=p,
    pitch_rate=q,
    yaw_rate=r
)

# Apply compensation to control commands
roll_cmd += roll_comp
pitch_cmd += pitch_comp
yaw_cmd += yaw_comp

# Check flight envelope limits
limits = hs_controller.get_flight_envelope_limits()
if current_dynamic_pressure > limits['max_dynamic_pressure']:
    # Reduce maneuverability
    pass
```

#### Configuration:

```yaml
high_speed_control:
  enabled: true
  adaptive_gain: 1.0
  adaptation_rate: 0.1
  
  base_gains:
    roll: {kp: 2.0, ki: 0.1, kd: 0.5}
    pitch: {kp: 2.0, ki: 0.1, kd: 0.5}
    yaw: {kp: 1.5, ki: 0.05, kd: 0.3}
  
  regime_multipliers:
    subsonic: 1.0
    transonic: 1.5
    supersonic: 1.2
    hypersonic: 0.8
  
  enable_aero_compensation: true
  max_mach: 6.0
  max_dynamic_pressure: 50000.0  # Pa
```

---

### 5. أدوات تحويل الإحداثيات - Coordinate Transformation Utilities

Comprehensive coordinate transformation utilities for converting between different reference frames.

#### Supported Transformations:

1. **LLA ↔ ECEF**: Latitude/Longitude/Altitude ↔ Earth-Centered Earth-Fixed
2. **LLA ↔ NED**: Latitude/Longitude/Altitude ↔ North/East/Down (local frame)
3. **NED ↔ ENU**: North/East/Down ↔ East/North/Up
4. **Body ↔ NED**: Body frame ↔ North/East/Down
5. **Quaternion ↔ Euler**: Quaternion ↔ Roll/Pitch/Yaw
6. **Distance and Bearing**: Calculate distance and bearing between GPS coordinates

#### Usage:

```python
from utils.coordinate_transforms import CoordinateTransforms, LLA, ECEF, NED

# GPS to local NED coordinates
current_lla = LLA(latitude=40.7128, longitude=-74.0060, altitude=100.0)
reference_lla = LLA(latitude=40.7100, longitude=-74.0050, altitude=50.0)
ned = CoordinateTransforms.lla_to_ned(current_lla, reference_lla)
print(f"NED: North={ned.north}m, East={ned.east}m, Down={ned.down}m")

# Body frame to NED frame
body_vector = (1.0, 0.0, 0.0)  # 1m forward in body frame
roll, pitch, yaw = 0.0, 10.0, 45.0  # degrees
ned_vector = CoordinateTransforms.body_to_ned(body_vector, roll, pitch, yaw)

# Quaternion to Euler angles
q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0  # Identity quaternion
roll, pitch, yaw = CoordinateTransforms.quaternion_to_euler(q0, q1, q2, q3)

# Calculate distance and bearing
distance = CoordinateTransforms.calculate_distance(
    lat1=40.7128, lon1=-74.0060,
    lat2=40.7500, lon2=-74.0200
)
bearing = CoordinateTransforms.calculate_bearing(
    lat1=40.7128, lon1=-74.0060,
    lat2=40.7500, lon2=-74.0200
)

# Offset position by bearing and distance
new_lat, new_lon = CoordinateTransforms.offset_position(
    lat=40.7128, lon=-74.0060,
    bearing=45.0,  # degrees
    distance=1000.0  # meters
)
```

---

## دمج المميزات - Feature Integration

### Complete System Integration Example

```python
import asyncio
from navigation.gps_navigation import GPSNavigation
from navigation.target_tracking import TargetTracking
from navigation.trajectory_control import TrajectoryControl
from controllers.high_speed_controller import HighSpeedController
from utils.coordinate_transforms import CoordinateTransforms

class AdvancedRocketControlSystem:
    def __init__(self):
        # Initialize all subsystems
        self.gps_nav = GPSNavigation()
        self.target_tracker = TargetTracking()
        self.trajectory_control = TrajectoryControl()
        self.high_speed_controller = HighSpeedController()
        
        # Set mission mode
        self.mission_mode = "waypoint"  # waypoint, target, trajectory
    
    async def control_loop(self):
        while True:
            # Get current state from PX4
            position = await self.get_position()
            velocity = await self.get_velocity()
            attitude = await self.get_attitude()
            
            # Update high-speed controller
            speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
            self.high_speed_controller.update_flight_conditions(speed, position[2])
            
            # Get adaptive gains
            roll_gains, pitch_gains, yaw_gains = self.high_speed_controller.get_adaptive_gains()
            
            # Mission-specific guidance
            if self.mission_mode == "waypoint":
                # GPS waypoint navigation
                self.gps_nav.update_current_position(*position, attitude[2])
                heading_cmd, speed_cmd, alt_cmd = self.gps_nav.get_navigation_command()
                
            elif self.mission_mode == "target":
                # Target tracking
                bearing = self.target_tracker.get_bearing_to_target(position[0], position[1])
                distance = self.target_tracker.get_distance_to_target(position[0], position[1])
                
                # Proportional navigation
                lat_accel, vert_accel = self.target_tracker.get_proportional_navigation_command(
                    position[0], position[1], position[2], speed, attitude[2]
                )
                
            elif self.mission_mode == "trajectory":
                # Trajectory following
                pitch_cmd, yaw_cmd, thrust_cmd = self.trajectory_control.calculate_guidance_command(
                    position, velocity, self.current_time
                )
            
            # Apply control commands
            await self.send_control_commands(roll_cmd, pitch_cmd, yaw_cmd)
            
            await asyncio.sleep(0.01)  # 100 Hz
```

---

## الاختبار - Testing

### Unit Tests

Run unit tests for each module:

```bash
# Test GPS navigation
python -m pytest tests/test_gps_navigation.py

# Test target tracking
python -m pytest tests/test_target_tracking.py

# Test trajectory control
python -m pytest tests/test_trajectory_control.py

# Test high-speed controller
python -m pytest tests/test_high_speed_controller.py

# Test coordinate transforms
python -m pytest tests/test_coordinate_transforms.py
```

### Simulation Testing

Test in PX4 SITL (Software-In-The-Loop):

```bash
# Start PX4 SITL
cd ~/repos/PX4-Autopilot
make px4_sitl gazebo

# In another terminal, run the control system
cd ~/repos/PX4
python main_enhanced.py --config config/advanced_features_config.yaml --simulation
```

### Hardware Testing Checklist

⚠️ **CRITICAL**: Complete all tests before flight!

1. ☐ **Ground Tests**
   - [ ] Verify GPS lock and accuracy
   - [ ] Test servo response to navigation commands
   - [ ] Verify telemetry data logging
   - [ ] Test emergency abort procedures

2. ☐ **Bench Tests**
   - [ ] Test all flight regimes in simulation
   - [ ] Verify gain scheduling transitions
   - [ ] Test waypoint navigation logic
   - [ ] Test target tracking algorithms

3. ☐ **Static Fire Tests**
   - [ ] Verify motor ignition and thrust
   - [ ] Test fin deflection under thrust
   - [ ] Verify data logging during burn

4. ☐ **Low-Altitude Tests**
   - [ ] Test basic stability control
   - [ ] Verify GPS navigation at low speed
   - [ ] Test parachute deployment
   - [ ] Verify recovery procedures

5. ☐ **Progressive Flight Tests**
   - [ ] Gradually increase altitude and speed
   - [ ] Test each flight regime separately
   - [ ] Verify adaptive control performance
   - [ ] Test emergency procedures

---

## الأمان - Safety Considerations

### Flight Envelope Limits

The system automatically enforces safety limits:

- **Maximum Altitude**: Configurable (default 3000m AGL)
- **Maximum Range**: Configurable (default 5000m from home)
- **Maximum Mach**: 6.0 (configurable)
- **Maximum Dynamic Pressure**: 50 kPa (structural limit)
- **Maximum Angle of Attack**: Varies by regime (8-20 degrees)

### Abort Conditions

The system will automatically abort if:

- Tilt angle exceeds 45 degrees
- Battery voltage drops below threshold
- Communication timeout (5 seconds)
- Geofence breach
- Excessive descent rate

### Emergency Procedures

On abort:
1. Deploy drogue parachute immediately
2. Cut motor thrust
3. Log abort reason and state
4. Deploy main parachute at safe altitude
5. Transmit emergency telemetry

---

## استكشاف الأخطاء - Troubleshooting

### GPS Navigation Issues

**Problem**: Waypoints not being reached
- Check GPS accuracy (HDOP < 2.0)
- Verify acceptance radius is reasonable
- Check wind compensation

**Problem**: Erratic navigation commands
- Verify GPS update rate (≥ 5 Hz)
- Check for GPS multipath interference
- Verify coordinate transformations

### Target Tracking Issues

**Problem**: Unable to intercept moving target
- Verify target velocity is accurate
- Increase navigation constant (3-5)
- Check rocket speed is sufficient

**Problem**: Oscillating around target
- Reduce navigation constant
- Increase acceptance radius
- Check for sensor noise

### Trajectory Control Issues

**Problem**: Large trajectory deviations
- Verify rocket parameters are accurate
- Check drag coefficient estimation
- Increase control gains
- Verify wind model

**Problem**: Trajectory not converging
- Check reference trajectory generation
- Verify guidance law implementation
- Increase trajectory update rate

### High-Speed Control Issues

**Problem**: Instability in transonic regime
- Verify gain scheduling is active
- Increase transonic gain multiplier
- Enable aerodynamic compensation
- Reduce maximum angle of attack

**Problem**: Excessive control activity
- Reduce adaptation rate
- Increase reference model damping
- Enable rate limiting
- Check for sensor noise

---

## المراجع - References

### Guidance and Control
- Zarchan, P. (2012). *Tactical and Strategic Missile Guidance*
- Tewari, A. (2011). *Advanced Control of Aircraft, Spacecraft and Rockets*
- Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation*

### Coordinate Transformations
- Farrell, J. A. (2008). *Aided Navigation: GPS with High Rate Sensors*
- Groves, P. D. (2013). *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*

### High-Speed Aerodynamics
- Anderson, J. D. (2003). *Modern Compressible Flow*
- Fleeman, E. L. (2012). *Tactical Missile Design*

---

## الدعم - Support

For questions or issues:
1. Check this documentation
2. Review configuration files
3. Check log files in `logs/` directory
4. Review PR comments on GitHub
5. Contact system developer

---

## الترخيص - License

This advanced features package is part of the PX4 Rocket Control System and follows the same license terms.
