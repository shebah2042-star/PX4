# Comprehensive Control Systems Documentation

This document provides detailed information about the advanced control systems including control surfaces, TVC, thrust modeling, aerodynamics, and atmosphere modeling.

## Table of Contents

1. [Control Allocator](#control-allocator)
2. [Thrust Vector Control (TVC)](#thrust-vector-control-tvc)
3. [Thrust Curve Model](#thrust-curve-model)
4. [Aerodynamic Model](#aerodynamic-model)
5. [Standard Atmosphere Model](#standard-atmosphere-model)
6. [Configuration Guide](#configuration-guide)
7. [Integration Examples](#integration-examples)

---

## Control Allocator

The Control Allocator distributes control commands to multiple actuator types including control surfaces (canards/tail fins) and TVC.

### Supported Configurations

1. **Canard Only**: Forward control surfaces only
2. **Tail Only**: Aft control surfaces only (standard configuration)
3. **Canard + Tail**: Combined configuration for enhanced control authority

### Features

- **X-Pattern Mixing**: Standard quad fin configuration with proper moment distribution
- **Control Effectiveness Modeling**: Adjusts control authority based on:
  - Mach number (reduced effectiveness in transonic regime)
  - Dynamic pressure (reduced at low q, flutter protection at high q)
  - Angle of attack (reduced at high AoA due to flow separation)
- **Rate Limiting**: Prevents excessive actuator rates
- **Saturation Protection**: Limits deflection angles to prevent damage

### Configuration Parameters

```yaml
control_allocator:
  configuration: tail_only  # canard_only, tail_only, canard_and_tail
  
  surfaces:
    - type: tail
      index: 0
      channel: 1
      max_deflection: 30.0    # degrees
      min_deflection: -30.0
      effectiveness_factor: 1.0
      position_x: -0.9        # meters from nose
  
  mixing_weights:
    canard: 0.5
    tail: 0.5
    tvc: 0.3
  
  rate_limits:
    surface_rate: 200.0       # degrees/second
```

### Usage Example

```python
from controllers.control_allocator import ControlAllocator

# Initialize
allocator = ControlAllocator("config/control_systems_config.yaml")

# Allocate control commands
actuator_commands = allocator.allocate_control(
    roll_command=0.5,      # -1 to 1
    pitch_command=0.3,
    yaw_command=0.2,
    mach=0.8,
    dynamic_pressure=5000.0,  # Pa
    angle_of_attack=5.0,      # degrees
    dt=0.01                   # seconds
)

# Convert to PWM
pwm_commands = allocator.angles_to_pwm(actuator_commands)
```

---

## Thrust Vector Control (TVC)

TVC provides additional control authority through engine gimbal deflection.

### Features

- **2-Axis Gimbal**: Independent pitch and yaw control
- **Enable/Disable**: Can be turned on/off via configuration
- **Integration with Surfaces**: Works alongside canards/tail fins
- **Rate Limiting**: Prevents excessive gimbal rates
- **Configurable Limits**: Adjustable maximum deflection angles

### Configuration Parameters

```yaml
tvc:
  enabled: true
  pitch_channel: 9
  yaw_channel: 10
  max_pitch_angle: 10.0      # degrees
  max_yaw_angle: 10.0
  min_pitch_angle: -10.0
  min_yaw_angle: -10.0
  pitch_pwm_min: 1000        # microseconds
  pitch_pwm_max: 2000
  pitch_pwm_center: 1500
  effectiveness_factor: 1.0
  position_x: -0.95          # meters from nose
```

### When to Use TVC

- **High-Speed Flight**: TVC remains effective when aerodynamic surfaces lose effectiveness
- **Low Dynamic Pressure**: Provides control authority during launch and high-altitude flight
- **Enhanced Maneuverability**: Increases control authority for aggressive maneuvers
- **Backup Control**: Redundancy if aerodynamic surfaces fail

### Mixing with Aerodynamic Surfaces

The control allocator automatically distributes commands between TVC and surfaces based on mixing weights:

```yaml
mixing_weights:
  canard: 0.4
  tail: 0.3
  tvc: 0.3
```

This ensures smooth coordination between all control effectors.

---

## Thrust Curve Model

Comprehensive thrust modeling with mass depletion and CG shift calculations.

### Supported Formats

1. **RASP .eng files**: Standard motor files from thrustcurve.org
2. **CSV files**: Custom thrust curves (time, thrust columns)
3. **YAML files**: Inline thrust curve data
4. **Analytical curves**: Progressive, regressive, or neutral burn profiles

### Features

- **Mass Depletion**: Tracks propellant consumption over time
- **CG Shift**: Calculates center of gravity changes as propellant burns
- **Static Margin**: Computes stability margin (CP - CG)
- **Total Impulse**: Automatic calculation from thrust curve
- **Burn Fraction**: Real-time burn progress tracking

### Configuration Parameters

```yaml
thrust_model:
  curve_type: time_based
  
  # Option 1: Load from file
  curve_file: "motors/Cesaroni_M1670.eng"
  
  # Option 2: Inline data
  curve_points:
    - time: 0.0
      thrust: 0.0
    - time: 0.05
      thrust: 800.0
    - time: 4.0
      thrust: 0.0
  
  motor_parameters:
    name: "Generic Motor"
    propellant_mass: 1.5      # kg
    motor_mass: 0.5           # kg (empty)
    burn_time: 4.0            # seconds
  
  mass_properties:
    dry_mass: 10.0            # kg
    propellant_mass: 1.5
    cg_dry: 0.6               # meters from nose
    cg_wet: 0.65
    cp_position: 0.85
```

### Usage Example

```python
from propulsion.thrust_model import ThrustModel

# Initialize
thrust_model = ThrustModel("config/control_systems_config.yaml")

# Ignite motor
thrust_model.ignite(current_time=1.0)

# Update each timestep
thrust = thrust_model.update(current_time, dt=0.01)
current_mass = thrust_model.get_mass()
current_cg = thrust_model.get_cg()
static_margin = thrust_model.get_static_margin()
```

### Creating Custom Thrust Curves

```python
# Constant thrust
thrust_model.create_constant_thrust_curve(
    thrust=750.0,           # N
    burn_time=4.0,          # seconds
    propellant_mass=1.5     # kg
)

# Typical curve shapes
thrust_model.create_typical_curve(
    peak_thrust=800.0,
    burn_time=4.0,
    propellant_mass=1.5,
    curve_shape="progressive"  # or "regressive", "neutral"
)
```

---

## Aerodynamic Model

Comprehensive aerodynamic coefficient modeling for all flight regimes.

### Coefficients Provided

- **CL**: Lift coefficient
- **CD**: Drag coefficient (base + skin friction + wave + induced)
- **Cm**: Pitching moment coefficient
- **CY**: Side force coefficient
- **Cl**: Rolling moment coefficient
- **Cn**: Yawing moment coefficient

### Model Types

1. **Analytical Model**: Physics-based calculations suitable for most rockets
2. **Table-Based Model**: Lookup tables from CFD or wind tunnel data

### Features

- **Compressibility Effects**: Prandtl-Glauert correction for subsonic/supersonic
- **Wave Drag**: Transonic drag rise and supersonic wave drag
- **Induced Drag**: Based on fin configuration and aspect ratio
- **Center of Pressure**: Automatic CP estimation
- **Reynolds Number**: Viscous effects modeling

### Configuration Parameters

```yaml
aero_model:
  reference_geometry:
    area: 0.00785             # m²
    length: 1.0               # meters
    diameter: 0.1
  
  body:
    length: 1.0
    diameter: 0.1
    nose_length: 0.2
    nose_shape: ogive         # cone, ogive, parabolic
  
  fins:
    count: 4
    area: 0.0025              # m² per fin
    span: 0.05                # meters
    root_chord: 0.1
    tip_chord: 0.05
    position: 0.85            # fraction from nose
  
  analytical_model:
    enabled: true
    base_drag: 0.15
    skin_friction: 0.02
    wave_drag: 0.1
    lift_slope: 2.0           # per radian
    moment_slope: -0.5        # per radian
```

### Usage Example

```python
from aero.aero_model import AeroModel

# Initialize
aero_model = AeroModel("config/control_systems_config.yaml")

# Get coefficients
coeffs = aero_model.get_coefficients(
    mach=0.8,
    angle_of_attack=5.0,      # degrees
    sideslip_angle=0.0
)

# Calculate forces and moments
forces = aero_model.calculate_forces(
    coefficients=coeffs,
    dynamic_pressure=5000.0,  # Pa
    velocity=250.0,           # m/s
    angle_of_attack=5.0,
    sideslip_angle=0.0
)

# Get center of pressure
cp = aero_model.estimate_center_of_pressure(mach=0.8)
```

### Drag Components

The total drag coefficient includes:

1. **Base Drag**: Pressure drag from base area
2. **Skin Friction**: Viscous drag along body
3. **Wave Drag**: Compressibility effects (transonic/supersonic)
4. **Induced Drag**: Drag due to lift generation

```
CD_total = CD_base + CD_skin + CD_wave + CD_induced
```

---

## Standard Atmosphere Model

US Standard Atmosphere 1976 implementation for accurate atmospheric properties.

### Features

- **Altitude Range**: Sea level to 86 km
- **Temperature Profile**: Accurate temperature vs altitude
- **Pressure Profile**: Exponential pressure decay with altitude layers
- **Density Calculation**: From ideal gas law
- **Speed of Sound**: Temperature-dependent
- **Viscosity**: Sutherland's formula for dynamic viscosity

### Model Types

1. **Standard (ISA/US 1976)**: Accurate multi-layer model
2. **Simple**: Exponential model for quick calculations

### Configuration Parameters

```yaml
atmosphere_model:
  model_type: standard        # standard or simple
  
  simple_model:
    scale_height: 8500.0      # meters
    temp_lapse_rate: 0.0065   # K/m
```

### Usage Example

```python
from atmosphere.standard_atmosphere import StandardAtmosphere, AtmosphereModel

# Initialize
atmosphere = StandardAtmosphere(AtmosphereModel.STANDARD)

# Get conditions at altitude
conditions = atmosphere.get_conditions(altitude=10000.0)  # meters

print(f"Temperature: {conditions.temperature} K")
print(f"Pressure: {conditions.pressure} Pa")
print(f"Density: {conditions.density} kg/m³")
print(f"Speed of Sound: {conditions.speed_of_sound} m/s")

# Calculate Mach number
mach = atmosphere.get_mach_number(velocity=300.0, altitude=10000.0)

# Calculate dynamic pressure
q = atmosphere.get_dynamic_pressure(velocity=300.0, altitude=10000.0)

# Calculate Reynolds number
Re = atmosphere.get_reynolds_number(
    velocity=300.0,
    altitude=10000.0,
    characteristic_length=1.0
)
```

### Atmospheric Layers

The standard atmosphere model includes 7 layers:

| Layer | Altitude (km) | Temperature | Lapse Rate (K/km) |
|-------|---------------|-------------|-------------------|
| Troposphere | 0 - 11 | 288.15 K | -6.5 |
| Tropopause | 11 - 20 | 216.65 K | 0.0 |
| Stratosphere 1 | 20 - 32 | 216.65 K | +1.0 |
| Stratosphere 2 | 32 - 47 | 228.65 K | +2.8 |
| Stratopause | 47 - 51 | 270.65 K | 0.0 |
| Mesosphere 1 | 51 - 71 | 270.65 K | -2.8 |
| Mesosphere 2 | 71 - 84.852 | 214.65 K | -2.0 |

---

## Configuration Guide

### Complete Configuration File Structure

```yaml
# Control Allocator
control_allocator:
  configuration: tail_only
  surfaces: [...]
  tvc: {...}
  mixing_weights: {...}
  rate_limits: {...}
  effectiveness_curves: {...}

# Thrust Model
thrust_model:
  curve_type: time_based
  curve_points: [...]
  motor_parameters: {...}
  mass_properties: {...}

# Aerodynamic Model
aero_model:
  reference_geometry: {...}
  body: {...}
  fins: {...}
  analytical_model: {...}

# Atmosphere Model
atmosphere_model:
  model_type: standard
```

### Configuration for Different Rocket Types

#### High-Power Hobby Rocket

```yaml
control_allocator:
  configuration: tail_only
  tvc:
    enabled: false

thrust_model:
  motor_parameters:
    name: "Cesaroni M1670"
    propellant_mass: 2.0
    burn_time: 3.5

aero_model:
  body:
    diameter: 0.15
    length: 1.5
```

#### Advanced Guided Rocket

```yaml
control_allocator:
  configuration: canard_and_tail
  tvc:
    enabled: true
  mixing_weights:
    canard: 0.3
    tail: 0.4
    tvc: 0.3
```

#### High-Speed Rocket (Supersonic/Hypersonic)

```yaml
thrust_model:
  motor_parameters:
    average_thrust: 5000.0
    burn_time: 10.0

aero_model:
  body:
    nose_shape: cone
  analytical_model:
    wave_drag: 0.15
```

---

## Integration Examples

### Basic Integration

```python
from controllers.control_allocator import ControlAllocator
from propulsion.thrust_model import ThrustModel
from aero.aero_model import AeroModel
from atmosphere.standard_atmosphere import StandardAtmosphere

# Initialize all systems
allocator = ControlAllocator("config/control_systems_config.yaml")
thrust_model = ThrustModel("config/control_systems_config.yaml")
aero_model = AeroModel("config/control_systems_config.yaml")
atmosphere = StandardAtmosphere()

# In control loop
def control_loop(altitude, velocity, aoa, roll_cmd, pitch_cmd, yaw_cmd, dt):
    # Get atmospheric conditions
    mach = atmosphere.get_mach_number(velocity, altitude)
    q = atmosphere.get_dynamic_pressure(velocity, altitude)
    
    # Update thrust
    thrust = thrust_model.update(time, dt)
    mass = thrust_model.get_mass()
    
    # Get aero coefficients
    coeffs = aero_model.get_coefficients(mach, aoa, 0.0)
    
    # Allocate control
    commands = allocator.allocate_control(
        roll_cmd, pitch_cmd, yaw_cmd,
        mach, q, aoa, dt
    )
    
    # Convert to PWM
    pwm = allocator.angles_to_pwm(commands)
    
    return pwm, thrust, mass
```

### Complete Example

See `examples/advanced_integration_example.py` for a complete working example showing:
- Full flight simulation
- TVC configuration
- High-speed flight regimes
- All system integration

---

## Troubleshooting

### Control Surface Not Responding

1. Check PWM channel mapping in configuration
2. Verify servo connections
3. Check rate limits (may be too restrictive)
4. Verify effectiveness curves (may be reducing authority)

### TVC Not Working

1. Ensure `tvc.enabled: true` in configuration
2. Check PWM channels for gimbal servos
3. Verify gimbal angle limits
4. Check mixing weights (TVC weight may be too low)

### Thrust Curve Issues

1. Verify curve file format (.eng, .csv, .yaml)
2. Check propellant mass matches curve
3. Ensure burn time is correct
4. Verify motor ignition was called

### Aerodynamic Coefficient Errors

1. Check Mach number range (tables may not cover it)
2. Verify angle of attack is reasonable
3. Check reference geometry (area, length)
4. Ensure nose shape is valid (cone, ogive, parabolic)

### Atmosphere Model Issues

1. Check altitude range (0-86 km for standard model)
2. Verify model type (standard vs simple)
3. Ensure velocity is positive for Mach calculation

---

## References

1. US Standard Atmosphere 1976
2. Sutherland's Viscosity Formula
3. Prandtl-Glauert Compressibility Correction
4. Barrowman Stability Equations
5. Rocket Propulsion Elements (Sutton & Biblarz)
6. Modern Compressible Flow (Anderson)

---

## Support

For questions or issues:
1. Check the configuration examples
2. Review the integration example
3. Consult the troubleshooting section
4. Check the main README.md and ADVANCED_FEATURES.md
