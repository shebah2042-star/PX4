"""
Multi-Stage Propulsion System

This module provides comprehensive multi-stage rocket support with:
- Multiple propulsion stages (2-3 stages typical)
- Automatic stage separation based on burnout detection
- Mass and CG updates after separation
- Stage-specific thrust curves and configurations
- Enable/disable multi-stage functionality

Author: Devin
Date: 2025-10-25
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum
import yaml
import logging

from propulsion.thrust_model import ThrustModel, MotorParameters, VehicleMassProperties


class StageState(Enum):
    """Stage states"""
    INACTIVE = "inactive"      # Stage not yet active
    ACTIVE = "active"          # Stage currently burning
    BURNOUT = "burnout"        # Stage burned out but not separated
    SEPARATED = "separated"    # Stage separated from vehicle


@dataclass
class StageConfiguration:
    """Configuration for a single stage"""
    stage_number: int                    # Stage number (1, 2, 3, etc.)
    name: str                           # Stage name (e.g., "Booster", "Sustainer")
    
    thrust_model: ThrustModel           # Thrust model for this stage
    
    dry_mass: float                     # Stage dry mass (kg)
    propellant_mass: float              # Propellant mass (kg)
    structural_mass: float              # Mass of stage structure that separates (kg)
    
    separation_delay: float = 0.5       # Delay after burnout before separation (s)
    separation_impulse: float = 0.0     # Impulse from separation mechanism (N·s)
    
    length: float = 0.0                 # Stage length (m)
    diameter: float = 0.0               # Stage diameter (m)
    
    state: StageState = StageState.INACTIVE
    ignition_time: Optional[float] = None
    burnout_time: Optional[float] = None
    separation_time: Optional[float] = None


@dataclass
class MultiStageConfiguration:
    """Configuration for multi-stage system"""
    enabled: bool = False               # Enable/disable multi-stage
    num_stages: int = 1                 # Number of stages
    stages: List[StageConfiguration] = field(default_factory=list)
    
    auto_ignition: bool = True          # Auto-ignite next stage after separation
    ignition_delay: float = 0.1         # Delay after separation before next ignition (s)
    
    min_altitude_for_staging: float = 100.0  # Minimum altitude for staging (m)
    max_staging_velocity: float = 1000.0     # Maximum velocity for staging (m/s)


class MultiStageSystem:
    """
    Multi-stage propulsion system manager
    
    Handles:
    - Multiple propulsion stages
    - Stage ignition and burnout detection
    - Stage separation with mass/CG updates
    - Automatic staging sequence
    """
    
    def __init__(self, config: Optional[MultiStageConfiguration] = None):
        """
        Initialize multi-stage system
        
        Args:
            config: Multi-stage configuration
        """
        self.logger = logging.getLogger(__name__)
        
        self.config = config or MultiStageConfiguration()
        
        self.current_stage_index = 0
        self.mission_time = 0.0
        self.total_mass = 0.0
        self.total_cg = np.array([0.0, 0.0, 0.0])
        
        self._calculate_total_mass_cg()
        
        self.logger.info(f"Multi-stage system initialized: {self.config.num_stages} stages, "
                        f"enabled={self.config.enabled}")
    
    @classmethod
    def from_yaml(cls, config_file: str) -> 'MultiStageSystem':
        """
        Load configuration from YAML file
        
        Args:
            config_file: Path to YAML configuration file
            
        Returns:
            MultiStageSystem instance
        """
        with open(config_file, 'r') as f:
            config_dict = yaml.safe_load(f)
        
        multi_stage_config = config_dict.get('multi_stage', {})
        
        config = MultiStageConfiguration(
            enabled=multi_stage_config.get('enabled', False),
            num_stages=multi_stage_config.get('num_stages', 1),
            auto_ignition=multi_stage_config.get('auto_ignition', True),
            ignition_delay=multi_stage_config.get('ignition_delay', 0.1),
            min_altitude_for_staging=multi_stage_config.get('min_altitude_for_staging', 100.0),
            max_staging_velocity=multi_stage_config.get('max_staging_velocity', 1000.0)
        )
        
        stages_config = multi_stage_config.get('stages', [])
        for i, stage_dict in enumerate(stages_config):
            thrust_model = ThrustModel()
            
            if 'thrust_curve_file' in stage_dict:
                curve_file = stage_dict['thrust_curve_file']
                if curve_file.endswith('.eng'):
                    thrust_model.load_eng_file(curve_file)
                elif curve_file.endswith('.csv'):
                    thrust_model.load_csv_file(curve_file)
                elif curve_file.endswith('.yaml'):
                    thrust_model.load_yaml_curve(curve_file)
            
            stage = StageConfiguration(
                stage_number=i + 1,
                name=stage_dict.get('name', f'Stage {i+1}'),
                thrust_model=thrust_model,
                dry_mass=stage_dict.get('dry_mass', 0.0),
                propellant_mass=stage_dict.get('propellant_mass', 0.0),
                structural_mass=stage_dict.get('structural_mass', 0.0),
                separation_delay=stage_dict.get('separation_delay', 0.5),
                separation_impulse=stage_dict.get('separation_impulse', 0.0),
                length=stage_dict.get('length', 0.0),
                diameter=stage_dict.get('diameter', 0.0)
            )
            
            config.stages.append(stage)
        
        return cls(config)
    
    def _calculate_total_mass_cg(self):
        """Calculate total vehicle mass and CG"""
        if not self.config.enabled or len(self.config.stages) == 0:
            self.total_mass = 0.0
            self.total_cg = np.array([0.0, 0.0, 0.0])
            return
        
        total_mass = 0.0
        weighted_cg = np.array([0.0, 0.0, 0.0])
        
        for stage in self.config.stages:
            if stage.state != StageState.SEPARATED:
                stage_mass = stage.thrust_model.get_mass()
                total_mass += stage_mass
                
                stage_cg = stage.thrust_model.get_cg()
                weighted_cg += stage_cg * stage_mass
        
        if total_mass > 0:
            self.total_cg = weighted_cg / total_mass
        
        self.total_mass = total_mass
    
    def update(self, dt: float, altitude: float = 0.0, velocity: float = 0.0) -> Dict[str, Any]:
        """
        Update multi-stage system
        
        Args:
            dt: Time step (s)
            altitude: Current altitude (m)
            velocity: Current velocity (m/s)
            
        Returns:
            Dictionary with system status
        """
        self.mission_time += dt
        
        if not self.config.enabled:
            return self._get_status()
        
        current_stage = self._get_current_stage()
        if current_stage is None:
            return self._get_status()
        
        if current_stage.state == StageState.ACTIVE:
            current_stage.thrust_model.update(self.mission_time, dt)
            
            if current_stage.thrust_model.is_burnout():
                current_stage.state = StageState.BURNOUT
                current_stage.burnout_time = self.mission_time
                self.logger.info(f"Stage {current_stage.stage_number} burnout at t={self.mission_time:.2f}s")
        
        if current_stage.state == StageState.BURNOUT:
            time_since_burnout = self.mission_time - current_stage.burnout_time
            
            can_separate = True
            if altitude < self.config.min_altitude_for_staging:
                can_separate = False
                self.logger.warning(f"Altitude {altitude:.1f}m below minimum for staging")
            
            if velocity > self.config.max_staging_velocity:
                can_separate = False
                self.logger.warning(f"Velocity {velocity:.1f}m/s above maximum for staging")
            
            if can_separate and time_since_burnout >= current_stage.separation_delay:
                self._separate_stage(current_stage)
                
                if self.config.auto_ignition:
                    next_stage = self._get_next_stage()
                    if next_stage is not None:
                        if time_since_burnout >= (current_stage.separation_delay + 
                                                 self.config.ignition_delay):
                            self._ignite_stage(next_stage)
        
        self._calculate_total_mass_cg()
        
        return self._get_status()
    
    def _get_current_stage(self) -> Optional[StageConfiguration]:
        """Get current active or burning stage"""
        if self.current_stage_index >= len(self.config.stages):
            return None
        return self.config.stages[self.current_stage_index]
    
    def _get_next_stage(self) -> Optional[StageConfiguration]:
        """Get next stage to be ignited"""
        next_index = self.current_stage_index + 1
        if next_index >= len(self.config.stages):
            return None
        return self.config.stages[next_index]
    
    def _ignite_stage(self, stage: StageConfiguration):
        """Ignite a stage"""
        stage.state = StageState.ACTIVE
        stage.ignition_time = self.mission_time
        stage.thrust_model.ignite(self.mission_time)
        self.logger.info(f"Stage {stage.stage_number} ignited at t={self.mission_time:.2f}s")
    
    def _separate_stage(self, stage: StageConfiguration):
        """Separate a stage"""
        stage.state = StageState.SEPARATED
        stage.separation_time = self.mission_time
        self.current_stage_index += 1
        
        self.logger.info(f"Stage {stage.stage_number} separated at t={self.mission_time:.2f}s, "
                        f"mass reduction: {stage.structural_mass:.2f}kg")
    
    def ignite_first_stage(self):
        """Ignite the first stage"""
        if not self.config.enabled or len(self.config.stages) == 0:
            return
        
        first_stage = self.config.stages[0]
        if first_stage.state == StageState.INACTIVE:
            self._ignite_stage(first_stage)
    
    def get_current_thrust(self) -> float:
        """
        Get current thrust from active stage
        
        Returns:
            Current thrust (N)
        """
        if not self.config.enabled:
            return 0.0
        
        current_stage = self._get_current_stage()
        if current_stage is None or current_stage.state != StageState.ACTIVE:
            return 0.0
        
        return current_stage.thrust_model.get_current_thrust()
    
    def get_total_mass(self) -> float:
        """
        Get total vehicle mass
        
        Returns:
            Total mass (kg)
        """
        return self.total_mass
    
    def get_total_cg(self) -> np.ndarray:
        """
        Get total vehicle center of gravity
        
        Returns:
            CG position (m) as [x, y, z]
        """
        return self.total_cg.copy()
    
    def is_all_stages_complete(self) -> bool:
        """
        Check if all stages are complete
        
        Returns:
            True if all stages burned out and separated
        """
        if not self.config.enabled:
            return False
        
        for stage in self.config.stages:
            if stage.state != StageState.SEPARATED:
                return False
        
        return True
    
    def get_active_stage_number(self) -> int:
        """
        Get current active stage number
        
        Returns:
            Stage number (1-indexed), or 0 if no active stage
        """
        current_stage = self._get_current_stage()
        if current_stage is None:
            return 0
        return current_stage.stage_number
    
    def _get_status(self) -> Dict[str, Any]:
        """Get system status"""
        status = {
            'enabled': self.config.enabled,
            'mission_time': self.mission_time,
            'total_mass': self.total_mass,
            'total_cg': self.total_cg.tolist(),
            'current_stage': self.get_active_stage_number(),
            'current_thrust': self.get_current_thrust(),
            'all_stages_complete': self.is_all_stages_complete(),
            'stages': []
        }
        
        for stage in self.config.stages:
            stage_status = {
                'number': stage.stage_number,
                'name': stage.name,
                'state': stage.state.value,
                'ignition_time': stage.ignition_time,
                'burnout_time': stage.burnout_time,
                'separation_time': stage.separation_time
            }
            status['stages'].append(stage_status)
        
        return status


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    print("Multi-Stage Propulsion System Example")
    print("=" * 50)
    
    booster = ThrustModel()
    booster.motor_params = MotorParameters(
        name="Booster",
        total_impulse=10000.0,
        burn_time=5.0,
        propellant_mass=5.0,
        diameter=0.1,
        length=0.5
    )
    
    sustainer = ThrustModel()
    sustainer.motor_params = MotorParameters(
        name="Sustainer",
        total_impulse=5000.0,
        burn_time=10.0,
        propellant_mass=2.0,
        diameter=0.08,
        length=0.4
    )
    
    config = MultiStageConfiguration(
        enabled=True,
        num_stages=2,
        auto_ignition=True,
        ignition_delay=0.5
    )
    
    stage1 = StageConfiguration(
        stage_number=1,
        name="Booster",
        thrust_model=booster,
        dry_mass=2.0,
        propellant_mass=5.0,
        structural_mass=1.5,
        separation_delay=0.5
    )
    
    stage2 = StageConfiguration(
        stage_number=2,
        name="Sustainer",
        thrust_model=sustainer,
        dry_mass=1.0,
        propellant_mass=2.0,
        structural_mass=0.5,
        separation_delay=0.5
    )
    
    config.stages = [stage1, stage2]
    
    system = MultiStageSystem(config)
    
    system.ignite_first_stage()
    
    dt = 0.1
    for i in range(200):
        t = i * dt
        altitude = 1000.0  # Simplified
        velocity = 100.0   # Simplified
        
        status = system.update(dt, altitude, velocity)
        
        if i % 10 == 0:
            print(f"\nTime: {t:.1f}s")
            print(f"  Stage: {status['current_stage']}")
            print(f"  Thrust: {status['current_thrust']:.1f}N")
            print(f"  Mass: {status['total_mass']:.2f}kg")
            print(f"  All complete: {status['all_stages_complete']}")
