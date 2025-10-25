"""
Comprehensive Aerodynamic Model

This module provides complete aerodynamic coefficient modeling including:
- Lift coefficient (CL) vs Mach and angle of attack
- Drag coefficient (CD) vs Mach and angle of attack
- Pitching moment coefficient (Cm) vs Mach and angle of attack
- Side force coefficient (CY) vs Mach and sideslip
- Rolling moment coefficient (Cl) vs Mach and sideslip
- Yawing moment coefficient (Cn) vs Mach and sideslip
- Support for lookup tables and analytical models
- Reynolds number effects
- Compressibility effects

Author: Rocket Control System
Date: 2025-10-25
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Callable
import yaml


@dataclass
class AeroCoefficients:
    """Complete set of aerodynamic coefficients"""
    CL: float = 0.0  # Lift coefficient
    CD: float = 0.0  # Drag coefficient
    Cm: float = 0.0  # Pitching moment coefficient
    CY: float = 0.0  # Side force coefficient
    Cl: float = 0.0  # Rolling moment coefficient
    Cn: float = 0.0  # Yawing moment coefficient
    
    CL_alpha: float = 0.0  # dCL/dα
    CD_alpha: float = 0.0  # dCD/dα
    Cm_alpha: float = 0.0  # dCm/dα
    CY_beta: float = 0.0   # dCY/dβ
    Cl_beta: float = 0.0   # dCl/dβ
    Cn_beta: float = 0.0   # dCn/dβ


@dataclass
class AeroForces:
    """Aerodynamic forces and moments in body frame"""
    force_x: float = 0.0  # Axial force (N)
    force_y: float = 0.0  # Side force (N)
    force_z: float = 0.0  # Normal force (N)
    moment_x: float = 0.0  # Rolling moment (N·m)
    moment_y: float = 0.0  # Pitching moment (N·m)
    moment_z: float = 0.0  # Yawing moment (N·m)


class AeroModel:
    """
    Comprehensive aerodynamic model for rockets
    
    Features:
    - Lookup tables for coefficients vs Mach and AoA
    - Analytical models for simple geometries
    - Reynolds number corrections
    - Compressibility effects
    - Configurable reference areas and lengths
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize aerodynamic model
        
        Args:
            config_file: Path to YAML configuration file
        """
        self.reference_area: float = 0.01  # Reference area (m²)
        self.reference_length: float = 1.0  # Reference length (m)
        self.reference_diameter: float = 0.1  # Reference diameter (m)
        self.moment_reference_point: float = 0.5  # Moment reference point (m from nose)
        
        self.CL_table: Optional[np.ndarray] = None
        self.CD_table: Optional[np.ndarray] = None
        self.Cm_table: Optional[np.ndarray] = None
        self.CY_table: Optional[np.ndarray] = None
        self.Cl_table: Optional[np.ndarray] = None
        self.Cn_table: Optional[np.ndarray] = None
        
        self.mach_values: Optional[np.ndarray] = None
        self.aoa_values: Optional[np.ndarray] = None
        self.beta_values: Optional[np.ndarray] = None
        
        self.use_analytical_model: bool = True
        self.base_drag_coefficient: float = 0.15
        self.skin_friction_coefficient: float = 0.02
        self.wave_drag_coefficient: float = 0.1
        self.lift_slope: float = 2.0  # per radian
        self.moment_slope: float = -0.5  # per radian
        
        self.num_fins: int = 4
        self.fin_area: float = 0.001  # Single fin area (m²)
        self.fin_span: float = 0.05  # Fin span (m)
        self.fin_root_chord: float = 0.05  # Fin root chord (m)
        self.fin_tip_chord: float = 0.02  # Fin tip chord (m)
        self.fin_position: float = 0.9  # Fin position (fraction of length from nose)
        
        self.body_length: float = 1.0  # Body length (m)
        self.body_diameter: float = 0.1  # Body diameter (m)
        self.nose_length: float = 0.2  # Nose cone length (m)
        self.nose_shape: str = "ogive"  # Nose shape: cone, ogive, parabolic
        
        self.prandtl_glauert_correction: bool = True
        self.transonic_drag_peak: float = 1.2  # Mach number of peak drag
        
        if config_file:
            self.load_configuration(config_file)
    
    def load_configuration(self, config_file: str):
        """Load configuration from YAML file"""
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        aero_config = config.get('aero_model', {})
        
        geometry = aero_config.get('reference_geometry', {})
        self.reference_area = geometry.get('area', 0.01)
        self.reference_length = geometry.get('length', 1.0)
        self.reference_diameter = geometry.get('diameter', 0.1)
        self.moment_reference_point = geometry.get('moment_reference', 0.5)
        
        body = aero_config.get('body', {})
        self.body_length = body.get('length', 1.0)
        self.body_diameter = body.get('diameter', 0.1)
        self.nose_length = body.get('nose_length', 0.2)
        self.nose_shape = body.get('nose_shape', 'ogive')
        
        fins = aero_config.get('fins', {})
        self.num_fins = fins.get('count', 4)
        self.fin_area = fins.get('area', 0.001)
        self.fin_span = fins.get('span', 0.05)
        self.fin_root_chord = fins.get('root_chord', 0.05)
        self.fin_tip_chord = fins.get('tip_chord', 0.02)
        self.fin_position = fins.get('position', 0.9)
        
        analytical = aero_config.get('analytical_model', {})
        self.use_analytical_model = analytical.get('enabled', True)
        self.base_drag_coefficient = analytical.get('base_drag', 0.15)
        self.skin_friction_coefficient = analytical.get('skin_friction', 0.02)
        self.wave_drag_coefficient = analytical.get('wave_drag', 0.1)
        self.lift_slope = analytical.get('lift_slope', 2.0)
        self.moment_slope = analytical.get('moment_slope', -0.5)
        
        if 'coefficient_tables' in aero_config:
            self._load_coefficient_tables(aero_config['coefficient_tables'])
    
    def _load_coefficient_tables(self, tables_config: dict):
        """Load coefficient lookup tables"""
        if 'mach_values' in tables_config:
            self.mach_values = np.array(tables_config['mach_values'])
        if 'aoa_values' in tables_config:
            self.aoa_values = np.array(tables_config['aoa_values'])
        if 'beta_values' in tables_config:
            self.beta_values = np.array(tables_config['beta_values'])
        
        if 'CL' in tables_config:
            self.CL_table = np.array(tables_config['CL'])
        if 'CD' in tables_config:
            self.CD_table = np.array(tables_config['CD'])
        if 'Cm' in tables_config:
            self.Cm_table = np.array(tables_config['Cm'])
        if 'CY' in tables_config:
            self.CY_table = np.array(tables_config['CY'])
        if 'Cl' in tables_config:
            self.Cl_table = np.array(tables_config['Cl'])
        if 'Cn' in tables_config:
            self.Cn_table = np.array(tables_config['Cn'])
    
    def get_coefficients(self, mach: float, angle_of_attack: float, 
                        sideslip_angle: float = 0.0) -> AeroCoefficients:
        """
        Get aerodynamic coefficients at given flight conditions
        
        Args:
            mach: Mach number
            angle_of_attack: Angle of attack (degrees)
            sideslip_angle: Sideslip angle (degrees)
        
        Returns:
            AeroCoefficients object
        """
        if self.use_analytical_model or self.CL_table is None:
            return self._get_analytical_coefficients(mach, angle_of_attack, sideslip_angle)
        else:
            return self._get_table_coefficients(mach, angle_of_attack, sideslip_angle)
    
    def _get_analytical_coefficients(self, mach: float, aoa_deg: float, 
                                    beta_deg: float) -> AeroCoefficients:
        """Calculate coefficients using analytical models"""
        aoa_rad = np.radians(aoa_deg)
        beta_rad = np.radians(beta_deg)
        
        CL_subsonic = self.lift_slope * aoa_rad
        CL = self._apply_compressibility_correction(CL_subsonic, mach)
        
        CD_base = self.base_drag_coefficient
        CD_skin = self.skin_friction_coefficient * (1.0 + 0.144 * mach**2)
        CD_wave = self._calculate_wave_drag(mach)
        CD_induced = CL**2 / (np.pi * self._calculate_aspect_ratio())
        CD = CD_base + CD_skin + CD_wave + CD_induced
        
        Cm = self.moment_slope * aoa_rad
        
        CY = -self.lift_slope * beta_rad * 0.5  # Typically smaller than lift slope
        
        Cl = -0.1 * beta_rad  # Dihedral effect
        
        Cn = 0.05 * beta_rad  # Positive for stability
        
        CL_alpha = self.lift_slope
        CD_alpha = 2.0 * CL * CL_alpha / (np.pi * self._calculate_aspect_ratio())
        Cm_alpha = self.moment_slope
        CY_beta = -self.lift_slope * 0.5
        Cl_beta = -0.1
        Cn_beta = 0.05
        
        return AeroCoefficients(
            CL=CL, CD=CD, Cm=Cm, CY=CY, Cl=Cl, Cn=Cn,
            CL_alpha=CL_alpha, CD_alpha=CD_alpha, Cm_alpha=Cm_alpha,
            CY_beta=CY_beta, Cl_beta=Cl_beta, Cn_beta=Cn_beta
        )
    
    def _get_table_coefficients(self, mach: float, aoa_deg: float, 
                               beta_deg: float) -> AeroCoefficients:
        """Interpolate coefficients from lookup tables"""
        coeffs = AeroCoefficients()
        
        if self.CL_table is not None and self.mach_values is not None and self.aoa_values is not None:
            coeffs.CL = self._interpolate_2d(mach, aoa_deg, self.mach_values, 
                                             self.aoa_values, self.CL_table)
        
        if self.CD_table is not None:
            coeffs.CD = self._interpolate_2d(mach, aoa_deg, self.mach_values, 
                                             self.aoa_values, self.CD_table)
        
        if self.Cm_table is not None:
            coeffs.Cm = self._interpolate_2d(mach, aoa_deg, self.mach_values, 
                                             self.aoa_values, self.Cm_table)
        
        if self.CY_table is not None and self.beta_values is not None:
            coeffs.CY = self._interpolate_2d(mach, beta_deg, self.mach_values, 
                                             self.beta_values, self.CY_table)
        
        if self.Cl_table is not None:
            coeffs.Cl = self._interpolate_2d(mach, beta_deg, self.mach_values, 
                                             self.beta_values, self.Cl_table)
        
        if self.Cn_table is not None:
            coeffs.Cn = self._interpolate_2d(mach, beta_deg, self.mach_values, 
                                             self.beta_values, self.Cn_table)
        
        return coeffs
    
    def _interpolate_2d(self, x: float, y: float, x_values: np.ndarray, 
                       y_values: np.ndarray, table: np.ndarray) -> float:
        """2D bilinear interpolation"""
        x = np.clip(x, x_values[0], x_values[-1])
        y = np.clip(y, y_values[0], y_values[-1])
        
        i = np.searchsorted(x_values, x) - 1
        j = np.searchsorted(y_values, y) - 1
        
        i = np.clip(i, 0, len(x_values) - 2)
        j = np.clip(j, 0, len(y_values) - 2)
        
        wx = (x - x_values[i]) / (x_values[i + 1] - x_values[i]) if x_values[i + 1] != x_values[i] else 0.0
        wy = (y - y_values[j]) / (y_values[j + 1] - y_values[j]) if y_values[j + 1] != y_values[j] else 0.0
        
        value = (table[i, j] * (1 - wx) * (1 - wy) +
                table[i + 1, j] * wx * (1 - wy) +
                table[i, j + 1] * (1 - wx) * wy +
                table[i + 1, j + 1] * wx * wy)
        
        return value
    
    def _apply_compressibility_correction(self, coefficient: float, mach: float) -> float:
        """Apply Prandtl-Glauert compressibility correction"""
        if not self.prandtl_glauert_correction:
            return coefficient
        
        if mach < 0.8:
            beta = np.sqrt(1 - mach**2)
            return coefficient / beta
        elif mach < 1.2:
            return coefficient * (1.0 + 0.5 * (mach - 0.8) / 0.4)
        else:
            beta = np.sqrt(mach**2 - 1)
            return coefficient / beta * 0.7  # Reduced effectiveness
    
    def _calculate_wave_drag(self, mach: float) -> float:
        """Calculate wave drag coefficient"""
        if mach < 0.8:
            return 0.0
        elif mach < 1.2:
            return self.wave_drag_coefficient * np.sin(np.pi * (mach - 0.8) / 0.4) ** 2
        else:
            return self.wave_drag_coefficient * 0.5 / mach
    
    def _calculate_aspect_ratio(self) -> float:
        """Calculate effective aspect ratio for induced drag"""
        if self.num_fins == 0:
            return 4.0  # Body alone
        
        fin_span_total = self.fin_span * self.num_fins / 2.0
        fin_area_total = self.fin_area * self.num_fins
        
        if fin_area_total > 0:
            return fin_span_total**2 / fin_area_total
        return 4.0
    
    def calculate_forces(self, coefficients: AeroCoefficients, 
                        dynamic_pressure: float, velocity: float,
                        angle_of_attack: float, sideslip_angle: float) -> AeroForces:
        """
        Calculate aerodynamic forces and moments from coefficients
        
        Args:
            coefficients: Aerodynamic coefficients
            dynamic_pressure: Dynamic pressure (Pa)
            velocity: Velocity magnitude (m/s)
            angle_of_attack: Angle of attack (degrees)
            sideslip_angle: Sideslip angle (degrees)
        
        Returns:
            AeroForces object with forces and moments in body frame
        """
        aoa_rad = np.radians(angle_of_attack)
        beta_rad = np.radians(sideslip_angle)
        
        q_S = dynamic_pressure * self.reference_area
        
        force_x = -q_S * (coefficients.CD * np.cos(aoa_rad) - 
                         coefficients.CL * np.sin(aoa_rad))
        
        force_y = q_S * coefficients.CY
        
        force_z = -q_S * (coefficients.CD * np.sin(aoa_rad) + 
                         coefficients.CL * np.cos(aoa_rad))
        
        q_S_L = q_S * self.reference_length
        
        moment_x = q_S_L * coefficients.Cl
        
        moment_y = q_S_L * coefficients.Cm
        
        moment_z = q_S_L * coefficients.Cn
        
        return AeroForces(
            force_x=force_x,
            force_y=force_y,
            force_z=force_z,
            moment_x=moment_x,
            moment_y=moment_y,
            moment_z=moment_z
        )
    
    def estimate_center_of_pressure(self, mach: float) -> float:
        """
        Estimate center of pressure location
        
        Args:
            mach: Mach number
        
        Returns:
            Center of pressure position (m from nose)
        """
        
        if self.nose_shape == "cone":
            cp_nose = 0.67 * self.nose_length
        elif self.nose_shape == "ogive":
            cp_nose = 0.534 * self.nose_length
        else:  # parabolic
            cp_nose = 0.5 * self.nose_length
        
        cp_body = self.body_length * 0.5
        
        cp_fins = self.body_length * self.fin_position + self.fin_root_chord * 0.25
        
        cp = 0.1 * cp_nose + 0.1 * cp_body + 0.8 * cp_fins
        
        if mach > 1.0:
            cp += 0.05 * self.body_length * (mach - 1.0) / 2.0
        
        return cp
    
    def get_status(self, mach: float, aoa: float, beta: float = 0.0) -> dict:
        """Get comprehensive aerodynamic status"""
        coeffs = self.get_coefficients(mach, aoa, beta)
        cp = self.estimate_center_of_pressure(mach)
        
        return {
            'model_type': 'analytical' if self.use_analytical_model else 'table',
            'mach': mach,
            'angle_of_attack': aoa,
            'sideslip_angle': beta,
            'CL': coeffs.CL,
            'CD': coeffs.CD,
            'Cm': coeffs.Cm,
            'CY': coeffs.CY,
            'Cl': coeffs.Cl,
            'Cn': coeffs.Cn,
            'center_of_pressure': cp,
            'reference_area': self.reference_area,
            'reference_length': self.reference_length,
        }
