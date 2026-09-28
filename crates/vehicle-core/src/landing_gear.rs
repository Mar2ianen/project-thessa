//! Backend-neutral parametric wheel chassis, tires, struts, brakes, and drive.
//!
//! The module owns SI-valued vehicle data and deterministic component laws.
//! Rapier articulation/contact integration lives in `thessa-collision`.

use std::{error::Error, fmt};

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::ElectricMotorSpec;

/// Upper bound on authored wheels in one chassis assembly.
///
/// This is a format/resource guard, not a physical wheel-count coefficient.
pub const MAX_WHEELS_PER_CHASSIS: u16 = 64;
const UNIT_QUATERNION_TOLERANCE: f64 = 1.0e-6;
const SYMMETRY_TOLERANCE: f64 = 1.0e-10;

/// Spatial arrangement of the wheels belonging to one chassis assembly.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WheelLayout {
    /// Wheels are arranged in tandem along the chassis longitudinal axis.
    Inline,
    /// Wheels form left/right pairs at longitudinal axle stations.
    AxlePairs { track_width_m: f64 },
}

/// Airless wheel structure family. Effective stiffness remains a measured
/// property on [`WheelTireSpec`], rather than an arbitrary gain inferred from
/// a display-only structure name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AirlessWheelStructure {
    Spoked {
        spoke_count: u16,
    },
    Lattice {
        circumferential_cells: u16,
        axial_cells: u16,
    },
}

/// Tire construction parameters that differ between pneumatic and airless
/// wheels. Radial stiffness/damping are measured/equivalent values in both
/// cases and are stored separately on [`WheelTireSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum TireConstruction {
    Pneumatic {
        /// Sealed tire's absolute pressure at the reference temperature (Pa).
        inflation_pressure_pa: f64,
        reference_temperature_k: f64,
    },
    Airless {
        structure: AirlessWheelStructure,
        structure_density_kg_m3: f64,
        minimum_temperature_k: f64,
        maximum_temperature_k: f64,
    },
}

/// One wheel/tire assembly, with radial properties fitted at its authored
/// construction and nominal operating condition.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelTireSpec {
    pub construction: TireConstruction,
    pub radius_m: f64,
    pub width_m: f64,
    /// Installed tire/rim/hub mass per wheel (kg).
    pub mass_kg: f64,
    /// Spin inertia around the wheel axle (kg·m²).
    pub spin_inertia_kg_m2: f64,
    /// Equivalent radial stiffness at the authored reference condition (N/m).
    pub radial_stiffness_n_m: f64,
    /// Radial damping coefficient (N·s/m).
    pub radial_damping_n_s_m: f64,
    /// Longitudinal force slope against contact-patch slip velocity (N/(m/s)).
    pub longitudinal_slip_stiffness_n_per_mps: f64,
    /// Lateral force slope against contact-patch slip velocity (N/(m/s)).
    pub lateral_slip_stiffness_n_per_mps: f64,
    pub maximum_deflection_m: f64,
    pub maximum_load_n: f64,
    /// Tire-side friction material parameter consumed by the contact backend.
    pub surface_friction: f64,
}

impl WheelTireSpec {
    pub fn validate(self) -> Result<(), LandingGearError> {
        require_positive(self.radius_m, "wheel radius")?;
        require_positive(self.width_m, "wheel width")?;
        require_positive(self.mass_kg, "wheel/tire mass")?;
        require_positive(self.spin_inertia_kg_m2, "wheel spin inertia")?;
        require_positive(self.radial_stiffness_n_m, "tire radial stiffness")?;
        require_non_negative(self.radial_damping_n_s_m, "tire radial damping")?;
        require_positive(
            self.longitudinal_slip_stiffness_n_per_mps,
            "tire longitudinal slip stiffness",
        )?;
        require_positive(
            self.lateral_slip_stiffness_n_per_mps,
            "tire lateral slip stiffness",
        )?;
        require_positive(self.maximum_deflection_m, "tire maximum deflection")?;
        if self.maximum_deflection_m >= self.radius_m {
            return Err(LandingGearError::InvalidSpec(
                "tire maximum deflection must be smaller than its radius".into(),
            ));
        }
        require_positive(self.maximum_load_n, "tire maximum load")?;
        require_non_negative(self.surface_friction, "tire surface friction")?;
        match self.construction {
            TireConstruction::Pneumatic {
                inflation_pressure_pa,
                reference_temperature_k,
            } => {
                require_positive(inflation_pressure_pa, "tire absolute inflation pressure")?;
                require_positive(reference_temperature_k, "tire reference temperature")?;
            }
            TireConstruction::Airless {
                structure,
                structure_density_kg_m3,
                minimum_temperature_k,
                maximum_temperature_k,
            } => {
                match structure {
                    AirlessWheelStructure::Spoked { spoke_count } if spoke_count < 3 => {
                        return Err(LandingGearError::InvalidSpec(
                            "airless spoked wheel needs at least three spokes".into(),
                        ));
                    }
                    AirlessWheelStructure::Lattice {
                        circumferential_cells,
                        axial_cells,
                    } if circumferential_cells == 0 || axial_cells == 0 => {
                        return Err(LandingGearError::InvalidSpec(
                            "airless lattice cell counts must be positive".into(),
                        ));
                    }
                    _ => {}
                }
                require_positive(structure_density_kg_m3, "airless structure density")?;
                require_positive(minimum_temperature_k, "airless minimum temperature")?;
                require_positive(maximum_temperature_k, "airless maximum temperature")?;
                if minimum_temperature_k >= maximum_temperature_k {
                    return Err(LandingGearError::InvalidSpec(
                        "airless tire temperature limits must be ordered".into(),
                    ));
                }
            }
        }
        Ok(())
    }

    /// Resolve tire/terrain Coulomb friction using Rapier's default arithmetic
    /// mean combine rule. Custom terrain policies can still pass their
    /// resolved pair coefficient directly to the force evaluators.
    pub fn contact_friction(self, terrain_friction: f64) -> Result<f64, LandingGearError> {
        self.validate()?;
        require_non_negative(terrain_friction, "terrain surface friction")?;
        let contact_friction = 0.5 * self.surface_friction + 0.5 * terrain_friction;
        if !contact_friction.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "resolved tire/terrain friction overflowed".into(),
            ));
        }
        Ok(contact_friction)
    }

    /// Unilateral spring/damper reaction for a geometric tire compression.
    /// Positive compression rate means the wheel is moving farther into the
    /// terrain. Negative loads are clipped because a tire cannot pull on the
    /// ground. For wheel/terrain pairs the tire law owns the normal reaction;
    /// Rapier supplies contact geometry and integrates the resulting force.
    /// Such pairs must be query/sensor colliders, not a second solid response.
    pub fn normal_load(
        self,
        compression_m: f64,
        compression_rate_mps: f64,
    ) -> Result<TireLoadPoint, LandingGearError> {
        self.validate()?;
        validate_finite(compression_m, "tire compression")?;
        validate_finite(compression_rate_mps, "tire compression rate")?;
        if compression_m < 0.0 {
            return Ok(TireLoadPoint {
                compression_m: 0.0,
                normal_load_n: 0.0,
                saturated: false,
            });
        }
        let compression = compression_m.min(self.maximum_deflection_m);
        let raw_load = self.radial_stiffness_n_m * compression
            + self.radial_damping_n_s_m * compression_rate_mps;
        if !raw_load.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "tire load calculation overflowed".into(),
            ));
        }
        Ok(TireLoadPoint {
            compression_m: compression,
            normal_load_n: raw_load.clamp(0.0, self.maximum_load_n),
            saturated: compression_m > self.maximum_deflection_m
                || raw_load > self.maximum_load_n
                || raw_load < 0.0,
        })
    }

    /// Reduced-order tangent-plane tire force. Slip velocities are those of
    /// the wheel contact patch relative to the terrain in wheel-forward and
    /// wheel-lateral axes; returned forces oppose slip and stay inside the
    /// Coulomb friction circle `|F| <= μ N`.
    pub fn tangential_force(
        self,
        longitudinal_slip_velocity_mps: f64,
        lateral_slip_velocity_mps: f64,
        normal_load_n: f64,
        contact_friction: f64,
    ) -> Result<TireTangentForcePoint, LandingGearError> {
        self.validate()?;
        validate_finite(
            longitudinal_slip_velocity_mps,
            "longitudinal tire slip velocity",
        )?;
        validate_finite(lateral_slip_velocity_mps, "lateral tire slip velocity")?;
        require_non_negative(normal_load_n, "wheel normal load")?;
        require_non_negative(contact_friction, "contact friction")?;
        let requested_longitudinal_n =
            -self.longitudinal_slip_stiffness_n_per_mps * longitudinal_slip_velocity_mps;
        let requested_lateral_n =
            -self.lateral_slip_stiffness_n_per_mps * lateral_slip_velocity_mps;
        let friction_limit_n = contact_friction * normal_load_n;
        let requested_magnitude_n = requested_longitudinal_n.hypot(requested_lateral_n);
        if !requested_magnitude_n.is_finite() || !friction_limit_n.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "tire tangent force calculation overflowed".into(),
            ));
        }
        let scale = if requested_magnitude_n > friction_limit_n && requested_magnitude_n > 0.0 {
            friction_limit_n / requested_magnitude_n
        } else {
            1.0
        };
        Ok(TireTangentForcePoint {
            longitudinal_force_n: requested_longitudinal_n * scale,
            lateral_force_n: requested_lateral_n * scale,
            friction_limit_n,
            saturated: scale < 1.0,
        })
    }
}

/// Result of evaluating the tire's unilateral radial response.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TireLoadPoint {
    pub compression_m: f64,
    pub normal_load_n: f64,
    pub saturated: bool,
}

/// Resolved longitudinal/lateral tire force in the wheel contact frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TireTangentForcePoint {
    pub longitudinal_force_n: f64,
    pub lateral_force_n: f64,
    pub friction_limit_n: f64,
    pub saturated: bool,
}

/// One suspension strut, replicated once per wheel station in its chassis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelStrutSpec {
    /// Mount-to-axle distance at full extension (m).
    pub extended_length_m: f64,
    pub stroke_m: f64,
    pub spring_rate_n_m: f64,
    pub damping_n_s_m: f64,
    pub preload_n: f64,
    pub minimum_force_n: f64,
    pub maximum_force_n: f64,
    /// Strut unsprung/linked mass per wheel station (kg).
    pub mass_per_wheel_kg: f64,
}

impl WheelStrutSpec {
    pub fn validate(self) -> Result<(), LandingGearError> {
        require_positive(self.extended_length_m, "strut extended length")?;
        require_positive(self.stroke_m, "strut stroke")?;
        if self.stroke_m >= self.extended_length_m {
            return Err(LandingGearError::InvalidSpec(
                "strut stroke must be shorter than its extended length".into(),
            ));
        }
        require_positive(self.spring_rate_n_m, "strut spring rate")?;
        require_non_negative(self.damping_n_s_m, "strut damping")?;
        require_non_negative(self.preload_n, "strut preload")?;
        require_non_negative(self.minimum_force_n, "strut minimum force")?;
        require_positive(self.maximum_force_n, "strut maximum force")?;
        if self.minimum_force_n > self.maximum_force_n {
            return Err(LandingGearError::InvalidSpec(
                "strut minimum force must not exceed maximum force".into(),
            ));
        }
        require_non_negative(self.mass_per_wheel_kg, "strut mass")
    }

    /// Bounded axial spring/damper force. The Rapier prismatic joint owns the
    /// hard stroke stops; this method supplies only the continuous physical
    /// spring, preload and damper load.
    pub fn axial_force(
        self,
        compression_m: f64,
        compression_rate_mps: f64,
    ) -> Result<StrutLoadPoint, LandingGearError> {
        self.validate()?;
        validate_finite(compression_m, "strut compression")?;
        validate_finite(compression_rate_mps, "strut compression rate")?;
        let compression = compression_m.clamp(0.0, self.stroke_m);
        let raw_force = self.preload_n
            + self.spring_rate_n_m * compression
            + self.damping_n_s_m * compression_rate_mps;
        if !raw_force.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "strut force calculation overflowed".into(),
            ));
        }
        Ok(StrutLoadPoint {
            compression_m: compression,
            axial_force_n: raw_force.clamp(self.minimum_force_n, self.maximum_force_n),
            at_stroke_stop: compression_m >= self.stroke_m,
            saturated: raw_force > self.maximum_force_n || raw_force < self.minimum_force_n,
        })
    }
}

/// Evaluate the series tire/strut load for a wheel against a local terrain
/// plane. Compression inputs are measured along the terrain normal; the
/// alignment is `abs(normal · strut_axis)`.
impl WheelChassisSpec {
    pub fn contact_load(
        &self,
        radial_penetration_m: f64,
        radial_penetration_rate_mps: f64,
        normal_alignment: f64,
    ) -> Result<WheelContactLoadPoint, LandingGearError> {
        self.tire.validate()?;
        self.strut.validate()?;
        validate_finite(radial_penetration_m, "wheel radial penetration")?;
        validate_finite(radial_penetration_rate_mps, "wheel radial penetration rate")?;
        if !normal_alignment.is_finite()
            || normal_alignment <= 0.0
            || normal_alignment > 1.0 + 1.0e-9
        {
            return Err(LandingGearError::InvalidCommand(
                "wheel/terrain normal alignment must be in (0, 1]".into(),
            ));
        }
        let alignment = normal_alignment.min(1.0);
        if radial_penetration_m <= 0.0 {
            return Ok(WheelContactLoadPoint::zero());
        }

        // The radial overlap is the sum of the tire's normal compression and
        // the strut's axial compression projected onto the terrain normal.
        // Allocate the displacement by the two series spring compliances.
        let axial_penetration_m = radial_penetration_m / alignment;
        let axial_rate_mps = radial_penetration_rate_mps / alignment;
        let maximum_axial_compression_m =
            self.strut.stroke_m + self.tire.maximum_deflection_m / alignment;
        let resolved_axial_penetration_m = axial_penetration_m.min(maximum_axial_compression_m);
        let tire_axial_stiffness_n_m = self.tire.radial_stiffness_n_m * alignment * alignment;
        let strut_fraction =
            tire_axial_stiffness_n_m / (self.strut.spring_rate_n_m + tire_axial_stiffness_n_m);
        let unconstrained_strut_compression_m =
            (tire_axial_stiffness_n_m * resolved_axial_penetration_m - self.strut.preload_n)
                / (self.strut.spring_rate_n_m + tire_axial_stiffness_n_m);
        let strut_compression_m = unconstrained_strut_compression_m.clamp(0.0, self.strut.stroke_m);
        let strut_at_limit = strut_compression_m != unconstrained_strut_compression_m;
        let tire_compression_m = (alignment * (resolved_axial_penetration_m - strut_compression_m))
            .clamp(0.0, self.tire.maximum_deflection_m);
        let strut_rate_mps = if strut_at_limit {
            0.0
        } else {
            strut_fraction * axial_rate_mps
        };
        let tire_rate_mps = if strut_at_limit {
            alignment * axial_rate_mps
        } else {
            alignment * (1.0 - strut_fraction) * axial_rate_mps
        };
        let strut = self
            .strut
            .axial_force(strut_compression_m, strut_rate_mps)?;
        let tire = self.tire.normal_load(tire_compression_m, tire_rate_mps)?;
        let normal_load_n = tire.normal_load_n.min(strut.axial_force_n / alignment);
        if !normal_load_n.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "series wheel contact load overflowed".into(),
            ));
        }
        Ok(WheelContactLoadPoint {
            radial_penetration_m,
            strut_compression_m,
            tire_compression_m,
            normal_load_n,
            saturated: axial_penetration_m > maximum_axial_compression_m
                || strut.saturated
                || tire.saturated,
        })
    }
}

/// Tire/strut series response at one geometric wheel/terrain contact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelContactLoadPoint {
    pub radial_penetration_m: f64,
    pub strut_compression_m: f64,
    pub tire_compression_m: f64,
    pub normal_load_n: f64,
    pub saturated: bool,
}

impl WheelContactLoadPoint {
    fn zero() -> Self {
        Self {
            radial_penetration_m: 0.0,
            strut_compression_m: 0.0,
            tire_compression_m: 0.0,
            normal_load_n: 0.0,
            saturated: false,
        }
    }
}

/// Result of evaluating a strut's bounded spring/damper response.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StrutLoadPoint {
    pub compression_m: f64,
    pub axial_force_n: f64,
    pub at_stroke_stop: bool,
    pub saturated: bool,
}

/// Brake-actuator capacity fitted to each braked wheel in a chassis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelBrakeSpec {
    /// Maximum brake torque at one wheel hub (N·m).
    pub maximum_torque_nm: f64,
    pub response_time_s: f64,
    pub mass_per_wheel_kg: f64,
}

impl WheelBrakeSpec {
    pub fn validate(self) -> Result<(), LandingGearError> {
        require_non_negative(self.maximum_torque_nm, "wheel brake maximum torque")?;
        require_non_negative(self.response_time_s, "wheel brake response time")?;
        require_non_negative(self.mass_per_wheel_kg, "wheel brake mass")
    }

    /// Evaluate the requested hub brake torque and the maximum force the
    /// current tire load could transfer without slipping. The torque acts on
    /// wheel spin; it is not clipped to grip, so wheel lock remains possible.
    /// `actuator_fraction` is the achieved brake state after [`Self::advance`].
    pub fn braking_force(
        self,
        actuator_fraction: f64,
        wheel_radius_m: f64,
        normal_load_n: f64,
        contact_friction: f64,
    ) -> Result<BrakePoint, LandingGearError> {
        self.validate()?;
        if !actuator_fraction.is_finite() || !(0.0..=1.0).contains(&actuator_fraction) {
            return Err(LandingGearError::InvalidCommand(
                "achieved brake fraction must be finite and in [0, 1]".into(),
            ));
        }
        require_positive(wheel_radius_m, "wheel radius")?;
        require_non_negative(normal_load_n, "wheel normal load")?;
        require_non_negative(contact_friction, "contact friction")?;
        let brake_torque_nm = self.maximum_torque_nm * actuator_fraction;
        let traction_torque_nm = contact_friction * normal_load_n * wheel_radius_m;
        if !traction_torque_nm.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "tire traction limit overflowed".into(),
            ));
        }
        let maximum_braking_force_n =
            (brake_torque_nm / wheel_radius_m).min(traction_torque_nm / wheel_radius_m);
        if !maximum_braking_force_n.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "maximum tire braking force overflowed".into(),
            ));
        }
        Ok(BrakePoint {
            brake_torque_nm,
            maximum_braking_force_n,
            traction_limited: traction_torque_nm < brake_torque_nm,
            actuator_limited: actuator_fraction < 1.0,
        })
    }

    /// Advance the first-order brake actuator using the exact response over a
    /// fixed simulation interval. A zero response time means an ideal actuator.
    pub fn advance(
        self,
        state: WheelBrakeState,
        command: f64,
        dt_s: f64,
    ) -> Result<WheelBrakeState, LandingGearError> {
        self.validate()?;
        if !state.applied_fraction.is_finite()
            || !(0.0..=1.0).contains(&state.applied_fraction)
            || !command.is_finite()
            || !(0.0..=1.0).contains(&command)
            || !dt_s.is_finite()
            || dt_s < 0.0
        {
            return Err(LandingGearError::InvalidCommand(
                "brake state/command must be in [0, 1] and step duration finite/non-negative"
                    .into(),
            ));
        }
        let applied_fraction = if self.response_time_s == 0.0 {
            command
        } else {
            command + (state.applied_fraction - command) * (-dt_s / self.response_time_s).exp()
        };
        Ok(WheelBrakeState {
            applied_fraction: applied_fraction.clamp(0.0, 1.0),
        })
    }
}

/// Dynamic state of one brake actuator, as a normalized achieved clamp level.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelBrakeState {
    pub applied_fraction: f64,
}

impl Default for WheelBrakeState {
    fn default() -> Self {
        Self {
            applied_fraction: 0.0,
        }
    }
}

/// Resolved service-brake output at one wheel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BrakePoint {
    /// Applied hub torque, independent of the tire's current grip.
    pub brake_torque_nm: f64,
    /// Maximum force this tire load can transmit before longitudinal slip.
    pub maximum_braking_force_n: f64,
    pub traction_limited: bool,
    pub actuator_limited: bool,
}

/// Optional traction motor and final drive on a wheel chassis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelDriveSpec {
    /// Existing continuous-duty motor envelope; wheel traction adds the
    /// low-speed/stall-loss parameter below.
    pub motor: ElectricMotorSpec,
    /// Copper-loss heat at rated peak torque and zero shaft speed (W).
    pub stall_copper_loss_w: f64,
    /// Rotor spin inertia at the motor shaft (kg·m²).
    pub rotor_inertia_kg_m2: f64,
    pub final_drive_ratio: f64,
    pub drivetrain_efficiency: f64,
    pub driven_wheel_count: u16,
}

impl WheelDriveSpec {
    pub fn compile(self, chassis_wheel_count: u16) -> Result<CompiledWheelDrive, LandingGearError> {
        let motor = self
            .motor
            .compile()
            .map_err(|error| LandingGearError::InvalidSpec(error.to_string()))?;
        require_non_negative(self.stall_copper_loss_w, "motor stall copper loss")?;
        require_non_negative(self.rotor_inertia_kg_m2, "motor rotor inertia")?;
        require_positive(self.final_drive_ratio, "wheel final-drive ratio")?;
        require_unit_interval(self.drivetrain_efficiency, "wheel drivetrain efficiency")?;
        if self.driven_wheel_count == 0 || self.driven_wheel_count > chassis_wheel_count {
            return Err(LandingGearError::InvalidSpec(
                "driven wheel count must be in [1, chassis wheel count]".into(),
            ));
        }
        let derived_motor_speed_rad_s = motor.maximum_rpm * std::f64::consts::TAU / 60.0;
        if !(motor.peak_torque_nm * derived_motor_speed_rad_s).is_finite()
            || !(motor.rated_power_w / motor.peak_torque_nm).is_finite()
        {
            return Err(LandingGearError::InvalidSpec(
                "wheel motor torque/power envelope overflows".into(),
            ));
        }
        Ok(CompiledWheelDrive {
            motor,
            stall_copper_loss_w: self.stall_copper_loss_w,
            rotor_inertia_kg_m2: self.rotor_inertia_kg_m2,
            final_drive_ratio: self.final_drive_ratio,
            drivetrain_efficiency: self.drivetrain_efficiency,
            driven_wheel_count: self.driven_wheel_count,
        })
    }
}

/// Compiled traction-motor curve with nonzero torque at wheel standstill.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledWheelDrive {
    motor: crate::CompiledElectricMotor,
    pub stall_copper_loss_w: f64,
    pub rotor_inertia_kg_m2: f64,
    pub final_drive_ratio: f64,
    pub drivetrain_efficiency: f64,
    pub driven_wheel_count: u16,
}

impl CompiledWheelDrive {
    /// Evaluate motor torque at signed wheel speed and a signed command in
    /// `[-1, 1]`. Positive electrical power is drawn from the bus; negative
    /// electrical power is available for regenerative recovery.
    pub fn operating_point(
        self,
        wheel_rpm: f64,
        command: f64,
    ) -> Result<WheelDrivePoint, LandingGearError> {
        if !wheel_rpm.is_finite() || !command.is_finite() || !(-1.0..=1.0).contains(&command) {
            return Err(LandingGearError::InvalidCommand(
                "wheel speed must be finite and motor command in [-1, 1]".into(),
            ));
        }
        let motor_rpm = wheel_rpm * self.final_drive_ratio;
        if !motor_rpm.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "wheel motor speed overflowed".into(),
            ));
        }
        let motor_omega_rad_s = motor_rpm * std::f64::consts::TAU / 60.0;
        if command == 0.0 {
            return Ok(WheelDrivePoint::zero(motor_rpm));
        }
        if motor_rpm.abs() > self.motor.maximum_rpm {
            return Ok(WheelDrivePoint {
                motor_rpm,
                motor_torque_nm: 0.0,
                requested_wheel_torque_per_driven_wheel_nm: 0.0,
                mechanical_power_w: 0.0,
                electrical_power_w: 0.0,
                waste_heat_w: 0.0,
                power_limited: false,
                thermal_limited: false,
                speed_limited: true,
            });
        }

        let speed = motor_omega_rad_s.abs();
        let peak_torque_nm = self.motor.peak_torque_nm;
        let power_envelope_torque_nm = if speed > 0.0 {
            (self.motor.rated_power_w / speed).min(peak_torque_nm)
        } else {
            peak_torque_nm
        };
        let power_limited = power_envelope_torque_nm < peak_torque_nm;
        let requested_torque_nm = command.abs() * power_envelope_torque_nm;
        let motoring = command * motor_omega_rad_s >= 0.0;
        let conversion_loss_per_w = if motoring {
            1.0 / self.motor.efficiency - 1.0
        } else {
            1.0 - self.motor.efficiency
        };
        let thermal_fraction = self.thermal_torque_fraction(speed, conversion_loss_per_w);
        let thermal_limit_nm = peak_torque_nm * thermal_fraction;
        let motor_torque_magnitude_nm = requested_torque_nm.min(thermal_limit_nm);
        let thermal_limited = thermal_limit_nm < requested_torque_nm;
        let motor_torque_nm = motor_torque_magnitude_nm * command.signum();
        let mechanical_power_w = motor_torque_nm * motor_omega_rad_s;
        let conversion_loss_w = mechanical_power_w.abs() * conversion_loss_per_w;
        let copper_loss_w =
            self.stall_copper_loss_w * (motor_torque_magnitude_nm / peak_torque_nm).powi(2);
        let waste_heat_w = conversion_loss_w + copper_loss_w;
        let electrical_power_w = mechanical_power_w + waste_heat_w;
        let requested_wheel_torque_per_driven_wheel_nm =
            motor_torque_nm * self.final_drive_ratio * self.drivetrain_efficiency
                / f64::from(self.driven_wheel_count);
        if !mechanical_power_w.is_finite()
            || !electrical_power_w.is_finite()
            || !waste_heat_w.is_finite()
            || !requested_wheel_torque_per_driven_wheel_nm.is_finite()
        {
            return Err(LandingGearError::InvalidCommand(
                "wheel motor operating point overflowed".into(),
            ));
        }
        Ok(WheelDrivePoint {
            motor_rpm,
            motor_torque_nm,
            requested_wheel_torque_per_driven_wheel_nm,
            mechanical_power_w,
            electrical_power_w,
            waste_heat_w,
            power_limited,
            thermal_limited,
            speed_limited: false,
        })
    }

    fn thermal_torque_fraction(self, motor_speed_rad_s: f64, loss_factor: f64) -> f64 {
        let cooling_w = self.motor.cooling_capacity_w;
        let stall_loss_w = self.stall_copper_loss_w;
        let rated_mechanical_w_per_fraction =
            self.motor.peak_torque_nm * motor_speed_rad_s * loss_factor;
        if stall_loss_w == 0.0 {
            return if rated_mechanical_w_per_fraction > 0.0 {
                (cooling_w / rated_mechanical_w_per_fraction).clamp(0.0, 1.0)
            } else {
                1.0
            };
        }
        if cooling_w == 0.0 {
            return 0.0;
        }
        let root = rated_mechanical_w_per_fraction.hypot(2.0 * (stall_loss_w * cooling_w).sqrt());
        let denominator = root + rated_mechanical_w_per_fraction;
        if denominator == 0.0 {
            1.0
        } else {
            (2.0 * cooling_w / denominator).clamp(0.0, 1.0)
        }
    }
}

/// Motor, reduction and per-driven-wheel response for one fixed step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelDrivePoint {
    pub motor_rpm: f64,
    pub motor_torque_nm: f64,
    /// Requested output torque before per-wheel tire-grip limiting.
    pub requested_wheel_torque_per_driven_wheel_nm: f64,
    /// Signed mechanical output; negative means the motor is being back-driven.
    pub mechanical_power_w: f64,
    /// Signed bus power; negative means regenerative power is available.
    pub electrical_power_w: f64,
    pub waste_heat_w: f64,
    pub power_limited: bool,
    pub thermal_limited: bool,
    pub speed_limited: bool,
}

impl WheelDrivePoint {
    fn zero(motor_rpm: f64) -> Self {
        Self {
            motor_rpm,
            motor_torque_nm: 0.0,
            requested_wheel_torque_per_driven_wheel_nm: 0.0,
            mechanical_power_w: 0.0,
            electrical_power_w: 0.0,
            waste_heat_w: 0.0,
            power_limited: false,
            thermal_limited: false,
            speed_limited: false,
        }
    }

    /// Compare requested hub torque with the maximum torque transferable
    /// through the current tire contact. This reports a grip exceedance; it
    /// does not silently clamp motor torque or act as traction control.
    pub fn contact_torque_limit(
        self,
        wheel_radius_m: f64,
        normal_load_n: f64,
        contact_friction: f64,
    ) -> Result<WheelDriveTractionPoint, LandingGearError> {
        require_positive(wheel_radius_m, "wheel radius")?;
        require_non_negative(normal_load_n, "wheel normal load")?;
        require_non_negative(contact_friction, "contact friction")?;
        if !self.requested_wheel_torque_per_driven_wheel_nm.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "requested wheel torque must be finite".into(),
            ));
        }
        let maximum_torque_nm = contact_friction * normal_load_n * wheel_radius_m;
        if !maximum_torque_nm.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "wheel traction torque limit overflowed".into(),
            ));
        }
        Ok(WheelDriveTractionPoint {
            requested_torque_nm: self.requested_wheel_torque_per_driven_wheel_nm,
            maximum_torque_nm,
            would_exceed_grip: self.requested_wheel_torque_per_driven_wheel_nm.abs()
                > maximum_torque_nm,
        })
    }
}

/// Comparison of per-wheel motor torque with the available tire-ground grip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelDriveTractionPoint {
    pub requested_torque_nm: f64,
    /// Maximum no-slip torque transferable to the ground at this wheel.
    pub maximum_torque_nm: f64,
    pub would_exceed_grip: bool,
}

/// Parametric wheel-bearing assembly installed on a vehicle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WheelChassisSpec {
    pub name: String,
    pub mount_position_body_m: DVec3,
    pub mount_orientation_body: DQuat,
    /// Longitudinal span for tandem wheels or axle stations (m).
    pub length_m: f64,
    pub layout: WheelLayout,
    pub wheel_count: u16,
    pub structural_mass_kg: f64,
    /// Intrinsic chassis/frame inertia about the mount, in chassis-local axes.
    pub structural_inertia_local_kg_m2: DMat3,
    pub tire: WheelTireSpec,
    pub strut: WheelStrutSpec,
    pub brake: WheelBrakeSpec,
    /// No drive is a valid configuration for free-rolling or aircraft gear.
    pub drive: Option<WheelDriveSpec>,
    /// Optional fold actuator for aircraft-style retractable wheel gear.
    /// When absent, the chassis stays at its authored mount pose.
    #[serde(default)]
    pub retraction: Option<WheelChassisRetractionSpec>,
}

/// Hinge geometry and actuator limits for a retractable wheel chassis.
/// The chassis mount pose in [`WheelChassisSpec`] is its fully deployed pose.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelChassisRetractionSpec {
    pub pivot_position_body_m: DVec3,
    pub hinge_axis_body: DVec3,
    pub stowed_angle_rad: f64,
    pub deployed_angle_rad: f64,
    pub initially_deployed: bool,
    pub deployment_rate_rad_s: f64,
    pub actuator_max_torque_nm: f64,
}

impl WheelChassisRetractionSpec {
    pub fn validate(self) -> Result<(), LandingGearError> {
        if !self.pivot_position_body_m.is_finite()
            || !self.hinge_axis_body.is_finite()
            || (self.hinge_axis_body.length_squared() - 1.0).abs() > UNIT_QUATERNION_TOLERANCE
        {
            return Err(LandingGearError::InvalidSpec(
                "wheel retraction pivot and unit hinge axis must be finite".into(),
            ));
        }
        if !self.stowed_angle_rad.is_finite()
            || !self.deployed_angle_rad.is_finite()
            || (self.deployed_angle_rad - self.stowed_angle_rad).abs() <= 1.0e-6
            || (self.deployed_angle_rad - self.stowed_angle_rad).abs() > std::f64::consts::TAU
        {
            return Err(LandingGearError::InvalidSpec(
                "wheel retraction angles must be finite and distinct within one turn".into(),
            ));
        }
        require_positive(self.deployment_rate_rad_s, "wheel gear deployment rate")?;
        require_positive(self.actuator_max_torque_nm, "wheel gear actuator torque")
    }

    pub fn initial_state(self) -> WheelChassisState {
        WheelChassisState {
            deployment_fraction: if self.initially_deployed { 1.0 } else { 0.0 },
            actuator_stalled: false,
        }
    }

    /// Rotation that carries the authored deployed chassis pose to the
    /// requested point between stowed (0) and deployed (1).
    pub fn rotation_at(self, deployment_fraction: f64) -> DQuat {
        let fraction = deployment_fraction.clamp(0.0, 1.0);
        let angle =
            self.stowed_angle_rad + (self.deployed_angle_rad - self.stowed_angle_rad) * fraction;
        DQuat::from_axis_angle(self.hinge_axis_body, angle - self.deployed_angle_rad)
    }

    pub fn advance_deployment(
        self,
        state: WheelChassisState,
        deployed: bool,
        dt_s: f64,
        resisting_torque_nm: f64,
    ) -> Result<(WheelChassisState, WheelChassisActuatorPoint), LandingGearError> {
        self.validate()?;
        if !state.deployment_fraction.is_finite()
            || !(0.0..=1.0).contains(&state.deployment_fraction)
            || !dt_s.is_finite()
            || dt_s < 0.0
            || !resisting_torque_nm.is_finite()
            || resisting_torque_nm < 0.0
        {
            return Err(LandingGearError::InvalidCommand(
                "wheel gear state, interval and resisting torque must be finite and in range"
                    .into(),
            ));
        }
        let target_fraction = if deployed { 1.0 } else { 0.0 };
        let remaining_fraction = target_fraction - state.deployment_fraction;
        let moving_to_target = remaining_fraction.abs() > 1.0e-12;
        let stalled = moving_to_target && resisting_torque_nm >= self.actuator_max_torque_nm;
        let mut next = state;
        if !stalled {
            let travel_rad = (self.deployed_angle_rad - self.stowed_angle_rad).abs();
            let load_factor =
                (1.0 - resisting_torque_nm / self.actuator_max_torque_nm).clamp(0.0, 1.0);
            let fraction_step = self.deployment_rate_rad_s * load_factor * dt_s / travel_rad;
            next.deployment_fraction = if remaining_fraction >= 0.0 {
                (state.deployment_fraction + fraction_step).min(target_fraction)
            } else {
                (state.deployment_fraction - fraction_step).max(target_fraction)
            };
        }
        next.actuator_stalled = stalled;
        Ok((
            next,
            WheelChassisActuatorPoint {
                target_fraction,
                deployment_fraction: next.deployment_fraction,
                resisting_torque_nm,
                actuator_torque_nm: if moving_to_target {
                    resisting_torque_nm.min(self.actuator_max_torque_nm)
                } else {
                    0.0
                },
                stalled,
                moving: (target_fraction - next.deployment_fraction).abs() > 1.0e-12,
            },
        ))
    }
}

/// Persistent position and stall state for one retractable wheel chassis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelChassisState {
    /// 0 = stowed, 1 = fully deployed.
    pub deployment_fraction: f64,
    pub actuator_stalled: bool,
}

impl WheelChassisState {
    pub const fn deployed() -> Self {
        Self {
            deployment_fraction: 1.0,
            actuator_stalled: false,
        }
    }
}

/// Measured gear load and achieved state of one fold actuator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelChassisActuatorPoint {
    pub target_fraction: f64,
    pub deployment_fraction: f64,
    pub resisting_torque_nm: f64,
    pub actuator_torque_nm: f64,
    pub stalled: bool,
    pub moving: bool,
}

impl WheelChassisSpec {
    pub fn compile(self) -> Result<CompiledWheelChassis, LandingGearError> {
        if self.name.trim().is_empty() {
            return Err(LandingGearError::InvalidSpec(
                "wheel chassis needs a non-empty name".into(),
            ));
        }
        if !self.mount_position_body_m.is_finite()
            || !self.mount_orientation_body.is_finite()
            || (self.mount_orientation_body.length_squared() - 1.0).abs()
                > UNIT_QUATERNION_TOLERANCE
        {
            return Err(LandingGearError::InvalidSpec(
                "wheel chassis mount pose must be finite with a unit quaternion".into(),
            ));
        }
        require_positive(self.length_m, "wheel chassis length")?;
        if self.wheel_count == 0 || self.wheel_count > MAX_WHEELS_PER_CHASSIS {
            return Err(LandingGearError::InvalidSpec(format!(
                "wheel count must be in [1, {MAX_WHEELS_PER_CHASSIS}]"
            )));
        }
        self.tire.validate()?;
        match self.layout {
            WheelLayout::Inline => {
                if self.wheel_count > 1
                    && self.length_m / f64::from(self.wheel_count - 1) < 2.0 * self.tire.radius_m
                {
                    return Err(LandingGearError::InvalidSpec(
                        "inline wheel stations overlap along the chassis length".into(),
                    ));
                }
            }
            WheelLayout::AxlePairs { track_width_m } => {
                require_positive(track_width_m, "wheel track width")?;
                if !self.wheel_count.is_multiple_of(2) {
                    return Err(LandingGearError::InvalidSpec(
                        "axle-pair layout requires an even wheel count".into(),
                    ));
                }
                if track_width_m < self.tire.width_m {
                    return Err(LandingGearError::InvalidSpec(
                        "paired wheel centers overlap across the track width".into(),
                    ));
                }
                let axle_count = self.wheel_count / 2;
                if axle_count > 1
                    && self.length_m / f64::from(axle_count - 1) < 2.0 * self.tire.radius_m
                {
                    return Err(LandingGearError::InvalidSpec(
                        "axle stations overlap along the chassis length".into(),
                    ));
                }
            }
        }
        require_positive(self.structural_mass_kg, "wheel chassis structural mass")?;
        validate_inertia_matrix(self.structural_inertia_local_kg_m2)?;
        self.strut.validate()?;
        self.brake.validate()?;
        if let Some(retraction) = self.retraction {
            retraction.validate()?;
        }
        let drive = self
            .drive
            .map(|drive| drive.compile(self.wheel_count))
            .transpose()?;
        let wheel_stations = compile_wheel_stations(&self);
        if wheel_stations.iter().any(|station| {
            !station.position_body_m.is_finite() || !station.axle_axis_body.is_finite()
        }) {
            return Err(LandingGearError::InvalidSpec(
                "compiled wheel-station geometry overflowed".into(),
            ));
        }
        let mass_properties = chassis_mass_properties(&self, &wheel_stations, drive)?;
        Ok(CompiledWheelChassis {
            spec: self,
            wheel_stations,
            drive,
            mass_properties,
        })
    }

    /// Body-frame mount pose at the requested gear position.
    pub fn mount_pose_at_fraction(&self, deployment_fraction: f64) -> (DVec3, DQuat) {
        let Some(retraction) = self.retraction else {
            return (self.mount_position_body_m, self.mount_orientation_body);
        };
        let rotation = retraction.rotation_at(deployment_fraction);
        (
            retraction.pivot_position_body_m
                + rotation * (self.mount_position_body_m - retraction.pivot_position_body_m),
            (rotation * self.mount_orientation_body).normalize(),
        )
    }
}

/// One deterministically positioned wheel station in vehicle body axes.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelStation {
    pub index: u16,
    pub position_body_m: DVec3,
    /// Wheel center relative to the chassis mount, expressed in body axes.
    pub mount_relative_position_body_m: DVec3,
    /// Unit spin axis in vehicle body coordinates.
    pub axle_axis_body: DVec3,
}

/// Compiled wheel assembly and its derived physical contributions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledWheelChassis {
    pub spec: WheelChassisSpec,
    pub wheel_stations: Vec<WheelStation>,
    pub drive: Option<CompiledWheelDrive>,
    pub mass_properties: WheelChassisMassProperties,
}

impl CompiledWheelChassis {
    /// Resolve a runtime chassis pose without changing its authored deployed
    /// mass bake. Unsprung wheel positions and axes follow the fold hinge.
    pub fn at_deployment_fraction(&self, deployment_fraction: f64) -> Self {
        let mut resolved = self.clone();
        let Some(retraction) = self.spec.retraction else {
            return resolved;
        };
        let rotation = retraction.rotation_at(deployment_fraction);
        let (mount_position_body_m, mount_orientation_body) =
            self.spec.mount_pose_at_fraction(deployment_fraction);
        resolved.spec.mount_position_body_m = mount_position_body_m;
        resolved.spec.mount_orientation_body = mount_orientation_body;
        for station in &mut resolved.wheel_stations {
            station.position_body_m = retraction.pivot_position_body_m
                + rotation * (station.position_body_m - retraction.pivot_position_body_m);
            station.mount_relative_position_body_m =
                rotation * station.mount_relative_position_body_m;
            station.axle_axis_body = (rotation * station.axle_axis_body).normalize();
        }
        resolved
    }

    pub fn dry_mass_kg(&self) -> f64 {
        self.mass_properties.mass_kg
    }

    /// Compare cached compiled values with a fresh bake while tolerating the
    /// last-bit decimal round-trip introduced by JSON vehicle assets.
    pub(crate) fn matches_recompiled(&self, expected: &Self) -> bool {
        self.spec == expected.spec
            && self.wheel_stations.len() == expected.wheel_stations.len()
            && self
                .wheel_stations
                .iter()
                .zip(&expected.wheel_stations)
                .all(|(actual, expected)| {
                    actual.index == expected.index
                        && close_vec3(actual.position_body_m, expected.position_body_m)
                        && close_vec3(
                            actual.mount_relative_position_body_m,
                            expected.mount_relative_position_body_m,
                        )
                        && close_vec3(actual.axle_axis_body, expected.axle_axis_body)
                })
            && close_scalar(
                self.mass_properties.mass_kg,
                expected.mass_properties.mass_kg,
            )
            && close_vec3(
                self.mass_properties.center_of_mass_body_m,
                expected.mass_properties.center_of_mass_body_m,
            )
            && close_mat3(
                self.mass_properties.inertia_body_kg_m2,
                expected.mass_properties.inertia_body_kg_m2,
            )
            && self.drive == expected.drive
    }

    /// Rigid-body mass properties for the unsprung tire/brake body at each
    /// wheel station. Strut mass stays with the sprung assembly in this
    /// reduced mass split; it remains represented in the chassis aggregate.
    pub fn wheel_body_mass_properties(&self, chassis_index: usize) -> Vec<WheelBodyMassProperties> {
        self.wheel_stations
            .iter()
            .map(|station| {
                let mass_kg = self.spec.tire.mass_kg + self.spec.brake.mass_per_wheel_kg;
                let transverse_inertia = self.spec.tire.mass_kg
                    * (3.0 * self.spec.tire.radius_m.powi(2) + self.spec.tire.width_m.powi(2))
                    / 12.0;
                let axle_outer = outer_product(station.axle_axis_body);
                let inertia_body_kg_m2 = DMat3::IDENTITY * transverse_inertia
                    + axle_outer * (self.spec.tire.spin_inertia_kg_m2 - transverse_inertia);
                let reflected_drive_inertia = self
                    .drive
                    .filter(|drive| station.index < drive.driven_wheel_count)
                    .map(|drive| {
                        drive.rotor_inertia_kg_m2 * drive.final_drive_ratio.powi(2)
                            / f64::from(drive.driven_wheel_count)
                    })
                    .unwrap_or(0.0);
                WheelBodyMassProperties {
                    chassis_index,
                    wheel_index: station.index,
                    mass_kg,
                    center_of_mass_body_m: station.position_body_m,
                    inertia_body_kg_m2: inertia_body_kg_m2 + axle_outer * reflected_drive_inertia,
                    axle_axis_body: station.axle_axis_body,
                }
            })
            .collect()
    }
}

/// Authored/derived inertia data for one unsprung wheel body.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelBodyMassProperties {
    pub chassis_index: usize,
    pub wheel_index: u16,
    pub mass_kg: f64,
    pub center_of_mass_body_m: DVec3,
    pub inertia_body_kg_m2: DMat3,
    pub axle_axis_body: DVec3,
}

fn close_scalar(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= 1.0e-12 * actual.abs().max(expected.abs()).max(1.0)
}

fn close_vec3(actual: DVec3, expected: DVec3) -> bool {
    close_scalar(actual.x, expected.x)
        && close_scalar(actual.y, expected.y)
        && close_scalar(actual.z, expected.z)
}

fn close_mat3(actual: DMat3, expected: DMat3) -> bool {
    actual
        .to_cols_array()
        .into_iter()
        .zip(expected.to_cols_array())
        .all(|(actual, expected)| close_scalar(actual, expected))
}

/// Dry mass and full inertia contribution of one compiled wheel chassis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WheelChassisMassProperties {
    pub mass_kg: f64,
    pub center_of_mass_body_m: DVec3,
    pub inertia_body_kg_m2: DMat3,
}

fn compile_wheel_stations(spec: &WheelChassisSpec) -> Vec<WheelStation> {
    let orientation = spec.mount_orientation_body;
    let axle_axis_body = (orientation * DVec3::Y).normalize();
    let mut offsets = Vec::with_capacity(usize::from(spec.wheel_count));
    match spec.layout {
        WheelLayout::Inline => {
            for index in 0..spec.wheel_count {
                let x = evenly_spaced(index, spec.wheel_count, spec.length_m);
                offsets.push(DVec3::new(x, 0.0, -spec.strut.extended_length_m));
            }
        }
        WheelLayout::AxlePairs { track_width_m } => {
            let axle_count = spec.wheel_count / 2;
            for axle in 0..axle_count {
                let x = evenly_spaced(axle, axle_count, spec.length_m);
                offsets.push(DVec3::new(
                    x,
                    -0.5 * track_width_m,
                    -spec.strut.extended_length_m,
                ));
                offsets.push(DVec3::new(
                    x,
                    0.5 * track_width_m,
                    -spec.strut.extended_length_m,
                ));
            }
        }
    }
    offsets
        .into_iter()
        .enumerate()
        .map(|(index, local_offset)| {
            let mount_relative_position_body_m = orientation * local_offset;
            WheelStation {
                index: index as u16,
                position_body_m: spec.mount_position_body_m + mount_relative_position_body_m,
                mount_relative_position_body_m,
                axle_axis_body,
            }
        })
        .collect()
}

fn evenly_spaced(index: u16, count: u16, span_m: f64) -> f64 {
    if count <= 1 {
        0.0
    } else {
        -0.5 * span_m + f64::from(index) * span_m / f64::from(count - 1)
    }
}

fn chassis_mass_properties(
    spec: &WheelChassisSpec,
    stations: &[WheelStation],
    drive: Option<CompiledWheelDrive>,
) -> Result<WheelChassisMassProperties, LandingGearError> {
    let rotation = DMat3::from_quat(spec.mount_orientation_body);
    let mut mass_kg = spec.structural_mass_kg;
    let mut first_moment = DVec3::ZERO;
    let mut inertia_about_mount =
        rotation * spec.structural_inertia_local_kg_m2 * rotation.transpose();
    if let Some(drive) = drive {
        mass_kg += drive.motor.dry_mass_kg;
    }
    for station in stations {
        let wheel_position = station.mount_relative_position_body_m;
        let strut_position = wheel_position * 0.5;
        let wheel_and_brake_mass = spec.tire.mass_kg + spec.brake.mass_per_wheel_kg;
        mass_kg += wheel_and_brake_mass + spec.strut.mass_per_wheel_kg;
        first_moment +=
            wheel_and_brake_mass * wheel_position + spec.strut.mass_per_wheel_kg * strut_position;
        inertia_about_mount += point_mass_inertia(wheel_and_brake_mass, wheel_position)
            + wheel_intrinsic_inertia(&spec.tire, station.axle_axis_body)
            + point_mass_inertia(spec.strut.mass_per_wheel_kg, strut_position);
        if let Some(drive) = drive.filter(|drive| station.index < drive.driven_wheel_count) {
            let reflected_inertia = drive.rotor_inertia_kg_m2 * drive.final_drive_ratio.powi(2)
                / f64::from(drive.driven_wheel_count);
            inertia_about_mount += reflected_inertia * outer_product(station.axle_axis_body);
        }
    }
    if !mass_kg.is_finite() || mass_kg <= 0.0 || !first_moment.is_finite() {
        return Err(LandingGearError::InvalidSpec(
            "wheel chassis mass aggregation overflowed".into(),
        ));
    }
    let center_relative_to_mount = first_moment / mass_kg;
    let inertia_at_center =
        inertia_about_mount - point_mass_inertia(mass_kg, center_relative_to_mount);
    let center_of_mass_body_m = spec.mount_position_body_m + center_relative_to_mount;
    if !inertia_at_center.is_finite() || !center_of_mass_body_m.is_finite() {
        return Err(LandingGearError::InvalidSpec(
            "wheel chassis inertia aggregation overflowed".into(),
        ));
    }
    validate_inertia_matrix(inertia_at_center)?;
    Ok(WheelChassisMassProperties {
        mass_kg,
        center_of_mass_body_m,
        inertia_body_kg_m2: inertia_at_center,
    })
}

fn wheel_intrinsic_inertia(tire: &WheelTireSpec, axle_axis_body: DVec3) -> DMat3 {
    let transverse = tire.mass_kg * (3.0 * tire.radius_m.powi(2) + tire.width_m.powi(2)) / 12.0;
    DMat3::IDENTITY * transverse
        + outer_product(axle_axis_body) * (tire.spin_inertia_kg_m2 - transverse)
}

fn point_mass_inertia(mass_kg: f64, position_m: DVec3) -> DMat3 {
    mass_kg * (DMat3::IDENTITY * position_m.length_squared() - outer_product(position_m))
}

fn outer_product(vector: DVec3) -> DMat3 {
    DMat3::from_cols(vector * vector.x, vector * vector.y, vector * vector.z)
}

fn validate_inertia_matrix(inertia: DMat3) -> Result<(), LandingGearError> {
    if !inertia.is_finite() {
        return Err(LandingGearError::InvalidSpec(
            "wheel chassis structural inertia must be finite".into(),
        ));
    }
    let scale = inertia
        .to_cols_array()
        .into_iter()
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    let tolerance = SYMMETRY_TOLERANCE * scale;
    if (inertia.x_axis.y - inertia.y_axis.x).abs() > tolerance
        || (inertia.x_axis.z - inertia.z_axis.x).abs() > tolerance
        || (inertia.y_axis.z - inertia.z_axis.y).abs() > tolerance
    {
        return Err(LandingGearError::InvalidSpec(
            "wheel chassis structural inertia must be symmetric".into(),
        ));
    }
    let normalized = if scale > 0.0 {
        inertia / scale
    } else {
        inertia
    };
    let xx = normalized.x_axis.x;
    let yy = normalized.y_axis.y;
    let zz = normalized.z_axis.z;
    let xy = 0.5 * (normalized.x_axis.y + normalized.y_axis.x);
    let xz = 0.5 * (normalized.x_axis.z + normalized.z_axis.x);
    let yz = 0.5 * (normalized.y_axis.z + normalized.z_axis.y);
    let minors = [
        xx,
        yy,
        zz,
        xx * yy - xy * xy,
        xx * zz - xz * xz,
        yy * zz - yz * yz,
        normalized.determinant(),
    ];
    if minors.into_iter().any(|minor| minor < -SYMMETRY_TOLERANCE) {
        return Err(LandingGearError::InvalidSpec(
            "wheel chassis structural inertia must be positive semidefinite".into(),
        ));
    }
    Ok(())
}

/// Maximum independent fold-out supports compiled for one vehicle.
pub const MAX_LANDING_LEGS: usize = 16;

/// Shock-absorber architecture for a fold-out landing leg.
///
/// `Reusable` represents a spring/hydraulic-damper unit that returns its
/// stroke after unloading. `Crushable` represents a one-shot cellular or
/// honeycomb cartridge: plastic crush is retained in [`LandingLegState`] and
/// its spent length is removed from the leg on subsequent ticks.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum LandingShockAbsorberSpec {
    Reusable {
        stroke_m: f64,
        spring_rate_n_m: f64,
        damping_n_s_m: f64,
        preload_n: f64,
        bottom_out_stiffness_n_m: f64,
        maximum_force_n: f64,
    },
    Crushable {
        elastic_stiffness_n_m: f64,
        damping_n_s_m: f64,
        plateau_force_n: f64,
        maximum_crush_m: f64,
        bottom_out_stiffness_n_m: f64,
        maximum_force_n: f64,
    },
}

impl LandingShockAbsorberSpec {
    pub fn validate(self) -> Result<(), LandingGearError> {
        match self {
            Self::Reusable {
                stroke_m,
                spring_rate_n_m,
                damping_n_s_m,
                preload_n,
                bottom_out_stiffness_n_m,
                maximum_force_n,
            } => {
                require_positive(stroke_m, "reusable shock stroke")?;
                require_positive(spring_rate_n_m, "reusable shock spring rate")?;
                require_non_negative(damping_n_s_m, "reusable shock damping")?;
                require_non_negative(preload_n, "reusable shock preload")?;
                require_positive(bottom_out_stiffness_n_m, "reusable bottom-out stiffness")?;
                require_positive(maximum_force_n, "reusable shock maximum force")?;
                if preload_n > maximum_force_n {
                    return Err(LandingGearError::InvalidSpec(
                        "reusable shock preload exceeds its maximum force".into(),
                    ));
                }
            }
            Self::Crushable {
                elastic_stiffness_n_m,
                damping_n_s_m,
                plateau_force_n,
                maximum_crush_m,
                bottom_out_stiffness_n_m,
                maximum_force_n,
            } => {
                require_positive(elastic_stiffness_n_m, "crushable elastic stiffness")?;
                require_non_negative(damping_n_s_m, "crushable shock damping")?;
                require_positive(plateau_force_n, "crush plateau force")?;
                require_positive(maximum_crush_m, "maximum permanent crush")?;
                require_positive(bottom_out_stiffness_n_m, "crushable bottom-out stiffness")?;
                require_positive(maximum_force_n, "crushable maximum force")?;
                if plateau_force_n > maximum_force_n {
                    return Err(LandingGearError::InvalidSpec(
                        "crush plateau exceeds the shock maximum force".into(),
                    ));
                }
            }
        }
        Ok(())
    }

    /// Maximum recoverable or crush travel in the axial load path.
    pub fn usable_stroke_m(self) -> f64 {
        match self {
            Self::Reusable { stroke_m, .. } => stroke_m,
            Self::Crushable {
                elastic_stiffness_n_m,
                plateau_force_n,
                maximum_crush_m,
                ..
            } => plateau_force_n / elastic_stiffness_n_m + maximum_crush_m,
        }
    }

    pub fn maximum_permanent_crush_m(self) -> f64 {
        match self {
            Self::Reusable { .. } => 0.0,
            Self::Crushable {
                maximum_crush_m, ..
            } => maximum_crush_m,
        }
    }

    fn force_limit_n(self) -> f64 {
        match self {
            Self::Reusable {
                maximum_force_n, ..
            }
            | Self::Crushable {
                maximum_force_n, ..
            } => maximum_force_n,
        }
    }

    fn bottom_out_stiffness_n_m(self) -> f64 {
        match self {
            Self::Reusable {
                bottom_out_stiffness_n_m,
                ..
            }
            | Self::Crushable {
                bottom_out_stiffness_n_m,
                ..
            } => bottom_out_stiffness_n_m,
        }
    }

    /// Evaluate a shock's axial force and advance its irreversible energy
    /// state. Positive compression rate is an impact/compression event.
    pub fn evaluate(
        self,
        state: LandingLegState,
        compression_m: f64,
        compression_rate_mps: f64,
        dt_s: f64,
    ) -> Result<(LandingShockPoint, LandingLegState), LandingGearError> {
        self.validate()?;
        if !compression_m.is_finite()
            || compression_m < 0.0
            || !compression_rate_mps.is_finite()
            || !dt_s.is_finite()
            || dt_s < 0.0
            || !state.deployment_fraction.is_finite()
            || !(0.0..=1.0).contains(&state.deployment_fraction)
            || !state.permanent_crush_m.is_finite()
            || state.permanent_crush_m < 0.0
            || !state.absorbed_energy_j.is_finite()
            || state.absorbed_energy_j < 0.0
        {
            return Err(LandingGearError::InvalidCommand(
                "shock state and compression inputs must be finite and non-negative".into(),
            ));
        }
        match self {
            Self::Reusable { .. } if state.permanent_crush_m != 0.0 => {
                return Err(LandingGearError::InvalidCommand(
                    "reusable shock state cannot contain permanent crush".into(),
                ));
            }
            Self::Crushable {
                maximum_crush_m, ..
            } if state.permanent_crush_m > maximum_crush_m => {
                return Err(LandingGearError::InvalidCommand(
                    "permanent crush exceeds the shock cartridge capacity".into(),
                ));
            }
            _ => {}
        }

        let mut next = state;
        let (
            elastic_compression_m,
            overtravel_m,
            plastic_delta_m,
            spring_rate_n_m,
            viscous_damping_n_s_m,
            preload_n,
        ) = match self {
            Self::Reusable {
                stroke_m,
                spring_rate_n_m,
                damping_n_s_m,
                preload_n,
                ..
            } => {
                let elastic = compression_m.min(stroke_m);
                (
                    elastic,
                    (compression_m - stroke_m).max(0.0),
                    0.0,
                    spring_rate_n_m,
                    damping_n_s_m,
                    preload_n,
                )
            }
            Self::Crushable {
                elastic_stiffness_n_m,
                damping_n_s_m,
                plateau_force_n,
                maximum_crush_m,
                ..
            } => {
                let yield_compression_m = plateau_force_n / elastic_stiffness_n_m;
                let requested_crush_m = (compression_m - yield_compression_m)
                    .max(0.0)
                    .min(maximum_crush_m);
                let permanent_crush_m = state.permanent_crush_m.max(requested_crush_m);
                let plastic_delta_m = permanent_crush_m - state.permanent_crush_m;
                next.permanent_crush_m = permanent_crush_m;
                let recoverable_compression_m = (compression_m - permanent_crush_m).max(0.0);
                (
                    recoverable_compression_m.min(yield_compression_m),
                    (recoverable_compression_m - yield_compression_m).max(0.0),
                    plastic_delta_m,
                    elastic_stiffness_n_m,
                    damping_n_s_m,
                    0.0,
                )
            }
        };

        let spring_force_n = spring_rate_n_m * elastic_compression_m;
        let bottom_out_force_n = self.bottom_out_stiffness_n_m() * overtravel_m;
        let damping_force_n = viscous_damping_n_s_m * compression_rate_mps;
        let force_axial_n = (preload_n + spring_force_n + bottom_out_force_n + damping_force_n)
            .clamp(0.0, self.force_limit_n());
        if !force_axial_n.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "landing shock force overflowed".into(),
            ));
        }

        let plateau_force_n = match self {
            Self::Reusable { .. } => 0.0,
            Self::Crushable {
                plateau_force_n, ..
            } => plateau_force_n,
        };
        let plastic_energy_j = plateau_force_n * plastic_delta_m;
        let damping_energy_j =
            viscous_damping_n_s_m * compression_rate_mps * compression_rate_mps * dt_s;
        let absorbed_energy_delta_j = plastic_energy_j + damping_energy_j;
        next.absorbed_energy_j += absorbed_energy_delta_j;
        if !next.absorbed_energy_j.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "landing shock energy overflowed".into(),
            ));
        }
        let stored_energy_j = match self {
            Self::Reusable {
                spring_rate_n_m, ..
            } => {
                0.5 * spring_rate_n_m * elastic_compression_m.powi(2)
                    + 0.5 * self.bottom_out_stiffness_n_m() * overtravel_m.powi(2)
            }
            Self::Crushable {
                elastic_stiffness_n_m,
                ..
            } => {
                0.5 * elastic_stiffness_n_m * elastic_compression_m.powi(2)
                    + 0.5 * self.bottom_out_stiffness_n_m() * overtravel_m.powi(2)
            }
        };
        if !stored_energy_j.is_finite() || !absorbed_energy_delta_j.is_finite() {
            return Err(LandingGearError::InvalidCommand(
                "landing shock energy calculation overflowed".into(),
            ));
        }
        let exhausted = match self {
            Self::Reusable { stroke_m, .. } => compression_m >= stroke_m,
            Self::Crushable {
                maximum_crush_m, ..
            } => next.permanent_crush_m >= maximum_crush_m,
        };
        let point = LandingShockPoint {
            compression_m,
            compression_rate_mps,
            force_axial_n,
            permanent_crush_m: next.permanent_crush_m,
            plastic_crush_delta_m: plastic_delta_m,
            absorbed_energy_delta_j,
            stored_energy_j,
            exhausted,
            bottomed_out: overtravel_m > 0.0,
        };
        Ok((point, next))
    }
}

/// One authored fold-out support. Geometry and all axes are in vehicle body
/// coordinates; the mass model treats the leg as sprung mass at its deployed
/// pose while the fold actuator drives its explicit angular degree of freedom.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LandingLegSpec {
    pub name: String,
    pub mount_position_body_m: DVec3,
    /// Unit body-frame hinge axis.
    pub hinge_axis_body: DVec3,
    /// Unit leg direction at `stowed_angle_rad`.
    pub stowed_leg_axis_body: DVec3,
    pub stowed_angle_rad: f64,
    pub deployed_angle_rad: f64,
    pub initially_deployed: bool,
    pub deployment_rate_rad_s: f64,
    pub actuator_max_torque_nm: f64,
    /// Length of the structural leg at full extension (m).
    pub leg_length_m: f64,
    pub leg_mass_kg: f64,
    pub footpad_radius_m: f64,
    pub footpad_mass_kg: f64,
    pub footpad_friction: f64,
    pub footpad_slip_stiffness_n_per_mps: f64,
    pub shock_absorber: LandingShockAbsorberSpec,
}

impl LandingLegSpec {
    pub fn validate(&self) -> Result<(), LandingGearError> {
        if self.name.trim().is_empty() {
            return Err(LandingGearError::InvalidSpec(
                "landing leg needs a non-empty name".into(),
            ));
        }
        if !self.mount_position_body_m.is_finite()
            || !self.hinge_axis_body.is_finite()
            || !self.stowed_leg_axis_body.is_finite()
            || (self.hinge_axis_body.length_squared() - 1.0).abs() > UNIT_QUATERNION_TOLERANCE
            || (self.stowed_leg_axis_body.length_squared() - 1.0).abs() > UNIT_QUATERNION_TOLERANCE
            || self.hinge_axis_body.dot(self.stowed_leg_axis_body).abs() > UNIT_QUATERNION_TOLERANCE
        {
            return Err(LandingGearError::InvalidSpec(
                "landing leg mount and orthogonal hinge/leg axes must be finite and unit length"
                    .into(),
            ));
        }
        if !self.stowed_angle_rad.is_finite()
            || !self.deployed_angle_rad.is_finite()
            || (self.deployed_angle_rad - self.stowed_angle_rad).abs() <= 1.0e-6
            || (self.deployed_angle_rad - self.stowed_angle_rad).abs() > std::f64::consts::TAU
        {
            return Err(LandingGearError::InvalidSpec(
                "landing leg stowed/deployed angles must be finite and distinct within one turn"
                    .into(),
            ));
        }
        require_positive(self.deployment_rate_rad_s, "landing leg deployment rate")?;
        require_positive(self.actuator_max_torque_nm, "landing leg actuator torque")?;
        require_positive(self.leg_length_m, "landing leg length")?;
        require_positive(self.leg_mass_kg, "landing leg structural mass")?;
        require_positive(self.footpad_radius_m, "landing footpad radius")?;
        require_positive(self.footpad_mass_kg, "landing footpad mass")?;
        require_non_negative(self.footpad_friction, "landing footpad friction")?;
        require_positive(
            self.footpad_slip_stiffness_n_per_mps,
            "landing footpad slip stiffness",
        )?;
        self.shock_absorber.validate()?;
        if self.shock_absorber.usable_stroke_m() >= self.leg_length_m {
            return Err(LandingGearError::InvalidSpec(
                "landing shock stroke must be shorter than the structural leg".into(),
            ));
        }
        Ok(())
    }

    pub fn compile(self) -> Result<CompiledLandingLeg, LandingGearError> {
        self.validate()?;
        let deployed_axis_body = self.leg_axis_body_at_fraction(1.0);
        let leg_center =
            self.mount_position_body_m + deployed_axis_body * (0.5 * self.leg_length_m);
        let footpad_center = self.mount_position_body_m + deployed_axis_body * self.leg_length_m;
        let mass_kg = self.leg_mass_kg + self.footpad_mass_kg;
        let center_of_mass_body_m =
            (leg_center * self.leg_mass_kg + footpad_center * self.footpad_mass_kg) / mass_kg;
        let rod_inertia_at_center = (DMat3::IDENTITY - outer_product(deployed_axis_body))
            * (self.leg_mass_kg * self.leg_length_m.powi(2) / 12.0);
        let pad_inertia_at_center =
            DMat3::IDENTITY * (0.4 * self.footpad_mass_kg * self.footpad_radius_m.powi(2));
        let inertia_about_mount = rod_inertia_at_center
            + point_mass_inertia(self.leg_mass_kg, leg_center)
            + pad_inertia_at_center
            + point_mass_inertia(self.footpad_mass_kg, footpad_center);
        let inertia_body_kg_m2 =
            inertia_about_mount - point_mass_inertia(mass_kg, center_of_mass_body_m);
        validate_inertia_matrix(inertia_body_kg_m2)?;
        Ok(CompiledLandingLeg {
            mass_properties: LandingLegMassProperties {
                mass_kg,
                center_of_mass_body_m,
                inertia_body_kg_m2,
            },
            spec: self,
        })
    }

    /// Resolve the deployed or partially-folded body-frame leg direction.
    pub fn leg_axis_body_at_fraction(&self, deployment_fraction: f64) -> DVec3 {
        let fraction = deployment_fraction.clamp(0.0, 1.0);
        let angle =
            self.stowed_angle_rad + (self.deployed_angle_rad - self.stowed_angle_rad) * fraction;
        DQuat::from_axis_angle(self.hinge_axis_body, angle - self.stowed_angle_rad)
            * self.stowed_leg_axis_body
    }

    pub fn initial_state(&self) -> LandingLegState {
        LandingLegState {
            deployment_fraction: if self.initially_deployed { 1.0 } else { 0.0 },
            ..LandingLegState::default()
        }
    }

    /// Advance the fold actuator. `resisting_torque_nm` is the non-negative
    /// contact-load moment opposing the commanded angular direction.
    pub fn advance_deployment(
        &self,
        state: LandingLegState,
        deployed: bool,
        dt_s: f64,
        resisting_torque_nm: f64,
    ) -> Result<(LandingLegState, LandingGearActuatorPoint), LandingGearError> {
        self.validate()?;
        if !state.deployment_fraction.is_finite()
            || !(0.0..=1.0).contains(&state.deployment_fraction)
            || !state.permanent_crush_m.is_finite()
            || state.permanent_crush_m < 0.0
            || state.permanent_crush_m > self.shock_absorber.maximum_permanent_crush_m()
            || !state.absorbed_energy_j.is_finite()
            || state.absorbed_energy_j < 0.0
            || !dt_s.is_finite()
            || dt_s < 0.0
            || !resisting_torque_nm.is_finite()
            || resisting_torque_nm < 0.0
        {
            return Err(LandingGearError::InvalidCommand(
                "landing actuator state, interval and load torque must be finite and in range"
                    .into(),
            ));
        }
        let target_fraction = if deployed { 1.0 } else { 0.0 };
        let remaining_fraction = target_fraction - state.deployment_fraction;
        let moving_to_target = remaining_fraction.abs() > 1.0e-12;
        let stalled = moving_to_target && resisting_torque_nm >= self.actuator_max_torque_nm;
        let mut next = state;
        if !stalled {
            let full_travel_rad = (self.deployed_angle_rad - self.stowed_angle_rad).abs();
            let load_factor =
                (1.0 - resisting_torque_nm / self.actuator_max_torque_nm).clamp(0.0, 1.0);
            let fraction_step = self.deployment_rate_rad_s * load_factor * dt_s / full_travel_rad;
            next.deployment_fraction = if remaining_fraction >= 0.0 {
                (state.deployment_fraction + fraction_step).min(target_fraction)
            } else {
                (state.deployment_fraction - fraction_step).max(target_fraction)
            };
        }
        next.actuator_stalled = stalled;
        let actuator_torque_nm = if moving_to_target {
            resisting_torque_nm.min(self.actuator_max_torque_nm)
        } else {
            0.0
        };
        Ok((
            next,
            LandingGearActuatorPoint {
                target_fraction,
                deployment_fraction: next.deployment_fraction,
                resisting_torque_nm,
                actuator_torque_nm,
                stalled,
                moving: (target_fraction - next.deployment_fraction).abs() > 1.0e-12,
            },
        ))
    }
}

impl CompiledLandingLeg {
    pub fn matches_recompiled(&self, expected: &Self) -> bool {
        self.spec == expected.spec
            && close_scalar(
                self.mass_properties.mass_kg,
                expected.mass_properties.mass_kg,
            )
            && close_vec3(
                self.mass_properties.center_of_mass_body_m,
                expected.mass_properties.center_of_mass_body_m,
            )
            && close_mat3(
                self.mass_properties.inertia_body_kg_m2,
                expected.mass_properties.inertia_body_kg_m2,
            )
    }
}

/// Compiled one-leg geometry and its contribution to vehicle mass properties.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledLandingLeg {
    pub spec: LandingLegSpec,
    pub mass_properties: LandingLegMassProperties,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LandingLegMassProperties {
    pub mass_kg: f64,
    pub center_of_mass_body_m: DVec3,
    pub inertia_body_kg_m2: DMat3,
}

/// Persistent flight-authority state for one landing leg and its absorber.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LandingLegState {
    /// 0 = stowed, 1 = fully deployed.
    pub deployment_fraction: f64,
    /// Irreversible shortening for a crushable absorber (m).
    pub permanent_crush_m: f64,
    /// Cumulative plastic and damper energy absorbed (J).
    pub absorbed_energy_j: f64,
    pub actuator_stalled: bool,
}

impl Default for LandingLegState {
    fn default() -> Self {
        Self {
            deployment_fraction: 1.0,
            permanent_crush_m: 0.0,
            absorbed_energy_j: 0.0,
            actuator_stalled: false,
        }
    }
}

/// One shock response evaluated for a geometric landing-leg compression.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingShockPoint {
    pub compression_m: f64,
    pub compression_rate_mps: f64,
    pub force_axial_n: f64,
    pub permanent_crush_m: f64,
    pub plastic_crush_delta_m: f64,
    pub absorbed_energy_delta_j: f64,
    pub stored_energy_j: f64,
    pub exhausted: bool,
    pub bottomed_out: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingGearActuatorPoint {
    pub target_fraction: f64,
    pub deployment_fraction: f64,
    pub resisting_torque_nm: f64,
    pub actuator_torque_nm: f64,
    pub stalled: bool,
    pub moving: bool,
}

/// Validation/evaluation error for backend-neutral landing-gear data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LandingGearError {
    InvalidSpec(String),
    InvalidCommand(String),
}

impl fmt::Display for LandingGearError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpec(message) => write!(formatter, "invalid landing-gear spec: {message}"),
            Self::InvalidCommand(message) => {
                write!(formatter, "invalid landing-gear command: {message}")
            }
        }
    }
}

impl Error for LandingGearError {}

fn validate_finite(value: f64, label: &str) -> Result<(), LandingGearError> {
    if !value.is_finite() {
        return Err(LandingGearError::InvalidCommand(format!(
            "{label} must be finite"
        )));
    }
    Ok(())
}

fn require_positive(value: f64, label: &str) -> Result<(), LandingGearError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(LandingGearError::InvalidSpec(format!(
            "{label} must be positive and finite"
        )));
    }
    Ok(())
}

fn require_non_negative(value: f64, label: &str) -> Result<(), LandingGearError> {
    if !value.is_finite() || value < 0.0 {
        return Err(LandingGearError::InvalidSpec(format!(
            "{label} must be finite and non-negative"
        )));
    }
    Ok(())
}

fn require_unit_interval(value: f64, label: &str) -> Result<(), LandingGearError> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) || value == 0.0 {
        return Err(LandingGearError::InvalidSpec(format!(
            "{label} must be finite in (0, 1]"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pneumatic_tire() -> WheelTireSpec {
        WheelTireSpec {
            construction: TireConstruction::Pneumatic {
                inflation_pressure_pa: 220_000.0,
                reference_temperature_k: 293.15,
            },
            radius_m: 0.32,
            width_m: 0.18,
            mass_kg: 3.4,
            spin_inertia_kg_m2: 0.11,
            radial_stiffness_n_m: 200_000.0,
            radial_damping_n_s_m: 10_000.0,
            longitudinal_slip_stiffness_n_per_mps: 12_000.0,
            lateral_slip_stiffness_n_per_mps: 9_000.0,
            maximum_deflection_m: 0.08,
            maximum_load_n: 8_000.0,
            surface_friction: 0.9,
        }
    }

    fn airless_tire() -> WheelTireSpec {
        WheelTireSpec {
            construction: TireConstruction::Airless {
                structure: AirlessWheelStructure::Spoked { spoke_count: 24 },
                structure_density_kg_m3: 4_400.0,
                minimum_temperature_k: 80.0,
                maximum_temperature_k: 500.0,
            },
            ..pneumatic_tire()
        }
    }

    fn strut() -> WheelStrutSpec {
        WheelStrutSpec {
            extended_length_m: 0.42,
            stroke_m: 0.16,
            spring_rate_n_m: 31_000.0,
            damping_n_s_m: 2_800.0,
            preload_n: 80.0,
            minimum_force_n: 0.0,
            maximum_force_n: 12_000.0,
            mass_per_wheel_kg: 1.0,
        }
    }

    fn landing_leg(shock_absorber: LandingShockAbsorberSpec) -> LandingLegSpec {
        LandingLegSpec {
            name: "lander-leg".into(),
            mount_position_body_m: DVec3::new(0.5, 0.0, 0.0),
            hinge_axis_body: DVec3::Y,
            stowed_leg_axis_body: DVec3::Z,
            stowed_angle_rad: 0.0,
            deployed_angle_rad: std::f64::consts::PI,
            initially_deployed: false,
            deployment_rate_rad_s: 1.0,
            actuator_max_torque_nm: 12_000.0,
            leg_length_m: 2.0,
            leg_mass_kg: 18.0,
            footpad_radius_m: 0.2,
            footpad_mass_kg: 3.0,
            footpad_friction: 0.8,
            footpad_slip_stiffness_n_per_mps: 5_000.0,
            shock_absorber,
        }
    }

    fn chassis(layout: WheelLayout, wheel_count: u16) -> WheelChassisSpec {
        WheelChassisSpec {
            name: "test-bogie".into(),
            mount_position_body_m: DVec3::new(1.0, 2.0, 3.0),
            mount_orientation_body: DQuat::IDENTITY,
            length_m: 2.0,
            layout,
            wheel_count,
            structural_mass_kg: 12.0,
            structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0)),
            tire: pneumatic_tire(),
            strut: strut(),
            brake: WheelBrakeSpec {
                maximum_torque_nm: 500.0,
                response_time_s: 0.12,
                mass_per_wheel_kg: 0.5,
            },
            drive: None,
            retraction: None,
        }
    }

    fn wheel_drive(driven_wheel_count: u16) -> WheelDriveSpec {
        WheelDriveSpec {
            motor: ElectricMotorSpec {
                rated_power_w: 100_000.0,
                peak_torque_nm: 400.0,
                maximum_rpm: 12_000.0,
                efficiency: 0.94,
                cooling_capacity_w: 20_000.0,
                dry_mass_kg: 18.0,
            },
            stall_copper_loss_w: 1_000.0,
            rotor_inertia_kg_m2: 0.04,
            final_drive_ratio: 10.0,
            drivetrain_efficiency: 0.9,
            driven_wheel_count,
        }
    }

    #[test]
    fn wheel_layouts_are_symmetric_and_respect_chassis_length() {
        let inline = chassis(WheelLayout::Inline, 3)
            .compile()
            .expect("three-wheel bogie");
        assert_eq!(
            inline
                .wheel_stations
                .iter()
                .map(|station| station.position_body_m.x)
                .collect::<Vec<_>>(),
            vec![0.0, 1.0, 2.0]
        );
        assert!(
            inline
                .wheel_stations
                .iter()
                .all(|station| station.position_body_m.z == 2.58)
        );

        let pairs = chassis(WheelLayout::AxlePairs { track_width_m: 1.2 }, 4)
            .compile()
            .expect("two-axle rover bogie");
        assert_eq!(pairs.wheel_stations.len(), 4);
        assert_eq!(pairs.wheel_stations[0].position_body_m.y, 1.4);
        assert_eq!(pairs.wheel_stations[1].position_body_m.y, 2.6);
        assert_eq!(pairs.wheel_stations[2].position_body_m.x, 2.0);
        assert!(
            (pairs.wheel_stations[0].position_body_m.y + pairs.wheel_stations[1].position_body_m.y
                - 2.0 * chassis(WheelLayout::Inline, 1).mount_position_body_m.y)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn retractable_wheel_chassis_uses_authored_hinge_and_torque_limit() {
        let mut spec = chassis(WheelLayout::Inline, 1);
        spec.retraction = Some(WheelChassisRetractionSpec {
            pivot_position_body_m: DVec3::ZERO,
            hinge_axis_body: DVec3::Y,
            stowed_angle_rad: -std::f64::consts::FRAC_PI_2,
            deployed_angle_rad: 0.0,
            initially_deployed: true,
            deployment_rate_rad_s: 1.0,
            actuator_max_torque_nm: 1_000.0,
        });
        let compiled = spec.clone().compile().expect("retractable chassis");
        let deployed = compiled.at_deployment_fraction(1.0);
        assert_eq!(
            deployed.wheel_stations[0].position_body_m,
            compiled.wheel_stations[0].position_body_m
        );
        let stowed = compiled.at_deployment_fraction(0.0);
        let expected_stowed =
            spec.retraction.unwrap().rotation_at(0.0) * compiled.wheel_stations[0].position_body_m;
        assert!(close_vec3(
            stowed.wheel_stations[0].position_body_m,
            expected_stowed
        ));
        assert!(
            (stowed.wheel_stations[0].position_body_m - compiled.wheel_stations[0].position_body_m)
                .length()
                > 1.0
        );

        let retraction = spec.retraction.unwrap();
        let initial = WheelChassisState {
            deployment_fraction: 0.0,
            actuator_stalled: false,
        };
        let (moving, point) = retraction
            .advance_deployment(initial, true, 0.5, 500.0)
            .expect("loaded actuator moves toward deployment target");
        let expected_fraction = 0.5 * 0.5 / std::f64::consts::FRAC_PI_2;
        assert!((moving.deployment_fraction - expected_fraction).abs() < 1.0e-12);
        assert!(point.moving);
        let (stalled, point) = retraction
            .advance_deployment(moving, true, 0.5, 1_000.0)
            .expect("actuator stalls at its torque limit");
        assert_eq!(stalled.deployment_fraction, moving.deployment_fraction);
        assert!(stalled.actuator_stalled);
        assert!(point.stalled);
    }

    #[test]
    fn one_wheel_is_centered_and_mass_scales_with_wheel_count() {
        let single = chassis(WheelLayout::Inline, 1)
            .compile()
            .expect("single wheel chassis");
        assert_eq!(single.wheel_stations[0].position_body_m.x, 1.0);
        let four = chassis(WheelLayout::AxlePairs { track_width_m: 1.2 }, 4)
            .compile()
            .expect("four-wheel chassis");
        assert!((single.dry_mass_kg() - (12.0 + 3.4 + 1.0 + 0.5)).abs() < 1.0e-12);
        assert!((four.dry_mass_kg() - (12.0 + 4.0 * (3.4 + 1.0 + 0.5))).abs() < 1.0e-12);
        assert!(four.mass_properties.inertia_body_kg_m2.is_finite());
        assert!(four.mass_properties.center_of_mass_body_m.is_finite());

        let mut driven = chassis(WheelLayout::AxlePairs { track_width_m: 1.2 }, 4);
        driven.drive = Some(wheel_drive(4));
        let driven = driven.compile().expect("motor mass contribution");
        assert!((driven.dry_mass_kg() - four.dry_mass_kg() - 18.0).abs() < 1.0e-12);
    }

    #[test]
    fn tire_load_matches_spring_damper_anchor_and_never_pulls() {
        let tire = pneumatic_tire();
        let loaded = tire.normal_load(0.02, 0.1).expect("compression load");
        assert!((loaded.normal_load_n - 5_000.0).abs() < 1.0e-10);
        let rebound = tire.normal_load(0.02, -1.0).expect("rebound load");
        assert_eq!(rebound.normal_load_n, 0.0);
        let airborne = tire.normal_load(-0.01, 1.0e6).expect("airborne wheel");
        assert_eq!(airborne.normal_load_n, 0.0);
        let bump = tire.normal_load(1.0, 0.0).expect("tire travel cap");
        assert_eq!(bump.compression_m, tire.maximum_deflection_m);
        assert_eq!(bump.normal_load_n, tire.maximum_load_n);
        assert!(bump.saturated);
    }

    #[test]
    fn tire_tangent_force_opposes_slip_and_obeys_friction_circle() {
        let tire = pneumatic_tire();
        assert_eq!(tire.contact_friction(0.5).unwrap(), 0.7);
        let free = tire.tangential_force(0.05, 0.0, 1_000.0, 0.8).unwrap();
        assert_eq!(free.longitudinal_force_n, -600.0);
        assert_eq!(free.lateral_force_n, 0.0);
        assert!(!free.saturated);

        let saturated = tire.tangential_force(0.1, 0.1, 1_000.0, 0.8).unwrap();
        assert!(saturated.saturated);
        assert!(
            (saturated
                .longitudinal_force_n
                .hypot(saturated.lateral_force_n)
                - 800.0)
                .abs()
                < 1.0e-10
        );
        let airborne = tire.tangential_force(1.0, -1.0, 0.0, 0.8).unwrap();
        assert_eq!(airborne.longitudinal_force_n, 0.0);
        assert_eq!(airborne.lateral_force_n, 0.0);
    }

    #[test]
    fn tire_and_strut_share_contact_penetration_as_series_springs() {
        let chassis = chassis(WheelLayout::Inline, 1);
        let load = chassis.contact_load(0.1, 0.0, 1.0).unwrap();
        let expected_strut_compression = (chassis.tire.radial_stiffness_n_m * 0.1
            - chassis.strut.preload_n)
            / (chassis.strut.spring_rate_n_m + chassis.tire.radial_stiffness_n_m);
        let expected_tire_compression = 0.1 - expected_strut_compression;
        let expected_load = chassis.tire.radial_stiffness_n_m * expected_tire_compression;
        assert!((load.strut_compression_m - expected_strut_compression).abs() < 1.0e-12);
        assert!((load.tire_compression_m - expected_tire_compression).abs() < 1.0e-12);
        assert!((load.normal_load_n - expected_load).abs() < 1.0e-8);
        assert!(!load.saturated);

        let side_slope = chassis.contact_load(0.05, 0.0, 0.5).unwrap();
        assert!(side_slope.strut_compression_m > 0.0);
        assert!(side_slope.tire_compression_m > 0.0);
        assert!(side_slope.normal_load_n <= chassis.tire.maximum_load_n);
        assert_eq!(
            chassis.contact_load(0.0, 1.0, 1.0).unwrap().normal_load_n,
            0.0
        );
    }

    #[test]
    fn strut_response_is_bounded_and_damper_does_not_pull() {
        let known = WheelStrutSpec {
            extended_length_m: 0.5,
            stroke_m: 0.2,
            spring_rate_n_m: 30_000.0,
            damping_n_s_m: 3_000.0,
            preload_n: 500.0,
            minimum_force_n: 0.0,
            maximum_force_n: 6_000.0,
            mass_per_wheel_kg: 2.0,
        };
        assert_eq!(known.axial_force(0.02, 0.1).unwrap().axial_force_n, 1_400.0);
        assert_eq!(known.axial_force(0.02, -1.0).unwrap().axial_force_n, 0.0);
        let stop = known.axial_force(0.25, 0.0).unwrap();
        assert!(stop.at_stroke_stop);
        assert_eq!(stop.compression_m, known.stroke_m);
        assert_eq!(stop.axial_force_n, known.maximum_force_n);
    }

    #[test]
    fn brake_force_is_limited_by_both_actuator_and_tire_contact() {
        let brake = WheelBrakeSpec {
            maximum_torque_nm: 500.0,
            response_time_s: 0.1,
            mass_per_wheel_kg: 0.5,
        };
        let low_load = brake.braking_force(1.0, 0.25, 1_000.0, 0.8).unwrap();
        assert_eq!(low_load.brake_torque_nm, 500.0);
        assert_eq!(low_load.maximum_braking_force_n, 800.0);
        assert!(low_load.traction_limited);
        let high_load = brake.braking_force(1.0, 0.25, 4_000.0, 0.8).unwrap();
        assert_eq!(high_load.maximum_braking_force_n, 2_000.0);
        assert!(!high_load.traction_limited);
        let airborne = brake.braking_force(1.0, 0.25, 0.0, 0.8).unwrap();
        assert_eq!(airborne.maximum_braking_force_n, 0.0);
    }

    #[test]
    fn brake_actuator_response_matches_first_order_time_constant() {
        let brake = WheelBrakeSpec {
            maximum_torque_nm: 500.0,
            response_time_s: 0.1,
            mass_per_wheel_kg: 0.5,
        };
        let state = brake.advance(WheelBrakeState::default(), 1.0, 0.1).unwrap();
        assert!((state.applied_fraction - (1.0 - (-1.0_f64).exp())).abs() < 1.0e-14);

        let ideal = WheelBrakeSpec {
            response_time_s: 0.0,
            ..brake
        }
        .advance(WheelBrakeState::default(), 0.7, 0.0)
        .unwrap();
        assert_eq!(ideal.applied_fraction, 0.7);
    }

    #[test]
    fn pneumatic_pressure_is_required_but_airless_wheel_works_without_it() {
        assert!(pneumatic_tire().validate().is_ok());
        let airless = airless_tire();
        assert!(airless.validate().is_ok());
        assert_eq!(
            airless.normal_load(0.02, 0.1).unwrap().normal_load_n,
            pneumatic_tire()
                .normal_load(0.02, 0.1)
                .unwrap()
                .normal_load_n
        );
        assert!(
            chassis(WheelLayout::Inline, 1)
                .compile()
                .expect("airless design still compiles")
                .mass_properties
                .mass_kg
                > 0.0
        );
        let invalid = WheelTireSpec {
            construction: TireConstruction::Pneumatic {
                inflation_pressure_pa: 0.0,
                reference_temperature_k: 293.15,
            },
            ..pneumatic_tire()
        };
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn traction_motor_has_standstill_torque_and_respects_power_and_thermal_limits() {
        let drive = wheel_drive(4).compile(4).expect("four-wheel drive");
        let standstill = drive.operating_point(0.0, 1.0).unwrap();
        assert_eq!(standstill.motor_torque_nm, 400.0);
        assert_eq!(standstill.mechanical_power_w, 0.0);
        assert_eq!(standstill.electrical_power_w, 1_000.0);
        assert_eq!(standstill.requested_wheel_torque_per_driven_wheel_nm, 900.0);
        let traction_limited = standstill.contact_torque_limit(0.32, 1_000.0, 0.8).unwrap();
        assert_eq!(traction_limited.requested_torque_nm, 900.0);
        assert_eq!(traction_limited.maximum_torque_nm, 256.0);
        assert!(traction_limited.would_exceed_grip);
        let airborne = standstill.contact_torque_limit(0.32, 0.0, 0.8).unwrap();
        assert_eq!(airborne.maximum_torque_nm, 0.0);

        let power_limited = drive.operating_point(1_000.0, 1.0).unwrap();
        assert!((power_limited.mechanical_power_w - 100_000.0).abs() < 1.0e-8);
        assert!(power_limited.power_limited);
        let overspeed = drive.operating_point(1_300.0, 1.0).unwrap();
        assert!(overspeed.speed_limited);
        assert_eq!(overspeed.motor_torque_nm, 0.0);

        let low_cooling = WheelDriveSpec {
            motor: ElectricMotorSpec {
                cooling_capacity_w: 250.0,
                ..wheel_drive(4).motor
            },
            ..wheel_drive(4)
        }
        .compile(4)
        .unwrap()
        .operating_point(0.0, 1.0)
        .unwrap();
        assert!(low_cooling.thermal_limited);
        assert!((low_cooling.waste_heat_w - 250.0).abs() < 1.0e-9);
    }

    #[test]
    fn wheel_chassis_rejects_bad_layout_counts_and_unmatched_drive() {
        assert!(
            chassis(WheelLayout::AxlePairs { track_width_m: 1.0 }, 3)
                .compile()
                .is_err()
        );
        assert!(
            chassis(WheelLayout::AxlePairs { track_width_m: 0.0 }, 4)
                .compile()
                .is_err()
        );
        let mut bad_drive = chassis(WheelLayout::Inline, 2);
        bad_drive.drive = Some(wheel_drive(3));
        assert!(bad_drive.compile().is_err());
        let mut invalid_axis = chassis(WheelLayout::Inline, 1);
        invalid_axis.mount_orientation_body = DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0);
        assert!(invalid_axis.compile().is_err());
        let mut overlapping = chassis(WheelLayout::Inline, 2);
        overlapping.length_m = 0.5;
        assert!(overlapping.compile().is_err());
        let mut negative_inertia = chassis(WheelLayout::Inline, 1);
        negative_inertia.structural_inertia_local_kg_m2 =
            DMat3::from_diagonal(DVec3::new(-1.0, 1.0, 1.0));
        assert!(negative_inertia.compile().is_err());
    }

    #[test]
    fn reusable_and_crushable_landing_shocks_have_distinct_state_laws() {
        let reusable = LandingShockAbsorberSpec::Reusable {
            stroke_m: 0.25,
            spring_rate_n_m: 20_000.0,
            damping_n_s_m: 500.0,
            preload_n: 100.0,
            bottom_out_stiffness_n_m: 200_000.0,
            maximum_force_n: 40_000.0,
        };
        let (rebound, rebound_state) = reusable
            .evaluate(LandingLegState::default(), 0.05, -2.0, 0.1)
            .unwrap();
        assert_eq!(rebound.force_axial_n, 100.0);
        assert_eq!(rebound_state.permanent_crush_m, 0.0);
        assert_eq!(rebound.absorbed_energy_delta_j, 200.0);
        let (compression, _) = reusable
            .evaluate(LandingLegState::default(), 0.05, 2.0, 0.1)
            .unwrap();
        assert_eq!(compression.force_axial_n, 2_100.0);
        assert!(
            reusable
                .evaluate(
                    LandingLegState {
                        permanent_crush_m: 0.01,
                        ..LandingLegState::default()
                    },
                    0.05,
                    0.0,
                    0.1,
                )
                .is_err()
        );

        let crushable = LandingShockAbsorberSpec::Crushable {
            elastic_stiffness_n_m: 100_000.0,
            damping_n_s_m: 200.0,
            plateau_force_n: 10_000.0,
            maximum_crush_m: 0.25,
            bottom_out_stiffness_n_m: 300_000.0,
            maximum_force_n: 100_000.0,
        };
        let (first_impact, crushed_state) = crushable
            .evaluate(LandingLegState::default(), 0.25, 0.0, 0.1)
            .unwrap();
        assert_eq!(first_impact.force_axial_n, 10_000.0);
        assert!((first_impact.permanent_crush_m - 0.15).abs() < 1.0e-12);
        assert!((first_impact.absorbed_energy_delta_j - 1_500.0).abs() < 1.0e-12);
        assert_eq!(crushed_state.permanent_crush_m, 0.15);
        let (second_impact, further_crushed) =
            crushable.evaluate(crushed_state, 0.30, 0.0, 0.1).unwrap();
        assert_eq!(second_impact.force_axial_n, 10_000.0);
        assert!((further_crushed.permanent_crush_m - 0.20).abs() < 1.0e-12);
        assert!((further_crushed.absorbed_energy_j - 2_000.0).abs() < 1.0e-12);
        let (bottomed, _) = crushable.evaluate(further_crushed, 0.70, 0.0, 0.1).unwrap();
        assert!(bottomed.bottomed_out);
        assert!(bottomed.exhausted);
        assert!(bottomed.force_axial_n > second_impact.force_axial_n);
    }

    #[test]
    fn landing_leg_geometry_mass_and_fold_actuator_are_bounded() {
        let reusable = LandingShockAbsorberSpec::Reusable {
            stroke_m: 0.2,
            spring_rate_n_m: 40_000.0,
            damping_n_s_m: 1_000.0,
            preload_n: 0.0,
            bottom_out_stiffness_n_m: 250_000.0,
            maximum_force_n: 80_000.0,
        };
        let spec = landing_leg(reusable);
        let compiled = spec.clone().compile().unwrap();
        assert_eq!(compiled.mass_properties.mass_kg, 21.0);
        assert!(compiled.mass_properties.center_of_mass_body_m.z < 0.0);
        assert!(compiled.mass_properties.inertia_body_kg_m2.is_finite());
        assert!((spec.leg_axis_body_at_fraction(1.0) - DVec3::NEG_Z).length() < 1.0e-12);
        assert_eq!(spec.initial_state().deployment_fraction, 0.0);

        let initial = spec.initial_state();
        let (moving, point) = spec.advance_deployment(initial, true, 0.5, 0.0).unwrap();
        assert!((moving.deployment_fraction - 0.5 / std::f64::consts::PI).abs() < 1.0e-12);
        assert!(point.moving);
        let (loaded_moving, _) = spec
            .advance_deployment(initial, true, 0.5, 6_000.0)
            .unwrap();
        assert!((loaded_moving.deployment_fraction - 0.25 / std::f64::consts::PI).abs() < 1.0e-12);
        let (stalled, point) = spec
            .advance_deployment(initial, true, 1.0, 12_001.0)
            .unwrap();
        assert!(stalled.actuator_stalled);
        assert_eq!(stalled.deployment_fraction, initial.deployment_fraction);
        assert!(point.stalled);
        assert_eq!(point.actuator_torque_nm, 12_000.0);
        let (retracting, point) = spec
            .advance_deployment(
                LandingLegState {
                    deployment_fraction: 1.0,
                    ..LandingLegState::default()
                },
                false,
                0.5,
                0.0,
            )
            .unwrap();
        assert!(retracting.deployment_fraction < 1.0);
        assert_eq!(point.target_fraction, 0.0);
    }
}
