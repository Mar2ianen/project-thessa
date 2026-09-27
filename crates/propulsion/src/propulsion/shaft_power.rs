//! Shaft-power propulsion primitives (doc04 §9).
//!
//! This module deliberately distinguishes a geometry-driven source from
//! its propulsor. `CompiledPropeller` is an ideal actuator disk: it maps
//! delivered shaft power, disk area, and ambient density/airspeed to the
//! momentum-theory upper bound on thrust. Blade profile drag, pitch maps,
//! swirl, and transonic blade-section losses are outside that ideal-disk
//! model and are not hidden in empirical multipliers.
//!
//! `CompiledPistonEngine` evaluates a four-stroke air-standard Otto cycle
//! from the sampled atmosphere, fuel LHV/stoichiometry, compression ratio,
//! and geometric displacement. `CompiledElectricMotor` exposes a continuous
//! torque/power envelope and electrical/thermal bookkeeping. These are
//! steady operating-point components; shaft inertia, starting, and bus
//! energy storage remain explicit runtime state owned by the vehicle layer.

use serde::{Deserialize, Serialize};

use crate::atmosphere::GasKind;

use super::{
    AirOperatingPoint, AirbreathingSpec, CompiledAirbreather, FlightCondition, JetFuel,
    JetShaftState, PropulsionError, STANDARD_GRAVITY_MPS2, ShaftCommand, ShaftTelemetry,
    advance_jet_shaft_loaded, require_non_negative, require_positive, require_unit_interval,
};

const FOUR_STROKE_CYCLES_PER_REV: f64 = 0.5;

/// Geometric rotor and material properties for a propeller/fan disk.
///
/// The actuator-disk performance model uses the annular swept area; blade
/// geometry is used for dry mass. It is an explicitly idealized propulsor,
/// not a blade-element performance map.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PropellerSpec {
    /// Number of physical blades used in the geometric mass estimate.
    pub blade_count: u8,
    /// Outer disk diameter (m).
    pub diameter_m: f64,
    /// Hub diameter excluded from the active annular disk (m).
    pub hub_diameter_m: f64,
    /// Uniform blade chord for the geometric mass estimate (m).
    pub blade_chord_m: f64,
    /// Uniform blade thickness for the geometric mass estimate (m).
    pub blade_thickness_m: f64,
    /// Blade material density (kg/m³).
    pub blade_material_density_kg_m3: f64,
    /// Propeller gearbox efficiency (shaft power reaching the disk / input).
    pub gearbox_efficiency: f64,
    /// Installed gearbox mass (kg).
    pub gearbox_mass_kg: f64,
}

impl Default for PropellerSpec {
    fn default() -> Self {
        Self {
            blade_count: 4,
            diameter_m: 2.0,
            hub_diameter_m: 0.25,
            blade_chord_m: 0.12,
            blade_thickness_m: 0.018,
            blade_material_density_kg_m3: 1_600.0,
            gearbox_efficiency: 0.97,
            gearbox_mass_kg: 12.0,
        }
    }
}

/// Compiled ideal actuator disk with geometric blade mass.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledPropeller {
    pub blade_count: u8,
    pub diameter_m: f64,
    pub hub_diameter_m: f64,
    pub disk_area_m2: f64,
    pub gearbox_efficiency: f64,
    pub dry_mass_kg: f64,
}

/// One ideal actuator-disk operating point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropellerPoint {
    /// Axial thrust (N).
    pub thrust_n: f64,
    /// Mechanical power delivered to the ideal disk (W).
    pub disk_power_w: f64,
    /// Gearbox input power before its efficiency loss (W).
    pub shaft_power_w: f64,
    /// Torque at the propeller shaft (N·m).
    pub torque_nm: f64,
    /// Propeller shaft speed (rev/min).
    pub rpm: f64,
    /// True when zero ambient density leaves the disk with no air to load.
    pub density_limited: bool,
    /// Ideal propulsive efficiency T·V/P; zero at static conditions.
    pub efficiency: f64,
    /// Induced velocity at the disk (m/s).
    pub induced_velocity_mps: f64,
}

impl PropellerSpec {
    pub fn compile(self) -> Result<CompiledPropeller, PropulsionError> {
        if !(2..=16).contains(&self.blade_count) {
            return Err(PropulsionError::InvalidSpec(
                "propeller blade count must be in [2, 16]".into(),
            ));
        }
        require_positive(self.diameter_m, "propeller diameter")?;
        require_non_negative(self.hub_diameter_m, "propeller hub diameter")?;
        if self.hub_diameter_m >= self.diameter_m {
            return Err(PropulsionError::InvalidSpec(
                "propeller hub diameter must be smaller than disk diameter".into(),
            ));
        }
        require_positive(self.blade_chord_m, "propeller blade chord")?;
        require_positive(self.blade_thickness_m, "propeller blade thickness")?;
        require_positive(
            self.blade_material_density_kg_m3,
            "propeller blade material density",
        )?;
        if !self.gearbox_efficiency.is_finite()
            || !(0.0..=1.0).contains(&self.gearbox_efficiency)
            || self.gearbox_efficiency == 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "propeller gearbox efficiency must be finite in (0, 1]".into(),
            ));
        }
        require_non_negative(self.gearbox_mass_kg, "propeller gearbox mass")?;

        let span_m = 0.5 * (self.diameter_m - self.hub_diameter_m);
        let blade_volume_m3 =
            f64::from(self.blade_count) * span_m * self.blade_chord_m * self.blade_thickness_m;
        let dry_mass_kg =
            blade_volume_m3 * self.blade_material_density_kg_m3 + self.gearbox_mass_kg;
        let disk_area_m2 =
            std::f64::consts::PI / 4.0 * (self.diameter_m.powi(2) - self.hub_diameter_m.powi(2));
        if !dry_mass_kg.is_finite() || !disk_area_m2.is_finite() || disk_area_m2 <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "propeller geometry produced a non-finite mass or disk area".into(),
            ));
        }
        Ok(CompiledPropeller {
            blade_count: self.blade_count,
            diameter_m: self.diameter_m,
            hub_diameter_m: self.hub_diameter_m,
            disk_area_m2,
            gearbox_efficiency: self.gearbox_efficiency,
            dry_mass_kg,
        })
    }
}

impl CompiledPropeller {
    /// Evaluate the ideal annular actuator disk at delivered shaft power.
    ///
    /// The scalar equation `P = 2ρA v_i (V + v_i)²` is solved by bisection;
    /// thrust then follows as `T = 2ρA v_i (V + v_i)`. The enclosure starts
    /// at zero and `cbrt(P/(2ρA))`, which brackets the non-negative root for
    /// all `V >= 0`. A nonzero power command at zero RPM is refused because
    /// it would imply unbounded shaft torque.
    pub fn operating_point(
        &self,
        shaft_power_w: f64,
        rpm: f64,
        condition: &FlightCondition,
    ) -> Result<PropellerPoint, PropulsionError> {
        condition.validate()?;
        if !shaft_power_w.is_finite() || shaft_power_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "propeller shaft power must be finite and >= 0".into(),
            ));
        }
        if !rpm.is_finite() || rpm < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "propeller RPM must be finite and >= 0".into(),
            ));
        }
        if shaft_power_w > 0.0 && rpm <= 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "positive propeller power requires positive shaft RPM".into(),
            ));
        }

        let disk_power_w = shaft_power_w * self.gearbox_efficiency;
        let density = condition.density_kg_m3();
        if density <= 0.0 {
            return Ok(PropellerPoint {
                thrust_n: 0.0,
                disk_power_w: 0.0,
                shaft_power_w: 0.0,
                torque_nm: 0.0,
                rpm,
                density_limited: true,
                efficiency: 0.0,
                induced_velocity_mps: 0.0,
            });
        }
        if disk_power_w == 0.0 {
            return Ok(PropellerPoint {
                thrust_n: 0.0,
                disk_power_w,
                shaft_power_w,
                torque_nm: if rpm > 0.0 {
                    disk_power_w / (rpm * std::f64::consts::TAU / 60.0)
                } else {
                    0.0
                },
                rpm,
                density_limited: false,
                efficiency: 0.0,
                induced_velocity_mps: 0.0,
            });
        }
        if !density.is_finite() {
            return Err(PropulsionError::InvalidCommand(
                "ambient density is non-finite".into(),
            ));
        }

        let airspeed = condition.airspeed_mps;
        let power_scale = disk_power_w / (2.0 * density * self.disk_area_m2);
        let mut low = 0.0;
        let mut high = power_scale.cbrt();
        for _ in 0..80 {
            let induced = 0.5 * (low + high);
            let required =
                2.0 * density * self.disk_area_m2 * induced * (airspeed + induced).powi(2);
            if required < disk_power_w {
                low = induced;
            } else {
                high = induced;
            }
        }
        let induced_velocity_mps = 0.5 * (low + high);
        let thrust_n = 2.0
            * density
            * self.disk_area_m2
            * induced_velocity_mps
            * (airspeed + induced_velocity_mps);
        let torque_nm = disk_power_w / (rpm * std::f64::consts::TAU / 60.0);
        let efficiency = (thrust_n * airspeed / disk_power_w).clamp(0.0, 1.0);
        if !thrust_n.is_finite() || !torque_nm.is_finite() || !efficiency.is_finite() {
            return Err(PropulsionError::InvalidCommand(
                "actuator-disk evaluation produced a non-finite result".into(),
            ));
        }
        Ok(PropellerPoint {
            thrust_n,
            disk_power_w,
            shaft_power_w,
            torque_nm,
            rpm,
            density_limited: false,
            efficiency,
            induced_velocity_mps,
        })
    }
}

/// Four-stroke spark/pressure-ignition engine design inputs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PistonEngineSpec {
    pub fuel: JetFuel,
    pub bore_m: f64,
    pub stroke_m: f64,
    pub cylinder_count: u8,
    pub compression_ratio: f64,
    pub boost_pressure_ratio: f64,
    pub supercharger_efficiency: f64,
    pub volumetric_efficiency: f64,
    pub combustion_efficiency: f64,
    pub idle_rpm: f64,
    pub redline_rpm: f64,
    /// Friction mean effective pressure (Pa), deducted from cycle work.
    pub friction_mep_pa: f64,
    /// Fraction of fuel heat assigned to block/coolant rejection in this
    /// air-standard model (the remainder leaves with the exhaust).
    pub wall_heat_fraction: f64,
    /// Continuous heat-rejection capacity of the cooling installation (W).
    pub cooling_capacity_w: f64,
    /// Installed engine dry mass (kg), an explicit hardware property.
    pub dry_mass_kg: f64,
}

impl Default for PistonEngineSpec {
    fn default() -> Self {
        Self {
            fuel: JetFuel::Kerosene,
            bore_m: 0.086,
            stroke_m: 0.086,
            cylinder_count: 4,
            compression_ratio: 9.0,
            boost_pressure_ratio: 1.0,
            supercharger_efficiency: 0.70,
            volumetric_efficiency: 0.85,
            combustion_efficiency: 0.98,
            idle_rpm: 800.0,
            redline_rpm: 6_000.0,
            friction_mep_pa: 150_000.0,
            wall_heat_fraction: 0.22,
            cooling_capacity_w: 30_000.0,
            dry_mass_kg: 145.0,
        }
    }
}

/// Compiled piston engine, with total swept volume derived from bore/stroke.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledPistonEngine {
    pub fuel: JetFuel,
    pub displacement_m3: f64,
    pub compression_ratio: f64,
    pub boost_pressure_ratio: f64,
    pub supercharger_efficiency: f64,
    pub volumetric_efficiency: f64,
    pub combustion_efficiency: f64,
    pub idle_rpm: f64,
    pub redline_rpm: f64,
    pub friction_mep_pa: f64,
    pub wall_heat_fraction: f64,
    pub cooling_capacity_w: f64,
    pub dry_mass_kg: f64,
}

/// Per-cycle piston-engine telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PistonOperatingPoint {
    pub rpm: f64,
    pub displacement_m3: f64,
    pub shaft_power_w: f64,
    pub torque_nm: f64,
    pub fuel_flow_kg_s: f64,
    pub fuel_power_w: f64,
    pub cooling_load_w: f64,
    pub thermal_efficiency: f64,
    pub oxygen_limited: bool,
    pub cooling_limited: bool,
    /// True when the engine turns without delivering shaft power, or when
    /// the cooling installation cannot carry even the friction heat of the
    /// minimum running point (overheat stall).
    pub stalled: bool,
}

impl PistonEngineSpec {
    pub fn compile(self) -> Result<CompiledPistonEngine, PropulsionError> {
        require_positive(self.bore_m, "piston bore")?;
        require_positive(self.stroke_m, "piston stroke")?;
        if !(1..=32).contains(&self.cylinder_count) {
            return Err(PropulsionError::InvalidSpec(
                "piston cylinder count must be in [1, 32]".into(),
            ));
        }
        if !self.compression_ratio.is_finite()
            || !(1.0..=30.0).contains(&self.compression_ratio)
            || self.compression_ratio == 1.0
        {
            return Err(PropulsionError::InvalidSpec(
                "piston compression ratio must be finite in (1, 30]".into(),
            ));
        }
        if !self.boost_pressure_ratio.is_finite() || self.boost_pressure_ratio < 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "piston boost pressure ratio must be finite and >= 1".into(),
            ));
        }
        require_unit_interval(self.supercharger_efficiency, "supercharger efficiency")?;
        if self.supercharger_efficiency == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "supercharger efficiency must be in (0, 1]".into(),
            ));
        }
        if !self.volumetric_efficiency.is_finite()
            || !(0.0..=1.5).contains(&self.volumetric_efficiency)
            || self.volumetric_efficiency == 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "volumetric efficiency must be finite in (0, 1.5]".into(),
            ));
        }
        require_unit_interval(self.combustion_efficiency, "combustion efficiency")?;
        require_positive(self.idle_rpm, "piston idle RPM")?;
        require_positive(self.redline_rpm, "piston redline RPM")?;
        if self.redline_rpm <= self.idle_rpm {
            return Err(PropulsionError::InvalidSpec(
                "piston redline RPM must exceed idle RPM".into(),
            ));
        }
        require_non_negative(self.friction_mep_pa, "piston friction MEP")?;
        require_unit_interval(self.wall_heat_fraction, "piston wall heat fraction")?;
        require_non_negative(self.cooling_capacity_w, "piston cooling capacity")?;
        require_positive(self.dry_mass_kg, "piston engine dry mass")?;

        let displacement_m3 = std::f64::consts::PI / 4.0
            * self.bore_m.powi(2)
            * self.stroke_m
            * f64::from(self.cylinder_count);
        if !displacement_m3.is_finite() || displacement_m3 <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "piston geometry produced invalid displacement".into(),
            ));
        }
        Ok(CompiledPistonEngine {
            fuel: self.fuel,
            displacement_m3,
            compression_ratio: self.compression_ratio,
            boost_pressure_ratio: self.boost_pressure_ratio,
            supercharger_efficiency: self.supercharger_efficiency,
            volumetric_efficiency: self.volumetric_efficiency,
            combustion_efficiency: self.combustion_efficiency,
            idle_rpm: self.idle_rpm,
            redline_rpm: self.redline_rpm,
            friction_mep_pa: self.friction_mep_pa,
            wall_heat_fraction: self.wall_heat_fraction,
            cooling_capacity_w: self.cooling_capacity_w,
            dry_mass_kg: self.dry_mass_kg,
        })
    }
}

impl CompiledPistonEngine {
    /// Evaluate a running four-stroke engine at a commanded RPM and throttle.
    ///
    /// The air-standard Otto cycle uses the sampled mixture gas constant and
    /// heat-capacity ratio. Intake charge is compressed isentropically with
    /// the authored compressor efficiency; fuel flow is stoichiometric but
    /// is capped by the sampled oxygen mass fraction. Friction MEP and the
    /// supercharger work are subtracted from brake power. Cooling overload
    /// reduces throttle by bisection and is explicitly reported; when the
    /// cooling installation cannot carry even the friction heat of the
    /// minimum running point, no throttle keeps the engine cool and the
    /// point comes back `stalled`.
    pub fn operating_point(
        &self,
        condition: &FlightCondition,
        rpm: f64,
        throttle: f64,
    ) -> Result<PistonOperatingPoint, PropulsionError> {
        condition.validate()?;
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return Err(PropulsionError::InvalidCommand(
                "piston throttle must be finite in [0, 1]".into(),
            ));
        }
        if !rpm.is_finite()
            || (throttle > 0.0 && (rpm < self.idle_rpm || rpm > self.redline_rpm))
            || (throttle == 0.0 && rpm < 0.0)
        {
            return Err(PropulsionError::InvalidCommand(format!(
                "piston RPM must be finite and in [{}, {}] while running",
                self.idle_rpm, self.redline_rpm
            )));
        }
        if throttle == 0.0 {
            return Ok(PistonOperatingPoint {
                rpm,
                displacement_m3: self.displacement_m3,
                shaft_power_w: 0.0,
                torque_nm: 0.0,
                fuel_flow_kg_s: 0.0,
                fuel_power_w: 0.0,
                cooling_load_w: 0.0,
                thermal_efficiency: 0.0,
                oxygen_limited: false,
                cooling_limited: false,
                stalled: false,
            });
        }

        let requested = self.at_throttle(condition, rpm, throttle)?;
        if requested.cooling_load_w <= self.cooling_capacity_w {
            return Ok(requested);
        }
        // Heat balance at the least-hungry running point: fuel heat scales
        // with throttle, so as the throttle falls away the only heat left
        // to reject is cycle friction at this RPM. When the cooling
        // installation cannot carry even that, every running point
        // generates more heat than it can shed — temperature runs away and
        // the engine is reported stalled, not limping along. The returned
        // point is that minimum running state: no fuel, no shaft power,
        // and the friction heat load that already exceeds the capacity.
        let minimum_cooling_load_w =
            self.friction_mep_pa * self.displacement_m3 * rpm / 60.0 * FOUR_STROKE_CYCLES_PER_REV;
        if self.cooling_capacity_w <= minimum_cooling_load_w {
            return Ok(PistonOperatingPoint {
                rpm,
                displacement_m3: self.displacement_m3,
                shaft_power_w: 0.0,
                torque_nm: 0.0,
                fuel_flow_kg_s: 0.0,
                fuel_power_w: 0.0,
                cooling_load_w: minimum_cooling_load_w,
                thermal_efficiency: 0.0,
                oxygen_limited: false,
                cooling_limited: true,
                stalled: true,
            });
        }
        let mut low = 0.0;
        let mut high = throttle;
        for _ in 0..56 {
            let mid = 0.5 * (low + high);
            let point = self.at_throttle(condition, rpm, mid)?;
            if point.cooling_load_w <= self.cooling_capacity_w {
                low = mid;
            } else {
                high = mid;
            }
        }
        let mut limited = self.at_throttle(condition, rpm, low)?;
        limited.cooling_limited = true;
        Ok(limited)
    }

    fn at_throttle(
        &self,
        condition: &FlightCondition,
        rpm: f64,
        throttle: f64,
    ) -> Result<PistonOperatingPoint, PropulsionError> {
        if throttle == 0.0 || condition.ambient_pa == 0.0 {
            return Ok(PistonOperatingPoint {
                rpm,
                displacement_m3: self.displacement_m3,
                shaft_power_w: 0.0,
                torque_nm: 0.0,
                fuel_flow_kg_s: 0.0,
                fuel_power_w: 0.0,
                cooling_load_w: 0.0,
                thermal_efficiency: 0.0,
                oxygen_limited: false,
                cooling_limited: false,
                stalled: false,
            });
        }

        let gas_constant = condition.composition.gas_constant_j_kg_k();
        let gamma = condition.composition.mean_heat_capacity_ratio();
        if !gas_constant.is_finite() || gas_constant <= 0.0 || !gamma.is_finite() || gamma <= 1.0 {
            return Err(PropulsionError::InvalidCommand(
                "piston engine requires a finite, positive-density atmosphere".into(),
            ));
        }
        // Ram (stagnation) intake state: isentropic total temperature and
        // pressure at the flight Mach number — the same ram model the
        // airbreathing cycle uses (air.rs), with unity recovery because no
        // intake duct is authored here. The trapped charge then follows
        // ram density instead of the static atmosphere.
        let ram_temp_ratio = 1.0 + (gamma - 1.0) / 2.0 * condition.mach * condition.mach;
        let ram_temp_k = condition.ambient_temp_k * ram_temp_ratio;
        let ram_pressure_pa = condition.ambient_pa * ram_temp_ratio.powf(gamma / (gamma - 1.0));
        let density = ram_pressure_pa / (gas_constant * ram_temp_k);
        if !density.is_finite() || density <= 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "piston engine requires a finite, positive-density atmosphere".into(),
            ));
        }

        let pressure_ratio = self.boost_pressure_ratio;
        let compressor_exponent = (gamma - 1.0) / gamma;
        let ideal_temp_ratio = pressure_ratio.powf(compressor_exponent);
        let charge_temp_k =
            ram_temp_k * (1.0 + (ideal_temp_ratio - 1.0) / self.supercharger_efficiency);
        let charge_pressure_pa = ram_pressure_pa * pressure_ratio;
        let charge_density = charge_pressure_pa / (gas_constant * charge_temp_k);
        let air_mass_per_cycle_kg =
            charge_density * self.displacement_m3 * self.volumetric_efficiency * throttle;

        let (lhv_j_kg, stoich_fuel_air, oxygen_per_fuel, _, _, _) = self.fuel.properties();
        let stoich_fuel_per_cycle_kg = air_mass_per_cycle_kg * stoich_fuel_air;
        let oxygen_available_kg =
            air_mass_per_cycle_kg * condition.composition.mass_fraction(GasKind::Oxygen);
        let oxygen_limited_fuel_kg = oxygen_available_kg / oxygen_per_fuel;
        let fuel_per_cycle_kg = stoich_fuel_per_cycle_kg.min(oxygen_limited_fuel_kg);
        let oxygen_limited = fuel_per_cycle_kg + f64::EPSILON < stoich_fuel_per_cycle_kg;

        let cv_j_kg_k = gas_constant / (gamma - 1.0);
        let compression_temperature_k = charge_temp_k * self.compression_ratio.powf(gamma - 1.0);
        let heat_added_j = fuel_per_cycle_kg * lhv_j_kg * self.combustion_efficiency;
        let peak_temperature_k =
            compression_temperature_k + heat_added_j / (air_mass_per_cycle_kg * cv_j_kg_k);
        let expanded_temperature_k = peak_temperature_k / self.compression_ratio.powf(gamma - 1.0);
        let indicated_work_j = air_mass_per_cycle_kg
            * cv_j_kg_k
            * ((peak_temperature_k - compression_temperature_k)
                - (expanded_temperature_k - charge_temp_k));

        let cycle_rate_hz = rpm / 60.0 * FOUR_STROKE_CYCLES_PER_REV;
        let fuel_flow_kg_s = fuel_per_cycle_kg * cycle_rate_hz;
        let fuel_power_w = fuel_flow_kg_s * lhv_j_kg;
        let indicated_power_w = indicated_work_j * cycle_rate_hz;
        let friction_power_w = self.friction_mep_pa * self.displacement_m3 * cycle_rate_hz;
        let mass_flow_kg_s = air_mass_per_cycle_kg * cycle_rate_hz;
        let compressor_specific_work_j_kg =
            gas_constant / (gamma - 1.0) * ram_temp_k * (ideal_temp_ratio - 1.0)
                / self.supercharger_efficiency;
        let supercharger_power_w = mass_flow_kg_s * compressor_specific_work_j_kg;
        let raw_shaft_power_w = indicated_power_w - friction_power_w - supercharger_power_w;
        let shaft_power_w = raw_shaft_power_w.max(0.0);
        let angular_speed_rad_s = rpm * std::f64::consts::TAU / 60.0;
        let torque_nm = shaft_power_w / angular_speed_rad_s;
        let cooling_load_w = fuel_power_w * self.wall_heat_fraction + friction_power_w;
        let thermal_efficiency = if fuel_power_w > 0.0 {
            shaft_power_w / fuel_power_w
        } else {
            0.0
        };
        if [
            charge_temp_k,
            indicated_work_j,
            fuel_flow_kg_s,
            shaft_power_w,
            torque_nm,
            cooling_load_w,
            thermal_efficiency,
        ]
        .iter()
        .any(|value| !value.is_finite())
        {
            return Err(PropulsionError::InvalidCommand(
                "piston cycle evaluation produced a non-finite result".into(),
            ));
        }
        Ok(PistonOperatingPoint {
            rpm,
            displacement_m3: self.displacement_m3,
            shaft_power_w,
            torque_nm,
            fuel_flow_kg_s,
            fuel_power_w,
            cooling_load_w,
            thermal_efficiency,
            oxygen_limited,
            cooling_limited: false,
            stalled: raw_shaft_power_w <= 0.0,
        })
    }
}

/// Continuous-duty electric motor/controller design inputs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ElectricMotorSpec {
    /// Rated continuous mechanical output (W).
    pub rated_power_w: f64,
    /// Low-speed torque ceiling (N·m).
    pub peak_torque_nm: f64,
    /// Maximum rotor speed (rev/min).
    pub maximum_rpm: f64,
    /// Combined motor/controller conversion efficiency.
    pub efficiency: f64,
    /// Heat rejected by the cooling installation at continuous duty (W).
    pub cooling_capacity_w: f64,
    /// Motor/controller installed mass (kg).
    pub dry_mass_kg: f64,
}

impl Default for ElectricMotorSpec {
    fn default() -> Self {
        Self {
            rated_power_w: 100_000.0,
            peak_torque_nm: 400.0,
            maximum_rpm: 12_000.0,
            efficiency: 0.94,
            cooling_capacity_w: 6_400.0,
            dry_mass_kg: 35.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledElectricMotor {
    pub rated_power_w: f64,
    pub peak_torque_nm: f64,
    pub maximum_rpm: f64,
    pub efficiency: f64,
    pub cooling_capacity_w: f64,
    pub dry_mass_kg: f64,
}

/// Motor electrical/mechanical/thermal operating point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ElectricMotorPoint {
    pub rpm: f64,
    pub torque_nm: f64,
    pub mechanical_power_w: f64,
    pub electrical_power_w: f64,
    pub waste_heat_w: f64,
    pub efficiency: f64,
    pub thermal_limited: bool,
    pub speed_limited: bool,
}

impl ElectricMotorSpec {
    pub fn compile(self) -> Result<CompiledElectricMotor, PropulsionError> {
        require_positive(self.rated_power_w, "motor rated power")?;
        require_positive(self.peak_torque_nm, "motor peak torque")?;
        require_positive(self.maximum_rpm, "motor maximum RPM")?;
        if !self.efficiency.is_finite() || !(0.0..=1.0).contains(&self.efficiency) {
            return Err(PropulsionError::InvalidSpec(
                "motor efficiency must be finite in (0, 1]".into(),
            ));
        }
        if self.efficiency == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "motor efficiency must be finite in (0, 1]".into(),
            ));
        }
        require_non_negative(self.cooling_capacity_w, "motor cooling capacity")?;
        require_positive(self.dry_mass_kg, "motor dry mass")?;
        Ok(CompiledElectricMotor {
            rated_power_w: self.rated_power_w,
            peak_torque_nm: self.peak_torque_nm,
            maximum_rpm: self.maximum_rpm,
            efficiency: self.efficiency,
            cooling_capacity_w: self.cooling_capacity_w,
            dry_mass_kg: self.dry_mass_kg,
        })
    }
}

impl CompiledElectricMotor {
    /// Evaluate the continuous torque envelope at rotor speed and throttle.
    /// Below base speed (`P_rated / τ_peak`) torque is constant; above it,
    /// power is constant. Cooling overload caps mechanical output using
    /// `P_loss = P_mech (1/η - 1)` and sets `thermal_limited`.
    pub fn operating_point(
        &self,
        rpm: f64,
        throttle: f64,
    ) -> Result<ElectricMotorPoint, PropulsionError> {
        if !rpm.is_finite() || rpm < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "motor RPM must be finite and >= 0".into(),
            ));
        }
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return Err(PropulsionError::InvalidCommand(
                "motor throttle must be finite in [0, 1]".into(),
            ));
        }
        if rpm == 0.0 || throttle == 0.0 {
            return Ok(ElectricMotorPoint {
                rpm,
                torque_nm: 0.0,
                mechanical_power_w: 0.0,
                electrical_power_w: 0.0,
                waste_heat_w: 0.0,
                efficiency: self.efficiency,
                thermal_limited: false,
                speed_limited: false,
            });
        }
        if rpm > self.maximum_rpm {
            return Ok(ElectricMotorPoint {
                rpm,
                torque_nm: 0.0,
                mechanical_power_w: 0.0,
                electrical_power_w: 0.0,
                waste_heat_w: 0.0,
                efficiency: self.efficiency,
                thermal_limited: false,
                speed_limited: true,
            });
        }

        let omega = rpm * std::f64::consts::TAU / 60.0;
        let base_omega = self.rated_power_w / self.peak_torque_nm;
        let envelope_torque_nm = if omega <= base_omega {
            self.peak_torque_nm
        } else {
            self.rated_power_w / omega
        };
        let requested_torque_nm = envelope_torque_nm * throttle;
        let requested_power_w = requested_torque_nm * omega;
        let maximum_thermal_power_w = if self.efficiency < 1.0 {
            self.cooling_capacity_w * self.efficiency / (1.0 - self.efficiency)
        } else {
            f64::INFINITY
        };
        let mechanical_power_w = requested_power_w.min(maximum_thermal_power_w);
        let thermal_limited = mechanical_power_w + f64::EPSILON < requested_power_w;
        let torque_nm = mechanical_power_w / omega;
        let electrical_power_w = mechanical_power_w / self.efficiency;
        let waste_heat_w = electrical_power_w - mechanical_power_w;
        Ok(ElectricMotorPoint {
            rpm,
            torque_nm,
            mechanical_power_w,
            electrical_power_w,
            waste_heat_w,
            efficiency: self.efficiency,
            thermal_limited,
            speed_limited: false,
        })
    }
}

/// Mechanical shaft power converted to propulsive performance and fuel Isp.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropDrivePoint {
    pub propeller: PropellerPoint,
    pub source_rpm: f64,
    pub source_shaft_power_w: f64,
    pub source_torque_nm: f64,
    pub fuel_flow_kg_s: f64,
    pub electrical_power_w: f64,
    pub specific_impulse_s: f64,
    pub oxygen_limited: bool,
    pub thermal_limited: bool,
    pub source_speed_limited: bool,
}

/// Analyzer output for one altitude/airspeed row of a shaft-power drive.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropDriveAltitudePoint {
    pub altitude_m: f64,
    pub airspeed_mps: f64,
    pub ambient_pa: f64,
    pub thrust_n: f64,
    pub isp_s: f64,
    pub fuel_flow_kg_s: f64,
    pub electrical_power_w: f64,
    pub source_rpm: f64,
    pub propeller_rpm: f64,
    pub oxygen_limited: bool,
    pub thermal_limited: bool,
    pub density_limited: bool,
    pub source_speed_limited: bool,
}

/// Selectable steady shaft-power source for a propeller drive.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "spec", rename_all = "kebab-case")]
pub enum ShaftPowerSourceSpec {
    Piston(PistonEngineSpec),
    Electric(ElectricMotorSpec),
}

/// Reusable propulsor plus a shaft source and reduction ratio. Its operating
/// point assumes the source's available brake power is absorbed by the disk;
/// shaft speed is an explicit command because this steady model owns no rotor
/// inertia state.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropellerDriveSpec {
    pub propeller: PropellerSpec,
    pub source: ShaftPowerSourceSpec,
    /// Source RPM divided by propeller RPM (must be finite and > 0).
    pub reduction_ratio: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CompiledShaftPowerSource {
    Piston(CompiledPistonEngine),
    Electric(CompiledElectricMotor),
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledPropellerDrive {
    pub propeller: CompiledPropeller,
    pub source: CompiledShaftPowerSource,
    pub reduction_ratio: f64,
    pub dry_mass_kg: f64,
}

/// Propeller-drive installation on a vehicle, with body-frame station and
/// thrust axis. Its force and moment are evaluated through the vehicle wrench
/// path just like jets and rocket mounts.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PropellerDriveMount {
    pub name: String,
    pub drive: CompiledPropellerDrive,
    pub position_body_m: [f64; 3],
    pub thrust_axis_body: [f64; 3],
}

/// A geared propeller driven from a gas-turbine shaft. The airbreather's
/// `ShaftSpec::power_turbine_heat_fraction` reserves an explicit part of
/// combustor heat for the power turbine. Loaded evaluation extracts that work
/// from the exhaust gas, reducing core-nozzle temperature and pressure.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TurbopropDriveSpec {
    pub air: AirbreathingSpec,
    pub propeller: PropellerSpec,
    /// Gas-generator shaft RPM at normalized spool speed 1.0.
    pub shaft_rpm_at_full_spool: f64,
    /// Gas-generator shaft RPM divided by propeller RPM.
    pub reduction_ratio: f64,
    /// Installed power-turbine and coupling hardware (kg).
    pub power_turbine_mass_kg: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledTurbopropDrive {
    pub air: CompiledAirbreather,
    pub propeller: CompiledPropeller,
    pub shaft_rpm_at_full_spool: f64,
    pub reduction_ratio: f64,
    pub power_turbine_mass_kg: f64,
    pub dry_mass_kg: f64,
}

/// Stateful turbine-propeller command. `shaft_state` is supplied from the
/// previous step and returned in the next command by `with_state`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TurbopropCommand {
    pub shaft_state: JetShaftState,
    pub shaft: ShaftCommand,
    pub dt_s: f64,
    /// Gas-generator shaft-side power drawn by the propeller reduction gear
    /// (W), before the propeller gearbox efficiency.
    pub propeller_power_w: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TurbopropOperatingPoint {
    pub air: AirOperatingPoint,
    pub propeller: PropellerPoint,
    pub total_thrust_n: f64,
    pub propeller_rpm: f64,
    pub shaft_telemetry: ShaftTelemetry,
    pub power_takeoff_capacity_w: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TurbopropAltitudePoint {
    pub altitude_m: f64,
    pub airspeed_mps: f64,
    pub ambient_pa: f64,
    pub core_thrust_n: f64,
    pub propeller_thrust_n: f64,
    pub total_thrust_n: f64,
    pub fuel_flow_kg_s: f64,
    pub power_takeoff_capacity_w: f64,
    pub power_takeoff_w: f64,
    pub propeller_rpm: f64,
    pub spool_n: f64,
}

/// Turboprop installation on a vehicle, with body-frame station and thrust
/// axis for the combined core-jet and propeller force.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TurbopropMount {
    pub name: String,
    pub drive: CompiledTurbopropDrive,
    pub position_body_m: [f64; 3],
    pub thrust_axis_body: [f64; 3],
}

/// One steady propeller-drive command: source speed and normalized throttle.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropellerDriveCommand {
    pub throttle: f64,
    pub source_rpm: f64,
}

impl PropellerDriveSpec {
    pub fn compile(self) -> Result<CompiledPropellerDrive, PropulsionError> {
        require_positive(self.reduction_ratio, "shaft-power reduction ratio")?;
        let propeller = self.propeller.compile()?;
        let source = match self.source {
            ShaftPowerSourceSpec::Piston(spec) => CompiledShaftPowerSource::Piston(spec.compile()?),
            ShaftPowerSourceSpec::Electric(spec) => {
                CompiledShaftPowerSource::Electric(spec.compile()?)
            }
        };
        let source_mass_kg = match source {
            CompiledShaftPowerSource::Piston(engine) => engine.dry_mass_kg,
            CompiledShaftPowerSource::Electric(motor) => motor.dry_mass_kg,
        };
        Ok(CompiledPropellerDrive {
            propeller,
            source,
            reduction_ratio: self.reduction_ratio,
            dry_mass_kg: propeller.dry_mass_kg + source_mass_kg,
        })
    }
}

impl CompiledPropellerDrive {
    pub fn operating_point(
        &self,
        condition: &FlightCondition,
        command: PropellerDriveCommand,
    ) -> Result<PropDrivePoint, PropulsionError> {
        if !command.throttle.is_finite() || !(0.0..=1.0).contains(&command.throttle) {
            return Err(PropulsionError::InvalidCommand(
                "shaft-power throttle must be finite in [0, 1]".into(),
            ));
        }
        if !command.source_rpm.is_finite() || command.source_rpm < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "shaft-power source RPM must be finite and >= 0".into(),
            ));
        }
        let propeller_rpm = command.source_rpm / self.reduction_ratio;
        if condition.density_kg_m3() <= 0.0 {
            let propeller = self
                .propeller
                .operating_point(0.0, propeller_rpm, condition)?;
            return Ok(PropDrivePoint {
                propeller,
                source_rpm: command.source_rpm,
                source_shaft_power_w: 0.0,
                source_torque_nm: 0.0,
                fuel_flow_kg_s: 0.0,
                electrical_power_w: 0.0,
                specific_impulse_s: 0.0,
                oxygen_limited: false,
                thermal_limited: false,
                source_speed_limited: false,
            });
        }
        let (
            source_power_w,
            source_torque_nm,
            fuel_flow_kg_s,
            electrical_power_w,
            oxygen_limited,
            thermal_limited,
            source_speed_limited,
        ) = match self.source {
            CompiledShaftPowerSource::Piston(engine) => {
                let point =
                    engine.operating_point(condition, command.source_rpm, command.throttle)?;
                (
                    point.shaft_power_w,
                    point.torque_nm,
                    point.fuel_flow_kg_s,
                    0.0,
                    point.oxygen_limited,
                    point.cooling_limited,
                    false,
                )
            }
            CompiledShaftPowerSource::Electric(motor) => {
                let point = motor.operating_point(command.source_rpm, command.throttle)?;
                (
                    point.mechanical_power_w,
                    point.torque_nm,
                    0.0,
                    point.electrical_power_w,
                    false,
                    point.thermal_limited,
                    point.speed_limited,
                )
            }
        };
        let propeller = self
            .propeller
            .operating_point(source_power_w, propeller_rpm, condition)?;
        Ok(PropDrivePoint {
            propeller,
            source_rpm: command.source_rpm,
            source_shaft_power_w: source_power_w,
            source_torque_nm,
            fuel_flow_kg_s,
            electrical_power_w,
            specific_impulse_s: effective_propulsive_isp_s(propeller.thrust_n, fuel_flow_kg_s),
            oxygen_limited,
            thermal_limited,
            source_speed_limited,
        })
    }
}

/// Analyze a shaft-power drive over altitude × true-airspeed rows at fixed
/// throttle and source RPM. The atmosphere composition is sampled into each
/// flight condition, so piston oxygen availability and actuator-disk density
/// use the same authoritative species basis as jet propulsion.
pub fn analyze_propeller_drive(
    drive: &CompiledPropellerDrive,
    atmosphere: &crate::atmosphere::AtmosphereConfig,
    altitudes_m: &[f64],
    airspeeds_mps: &[f64],
    throttle: f64,
    source_rpm: f64,
) -> Result<Vec<PropDriveAltitudePoint>, PropulsionError> {
    if altitudes_m.is_empty() || airspeeds_mps.is_empty() {
        return Err(PropulsionError::InvalidSpec(
            "propeller-drive analyzer needs altitudes and airspeeds".into(),
        ));
    }
    if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
        return Err(PropulsionError::InvalidCommand(
            "analyzer throttle must be finite in [0, 1]".into(),
        ));
    }
    if !source_rpm.is_finite() || source_rpm < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "analyzer source RPM must be finite and >= 0".into(),
        ));
    }
    let mut rows = Vec::with_capacity(altitudes_m.len() * airspeeds_mps.len());
    for altitude_m in altitudes_m {
        if !altitude_m.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "analyzer altitudes must be finite".into(),
            ));
        }
        let sample = atmosphere
            .sample(*altitude_m)
            .map_err(|error| PropulsionError::InvalidSpec(format!("atmosphere sample: {error}")))?;
        for airspeed_mps in airspeeds_mps {
            if !airspeed_mps.is_finite() || *airspeed_mps < 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "analyzer airspeeds must be finite and >= 0".into(),
                ));
            }
            let condition = super::flight_condition(&sample, *airspeed_mps)?;
            let point = drive.operating_point(
                &condition,
                PropellerDriveCommand {
                    throttle,
                    source_rpm,
                },
            )?;
            rows.push(PropDriveAltitudePoint {
                altitude_m: *altitude_m,
                airspeed_mps: *airspeed_mps,
                ambient_pa: sample.pressure_pa,
                thrust_n: point.propeller.thrust_n,
                isp_s: point.specific_impulse_s,
                fuel_flow_kg_s: point.fuel_flow_kg_s,
                electrical_power_w: point.electrical_power_w,
                source_rpm: point.source_rpm,
                propeller_rpm: point.propeller.rpm,
                oxygen_limited: point.oxygen_limited,
                thermal_limited: point.thermal_limited,
                density_limited: point.propeller.density_limited,
                source_speed_limited: point.source_speed_limited,
            });
        }
    }
    Ok(rows)
}

/// Analyze a running turboprop at design spool over altitude × airspeed.
/// `power_takeoff_fraction` is a requested share of each row's physically
/// available power-turbine output (not a thrust scale); the loaded gas path
/// debits this work before evaluating the core nozzle.
pub fn analyze_turboprop_drive(
    drive: &CompiledTurbopropDrive,
    atmosphere: &crate::atmosphere::AtmosphereConfig,
    altitudes_m: &[f64],
    airspeeds_mps: &[f64],
    throttle: f64,
    power_takeoff_fraction: f64,
) -> Result<Vec<TurbopropAltitudePoint>, PropulsionError> {
    if altitudes_m.is_empty() || airspeeds_mps.is_empty() {
        return Err(PropulsionError::InvalidSpec(
            "turboprop analyzer needs altitudes and airspeeds".into(),
        ));
    }
    if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
        return Err(PropulsionError::InvalidCommand(
            "analyzer throttle must be finite in [0, 1]".into(),
        ));
    }
    if !power_takeoff_fraction.is_finite() || !(0.0..=1.0).contains(&power_takeoff_fraction) {
        return Err(PropulsionError::InvalidCommand(
            "analyzer power-takeoff fraction must be finite in [0, 1]".into(),
        ));
    }
    let mut rows = Vec::with_capacity(altitudes_m.len() * airspeeds_mps.len());
    for altitude_m in altitudes_m {
        if !altitude_m.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "analyzer altitudes must be finite".into(),
            ));
        }
        let sample = atmosphere
            .sample(*altitude_m)
            .map_err(|error| PropulsionError::InvalidSpec(format!("atmosphere sample: {error}")))?;
        for airspeed_mps in airspeeds_mps {
            if !airspeed_mps.is_finite() || *airspeed_mps < 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "analyzer airspeeds must be finite and >= 0".into(),
                ));
            }
            let condition = super::flight_condition(&sample, *airspeed_mps)?;
            let (_, balance) = drive.air.operating_point_at_spool_loaded(
                &condition,
                throttle,
                1.0,
                throttle > 0.0,
                0.0,
            )?;
            let power_takeoff_w = balance.power_takeoff_capacity_w * power_takeoff_fraction;
            let mut command = TurbopropCommand::running(drive);
            command.shaft.throttle = throttle;
            command.propeller_power_w = power_takeoff_w;
            let (_, point) = drive.advance(&condition, &command)?;
            rows.push(TurbopropAltitudePoint {
                altitude_m: *altitude_m,
                airspeed_mps: *airspeed_mps,
                ambient_pa: sample.pressure_pa,
                core_thrust_n: point.air.thrust_n,
                propeller_thrust_n: point.propeller.thrust_n,
                total_thrust_n: point.total_thrust_n,
                fuel_flow_kg_s: point.air.fuel_flow_kg_s,
                power_takeoff_capacity_w: point.power_takeoff_capacity_w,
                power_takeoff_w,
                propeller_rpm: point.propeller_rpm,
                spool_n: command.shaft_state.spool_n,
            });
        }
    }
    Ok(rows)
}

// Baked-JSON guards for the compiled shaft-power hardware. `compile`
// enforces these once at authoring time; a tampered or hand-edited baked
// definition reaches the formulas directly, so the mounts re-check the
// divisors, powf bases, and finiteness-critical values before any
// evaluation turns them into NaN instead of `InvalidSpec`.
impl CompiledPropeller {
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.disk_area_m2, "propeller disk area")?;
        if !self.gearbox_efficiency.is_finite()
            || !(0.0..=1.0).contains(&self.gearbox_efficiency)
            || self.gearbox_efficiency == 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "propeller gearbox efficiency must be finite in (0, 1]".into(),
            ));
        }
        require_non_negative(self.dry_mass_kg, "propeller dry mass")?;
        Ok(())
    }
}

impl CompiledPistonEngine {
    /// Validate baked values the cycle divides by, powf's with, or
    /// compares against (mirrors [`PistonEngineSpec::compile`]).
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.displacement_m3, "piston displacement")?;
        if !self.compression_ratio.is_finite() || self.compression_ratio <= 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "piston compression ratio must be finite and > 1".into(),
            ));
        }
        if !self.boost_pressure_ratio.is_finite() || self.boost_pressure_ratio < 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "piston boost pressure ratio must be finite and >= 1".into(),
            ));
        }
        require_unit_interval(
            self.supercharger_efficiency,
            "piston supercharger efficiency",
        )?;
        if self.supercharger_efficiency == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "piston supercharger efficiency must be finite in (0, 1]".into(),
            ));
        }
        if !self.volumetric_efficiency.is_finite()
            || !(0.0..=1.5).contains(&self.volumetric_efficiency)
            || self.volumetric_efficiency == 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "piston volumetric efficiency must be finite in (0, 1.5]".into(),
            ));
        }
        require_unit_interval(self.combustion_efficiency, "piston combustion efficiency")?;
        require_positive(self.idle_rpm, "piston idle RPM")?;
        require_positive(self.redline_rpm, "piston redline RPM")?;
        if self.redline_rpm <= self.idle_rpm {
            return Err(PropulsionError::InvalidSpec(
                "piston redline RPM must exceed idle RPM".into(),
            ));
        }
        require_non_negative(self.friction_mep_pa, "piston friction MEP")?;
        require_unit_interval(self.wall_heat_fraction, "piston wall heat fraction")?;
        require_non_negative(self.cooling_capacity_w, "piston cooling capacity")?;
        require_positive(self.dry_mass_kg, "piston engine dry mass")?;
        Ok(())
    }
}

impl CompiledElectricMotor {
    /// Validate baked values the torque/power envelope divides by
    /// (mirrors [`ElectricMotorSpec::compile`]).
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.rated_power_w, "motor rated power")?;
        require_positive(self.peak_torque_nm, "motor peak torque")?;
        require_positive(self.maximum_rpm, "motor maximum RPM")?;
        if !self.efficiency.is_finite()
            || !(0.0..=1.0).contains(&self.efficiency)
            || self.efficiency == 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "motor efficiency must be finite in (0, 1]".into(),
            ));
        }
        require_non_negative(self.cooling_capacity_w, "motor cooling capacity")?;
        require_positive(self.dry_mass_kg, "motor dry mass")?;
        Ok(())
    }
}

impl CompiledShaftPowerSource {
    fn validate(&self) -> Result<(), PropulsionError> {
        match self {
            Self::Piston(engine) => engine.validate(),
            Self::Electric(motor) => motor.validate(),
        }
    }
}

impl CompiledPropellerDrive {
    /// Validate baked drive data (`reduction_ratio` is a divisor in every
    /// operating point).
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.reduction_ratio, "shaft-power reduction ratio")?;
        self.propeller.validate()?;
        self.source.validate()?;
        require_non_negative(self.dry_mass_kg, "propeller-drive dry mass")?;
        Ok(())
    }
}

impl CompiledTurbopropDrive {
    /// Validate baked drive data (`reduction_ratio` and the air-core
    /// `spool_tau_s`/`shaft_reference_power_w` are divisors on the
    /// advance path).
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.shaft_rpm_at_full_spool, "turboprop shaft RPM")?;
        require_positive(self.reduction_ratio, "turboprop reduction ratio")?;
        require_non_negative(self.power_turbine_mass_kg, "turboprop power-turbine mass")?;
        require_non_negative(self.dry_mass_kg, "turboprop dry mass")?;
        self.propeller.validate()?;
        require_positive(self.air.intake_area_m2, "turboprop intake area")?;
        require_positive(self.air.spool_tau_s, "turboprop spool tau")?;
        require_non_negative(self.air.dry_mass_kg, "turboprop core dry mass")?;
        if self.air.cycle.has_shaft() {
            require_positive(
                self.air.shaft_reference_power_w,
                "turboprop shaft reference power",
            )?;
        }
        let heat_fraction = self.air.shaft.power_turbine_heat_fraction;
        if heat_fraction <= 0.0 || !heat_fraction.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "turboprop requires a positive power-turbine heat fraction".into(),
            ));
        }
        self.air.shaft.validate(self.air.cycle)?;
        Ok(())
    }
}

impl PropellerDriveMount {
    /// Validate the mount station plus the nested compiled drive (tampered
    /// baked JSON fails closed here, before formulas see it).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "propeller-drive mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "propeller-drive mount position must be finite".into(),
            ));
        }
        if self.thrust_axis_body.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "propeller-drive thrust axis must be finite".into(),
            ));
        }
        let axis = self.thrust_axis_body;
        let length = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if !length.is_finite() || (length - 1.0).abs() > 1.0e-9 {
            return Err(PropulsionError::InvalidSpec(
                "propeller-drive thrust axis must be unit length".into(),
            ));
        }
        self.drive.validate()?;
        Ok(())
    }

    pub fn operating_point(
        &self,
        condition: &FlightCondition,
        command: PropellerDriveCommand,
    ) -> Result<PropDrivePoint, PropulsionError> {
        self.validate()?;
        self.drive.operating_point(condition, command)
    }
}

impl TurbopropDriveSpec {
    pub fn compile(self) -> Result<CompiledTurbopropDrive, PropulsionError> {
        require_positive(self.shaft_rpm_at_full_spool, "turboprop shaft RPM")?;
        require_positive(self.reduction_ratio, "turboprop reduction ratio")?;
        require_non_negative(self.power_turbine_mass_kg, "power-turbine mass")?;
        if self.air.shaft.power_turbine_heat_fraction <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "turboprop requires a positive power-turbine heat fraction".into(),
            ));
        }
        let air = self.air.compile()?;
        let propeller = self.propeller.compile()?;
        let dry_mass_kg = air.dry_mass_kg + propeller.dry_mass_kg + self.power_turbine_mass_kg;
        if !dry_mass_kg.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "turboprop installation mass is non-finite".into(),
            ));
        }
        Ok(CompiledTurbopropDrive {
            air,
            propeller,
            shaft_rpm_at_full_spool: self.shaft_rpm_at_full_spool,
            reduction_ratio: self.reduction_ratio,
            power_turbine_mass_kg: self.power_turbine_mass_kg,
            dry_mass_kg,
        })
    }
}

impl TurbopropCommand {
    pub fn running(drive: &CompiledTurbopropDrive) -> Self {
        Self {
            shaft_state: JetShaftState::running(&drive.air),
            shaft: ShaftCommand {
                throttle: 1.0,
                starter_engaged: false,
                generator_load_w: 0.0,
            },
            dt_s: 0.0,
            propeller_power_w: 0.0,
        }
    }

    pub fn cold(drive: &CompiledTurbopropDrive) -> Self {
        Self {
            shaft_state: JetShaftState::cold(&drive.air),
            ..Self::running(drive)
        }
    }

    pub fn with_state(&self, shaft_state: JetShaftState) -> Self {
        Self {
            shaft_state,
            ..*self
        }
    }
}

impl CompiledTurbopropDrive {
    /// Advance the core shaft under propeller takeoff load and evaluate the
    /// coupled core-nozzle/propeller thrust for this physics step. The load
    /// must fit the authored power-turbine heat allocation and the current
    /// exhaust enthalpy; either limit fails closed rather than inventing
    /// shaft output.
    pub fn advance(
        &self,
        condition: &FlightCondition,
        command: &TurbopropCommand,
    ) -> Result<(JetShaftState, TurbopropOperatingPoint), PropulsionError> {
        condition.validate()?;
        if !command.propeller_power_w.is_finite() || command.propeller_power_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "turboprop power request must be finite and >= 0".into(),
            ));
        }
        if command.propeller_power_w > 0.0 && !command.shaft_state.lit {
            return Err(PropulsionError::InvalidCommand(
                "turboprop power takeoff requires an already-lit core".into(),
            ));
        }
        let (air, balance) = self.air.operating_point_at_spool_loaded(
            condition,
            command.shaft.throttle,
            command.shaft_state.spool_n,
            command.shaft.throttle > 0.0,
            command.propeller_power_w,
        )?;
        let (shaft_state, shaft_telemetry) = advance_jet_shaft_loaded(
            &self.air,
            command.shaft_state,
            &command.shaft,
            condition,
            command.dt_s,
            command.propeller_power_w,
        )?;
        let propeller_rpm =
            command.shaft_state.spool_n * self.shaft_rpm_at_full_spool / self.reduction_ratio;
        let propeller =
            self.propeller
                .operating_point(command.propeller_power_w, propeller_rpm, condition)?;
        let total_thrust_n = air.thrust_n + propeller.thrust_n;
        if !total_thrust_n.is_finite() {
            return Err(PropulsionError::InvalidCommand(
                "turboprop evaluation produced non-finite thrust".into(),
            ));
        }
        Ok((
            shaft_state,
            TurbopropOperatingPoint {
                air,
                propeller,
                total_thrust_n,
                propeller_rpm,
                shaft_telemetry,
                power_takeoff_capacity_w: balance.power_takeoff_capacity_w,
            },
        ))
    }
}

impl TurbopropMount {
    /// Validate the mount station plus the nested compiled drive (tampered
    /// baked JSON fails closed here, before formulas see it).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "turboprop mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "turboprop mount position must be finite".into(),
            ));
        }
        if self.thrust_axis_body.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "turboprop thrust axis must be finite".into(),
            ));
        }
        let axis = self.thrust_axis_body;
        let length = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if !length.is_finite() || (length - 1.0).abs() > 1.0e-9 {
            return Err(PropulsionError::InvalidSpec(
                "turboprop thrust axis must be unit length".into(),
            ));
        }
        self.drive.validate()?;
        Ok(())
    }

    pub fn advance(
        &self,
        condition: &FlightCondition,
        command: &TurbopropCommand,
    ) -> Result<(JetShaftState, TurbopropOperatingPoint), PropulsionError> {
        self.validate()?;
        self.drive.advance(condition, command)
    }
}

/// Convert thrust and fuel flow to a conventional effective propulsive Isp.
pub fn effective_propulsive_isp_s(thrust_n: f64, fuel_flow_kg_s: f64) -> f64 {
    if fuel_flow_kg_s > 0.0 {
        thrust_n / (fuel_flow_kg_s * STANDARD_GRAVITY_MPS2)
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::atmosphere::AtmosphereConfig;

    fn earth_condition(airspeed_mps: f64) -> FlightCondition {
        let sample = AtmosphereConfig::default()
            .sample(0.0)
            .expect("sea-level atmosphere");
        super::super::flight_condition(&sample, airspeed_mps).expect("flight condition")
    }

    fn thessa_condition(airspeed_mps: f64) -> FlightCondition {
        let sample = AtmosphereConfig::default()
            .with_composition(crate::atmosphere::AtmosphereComposition::thessa_air())
            .sample(0.0)
            .expect("Thessa-like atmosphere");
        super::super::flight_condition(&sample, airspeed_mps).expect("flight condition")
    }

    fn ideal_propeller() -> CompiledPropeller {
        PropellerSpec {
            gearbox_efficiency: 1.0,
            ..PropellerSpec::default()
        }
        .compile()
        .expect("valid propeller")
    }

    fn turboprop_drive() -> CompiledTurbopropDrive {
        TurbopropDriveSpec {
            air: crate::AirbreathingSpec {
                name: "test-turboprop-core".into(),
                cycle: crate::AirCycle::Turbojet,
                fuel: JetFuel::Kerosene,
                intake_area_m2: 0.8,
                intake: crate::IntakeKind::Pitot,
                compressor_ratio: 8.0,
                bypass_ratio: 0.0,
                fan_pressure_ratio: 1.0,
                turbine_inlet_temp_k: 1_400.0,
                afterburner: false,
                reheat_temp_k: 0.0,
                turbine_material: crate::ChamberMaterial::nickel_superalloy(),
                spool_tau_s: 4.0,
                shaft: crate::ShaftSpec {
                    power_turbine_heat_fraction: 0.15,
                    ..crate::ShaftSpec::default()
                },
            },
            propeller: PropellerSpec {
                diameter_m: 2.4,
                ..PropellerSpec::default()
            },
            shaft_rpm_at_full_spool: 12_000.0,
            reduction_ratio: 6.0,
            power_turbine_mass_kg: 40.0,
        }
        .compile()
        .expect("valid turboprop")
    }

    #[test]
    fn actuator_disk_static_thrust_matches_momentum_special_case() {
        let propeller = ideal_propeller();
        let power_w = 80_000.0;
        let point = propeller
            .operating_point(power_w, 2_400.0, &earth_condition(0.0))
            .expect("static disk point");
        let density = earth_condition(0.0).density_kg_m3();
        let expected = (2.0 * density * propeller.disk_area_m2 * power_w.powi(2)).cbrt();
        assert!((point.thrust_n - expected).abs() / expected < 2.0e-14);
        assert_eq!(point.efficiency, 0.0);
        assert!(point.induced_velocity_mps > 0.0);
    }

    #[test]
    fn actuator_disk_efficiency_and_thrust_follow_momentum_theory() {
        let propeller = ideal_propeller();
        let condition = earth_condition(90.0);
        let power_w = 120_000.0;
        let point = propeller
            .operating_point(power_w, 1_800.0, &condition)
            .expect("forward-flight disk point");
        let expected_efficiency =
            condition.airspeed_mps / (condition.airspeed_mps + point.induced_velocity_mps);
        assert!((point.efficiency - expected_efficiency).abs() < 2.0e-14);
        assert!(point.thrust_n > 0.0);
        assert!(point.efficiency > 0.0 && point.efficiency < 1.0);
    }

    #[test]
    fn propeller_rejects_positive_power_at_zero_rpm_and_bad_geometry() {
        let propeller = ideal_propeller();
        assert!(
            propeller
                .operating_point(10.0, 0.0, &earth_condition(0.0))
                .is_err()
        );
        assert!(
            PropellerSpec {
                hub_diameter_m: 2.0,
                ..PropellerSpec::default()
            }
            .compile()
            .is_err()
        );
    }

    #[test]
    fn piston_otto_cycle_matches_air_standard_efficiency_and_oxygen_gate() {
        let engine = PistonEngineSpec {
            friction_mep_pa: 0.0,
            wall_heat_fraction: 0.0,
            cooling_capacity_w: 1.0e9,
            combustion_efficiency: 1.0,
            boost_pressure_ratio: 1.0,
            ..PistonEngineSpec::default()
        }
        .compile()
        .expect("valid piston");
        let point = engine
            .operating_point(&thessa_condition(0.0), 2_400.0, 1.0)
            .expect("Otto point");
        let gamma = thessa_condition(0.0).composition.mean_heat_capacity_ratio();
        let expected_efficiency = 1.0 - engine.compression_ratio.powf(1.0 - gamma);
        assert!((point.thermal_efficiency - expected_efficiency).abs() < 1.0e-12);
        assert!(point.shaft_power_w > 0.0);
        assert!(point.fuel_flow_kg_s > 0.0);
        assert!(!point.oxygen_limited);

        let anoxic = crate::atmosphere::AtmosphereComposition::anoxic();
        let sample = AtmosphereConfig::default()
            .with_composition(anoxic)
            .sample(0.0)
            .expect("anoxic atmosphere");
        let condition = super::super::flight_condition(&sample, 0.0).expect("condition");
        let point = engine
            .operating_point(&condition, 2_400.0, 1.0)
            .expect("oxygen-limited piston");
        assert!(point.oxygen_limited);
        assert_eq!(point.shaft_power_w, 0.0);
        assert_eq!(point.fuel_flow_kg_s, 0.0);
    }

    #[test]
    fn piston_cooling_capacity_caps_output_and_reports_the_limit() {
        let engine = PistonEngineSpec {
            cooling_capacity_w: 15_000.0,
            ..PistonEngineSpec::default()
        }
        .compile()
        .expect("valid piston");
        let point = engine
            .operating_point(&earth_condition(0.0), 2_400.0, 1.0)
            .expect("cooled piston");
        assert!(point.cooling_limited);
        assert!(!point.stalled);
        assert!(point.cooling_load_w <= engine.cooling_capacity_w + 1.0e-7);
        assert!(point.shaft_power_w > 0.0);
    }

    #[test]
    fn piston_overheats_and_stalls_when_cooling_cannot_carry_friction_heat() {
        let engine = PistonEngineSpec {
            cooling_capacity_w: 1_000.0,
            ..PistonEngineSpec::default()
        }
        .compile()
        .expect("valid piston");
        // Friction heat alone at this RPM already exceeds the installation,
        // so no throttle keeps the engine below its cooling capacity.
        let friction_w = engine.friction_mep_pa * engine.displacement_m3 * 2_400.0 / 60.0
            * FOUR_STROKE_CYCLES_PER_REV;
        assert!(friction_w > engine.cooling_capacity_w);

        let point = engine
            .operating_point(&earth_condition(0.0), 2_400.0, 1.0)
            .expect("overheated piston");
        assert!(point.stalled);
        assert!(point.cooling_limited);
        assert!(point.cooling_load_w > engine.cooling_capacity_w);
        assert_eq!(point.shaft_power_w, 0.0);
        assert_eq!(point.fuel_flow_kg_s, 0.0);
    }

    #[test]
    fn piston_cycle_traps_ram_charge_and_senses_flight_speed() {
        let engine = PistonEngineSpec {
            cooling_capacity_w: 1.0e9,
            ..PistonEngineSpec::default()
        }
        .compile()
        .expect("valid piston");
        let still = earth_condition(0.0);
        let mach = 0.44;
        // Sample-based Mach so the condition carries exactly this Mach.
        let sample = AtmosphereConfig::default()
            .sample(0.0)
            .expect("sea-level atmosphere");
        let fast = super::super::flight_condition(&sample, mach * sample.speed_of_sound_mps)
            .expect("ram condition");
        assert!((fast.mach - mach).abs() < 1.0e-12);

        // Independent ram-density reference: isentropic stagnation at
        // Mach 0.44 packs the intake charge ~10% above the static column.
        let gamma = still.composition.mean_heat_capacity_ratio();
        let gas_constant = still.composition.gas_constant_j_kg_k();
        let ram_temp_ratio = 1.0 + (gamma - 1.0) / 2.0 * mach * mach;
        let ram_temp_k = still.ambient_temp_k * ram_temp_ratio;
        let ram_pressure_pa = still.ambient_pa * ram_temp_ratio.powf(gamma / (gamma - 1.0));
        let ram_density = ram_pressure_pa / (gas_constant * ram_temp_k);
        let static_density = still.density_kg_m3();
        assert!(ram_density > static_density);
        assert!(ram_density / static_density > 1.09);

        let still_point = engine
            .operating_point(&still, 2_400.0, 1.0)
            .expect("static piston point");
        let fast_point = engine
            .operating_point(&fast, 2_400.0, 1.0)
            .expect("ram piston point");
        assert!(fast_point.fuel_flow_kg_s > still_point.fuel_flow_kg_s);
        assert!(fast_point.shaft_power_w > still_point.shaft_power_w);
        // Boost pressure ratio is 1, so trapped charge density is exactly
        // ram density and fuel flow (proportional to trapped air) scales
        // with the same ratio.
        let fuel_ratio = fast_point.fuel_flow_kg_s / still_point.fuel_flow_kg_s;
        assert!((fuel_ratio - ram_density / static_density).abs() < 1.0e-9);
    }

    #[test]
    fn electric_motor_has_constant_torque_then_power_and_thermal_limit() {
        let motor = ElectricMotorSpec {
            rated_power_w: 100_000.0,
            peak_torque_nm: 500.0,
            cooling_capacity_w: 1.0e9,
            ..ElectricMotorSpec::default()
        }
        .compile()
        .expect("valid motor");
        let low = motor.operating_point(600.0, 1.0).expect("low speed");
        let high = motor.operating_point(6_000.0, 1.0).expect("high speed");
        assert!((low.torque_nm - 500.0).abs() < 1.0e-10);
        assert!((high.mechanical_power_w - 100_000.0).abs() < 1.0e-8);
        assert!(
            (high.electrical_power_w * motor.efficiency - high.mechanical_power_w).abs() < 1.0e-8
        );

        let cooled = ElectricMotorSpec {
            cooling_capacity_w: 1_000.0,
            ..ElectricMotorSpec::default()
        }
        .compile()
        .expect("valid thermally limited motor");
        let point = cooled.operating_point(6_000.0, 1.0).expect("limited point");
        assert!(point.thermal_limited);
        assert!(point.waste_heat_w <= cooled.cooling_capacity_w + 1.0e-9);
    }

    #[test]
    fn propeller_drive_couples_motor_power_through_gear_to_thrust_and_bus_draw() {
        let drive = PropellerDriveSpec {
            propeller: PropellerSpec {
                gearbox_efficiency: 0.95,
                ..PropellerSpec::default()
            },
            source: ShaftPowerSourceSpec::Electric(ElectricMotorSpec {
                rated_power_w: 80_000.0,
                peak_torque_nm: 500.0,
                maximum_rpm: 8_000.0,
                efficiency: 0.9,
                cooling_capacity_w: 100_000.0,
                dry_mass_kg: 24.0,
            }),
            reduction_ratio: 2.0,
        }
        .compile()
        .expect("coupled drive");
        let point = drive
            .operating_point(
                &earth_condition(70.0),
                PropellerDriveCommand {
                    throttle: 1.0,
                    source_rpm: 4_000.0,
                },
            )
            .expect("drive operating point");
        assert_eq!(point.propeller.rpm, 2_000.0);
        assert!(point.propeller.thrust_n > 0.0);
        assert!((point.propeller.disk_power_w - 76_000.0).abs() < 1.0e-8);
        assert!((point.electrical_power_w - point.source_shaft_power_w / 0.9).abs() < 1.0e-8);
        assert_eq!(point.fuel_flow_kg_s, 0.0);
        assert!(!point.source_speed_limited);
    }

    #[test]
    fn propeller_drive_analyzer_reports_grid_and_declared_vacuum() {
        let drive = PropellerDriveSpec {
            propeller: PropellerSpec::default(),
            source: ShaftPowerSourceSpec::Electric(ElectricMotorSpec::default()),
            reduction_ratio: 2.0,
        }
        .compile()
        .expect("drive");
        let atmosphere = AtmosphereConfig::default();
        let rows = analyze_propeller_drive(
            &drive,
            &atmosphere,
            &[0.0, atmosphere.top_altitude_m() + 10_000.0],
            &[0.0, 50.0],
            1.0,
            6_000.0,
        )
        .expect("analyzer grid");
        assert_eq!(rows.len(), 4);
        assert!(rows[0].thrust_n > 0.0);
        assert!(rows[1].thrust_n > 0.0);
        assert_eq!(rows[2].thrust_n, 0.0);
        assert!(rows[2].density_limited);
        assert_eq!(rows[2].electrical_power_w, 0.0);
    }

    #[test]
    fn turboprop_takeoff_extracts_gas_energy_and_loads_the_live_shaft() {
        let drive = turboprop_drive();
        let condition = earth_condition(0.0);
        let (unloaded_air, unloaded_balance) = drive
            .air
            .operating_point_at_spool_loaded(&condition, 1.0, 1.0, true, 0.0)
            .expect("unloaded core point");
        let available_w = unloaded_balance.power_takeoff_capacity_w;
        assert!(available_w > 100_000.0);
        let requested_w = 0.25 * available_w;

        let mut command = TurbopropCommand::running(&drive);
        command.dt_s = 0.1;
        command.propeller_power_w = requested_w;
        let (next_state, loaded) = drive
            .advance(&condition, &command)
            .expect("loaded turboprop point");
        assert!(next_state.lit);
        assert_eq!(loaded.propeller.rpm, 2_000.0);
        assert!(loaded.propeller.thrust_n > 0.0);
        assert!(loaded.total_thrust_n > loaded.propeller.thrust_n);
        assert!(loaded.air.exhaust_temp_k < unloaded_air.exhaust_temp_k);
        assert!(loaded.air.thrust_n < unloaded_air.thrust_n);
        assert!(
            (loaded.shaft_telemetry.demand_w - (unloaded_balance.demand_w + requested_w)).abs()
                < 1e-6
        );
        assert!(
            (loaded.shaft_telemetry.capacity_w - (unloaded_balance.capacity_w + requested_w)).abs()
                < 1e-6
        );
        assert!(loaded.shaft_telemetry.net_w.abs() < 1e-6);

        let mut overdrawn = command;
        overdrawn.propeller_power_w = available_w * 1.01;
        assert!(drive.advance(&condition, &overdrawn).is_err());
    }

    #[test]
    fn turboprop_requires_authored_takeoff_heat_and_analyzer_scales_load_by_capacity() {
        assert!(
            TurbopropDriveSpec {
                air: crate::AirbreathingSpec {
                    name: "no-power-turbine".into(),
                    cycle: crate::AirCycle::Turbojet,
                    fuel: JetFuel::Kerosene,
                    intake_area_m2: 0.8,
                    intake: crate::IntakeKind::Pitot,
                    compressor_ratio: 8.0,
                    bypass_ratio: 0.0,
                    fan_pressure_ratio: 1.0,
                    turbine_inlet_temp_k: 1_400.0,
                    afterburner: false,
                    reheat_temp_k: 0.0,
                    turbine_material: crate::ChamberMaterial::nickel_superalloy(),
                    spool_tau_s: 4.0,
                    shaft: crate::ShaftSpec::default(),
                },
                propeller: PropellerSpec::default(),
                shaft_rpm_at_full_spool: 12_000.0,
                reduction_ratio: 6.0,
                power_turbine_mass_kg: 40.0,
            }
            .compile()
            .is_err()
        );

        let drive = turboprop_drive();
        let atmosphere = AtmosphereConfig::default();
        let rows = analyze_turboprop_drive(
            &drive,
            &atmosphere,
            &[0.0, atmosphere.top_altitude_m() + 10_000.0],
            &[0.0, 70.0],
            1.0,
            0.25,
        )
        .expect("turboprop analyzer");
        assert_eq!(rows.len(), 4);
        assert!(rows[0].power_takeoff_w > 0.0);
        assert!(rows[0].propeller_thrust_n > 0.0);
        assert_eq!(rows[2].power_takeoff_w, 0.0);
        assert_eq!(rows[2].total_thrust_n, 0.0);
    }

    #[test]
    fn propeller_drive_mount_validate_rejects_tampered_baked_drive() {
        let drive = PropellerDriveSpec {
            propeller: PropellerSpec::default(),
            source: ShaftPowerSourceSpec::Piston(PistonEngineSpec::default()),
            reduction_ratio: 2.0,
        }
        .compile()
        .expect("drive");
        let mount = PropellerDriveMount {
            name: "baked-prop".into(),
            drive,
            position_body_m: [0.0, 1.0, 0.0],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        mount.validate().expect("clean bake validates");

        // Zero reduction ratio is a divisor in every drive point.
        let mut no_ratio = mount.clone();
        no_ratio.drive.reduction_ratio = 0.0;
        assert!(matches!(
            no_ratio.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));

        // Zero supercharger efficiency is a divisor in the piston cycle.
        let mut no_boost = mount.clone();
        match &mut no_boost.drive.source {
            CompiledShaftPowerSource::Piston(engine) => {
                engine.supercharger_efficiency = 0.0;
            }
            CompiledShaftPowerSource::Electric(_) => panic!("piston source expected"),
        }
        assert!(matches!(
            no_boost.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));

        // A NaN motor efficiency used to sail through to the formulas and
        // come back as an `Ok` point carrying NaN electrical power.
        let mut motor = ElectricMotorSpec::default().compile().expect("motor");
        motor.efficiency = f64::NAN;
        let mut nan_motor = mount.clone();
        nan_motor.drive.source = CompiledShaftPowerSource::Electric(motor);
        assert!(matches!(
            nan_motor.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        let error = nan_motor
            .operating_point(
                &earth_condition(0.0),
                PropellerDriveCommand {
                    throttle: 1.0,
                    source_rpm: 6_000.0,
                },
            )
            .expect_err("tampered mount must fail closed, not return NaN");
        assert!(matches!(error, PropulsionError::InvalidSpec(_)));

        mount.validate().expect("untampered bake still validates");
    }

    #[test]
    fn turboprop_mount_validate_rejects_tampered_baked_drive() {
        let mount = TurbopropMount {
            name: "baked-turboprop".into(),
            drive: turboprop_drive(),
            position_body_m: [0.0, 1.0, 0.0],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        mount.validate().expect("clean bake validates");

        let mut no_ratio = mount.clone();
        no_ratio.drive.reduction_ratio = f64::NAN;
        assert!(matches!(
            no_ratio.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        // The evaluation path routes through the same validation.
        let error = no_ratio
            .advance(
                &earth_condition(0.0),
                &TurbopropCommand::running(&no_ratio.drive),
            )
            .expect_err("tampered turboprop must fail closed");
        assert!(matches!(error, PropulsionError::InvalidSpec(_)));

        let mut no_area = mount.clone();
        no_area.drive.propeller.disk_area_m2 = 0.0;
        assert!(matches!(
            no_area.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));

        let mut no_spool = mount.clone();
        no_spool.drive.air.spool_tau_s = 0.0;
        assert!(matches!(
            no_spool.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));

        let mut no_takeoff = mount.clone();
        no_takeoff.drive.air.shaft.power_turbine_heat_fraction = 0.0;
        assert!(matches!(
            no_takeoff.validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));

        mount.validate().expect("untampered bake still validates");
    }
}
