//! Ideal vehicle electrical bus with batteries, ultracapacitors, fission
//! power, and solar cells.
//!
//! Parts declare a load and priority; a single vehicle-wide bus allocates
//! available power. There are no authored wire graphs or per-part connections.

use glam::{DMat3, DQuat, DVec2, DVec3};
use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

#[derive(Debug, Clone, PartialEq)]
pub enum ElectricalPowerError {
    InvalidSpec(String),
    InvalidCommand(String),
    InvalidState(String),
}

impl fmt::Display for ElectricalPowerError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpec(message) => write!(formatter, "invalid power-system spec: {message}"),
            Self::InvalidCommand(message) => write!(formatter, "invalid power command: {message}"),
            Self::InvalidState(message) => write!(formatter, "invalid power state: {message}"),
        }
    }
}

impl Error for ElectricalPowerError {}

/// Power-shedding order on the shared, idealized vehicle bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PowerPriority {
    LifeSupport,
    FlightControl,
    Propulsion,
    Utility,
}

impl PowerPriority {
    const ORDERED: [Self; 4] = [
        Self::LifeSupport,
        Self::FlightControl,
        Self::Propulsion,
        Self::Utility,
    ];
}

/// A powered part's rated load. Runtime commands request a fraction of this
/// rating; allocations are reported under this part name without any wiring.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PowerConsumerSpec {
    pub name: String,
    pub rated_power_w: f64,
    pub priority: PowerPriority,
}

impl PowerConsumerSpec {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        if self.name.trim().is_empty() {
            return Err(ElectricalPowerError::InvalidSpec(
                "power consumer name must not be empty".into(),
            ));
        }
        require_positive(self.rated_power_w, "consumer rated power")
    }
}

/// A rechargeable electrical store.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BatterySpec {
    pub name: String,
    /// Usable stored energy at full charge (J).
    pub capacity_j: f64,
    pub initial_charge_fraction: f64,
    pub maximum_charge_power_w: f64,
    pub maximum_discharge_power_w: f64,
    pub charge_efficiency: f64,
    pub discharge_efficiency: f64,
    /// Pack-level specific energy used to derive installed mass (J/kg).
    pub specific_energy_j_kg: f64,
    /// Pack dimensions along craft body axes (m), for cuboid inertia.
    pub dimensions_body_m: DVec3,
    pub position_body_m: DVec3,
}

impl BatterySpec {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        if self.name.trim().is_empty() {
            return Err(ElectricalPowerError::InvalidSpec(
                "battery name must not be empty".into(),
            ));
        }
        for (value, label) in [
            (self.capacity_j, "battery capacity"),
            (self.maximum_charge_power_w, "battery maximum charge power"),
            (
                self.maximum_discharge_power_w,
                "battery maximum discharge power",
            ),
            (self.specific_energy_j_kg, "battery specific energy"),
        ] {
            require_positive(value, label)?;
        }
        require_unit_interval(self.initial_charge_fraction, "initial battery charge")?;
        require_efficiency(self.charge_efficiency, "battery charge efficiency")?;
        require_efficiency(self.discharge_efficiency, "battery discharge efficiency")?;
        validate_positive_vector(self.dimensions_body_m, "battery dimensions")?;
        validate_finite_vector(self.position_body_m, "battery position")
    }

    pub fn mass_kg(&self) -> f64 {
        self.capacity_j / self.specific_energy_j_kg
    }

    pub fn inertia_body_kg_m2(&self) -> DMat3 {
        cuboid_inertia(self.mass_kg(), self.dimensions_body_m)
    }
}

/// A rechargeable ultracapacitor (supercapacitor/ionistor) bank.
///
/// Same energy/power-bound bus model as [`BatterySpec`], kept as a separate
/// authoring type so high-power/low-energy buffers for pulsed loads are
/// explicit. Installed mass derives from capacity and specific energy;
/// voltage dynamics, leakage/self-discharge, and cycle ageing are future work.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UltracapacitorSpec {
    pub name: String,
    /// Usable stored energy at full charge (J).
    pub capacity_j: f64,
    pub initial_charge_fraction: f64,
    pub maximum_charge_power_w: f64,
    pub maximum_discharge_power_w: f64,
    pub charge_efficiency: f64,
    pub discharge_efficiency: f64,
    /// Bank-level specific energy used to derive installed mass (J/kg).
    pub specific_energy_j_kg: f64,
    /// Bank dimensions along craft body axes (m), for cuboid inertia.
    pub dimensions_body_m: DVec3,
    pub position_body_m: DVec3,
}

impl UltracapacitorSpec {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        if self.name.trim().is_empty() {
            return Err(ElectricalPowerError::InvalidSpec(
                "ultracapacitor name must not be empty".into(),
            ));
        }
        for (value, label) in [
            (self.capacity_j, "ultracapacitor capacity"),
            (
                self.maximum_charge_power_w,
                "ultracapacitor maximum charge power",
            ),
            (
                self.maximum_discharge_power_w,
                "ultracapacitor maximum discharge power",
            ),
            (self.specific_energy_j_kg, "ultracapacitor specific energy"),
        ] {
            require_positive(value, label)?;
        }
        require_unit_interval(
            self.initial_charge_fraction,
            "initial ultracapacitor charge",
        )?;
        require_efficiency(self.charge_efficiency, "ultracapacitor charge efficiency")?;
        require_efficiency(
            self.discharge_efficiency,
            "ultracapacitor discharge efficiency",
        )?;
        validate_positive_vector(self.dimensions_body_m, "ultracapacitor dimensions")?;
        validate_finite_vector(self.position_body_m, "ultracapacitor position")
    }

    pub fn mass_kg(&self) -> f64 {
        self.capacity_j / self.specific_energy_j_kg
    }

    pub fn inertia_body_kg_m2(&self) -> DMat3 {
        cuboid_inertia(self.mass_kg(), self.dimensions_body_m)
    }
}

/// Fixed arrays are permanently exposed. Foldable arrays change exposed
/// cell fraction at their authored deployment rate and consume bus power.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum SolarArrayDeployment {
    Fixed,
    Foldable {
        deployment_rate_per_s: f64,
        actuator_power_w: f64,
        initial_fraction: f64,
    },
}

/// Rectangular set of identical photovoltaic cells, parameterized by cell
/// dimensions, integer cell counts, efficiency, and array orientation.
///
/// `panel_u_axis_body`/`panel_v_axis_body` are the reference (zero-angle)
/// in-plane axes; [`SolarArraySpec::orientation_at_angle`] rotates them about
/// the tracking axis. The tracking axis is assumed to pass through the array
/// center, so rotation changes orientation but not position.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolarArraySpec {
    pub name: String,
    pub cell_count_x: u32,
    pub cell_count_y: u32,
    pub cell_size_x_m: f64,
    pub cell_size_y_m: f64,
    pub cell_efficiency: f64,
    pub cell_areal_density_kg_m2: f64,
    pub support_areal_density_kg_m2: f64,
    /// In-plane unit axes. Their cross product faces the active side.
    pub panel_u_axis_body: DVec3,
    pub panel_v_axis_body: DVec3,
    /// Center of the cell field in body coordinates (m).
    pub position_body_m: DVec3,
    pub deployment: SolarArrayDeployment,
    /// Sun-tracking drive. `Fixed` keeps the reference orientation forever.
    #[serde(default)]
    pub tracking: SolarArrayTracking,
}

/// Sun-tracking drive for a solar array.
///
/// `Fixed` arrays never rotate. `SingleAxis` rotates the whole cell sheet
/// about a body-frame axis (typical alpha-joint topology) at a bounded slew
/// rate, drawing authored actuator power while moving. In automatic mode the
/// drive slews toward the angle that maximizes instantaneous incident power
/// over all supplied stellar sources; a manual angle target overrides it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum SolarArrayTracking {
    #[default]
    Fixed,
    SingleAxis {
        /// Unit rotation axis in body coordinates.
        rotation_axis_body: DVec3,
        minimum_angle_rad: f64,
        maximum_angle_rad: f64,
        slew_rate_rad_s: f64,
        actuator_power_w: f64,
        initial_angle_rad: f64,
    },
}

impl SolarArrayTracking {
    fn validate(&self, array_name: &str) -> Result<(), ElectricalPowerError> {
        let SolarArrayTracking::SingleAxis {
            rotation_axis_body,
            minimum_angle_rad,
            maximum_angle_rad,
            slew_rate_rad_s,
            actuator_power_w,
            initial_angle_rad,
        } = self
        else {
            return Ok(());
        };
        validate_unit_vector(*rotation_axis_body, "solar tracking axis")?;
        for (value, label) in [
            (*minimum_angle_rad, "solar tracking minimum angle"),
            (*maximum_angle_rad, "solar tracking maximum angle"),
            (*initial_angle_rad, "initial solar tracking angle"),
        ] {
            if !value.is_finite() {
                return Err(ElectricalPowerError::InvalidSpec(format!(
                    "solar array '{array_name}' {label} must be finite"
                )));
            }
        }
        if minimum_angle_rad > maximum_angle_rad {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "solar array '{array_name}' tracking limits must satisfy min <= max"
            )));
        }
        if !(*minimum_angle_rad..=*maximum_angle_rad).contains(initial_angle_rad) {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "solar array '{array_name}' initial tracking angle must lie within its limits"
            )));
        }
        require_positive(*slew_rate_rad_s, "solar tracking slew rate")?;
        require_positive(*actuator_power_w, "solar tracking actuator power")?;
        Ok(())
    }

    fn initial_angle_rad(&self) -> f64 {
        match self {
            SolarArrayTracking::Fixed => 0.0,
            SolarArrayTracking::SingleAxis {
                initial_angle_rad, ..
            } => *initial_angle_rad,
        }
    }
}

impl SolarArraySpec {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        if self.name.trim().is_empty() {
            return Err(ElectricalPowerError::InvalidSpec(
                "solar-array name must not be empty".into(),
            ));
        }
        if self.cell_count_x == 0 || self.cell_count_y == 0 {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "solar array '{}' must contain at least one cell on each axis",
                self.name
            )));
        }
        for (value, label) in [
            (self.cell_size_x_m, "solar cell x dimension"),
            (self.cell_size_y_m, "solar cell y dimension"),
        ] {
            require_positive(value, label)?;
        }
        require_efficiency(self.cell_efficiency, "solar cell efficiency")?;
        require_non_negative(self.cell_areal_density_kg_m2, "solar cell areal density")?;
        require_non_negative(
            self.support_areal_density_kg_m2,
            "solar support areal density",
        )?;
        validate_unit_vector(self.panel_u_axis_body, "solar array u axis")?;
        validate_unit_vector(self.panel_v_axis_body, "solar array v axis")?;
        if self.panel_u_axis_body.dot(self.panel_v_axis_body).abs() > 1.0e-9 {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "solar array '{}' panel axes must be perpendicular",
                self.name
            )));
        }
        validate_finite_vector(self.position_body_m, "solar array position")?;
        self.tracking.validate(&self.name)?;
        // A tracking axis parallel to the panel normal cannot change
        // incidence; reject the degenerate authoring instead of burning
        // actuator power for no effect.
        if let SolarArrayTracking::SingleAxis {
            rotation_axis_body, ..
        } = self.tracking
            && rotation_axis_body
                .dot(self.panel_u_axis_body.cross(self.panel_v_axis_body))
                .abs()
                > 1.0 - 1.0e-6
        {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "solar array '{}' tracking axis must not be parallel to the panel normal",
                self.name
            )));
        }
        match self.deployment {
            SolarArrayDeployment::Fixed => {}
            SolarArrayDeployment::Foldable {
                deployment_rate_per_s,
                actuator_power_w,
                initial_fraction,
            } => {
                require_positive(deployment_rate_per_s, "solar deployment rate")?;
                require_positive(actuator_power_w, "solar deployment actuator power")?;
                require_unit_interval(initial_fraction, "initial solar deployment")?;
            }
        }
        if !self.area_m2().is_finite() || !self.mass_kg().is_finite() {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "solar array '{}' dimensions overflow",
                self.name
            )));
        }
        Ok(())
    }

    pub fn area_m2(&self) -> f64 {
        f64::from(self.cell_count_x)
            * f64::from(self.cell_count_y)
            * self.cell_size_x_m
            * self.cell_size_y_m
    }

    pub fn width_m(&self) -> f64 {
        f64::from(self.cell_count_x) * self.cell_size_x_m
    }

    pub fn height_m(&self) -> f64 {
        f64::from(self.cell_count_y) * self.cell_size_y_m
    }

    pub fn normal_body(&self) -> DVec3 {
        self.panel_u_axis_body
            .cross(self.panel_v_axis_body)
            .normalize()
    }

    /// Reference-frame axes rotated about the tracking axis by `angle_rad`.
    /// Fixed arrays ignore the angle and return the authored axes.
    pub fn orientation_at_angle(&self, angle_rad: f64) -> (DVec3, DVec3, DVec3) {
        let SolarArrayTracking::SingleAxis {
            rotation_axis_body, ..
        } = self.tracking
        else {
            return (
                self.panel_u_axis_body,
                self.panel_v_axis_body,
                self.normal_body(),
            );
        };
        let rotation = DQuat::from_axis_angle(rotation_axis_body, angle_rad);
        let u = rotation * self.panel_u_axis_body;
        let v = rotation * self.panel_v_axis_body;
        (u, v, u.cross(v).normalize())
    }

    /// Active-side normal at the given tracking angle.
    pub fn normal_at_angle(&self, angle_rad: f64) -> DVec3 {
        self.orientation_at_angle(angle_rad).2
    }

    pub fn mass_kg(&self) -> f64 {
        self.area_m2() * (self.cell_areal_density_kg_m2 + self.support_areal_density_kg_m2)
    }

    /// Thin rectangular cell/support sheet inertia, rotated into craft axes.
    pub fn inertia_body_kg_m2(&self) -> DMat3 {
        let mass = self.mass_kg();
        let width = self.width_m();
        let height = self.height_m();
        let normal = self.normal_body();
        let rotation = DMat3::from_cols(self.panel_u_axis_body, self.panel_v_axis_body, normal);
        let local = DMat3::from_diagonal(DVec3::new(
            mass * height.powi(2) / 12.0,
            mass * width.powi(2) / 12.0,
            mass * (width.powi(2) + height.powi(2)) / 12.0,
        ));
        rotation * local * rotation.transpose()
    }

    fn initial_deployment_fraction(&self) -> f64 {
        match self.deployment {
            SolarArrayDeployment::Fixed => 1.0,
            SolarArrayDeployment::Foldable {
                initial_fraction, ..
            } => initial_fraction,
        }
    }
}

/// A parameterized fission-electric power source. Core thermal output is
/// limited by rated power and radiator capacity; fissile inventory limits
/// the integrated energy available over a time step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReactorSpec {
    pub name: String,
    pub rated_thermal_power_w: f64,
    pub electric_efficiency: f64,
    pub radiator_capacity_w: f64,
    pub initial_fuel_mass_kg: f64,
    /// Usable fission heat per kilogram of fissile inventory (J/kg).
    pub fuel_specific_energy_j_kg: f64,
    pub dry_mass_kg: f64,
    /// Reactor package dimensions along craft body axes (m).
    pub dimensions_body_m: DVec3,
    pub position_body_m: DVec3,
}

impl ReactorSpec {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        if self.name.trim().is_empty() {
            return Err(ElectricalPowerError::InvalidSpec(
                "reactor name must not be empty".into(),
            ));
        }
        require_positive(self.rated_thermal_power_w, "reactor thermal rating")?;
        require_efficiency(self.electric_efficiency, "reactor electric efficiency")?;
        require_non_negative(self.radiator_capacity_w, "reactor radiator capacity")?;
        require_non_negative(self.initial_fuel_mass_kg, "reactor initial fuel mass")?;
        require_positive(
            self.fuel_specific_energy_j_kg,
            "reactor fuel specific energy",
        )?;
        require_positive(self.dry_mass_kg, "reactor dry mass")?;
        validate_positive_vector(self.dimensions_body_m, "reactor dimensions")?;
        validate_finite_vector(self.position_body_m, "reactor position")
    }

    /// Rated electric output after conversion and steady heat-rejection limits.
    pub fn maximum_electrical_power_w(&self) -> f64 {
        let thermal_limit = self.rated_thermal_power_w * self.electric_efficiency;
        let radiator_limit = if self.electric_efficiency >= 1.0 {
            f64::INFINITY
        } else {
            self.radiator_capacity_w * self.electric_efficiency / (1.0 - self.electric_efficiency)
        };
        thermal_limit.min(radiator_limit)
    }

    pub fn installed_mass_kg(&self) -> f64 {
        self.dry_mass_kg + self.initial_fuel_mass_kg
    }

    pub fn inertia_body_kg_m2(&self) -> DMat3 {
        cuboid_inertia(self.installed_mass_kg(), self.dimensions_body_m)
    }
}

/// Hydrogen/oxygen fuel cell attached to the shared vehicle bus.
///
/// The cell consumes liquid-hydrogen inventory and LOX at the water-forming
/// mass ratio (8 kg O2 per kg H2); product water, electrical output, and
/// conversion heat are reported per step. Reactants remain in the common
/// installed-tank inventory rather than being duplicated in power state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FuelCellSpec {
    pub name: String,
    pub rated_electrical_power_w: f64,
    pub electrical_efficiency: f64,
    pub dry_mass_kg: f64,
    pub dimensions_body_m: DVec3,
    pub position_body_m: DVec3,
    /// Optional assembly engine/feed-port endpoint restricting its tank path.
    #[serde(default)]
    pub feed_port_name: Option<String>,
}

impl FuelCellSpec {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        if self.name.trim().is_empty() {
            return Err(ElectricalPowerError::InvalidSpec(
                "fuel-cell name must not be empty".into(),
            ));
        }
        if self
            .feed_port_name
            .as_ref()
            .is_some_and(|name| name.trim().is_empty())
        {
            return Err(ElectricalPowerError::InvalidSpec(format!(
                "fuel cell '{}' feed-port name must not be empty",
                self.name
            )));
        }
        require_positive(self.rated_electrical_power_w, "fuel-cell electrical rating")?;
        require_efficiency(
            self.electrical_efficiency,
            "fuel-cell electrical efficiency",
        )?;
        require_positive(self.dry_mass_kg, "fuel-cell dry mass")?;
        validate_positive_vector(self.dimensions_body_m, "fuel-cell dimensions")?;
        validate_finite_vector(self.position_body_m, "fuel-cell position")
    }

    pub fn inertia_body_kg_m2(&self) -> DMat3 {
        cuboid_inertia(self.dry_mass_kg, self.dimensions_body_m)
    }
}

/// Hydrogen lower heating value used by the idealized fuel-cell balance (J/kg).
pub const FUEL_CELL_HYDROGEN_LHV_J_KG: f64 = 120.0e6;
/// Stoichiometric oxygen/hydrogen mass ratio for forming water.
pub const FUEL_CELL_OXYGEN_HYDROGEN_RATIO: f64 = 8.0;

/// A geometric occluder dimming one stellar source: a planet, moon, or
/// another vehicle seen from the receiver.
///
/// The caller supplies body-frame geometry (directions from an authoritative
/// ephemeris/attitude plus angular radii from body size and range); the bus
/// derives the combined eclipse factor from disc overlap. Own-vehicle
/// self-shadowing (panel hidden behind its own hull) is future work.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SolarOccluder {
    /// Unit vector from the receiver toward the occluder center (body frame).
    pub direction_body: DVec3,
    /// Apparent angular radius of the occluder (rad, > 0).
    pub angular_radius_rad: f64,
}

impl SolarOccluder {
    pub fn new(
        direction_body: DVec3,
        angular_radius_rad: f64,
    ) -> Result<Self, ElectricalPowerError> {
        validate_unit_vector(direction_body, "occluder direction")?;
        if !angular_radius_rad.is_finite() || angular_radius_rad <= 0.0 {
            return Err(ElectricalPowerError::InvalidSpec(
                "occluder angular radius must be finite and positive".into(),
            ));
        }
        Ok(Self {
            direction_body,
            angular_radius_rad,
        })
    }

    /// Build an occluder from body radius and range using the guarded
    /// small-angle formula shared with the lighting pipeline.
    pub fn from_geometry(
        occluder_radius_m: f64,
        occluder_distance_m: f64,
        direction_body: DVec3,
    ) -> Result<Self, ElectricalPowerError> {
        Self::new(
            direction_body,
            apparent_angular_radius(occluder_radius_m, occluder_distance_m),
        )
    }
}

/// A stellar source direction with broadband irradiance in the vehicle body
/// frame. Occlusion is derived from supplied occluder geometry so power
/// shares the same authoritative ephemeris/eclipse result as lighting:
/// `effective = irradiance × visibility × (1 - covered stellar-disc fraction)`.
///
/// `visibility` carries any non-geometric dimming the caller already applies
/// (usually 1.0 when occluders are given explicitly). Supplying occluders
/// without the stellar angular radius fails closed: a point source cannot
/// produce a penumbra, so the step is rejected instead of silently ignoring
/// the shadow.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolarFluxSource {
    pub irradiance_w_m2: f64,
    pub direction_body: DVec3,
    pub visibility: f64,
    /// Apparent angular radius of the stellar disc (rad). Required when
    /// `occluders` is non-empty; 0.0 disables geometric occlusion.
    #[serde(default)]
    pub light_angular_radius_rad: f64,
    /// Planets, moons, and other vehicles shadowing this source.
    #[serde(default)]
    pub occluders: Vec<SolarOccluder>,
}

impl SolarFluxSource {
    pub fn new(
        irradiance_w_m2: f64,
        direction_body: DVec3,
        visibility: f64,
    ) -> Result<Self, ElectricalPowerError> {
        require_non_negative(irradiance_w_m2, "solar irradiance")?;
        require_unit_interval(visibility, "solar visibility")?;
        validate_unit_vector(direction_body, "solar direction")?;
        Ok(Self {
            irradiance_w_m2,
            direction_body,
            visibility,
            light_angular_radius_rad: 0.0,
            occluders: Vec::new(),
        })
    }

    /// Construct incident stellar flux from luminosity using the inverse-
    /// square law. Direction points from the receiver toward the star.
    pub fn from_luminosity(
        luminosity_w: f64,
        distance_m: f64,
        direction_body: DVec3,
        visibility: f64,
    ) -> Result<Self, ElectricalPowerError> {
        require_positive(luminosity_w, "stellar luminosity")?;
        require_positive(distance_m, "stellar distance")?;
        validate_unit_vector(direction_body, "solar direction")?;
        require_unit_interval(visibility, "solar visibility")?;
        let irradiance = luminosity_w / (4.0 * std::f64::consts::PI * distance_m.powi(2));
        if !irradiance.is_finite() {
            return Err(ElectricalPowerError::InvalidCommand(
                "stellar irradiance overflowed".into(),
            ));
        }
        Self::new(irradiance, direction_body, visibility)
    }

    /// Incident flux with explicit stellar disc and occluder geometry.
    /// `star_radius_m`/`distance_m` derive both irradiance and the stellar
    /// angular radius; each occluder contributes its disc overlap factor.
    pub fn from_luminosity_with_occluders(
        luminosity_w: f64,
        star_radius_m: f64,
        distance_m: f64,
        direction_body: DVec3,
        visibility: f64,
        occluders: Vec<SolarOccluder>,
    ) -> Result<Self, ElectricalPowerError> {
        let mut source =
            Self::from_luminosity(luminosity_w, distance_m, direction_body, visibility)?;
        source.light_angular_radius_rad = apparent_angular_radius(star_radius_m, distance_m);
        if !source.light_angular_radius_rad.is_finite() || source.light_angular_radius_rad < 0.0 {
            return Err(ElectricalPowerError::InvalidCommand(
                "stellar angular radius overflowed".into(),
            ));
        }
        source.occluders = occluders;
        source.validate_occluders()?;
        Ok(source)
    }

    /// Combined dimming: caller visibility times the visible fraction left
    /// after taking the union of all geometric occluder discs.
    pub fn effective_visibility(&self) -> f64 {
        (self.visibility.clamp(0.0, 1.0)
            * eclipse_visibility(
                self.direction_body,
                self.light_angular_radius_rad,
                &self.occluders,
            ))
        .clamp(0.0, 1.0)
    }

    /// Broadband irradiance actually reaching the receiver (W/m^2).
    pub fn effective_irradiance_w_m2(&self) -> f64 {
        (self.irradiance_w_m2 * self.effective_visibility()).max(0.0)
    }

    fn validate_occluders(&self) -> Result<(), ElectricalPowerError> {
        for occluder in &self.occluders {
            validate_unit_vector(occluder.direction_body, "occluder direction")?;
            if !occluder.angular_radius_rad.is_finite() || occluder.angular_radius_rad <= 0.0 {
                return Err(ElectricalPowerError::InvalidSpec(
                    "occluder angular radius must be finite and positive".into(),
                ));
            }
        }
        Ok(())
    }

    pub(crate) fn validate_for_command(&self) -> Result<(), ElectricalPowerError> {
        require_non_negative_command(self.irradiance_w_m2, "solar irradiance")?;
        require_unit_interval_command(self.visibility, "solar visibility")?;
        validate_unit_vector_command(self.direction_body, "solar direction")?;
        if !self.light_angular_radius_rad.is_finite() || self.light_angular_radius_rad < 0.0 {
            return Err(ElectricalPowerError::InvalidCommand(
                "stellar angular radius must be finite and non-negative".into(),
            ));
        }
        if !self.occluders.is_empty() && self.light_angular_radius_rad <= 0.0 {
            return Err(ElectricalPowerError::InvalidCommand(
                "stellar angular radius is required when occluders are supplied".into(),
            ));
        }
        for occluder in &self.occluders {
            validate_unit_vector_command(occluder.direction_body, "occluder direction")?;
            if !occluder.angular_radius_rad.is_finite() || occluder.angular_radius_rad <= 0.0 {
                return Err(ElectricalPowerError::InvalidCommand(
                    "occluder angular radius must be finite and positive".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Guarded small-angle apparent radius, same definition as the lighting
/// pipeline: degenerate geometry yields 0.0 instead of NaN.
fn apparent_angular_radius(body_radius_m: f64, distance_m: f64) -> f64 {
    if !body_radius_m.is_finite()
        || !distance_m.is_finite()
        || body_radius_m <= 0.0
        || distance_m <= body_radius_m
    {
        return 0.0;
    }
    (body_radius_m / distance_m).asin().max(0.0)
}

/// Fraction of a stellar disc remaining visible after the union of circular
/// occluders is removed: 1.0 clear, 0.0 fully eclipsed, smooth penumbra
/// between. Apparent discs use a common tangent-plane angular model; the
/// exposed boundary arcs give the union area without counting shared shadow
/// regions more than once. This remains independent of visual-crate code.
fn eclipse_visibility(
    light_dir: DVec3,
    light_angular_radius_rad: f64,
    occluders: &[SolarOccluder],
) -> f64 {
    let rl = light_angular_radius_rad.max(0.0);
    if rl <= 0.0 || occluders.is_empty() {
        return 1.0;
    }

    let light_dir = light_dir.normalize_or_zero();
    if light_dir == DVec3::ZERO {
        return 1.0;
    }
    let helper_axis =
        if light_dir.x.abs() <= light_dir.y.abs() && light_dir.x.abs() <= light_dir.z.abs() {
            DVec3::X
        } else if light_dir.y.abs() <= light_dir.z.abs() {
            DVec3::Y
        } else {
            DVec3::Z
        };
    let tangent_u = light_dir.cross(helper_axis).normalize();
    let tangent_v = light_dir.cross(tangent_u);

    let mut discs = Vec::with_capacity(occluders.len());
    for occluder in occluders {
        let radius = occluder.angular_radius_rad.max(0.0);
        if radius <= 0.0 {
            continue;
        }
        let cosine = light_dir
            .dot(occluder.direction_body.normalize_or_zero())
            .clamp(-1.0, 1.0);
        let separation = cosine.acos();
        if separation > rl + radius {
            continue;
        }
        if radius >= separation + rl {
            return 0.0;
        }

        let tangent = occluder.direction_body.normalize_or_zero() - light_dir * cosine;
        let tangent_length = tangent.length();
        let offset_direction = if tangent_length > 1.0e-15 {
            tangent / tangent_length
        } else {
            // At the antipode the tangent direction is undefined. Any radial
            // orientation is equivalent in the circular angular-disc model.
            tangent_u
        };
        let center = DVec2::new(
            separation * offset_direction.dot(tangent_u),
            separation * offset_direction.dot(tangent_v),
        );
        let disc = AngularDisc { center, radius };
        if !discs.iter().any(|existing: &AngularDisc| {
            existing.center == disc.center && existing.radius == disc.radius
        }) {
            discs.push(disc);
        }
    }

    match discs.as_slice() {
        [] => 1.0,
        [disc] => {
            let overlap = circle_overlap_area(rl, disc.radius, disc.center.length());
            (1.0 - overlap / (std::f64::consts::PI * rl * rl)).clamp(0.0, 1.0)
        }
        _ => {
            let occulted_area = clipped_circle_union_area(rl, &discs);
            (1.0 - occulted_area / (std::f64::consts::PI * rl * rl)).clamp(0.0, 1.0)
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct AngularDisc {
    center: DVec2,
    radius: f64,
}

/// Area covered by the union of `occluders` inside the centered stellar disc.
/// The boundary of the clipped union consists of source-circle arcs covered
/// by at least one occluder and occluder-circle arcs inside the source and
/// outside every other occluder. Green's theorem integrates those arcs exactly
/// in the angular-disc model.
fn clipped_circle_union_area(light_radius: f64, occluders: &[AngularDisc]) -> f64 {
    let source = AngularDisc {
        center: DVec2::ZERO,
        radius: light_radius,
    };
    let mut circles = Vec::with_capacity(occluders.len() + 1);
    circles.push(source);
    circles.extend_from_slice(occluders);

    let mut intersection_angles = vec![Vec::new(); circles.len()];
    for first in 0..circles.len() {
        for second in (first + 1)..circles.len() {
            if let Some(intersections) = circle_intersection_angles(circles[first], circles[second])
            {
                for (first_angle, second_angle) in intersections {
                    intersection_angles[first].push(first_angle);
                    intersection_angles[second].push(second_angle);
                }
            }
        }
    }

    let mut covered_area = 0.0;
    for (circle_index, circle) in circles.iter().copied().enumerate() {
        let mut boundaries = vec![0.0];
        boundaries.extend(
            intersection_angles[circle_index]
                .iter()
                .copied()
                .filter(|angle| {
                    *angle > 16.0 * f64::EPSILON
                        && *angle < std::f64::consts::TAU - 16.0 * f64::EPSILON
                }),
        );
        boundaries.sort_by(f64::total_cmp);
        boundaries.dedup_by(|left, right| (*left - *right).abs() <= 16.0 * f64::EPSILON);
        boundaries.push(std::f64::consts::TAU);

        for interval in boundaries.windows(2) {
            let (start, end) = (interval[0], interval[1]);
            if end <= start {
                continue;
            }
            let angle = (start + end) * 0.5;
            let point = circle.center + DVec2::new(angle.cos(), angle.sin()) * circle.radius;
            let exposed = if circle_index == 0 {
                occluders
                    .iter()
                    .any(|occluder| disc_contains(*occluder, point))
            } else {
                disc_contains(source, point)
                    && occluders.iter().enumerate().all(|(other_index, other)| {
                        other_index == circle_index - 1 || !disc_contains(*other, point)
                    })
            };
            if exposed {
                covered_area += circle_arc_signed_area(circle, start, end);
            }
        }
    }

    covered_area.clamp(0.0, std::f64::consts::PI * light_radius * light_radius)
}

fn circle_intersection_angles(first: AngularDisc, second: AngularDisc) -> Option<[(f64, f64); 2]> {
    let delta = second.center - first.center;
    let distance = delta.length();
    if distance <= 0.0
        || distance > first.radius + second.radius
        || distance < (first.radius - second.radius).abs()
    {
        return None;
    }

    let along = ((first.radius * first.radius - second.radius * second.radius
        + distance * distance)
        / (2.0 * distance))
        .clamp(-first.radius, first.radius);
    let height = (first.radius * first.radius - along * along)
        .max(0.0)
        .sqrt();
    let axis = delta / distance;
    let perpendicular = DVec2::new(-axis.y, axis.x);
    let mut angles = [(0.0, 0.0); 2];
    for (index, sign) in [-1.0, 1.0].into_iter().enumerate() {
        let point = first.center + axis * along + perpendicular * (sign * height);
        angles[index] = (
            (point.y - first.center.y)
                .atan2(point.x - first.center.x)
                .rem_euclid(std::f64::consts::TAU),
            (point.y - second.center.y)
                .atan2(point.x - second.center.x)
                .rem_euclid(std::f64::consts::TAU),
        );
    }
    Some(angles)
}

fn disc_contains(disc: AngularDisc, point: DVec2) -> bool {
    point.distance_squared(disc.center) <= disc.radius * disc.radius
}

fn circle_arc_signed_area(circle: AngularDisc, start: f64, end: f64) -> f64 {
    let (start_sin, start_cos) = start.sin_cos();
    let (end_sin, end_cos) = end.sin_cos();
    0.5 * (circle.radius * circle.center.x * (end_sin - start_sin)
        + circle.radius * circle.center.y * (start_cos - end_cos)
        + circle.radius * circle.radius * (end - start))
}

/// Intersection area of two circles (standard lens formula).
fn circle_overlap_area(r0: f64, r1: f64, separation: f64) -> f64 {
    let d = separation.max(0.0);
    if d >= r0 + r1 {
        return 0.0;
    }
    if d <= (r0 - r1).abs() {
        return std::f64::consts::PI * r0.min(r1).powi(2);
    }
    let arg0 = ((d * d + r0 * r0 - r1 * r1) / (2.0 * d * r0)).clamp(-1.0, 1.0);
    let arg1 = ((d * d + r1 * r1 - r0 * r0) / (2.0 * d * r1)).clamp(-1.0, 1.0);
    let term = ((-d + r0 + r1).max(0.0)
        * (d + r0 - r1).max(0.0)
        * (d - r0 + r1).max(0.0)
        * (d + r0 + r1).max(0.0))
    .sqrt();
    (r0 * r0 * arg0.acos() + r1 * r1 * arg1.acos() - 0.5 * term).max(0.0)
}

/// Static electrical authoring for one vessel. Every load sees the same
/// ideal bus; sources and consumers are connected implicitly at vehicle level.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ElectricalPowerSystem {
    pub batteries: Vec<BatterySpec>,
    pub ultracapacitors: Vec<UltracapacitorSpec>,
    pub solar_arrays: Vec<SolarArraySpec>,
    pub reactors: Vec<ReactorSpec>,
    pub fuel_cells: Vec<FuelCellSpec>,
    pub consumers: Vec<PowerConsumerSpec>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PowerSystemMassProperties {
    pub mass_kg: f64,
    pub center_of_mass_body_m: DVec3,
    pub inertia_body_kg_m2: DMat3,
}

impl ElectricalPowerSystem {
    pub fn validate(&self) -> Result<(), ElectricalPowerError> {
        let mut names = std::collections::HashSet::new();
        for (name, validation) in self
            .batteries
            .iter()
            .map(|part| (part.name.as_str(), part.validate()))
            .chain(
                self.ultracapacitors
                    .iter()
                    .map(|part| (part.name.as_str(), part.validate())),
            )
            .chain(
                self.solar_arrays
                    .iter()
                    .map(|part| (part.name.as_str(), part.validate())),
            )
            .chain(
                self.reactors
                    .iter()
                    .map(|part| (part.name.as_str(), part.validate())),
            )
            .chain(
                self.fuel_cells
                    .iter()
                    .map(|part| (part.name.as_str(), part.validate())),
            )
            .chain(
                self.consumers
                    .iter()
                    .map(|part| (part.name.as_str(), part.validate())),
            )
        {
            validation?;
            if !names.insert(name) {
                return Err(ElectricalPowerError::InvalidSpec(format!(
                    "power-system part name '{name}' is duplicated"
                )));
            }
        }
        Ok(())
    }

    pub fn initial_state(&self) -> Result<ElectricalPowerState, ElectricalPowerError> {
        self.validate()?;
        Ok(ElectricalPowerState {
            battery_energy_j: self
                .batteries
                .iter()
                .map(|battery| battery.capacity_j * battery.initial_charge_fraction)
                .collect(),
            capacitor_energy_j: self
                .ultracapacitors
                .iter()
                .map(|capacitor| capacitor.capacity_j * capacitor.initial_charge_fraction)
                .collect(),
            reactor_fuel_mass_kg: self
                .reactors
                .iter()
                .map(|reactor| reactor.initial_fuel_mass_kg)
                .collect(),
            solar_array_deployed_fraction: self
                .solar_arrays
                .iter()
                .map(SolarArraySpec::initial_deployment_fraction)
                .collect(),
            solar_array_tracking_angle_rad: self
                .solar_arrays
                .iter()
                .map(|array| array.tracking.initial_angle_rad())
                .collect(),
        })
    }

    pub fn mass_properties(&self) -> Result<PowerSystemMassProperties, ElectricalPowerError> {
        self.validate()?;
        let mut mass_kg = 0.0;
        let mut first_moment = DVec3::ZERO;
        let mut inertia_about_origin = DMat3::ZERO;
        for (mass, position, intrinsic) in self
            .batteries
            .iter()
            .map(|part| {
                (
                    part.mass_kg(),
                    part.position_body_m,
                    part.inertia_body_kg_m2(),
                )
            })
            .chain(self.ultracapacitors.iter().map(|part| {
                (
                    part.mass_kg(),
                    part.position_body_m,
                    part.inertia_body_kg_m2(),
                )
            }))
            .chain(self.solar_arrays.iter().map(|part| {
                (
                    part.mass_kg(),
                    part.position_body_m,
                    part.inertia_body_kg_m2(),
                )
            }))
            .chain(self.reactors.iter().map(|part| {
                (
                    part.installed_mass_kg(),
                    part.position_body_m,
                    part.inertia_body_kg_m2(),
                )
            }))
            .chain(self.fuel_cells.iter().map(|part| {
                (
                    part.dry_mass_kg,
                    part.position_body_m,
                    part.inertia_body_kg_m2(),
                )
            }))
        {
            mass_kg += mass;
            first_moment += position * mass;
            inertia_about_origin += intrinsic + parallel_axis(mass, position);
        }
        if !mass_kg.is_finite() || !first_moment.is_finite() || !inertia_about_origin.is_finite() {
            return Err(ElectricalPowerError::InvalidSpec(
                "power-system mass aggregation overflowed".into(),
            ));
        }
        if mass_kg == 0.0 {
            return Ok(PowerSystemMassProperties {
                mass_kg: 0.0,
                center_of_mass_body_m: DVec3::ZERO,
                inertia_body_kg_m2: DMat3::ZERO,
            });
        }
        let center = first_moment / mass_kg;
        Ok(PowerSystemMassProperties {
            mass_kg,
            center_of_mass_body_m: center,
            inertia_body_kg_m2: inertia_about_origin - parallel_axis(mass_kg, center),
        })
    }

    /// Advance the ideal shared bus by one positive simulation-time step.
    /// All validation and integration are transactional: failure leaves the
    /// caller's state untouched.
    pub fn advance(
        &self,
        state: &ElectricalPowerState,
        command: &ElectricalPowerCommand,
    ) -> Result<(ElectricalPowerState, ElectricalPowerTelemetry), ElectricalPowerError> {
        self.advance_with_fuel_cell_inventory(state, command, f64::MAX, f64::MAX)
    }

    /// Advance the bus with fuel-cell reactant availability taken from the
    /// installed vehicle resource inventory. The returned fuel-cell telemetry
    /// is the exact reactant flow to commit through that shared inventory.
    pub fn advance_with_fuel_cell_inventory(
        &self,
        state: &ElectricalPowerState,
        command: &ElectricalPowerCommand,
        available_hydrogen_kg: f64,
        available_oxygen_kg: f64,
    ) -> Result<(ElectricalPowerState, ElectricalPowerTelemetry), ElectricalPowerError> {
        self.validate()?;
        state.validate_for(self)?;
        command.validate_for(self)?;
        require_non_negative_command(available_hydrogen_kg, "available fuel-cell hydrogen")?;
        require_non_negative_command(available_oxygen_kg, "available fuel-cell oxygen")?;

        // Effective post-occlusion irradiance per stellar source. Tracking
        // targets and panel power both use these weights, so an eclipsed sun
        // is never chased and never generates power.
        let source_weights_w_m2: Vec<f64> = command
            .solar_flux
            .iter()
            .map(SolarFluxSource::effective_irradiance_w_m2)
            .collect();
        for weight in &source_weights_w_m2 {
            if !weight.is_finite() {
                return Err(ElectricalPowerError::InvalidCommand(
                    "effective solar irradiance overflowed".into(),
                ));
            }
        }

        let mut array_motor_request_w = vec![0.0; self.solar_arrays.len()];
        let mut planned_targets = state.solar_array_deployed_fraction.clone();
        for (index, (array, target)) in self
            .solar_arrays
            .iter()
            .zip(&command.solar_deployment_targets)
            .enumerate()
        {
            if let (
                SolarArrayDeployment::Foldable {
                    deployment_rate_per_s,
                    actuator_power_w,
                    ..
                },
                Some(target),
            ) = (array.deployment, target)
            {
                let current = state.solar_array_deployed_fraction[index];
                let requested_delta = target - current;
                let maximum_delta = deployment_rate_per_s * command.dt_s;
                let planned_delta =
                    requested_delta.signum() * requested_delta.abs().min(maximum_delta);
                let duty = (planned_delta.abs() / maximum_delta).clamp(0.0, 1.0);
                planned_targets[index] = current + planned_delta;
                array_motor_request_w[index] = actuator_power_w * duty;
            }
        }

        // Sun-tracking drives: resolve each array's target angle (manual
        // target wins, otherwise the instantaneous optimum when enabled,
        // otherwise hold), then convert the rate-limited slew into a utility
        // motor load. Stowed arrays hold: no incidence, no reason to move.
        let mut tracking_motor_request_w = vec![0.0; self.solar_arrays.len()];
        let mut tracking_targets_rad = state.solar_array_tracking_angle_rad.clone();
        for (index, array) in self.solar_arrays.iter().enumerate() {
            let SolarArrayTracking::SingleAxis {
                minimum_angle_rad,
                maximum_angle_rad,
                slew_rate_rad_s,
                actuator_power_w,
                ..
            } = array.tracking
            else {
                continue;
            };
            let current = state.solar_array_tracking_angle_rad[index];
            let deployed = state.solar_array_deployed_fraction[index];
            let desired = if let Some(manual) = command.solar_tracking_targets[index] {
                manual.clamp(minimum_angle_rad, maximum_angle_rad)
            } else if command.solar_tracking_auto[index] && deployed > 0.0 {
                best_tracking_angle(array, current, &command.solar_flux, &source_weights_w_m2)
            } else {
                current
            };
            let maximum_delta = slew_rate_rad_s * command.dt_s;
            let planned_delta = (desired - current).clamp(-maximum_delta, maximum_delta);
            let duty = if maximum_delta > 0.0 {
                (planned_delta.abs() / maximum_delta).clamp(0.0, 1.0)
            } else {
                0.0
            };
            tracking_targets_rad[index] = current + planned_delta;
            tracking_motor_request_w[index] = actuator_power_w * duty;
        }

        // Explicit Euler uses exposed fraction and tracking angle at the
        // beginning of this step; mechanisms contribute from the next step.
        let solar_available_by_array: Vec<f64> = self
            .solar_arrays
            .iter()
            .zip(&state.solar_array_deployed_fraction)
            .zip(&state.solar_array_tracking_angle_rad)
            .map(|((array, deployed), angle)| {
                solar_array_power_w_at_angle(array, *deployed, *angle, &command.solar_flux)
            })
            .collect();
        let solar_available_power_w = finite_sum(&solar_available_by_array, "solar power")?;

        let reactor_available_by_source: Vec<f64> = self
            .reactors
            .iter()
            .zip(&state.reactor_fuel_mass_kg)
            .zip(&command.reactor_power_fraction)
            .map(|((reactor, fuel), fraction)| {
                let thermal_rating = reactor.maximum_electrical_power_w();
                let fuel_limited =
                    fuel * reactor.fuel_specific_energy_j_kg * reactor.electric_efficiency
                        / command.dt_s;
                (thermal_rating.min(fuel_limited) * fraction).max(0.0)
            })
            .collect();
        let reactor_available_power_w = finite_sum(&reactor_available_by_source, "reactor power")?;

        let fuel_cell_requested_by_source: Vec<f64> = self
            .fuel_cells
            .iter()
            .zip(&command.fuel_cell_power_fraction)
            .map(|(cell, fraction)| cell.rated_electrical_power_w * fraction)
            .collect();
        let hydrogen_required_at_full_kg = self
            .fuel_cells
            .iter()
            .zip(&fuel_cell_requested_by_source)
            .map(|(cell, power)| {
                power * command.dt_s / (cell.electrical_efficiency * FUEL_CELL_HYDROGEN_LHV_J_KG)
            })
            .sum::<f64>();
        let oxygen_required_at_full_kg =
            hydrogen_required_at_full_kg * FUEL_CELL_OXYGEN_HYDROGEN_RATIO;
        if !hydrogen_required_at_full_kg.is_finite() || !oxygen_required_at_full_kg.is_finite() {
            return Err(ElectricalPowerError::InvalidCommand(
                "fuel-cell reactant demand overflowed".into(),
            ));
        }
        let hydrogen_scale = if hydrogen_required_at_full_kg > 0.0 {
            (available_hydrogen_kg / hydrogen_required_at_full_kg).min(1.0)
        } else {
            1.0
        };
        let oxygen_scale = if oxygen_required_at_full_kg > 0.0 {
            (available_oxygen_kg / oxygen_required_at_full_kg).min(1.0)
        } else {
            1.0
        };
        let fuel_cell_resource_scale = hydrogen_scale.min(oxygen_scale).clamp(0.0, 1.0);
        let fuel_cell_available_by_source: Vec<f64> = fuel_cell_requested_by_source
            .iter()
            .map(|power| power * fuel_cell_resource_scale)
            .collect();
        let fuel_cell_available_power_w =
            finite_sum(&fuel_cell_available_by_source, "fuel-cell power")?;

        let battery_discharge_limit_by_store: Vec<f64> = self
            .batteries
            .iter()
            .zip(&state.battery_energy_j)
            .map(|(battery, energy)| {
                battery
                    .maximum_discharge_power_w
                    .min(energy * battery.discharge_efficiency / command.dt_s)
            })
            .collect();
        let capacitor_discharge_limit_by_store: Vec<f64> = self
            .ultracapacitors
            .iter()
            .zip(&state.capacitor_energy_j)
            .map(|(capacitor, energy)| {
                capacitor
                    .maximum_discharge_power_w
                    .min(energy * capacitor.discharge_efficiency / command.dt_s)
            })
            .collect();
        let storage_discharge_available_w =
            finite_sum(&battery_discharge_limit_by_store, "battery discharge power")?
                + finite_sum(
                    &capacitor_discharge_limit_by_store,
                    "ultracapacitor discharge power",
                )?;

        let mut requested_power_w = command.consumer_power_w.clone();
        let mut load_priorities: Vec<_> = self.consumers.iter().map(|part| part.priority).collect();
        for motor_request_w in array_motor_request_w
            .iter()
            .chain(&tracking_motor_request_w)
        {
            requested_power_w.push(*motor_request_w);
            load_priorities.push(PowerPriority::Utility);
        }
        let available_bus_power_w = solar_available_power_w
            + command.auxiliary_generation_power_w
            + reactor_available_power_w
            + fuel_cell_available_power_w
            + storage_discharge_available_w;
        if !available_bus_power_w.is_finite() {
            return Err(ElectricalPowerError::InvalidCommand(
                "available bus power overflowed".into(),
            ));
        }
        let allocated_power_w =
            allocate_by_priority(&requested_power_w, &load_priorities, available_bus_power_w);
        let total_requested_power_w = finite_sum(&requested_power_w, "requested power")?;
        let total_delivered_power_w = finite_sum(&allocated_power_w, "delivered power")?;

        let solar_to_load_w = solar_available_power_w.min(total_delivered_power_w);
        let auxiliary_to_load_w = (total_delivered_power_w - solar_to_load_w)
            .max(0.0)
            .min(command.auxiliary_generation_power_w);
        let reactor_to_load_w = (total_delivered_power_w - solar_to_load_w - auxiliary_to_load_w)
            .max(0.0)
            .min(reactor_available_power_w);
        let fuel_cell_to_load_w =
            (total_delivered_power_w - solar_to_load_w - auxiliary_to_load_w - reactor_to_load_w)
                .max(0.0)
                .min(fuel_cell_available_power_w);
        let battery_discharge_power_w = (total_delivered_power_w
            - solar_to_load_w
            - auxiliary_to_load_w
            - reactor_to_load_w
            - fuel_cell_to_load_w)
            .max(0.0);

        let battery_charge_limits_w: Vec<f64> = self
            .batteries
            .iter()
            .zip(&state.battery_energy_j)
            .map(|(battery, energy)| {
                battery.maximum_charge_power_w.min(
                    (battery.capacity_j - energy).max(0.0)
                        / (battery.charge_efficiency * command.dt_s),
                )
            })
            .collect();
        let capacitor_charge_limits_w: Vec<f64> = self
            .ultracapacitors
            .iter()
            .zip(&state.capacitor_energy_j)
            .map(|(capacitor, energy)| {
                capacitor.maximum_charge_power_w.min(
                    (capacitor.capacity_j - energy).max(0.0)
                        / (capacitor.charge_efficiency * command.dt_s),
                )
            })
            .collect();
        let total_storage_charge_limit_w =
            finite_sum(&battery_charge_limits_w, "battery charge power")?
                + finite_sum(&capacitor_charge_limits_w, "ultracapacitor charge power")?;
        let solar_surplus_w = (solar_available_power_w - solar_to_load_w).max(0.0);
        let solar_to_storage_w = solar_surplus_w.min(total_storage_charge_limit_w);
        let auxiliary_surplus_w =
            (command.auxiliary_generation_power_w - auxiliary_to_load_w).max(0.0);
        let auxiliary_to_storage_w =
            auxiliary_surplus_w.min((total_storage_charge_limit_w - solar_to_storage_w).max(0.0));
        let reactor_surplus_capacity_w = (reactor_available_power_w - reactor_to_load_w).max(0.0);
        let reactor_to_storage_w = reactor_surplus_capacity_w.min(
            (total_storage_charge_limit_w - solar_to_storage_w - auxiliary_to_storage_w).max(0.0),
        );
        let fuel_cell_surplus_capacity_w =
            (fuel_cell_available_power_w - fuel_cell_to_load_w).max(0.0);
        let fuel_cell_to_storage_w = fuel_cell_surplus_capacity_w.min(
            (total_storage_charge_limit_w
                - solar_to_storage_w
                - auxiliary_to_storage_w
                - reactor_to_storage_w)
                .max(0.0),
        );
        let storage_charge_power_w = solar_to_storage_w
            + auxiliary_to_storage_w
            + reactor_to_storage_w
            + fuel_cell_to_storage_w;
        let reactor_output_power_w = reactor_to_load_w + reactor_to_storage_w;
        let fuel_cell_output_power_w = fuel_cell_to_load_w + fuel_cell_to_storage_w;

        let reactor_outputs = share_with_caps(reactor_output_power_w, &reactor_available_by_source);
        let fuel_cell_outputs =
            share_with_caps(fuel_cell_output_power_w, &fuel_cell_available_by_source);
        // Batteries and ultracapacitors share bus charge/discharge in
        // proportion to their instantaneous headroom: no chemistry priority
        // is hardcoded into the ideal bus.
        let pooled_discharge_limits: Vec<f64> = battery_discharge_limit_by_store
            .iter()
            .chain(&capacitor_discharge_limit_by_store)
            .copied()
            .collect();
        let pooled_discharge = share_with_caps(battery_discharge_power_w, &pooled_discharge_limits);
        let pooled_charge_limits: Vec<f64> = battery_charge_limits_w
            .iter()
            .chain(&capacitor_charge_limits_w)
            .copied()
            .collect();
        let pooled_charge = share_with_caps(storage_charge_power_w, &pooled_charge_limits);
        let battery_charge: Vec<f64> = pooled_charge[..self.batteries.len()].to_vec();
        let capacitor_charge: Vec<f64> = pooled_charge[self.batteries.len()..].to_vec();
        let battery_discharge: Vec<f64> = pooled_discharge[..self.batteries.len()].to_vec();
        let capacitor_discharge: Vec<f64> = pooled_discharge[self.batteries.len()..].to_vec();
        let solar_to_load = share_with_caps(solar_to_load_w, &solar_available_by_array);
        let solar_to_store = share_with_caps(solar_to_storage_w, &solar_available_by_array);

        let mut next = state.clone();
        let mut battery_telemetry = Vec::with_capacity(self.batteries.len());
        for (index, battery) in self.batteries.iter().enumerate() {
            let initial_energy_j = state.battery_energy_j[index];
            let stored = initial_energy_j
                + battery_charge[index] * battery.charge_efficiency * command.dt_s
                - battery_discharge[index] * command.dt_s / battery.discharge_efficiency;
            let final_energy_j = stored.clamp(0.0, battery.capacity_j);
            next.battery_energy_j[index] = final_energy_j;
            battery_telemetry.push(BatteryPowerTelemetry {
                name: battery.name.clone(),
                initial_energy_j,
                final_energy_j,
                charge_power_w: battery_charge[index],
                discharge_power_w: battery_discharge[index],
            });
        }

        let mut capacitor_telemetry = Vec::with_capacity(self.ultracapacitors.len());
        for (index, capacitor) in self.ultracapacitors.iter().enumerate() {
            let initial_energy_j = state.capacitor_energy_j[index];
            let stored = initial_energy_j
                + capacitor_charge[index] * capacitor.charge_efficiency * command.dt_s
                - capacitor_discharge[index] * command.dt_s / capacitor.discharge_efficiency;
            let final_energy_j = stored.clamp(0.0, capacitor.capacity_j);
            next.capacitor_energy_j[index] = final_energy_j;
            capacitor_telemetry.push(UltracapacitorPowerTelemetry {
                name: capacitor.name.clone(),
                initial_energy_j,
                final_energy_j,
                charge_power_w: capacitor_charge[index],
                discharge_power_w: capacitor_discharge[index],
            });
        }

        let mut reactor_telemetry = Vec::with_capacity(self.reactors.len());
        for (index, reactor) in self.reactors.iter().enumerate() {
            let electrical_power_w = reactor_outputs[index];
            let thermal_power_w = electrical_power_w / reactor.electric_efficiency;
            let fuel_consumed_kg =
                thermal_power_w * command.dt_s / reactor.fuel_specific_energy_j_kg;
            let remaining_fuel = (state.reactor_fuel_mass_kg[index] - fuel_consumed_kg).max(0.0);
            next.reactor_fuel_mass_kg[index] = remaining_fuel;
            reactor_telemetry.push(ReactorPowerTelemetry {
                name: reactor.name.clone(),
                available_electrical_power_w: reactor_available_by_source[index],
                electrical_power_w,
                thermal_power_w,
                waste_heat_w: (thermal_power_w - electrical_power_w).max(0.0),
                fuel_consumed_kg,
                remaining_fuel_mass_kg: remaining_fuel,
            });
        }

        let mut fuel_cell_telemetry = Vec::with_capacity(self.fuel_cells.len());
        for (cell, electrical_power_w) in self.fuel_cells.iter().zip(&fuel_cell_outputs) {
            let chemical_power_w = electrical_power_w / cell.electrical_efficiency;
            let hydrogen_flow_kg_s = chemical_power_w / FUEL_CELL_HYDROGEN_LHV_J_KG;
            let oxygen_flow_kg_s = hydrogen_flow_kg_s * FUEL_CELL_OXYGEN_HYDROGEN_RATIO;
            let waste_heat_w = (chemical_power_w - electrical_power_w).max(0.0);
            fuel_cell_telemetry.push(FuelCellPowerTelemetry {
                name: cell.name.clone(),
                electrical_power_w: *electrical_power_w,
                chemical_power_w,
                hydrogen_flow_kg_s,
                oxygen_flow_kg_s,
                water_production_kg_s: hydrogen_flow_kg_s + oxygen_flow_kg_s,
                waste_heat_w,
            });
        }

        let mut solar_telemetry = Vec::with_capacity(self.solar_arrays.len());
        let mut deployment_power_w = 0.0;
        let mut tracking_power_w = 0.0;
        for (index, array) in self.solar_arrays.iter().enumerate() {
            let initial_fraction = state.solar_array_deployed_fraction[index];
            let initial_angle = state.solar_array_tracking_angle_rad[index];
            let fold_allocation = allocated_power_w[self.consumers.len() + index];
            let tracking_allocation =
                allocated_power_w[self.consumers.len() + self.solar_arrays.len() + index];
            let fold_fraction = if array_motor_request_w[index] > 0.0 {
                (fold_allocation / array_motor_request_w[index]).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let final_fraction = match array.deployment {
                SolarArrayDeployment::Fixed => 1.0,
                SolarArrayDeployment::Foldable {
                    deployment_rate_per_s,
                    ..
                } => {
                    let target = planned_targets[index];
                    let delta = target - initial_fraction;
                    let actual_delta = delta.signum()
                        * (deployment_rate_per_s * command.dt_s * fold_fraction).min(delta.abs());
                    (initial_fraction + actual_delta).clamp(0.0, 1.0)
                }
            };
            next.solar_array_deployed_fraction[index] = final_fraction;
            let track_fraction = if tracking_motor_request_w[index] > 0.0 {
                (tracking_allocation / tracking_motor_request_w[index]).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let final_angle = match array.tracking {
                SolarArrayTracking::Fixed => 0.0,
                SolarArrayTracking::SingleAxis {
                    slew_rate_rad_s, ..
                } => {
                    let target = tracking_targets_rad[index];
                    let delta = target - initial_angle;
                    let actual_delta = delta.signum()
                        * (slew_rate_rad_s * command.dt_s * track_fraction).min(delta.abs());
                    initial_angle + actual_delta
                }
            };
            next.solar_array_tracking_angle_rad[index] = final_angle;
            let charged_power_w = solar_to_store[index];
            let spilled_power_w =
                (solar_available_by_array[index] - solar_to_load[index] - charged_power_w).max(0.0);
            deployment_power_w += fold_allocation;
            tracking_power_w += tracking_allocation;
            solar_telemetry.push(SolarArrayPowerTelemetry {
                name: array.name.clone(),
                deployed_fraction: final_fraction,
                tracking_angle_rad: final_angle,
                tracking_target_rad: tracking_targets_rad[index],
                available_power_w: solar_available_by_array[index],
                effective_incident_irradiance_w_m2: projected_irradiance_at_angle(
                    array,
                    initial_angle,
                    &command.solar_flux,
                ),
                power_to_load_w: solar_to_load[index],
                power_to_storage_w: charged_power_w,
                spilled_power_w,
                deployment_power_w: fold_allocation,
                tracking_power_w: tracking_allocation,
            });
        }

        if next
            .battery_energy_j
            .iter()
            .chain(&next.capacitor_energy_j)
            .chain(&next.reactor_fuel_mass_kg)
            .chain(&next.solar_array_deployed_fraction)
            .chain(&next.solar_array_tracking_angle_rad)
            .any(|value| !value.is_finite())
        {
            return Err(ElectricalPowerError::InvalidState(
                "power integration produced non-finite state".into(),
            ));
        }
        let consumer_telemetry = self
            .consumers
            .iter()
            .zip(&command.consumer_power_w)
            .zip(&allocated_power_w)
            .map(|((consumer, requested), supplied)| PowerAllocation {
                name: consumer.name.clone(),
                requested_power_w: *requested,
                supplied_power_w: *supplied,
                priority: consumer.priority,
            })
            .collect();
        let total_spilled_solar_power_w = solar_telemetry
            .iter()
            .map(|array| array.spilled_power_w)
            .sum();
        let total_reactor_waste_heat_w = reactor_telemetry
            .iter()
            .map(|reactor| reactor.waste_heat_w)
            .sum();
        let total_fuel_cell_waste_heat_w = fuel_cell_telemetry
            .iter()
            .map(|cell| cell.waste_heat_w)
            .sum();
        let unserved_power_w = (total_requested_power_w - total_delivered_power_w).max(0.0);
        let telemetry = ElectricalPowerTelemetry {
            total_requested_power_w,
            total_delivered_power_w,
            unserved_power_w,
            solar_available_power_w,
            solar_to_load_power_w: solar_to_load_w,
            solar_to_storage_power_w: solar_to_storage_w,
            spilled_solar_power_w: total_spilled_solar_power_w,
            auxiliary_generation_power_w: command.auxiliary_generation_power_w,
            auxiliary_to_load_power_w: auxiliary_to_load_w,
            auxiliary_to_storage_power_w: auxiliary_to_storage_w,
            spilled_auxiliary_power_w: (command.auxiliary_generation_power_w
                - auxiliary_to_load_w
                - auxiliary_to_storage_w)
                .max(0.0),
            reactor_available_power_w,
            reactor_output_power_w,
            reactor_waste_heat_w: total_reactor_waste_heat_w,
            fuel_cell_available_power_w,
            fuel_cell_output_power_w,
            fuel_cell_waste_heat_w: total_fuel_cell_waste_heat_w,
            battery_charge_power_w: finite_sum(&battery_charge, "battery charge telemetry")?,
            battery_discharge_power_w: finite_sum(
                &battery_discharge,
                "battery discharge telemetry",
            )?,
            capacitor_charge_power_w: finite_sum(&capacitor_charge, "capacitor charge telemetry")?,
            capacitor_discharge_power_w: finite_sum(
                &capacitor_discharge,
                "capacitor discharge telemetry",
            )?,
            deployment_power_w,
            tracking_power_w,
            consumer_allocations: consumer_telemetry,
            solar_arrays: solar_telemetry,
            reactors: reactor_telemetry,
            fuel_cells: fuel_cell_telemetry,
            batteries: battery_telemetry,
            ultracapacitors: capacitor_telemetry,
        };
        Ok((next, telemetry))
    }
}

/// Persistent authoritative energy and mechanism state for one vessel.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ElectricalPowerState {
    pub battery_energy_j: Vec<f64>,
    #[serde(default)]
    pub capacitor_energy_j: Vec<f64>,
    pub reactor_fuel_mass_kg: Vec<f64>,
    pub solar_array_deployed_fraction: Vec<f64>,
    #[serde(default)]
    pub solar_array_tracking_angle_rad: Vec<f64>,
}

impl ElectricalPowerState {
    pub fn validate_for(&self, system: &ElectricalPowerSystem) -> Result<(), ElectricalPowerError> {
        if self.battery_energy_j.len() != system.batteries.len()
            || self.capacitor_energy_j.len() != system.ultracapacitors.len()
            || self.reactor_fuel_mass_kg.len() != system.reactors.len()
            || self.solar_array_deployed_fraction.len() != system.solar_arrays.len()
            || self.solar_array_tracking_angle_rad.len() != system.solar_arrays.len()
        {
            return Err(ElectricalPowerError::InvalidState(
                "state array lengths do not match the authored power system".into(),
            ));
        }
        for (battery, energy) in system.batteries.iter().zip(&self.battery_energy_j) {
            if !energy.is_finite() || *energy < 0.0 || *energy > battery.capacity_j {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "battery '{}' energy is outside [0, capacity]",
                    battery.name
                )));
            }
        }
        for (capacitor, energy) in system.ultracapacitors.iter().zip(&self.capacitor_energy_j) {
            if !energy.is_finite() || *energy < 0.0 || *energy > capacitor.capacity_j {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "ultracapacitor '{}' energy is outside [0, capacity]",
                    capacitor.name
                )));
            }
        }
        for (reactor, fuel) in system.reactors.iter().zip(&self.reactor_fuel_mass_kg) {
            if !fuel.is_finite() || *fuel < 0.0 || *fuel > reactor.initial_fuel_mass_kg {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "reactor '{}' fuel inventory is outside [0, initial]",
                    reactor.name
                )));
            }
        }
        for ((array, deployed), angle) in system
            .solar_arrays
            .iter()
            .zip(&self.solar_array_deployed_fraction)
            .zip(&self.solar_array_tracking_angle_rad)
        {
            if !deployed.is_finite() || !(0.0..=1.0).contains(deployed) {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "solar array '{}' deployment must be in [0, 1]",
                    array.name
                )));
            }
            if array.deployment == SolarArrayDeployment::Fixed && *deployed != 1.0 {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "fixed solar array '{}' must remain fully exposed",
                    array.name
                )));
            }
            if !angle.is_finite() {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "solar array '{}' tracking angle must be finite",
                    array.name
                )));
            }
            if let SolarArrayTracking::SingleAxis {
                minimum_angle_rad,
                maximum_angle_rad,
                ..
            } = array.tracking
                && !(minimum_angle_rad..=maximum_angle_rad).contains(angle)
            {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "solar array '{}' tracking angle is outside its limits",
                    array.name
                )));
            }
            if array.tracking == SolarArrayTracking::Fixed && *angle != 0.0 {
                return Err(ElectricalPowerError::InvalidState(format!(
                    "fixed solar array '{}' tracking angle must be zero",
                    array.name
                )));
            }
        }
        Ok(())
    }
}

/// Per-step commands use the same order as the corresponding authored lists.
/// Empty vectors are valid when that device class has no authored entries.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ElectricalPowerCommand {
    pub dt_s: f64,
    pub solar_flux: Vec<SolarFluxSource>,
    pub consumer_power_w: Vec<f64>,
    /// Electrical output from externally advanced mounted generators (for
    /// example an APU operating point). This is actual generated power, not
    /// a user-authored bus coefficient.
    pub auxiliary_generation_power_w: f64,
    /// Requested reactor output as a fraction of its physical capacity.
    pub reactor_power_fraction: Vec<f64>,
    /// Requested fuel-cell output as a fraction of each cell's rating.
    pub fuel_cell_power_fraction: Vec<f64>,
    /// `None` holds the current position; `Some(0..=1)` targets a foldable array.
    pub solar_deployment_targets: Vec<Option<f64>>,
    /// `true` slews a single-axis array toward its instantaneous optimum.
    /// A manual angle in `solar_tracking_targets` wins over automatic mode.
    pub solar_tracking_auto: Vec<bool>,
    /// `None` holds (or auto-tracks); `Some(angle)` drives a single-axis
    /// array toward that body-frame angle in radians.
    pub solar_tracking_targets: Vec<Option<f64>>,
}

impl ElectricalPowerCommand {
    pub fn idle_for(system: &ElectricalPowerSystem, dt_s: f64) -> Self {
        Self {
            dt_s,
            solar_flux: Vec::new(),
            consumer_power_w: vec![0.0; system.consumers.len()],
            auxiliary_generation_power_w: 0.0,
            reactor_power_fraction: vec![1.0; system.reactors.len()],
            fuel_cell_power_fraction: vec![1.0; system.fuel_cells.len()],
            solar_deployment_targets: vec![None; system.solar_arrays.len()],
            solar_tracking_auto: vec![false; system.solar_arrays.len()],
            solar_tracking_targets: vec![None; system.solar_arrays.len()],
        }
    }

    fn validate_for(&self, system: &ElectricalPowerSystem) -> Result<(), ElectricalPowerError> {
        require_positive_command(self.dt_s, "power step duration")?;
        require_non_negative_command(
            self.auxiliary_generation_power_w,
            "auxiliary generator output",
        )?;
        if self.consumer_power_w.len() != system.consumers.len()
            || self.reactor_power_fraction.len() != system.reactors.len()
            || self.fuel_cell_power_fraction.len() != system.fuel_cells.len()
            || self.solar_deployment_targets.len() != system.solar_arrays.len()
            || self.solar_tracking_auto.len() != system.solar_arrays.len()
            || self.solar_tracking_targets.len() != system.solar_arrays.len()
        {
            return Err(ElectricalPowerError::InvalidCommand(
                "command vector lengths do not match the authored power system".into(),
            ));
        }
        for (consumer, request) in system.consumers.iter().zip(&self.consumer_power_w) {
            require_non_negative_command(*request, "consumer requested power")?;
            if *request > consumer.rated_power_w {
                return Err(ElectricalPowerError::InvalidCommand(format!(
                    "consumer '{}' request exceeds its rated power",
                    consumer.name
                )));
            }
        }
        for fraction in &self.reactor_power_fraction {
            require_unit_interval_command(*fraction, "reactor power fraction")?;
        }
        for fraction in &self.fuel_cell_power_fraction {
            require_unit_interval_command(*fraction, "fuel-cell power fraction")?;
        }
        for (array, target) in system
            .solar_arrays
            .iter()
            .zip(&self.solar_deployment_targets)
        {
            if let Some(target) = target {
                require_unit_interval_command(*target, "solar-array deployment target")?;
                if array.deployment == SolarArrayDeployment::Fixed {
                    return Err(ElectricalPowerError::InvalidCommand(format!(
                        "fixed solar array '{}' cannot be deployed or folded",
                        array.name
                    )));
                }
            }
        }
        for ((array, target), auto) in system
            .solar_arrays
            .iter()
            .zip(&self.solar_tracking_targets)
            .zip(&self.solar_tracking_auto)
        {
            if let Some(target) = target {
                if !target.is_finite() {
                    return Err(ElectricalPowerError::InvalidCommand(format!(
                        "solar array '{}' tracking target must be finite",
                        array.name
                    )));
                }
                if array.tracking == SolarArrayTracking::Fixed {
                    return Err(ElectricalPowerError::InvalidCommand(format!(
                        "fixed solar array '{}' cannot slew",
                        array.name
                    )));
                }
            }
            if *auto && array.tracking == SolarArrayTracking::Fixed {
                return Err(ElectricalPowerError::InvalidCommand(format!(
                    "fixed solar array '{}' cannot auto-track",
                    array.name
                )));
            }
        }
        for source in &self.solar_flux {
            source.validate_for_command()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PowerAllocation {
    pub name: String,
    pub requested_power_w: f64,
    pub supplied_power_w: f64,
    pub priority: PowerPriority,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolarArrayPowerTelemetry {
    pub name: String,
    pub deployed_fraction: f64,
    pub tracking_angle_rad: f64,
    pub tracking_target_rad: f64,
    pub available_power_w: f64,
    /// Post-occlusion projected irradiance on the active side (W/m^2),
    /// before cell efficiency and exposed fraction.
    pub effective_incident_irradiance_w_m2: f64,
    pub power_to_load_w: f64,
    pub power_to_storage_w: f64,
    pub spilled_power_w: f64,
    pub deployment_power_w: f64,
    pub tracking_power_w: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReactorPowerTelemetry {
    pub name: String,
    pub available_electrical_power_w: f64,
    pub electrical_power_w: f64,
    pub thermal_power_w: f64,
    pub waste_heat_w: f64,
    pub fuel_consumed_kg: f64,
    pub remaining_fuel_mass_kg: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FuelCellPowerTelemetry {
    pub name: String,
    pub electrical_power_w: f64,
    pub chemical_power_w: f64,
    pub hydrogen_flow_kg_s: f64,
    pub oxygen_flow_kg_s: f64,
    /// Product water before any external recovery/venting model (kg/s).
    pub water_production_kg_s: f64,
    pub waste_heat_w: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BatteryPowerTelemetry {
    pub name: String,
    pub initial_energy_j: f64,
    pub final_energy_j: f64,
    /// Bus-side charging power before the authored charge efficiency.
    pub charge_power_w: f64,
    /// Bus-side delivered power; stored energy falls by `P * dt / efficiency`.
    pub discharge_power_w: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UltracapacitorPowerTelemetry {
    pub name: String,
    pub initial_energy_j: f64,
    pub final_energy_j: f64,
    /// Bus-side charging power before the authored charge efficiency.
    pub charge_power_w: f64,
    /// Bus-side delivered power; stored energy falls by `P * dt / efficiency`.
    pub discharge_power_w: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ElectricalPowerTelemetry {
    pub total_requested_power_w: f64,
    pub total_delivered_power_w: f64,
    pub unserved_power_w: f64,
    pub solar_available_power_w: f64,
    pub solar_to_load_power_w: f64,
    pub solar_to_storage_power_w: f64,
    pub spilled_solar_power_w: f64,
    pub auxiliary_generation_power_w: f64,
    pub auxiliary_to_load_power_w: f64,
    pub auxiliary_to_storage_power_w: f64,
    pub spilled_auxiliary_power_w: f64,
    pub reactor_available_power_w: f64,
    pub reactor_output_power_w: f64,
    pub reactor_waste_heat_w: f64,
    pub fuel_cell_available_power_w: f64,
    pub fuel_cell_output_power_w: f64,
    pub fuel_cell_waste_heat_w: f64,
    pub battery_charge_power_w: f64,
    pub battery_discharge_power_w: f64,
    pub capacitor_charge_power_w: f64,
    pub capacitor_discharge_power_w: f64,
    pub deployment_power_w: f64,
    pub tracking_power_w: f64,
    pub consumer_allocations: Vec<PowerAllocation>,
    pub solar_arrays: Vec<SolarArrayPowerTelemetry>,
    pub reactors: Vec<ReactorPowerTelemetry>,
    pub fuel_cells: Vec<FuelCellPowerTelemetry>,
    pub batteries: Vec<BatteryPowerTelemetry>,
    pub ultracapacitors: Vec<UltracapacitorPowerTelemetry>,
}

impl ElectricalPowerTelemetry {
    pub fn supplied_power_w(&self, consumer_name: &str) -> Option<f64> {
        self.consumer_allocations
            .iter()
            .find(|allocation| allocation.name == consumer_name)
            .map(|allocation| allocation.supplied_power_w)
    }
}

/// Post-occlusion irradiance projected onto the active side (W/m^2),
/// before cell efficiency and exposed fraction.
fn projected_irradiance_at_angle(
    array: &SolarArraySpec,
    angle_rad: f64,
    sources: &[SolarFluxSource],
) -> f64 {
    let normal = array.normal_at_angle(angle_rad);
    sources
        .iter()
        .map(|source| {
            source.effective_irradiance_w_m2() * normal.dot(source.direction_body).max(0.0)
        })
        .sum()
}

fn solar_array_power_w_at_angle(
    array: &SolarArraySpec,
    deployed_fraction: f64,
    angle_rad: f64,
    sources: &[SolarFluxSource],
) -> f64 {
    projected_irradiance_at_angle(array, angle_rad, sources)
        * array.area_m2()
        * array.cell_efficiency
        * deployed_fraction
}

/// Objective maximized by sun tracking: post-occlusion projected
/// irradiance (W/m^2) at a candidate angle.
fn tracking_objective(
    array: &SolarArraySpec,
    angle_rad: f64,
    sources: &[SolarFluxSource],
    weights_w_m2: &[f64],
) -> f64 {
    let normal = array.normal_at_angle(angle_rad);
    sources
        .iter()
        .zip(weights_w_m2)
        .map(|(source, weight)| weight * normal.dot(source.direction_body).max(0.0))
        .sum()
}

/// Best single-axis angle in `[min, max]` for the current stellar geometry.
/// Deterministic coarse scan (72 samples) plus local halving refinement;
/// holds the current angle when nothing improves it (night, eclipse, or sun
/// along the slew axis), so the drive never burns power for zero gain.
fn best_tracking_angle(
    array: &SolarArraySpec,
    current_angle_rad: f64,
    sources: &[SolarFluxSource],
    weights_w_m2: &[f64],
) -> f64 {
    let SolarArrayTracking::SingleAxis {
        minimum_angle_rad,
        maximum_angle_rad,
        ..
    } = array.tracking
    else {
        return 0.0;
    };
    let total_weight: f64 = weights_w_m2.iter().sum();
    if !total_weight.is_finite() || total_weight <= 0.0 {
        return current_angle_rad;
    }
    let current_value = tracking_objective(array, current_angle_rad, sources, weights_w_m2);
    const SAMPLES: usize = 72;
    let mut best_angle = current_angle_rad;
    let mut best_value = current_value;
    for sample in 0..=SAMPLES {
        let angle = minimum_angle_rad
            + (maximum_angle_rad - minimum_angle_rad) * sample as f64 / SAMPLES as f64;
        let value = tracking_objective(array, angle, sources, weights_w_m2);
        if value > best_value {
            best_value = value;
            best_angle = angle;
        }
    }
    // Local refinement around the scan winner; the clipped-sinusoid sum is
    // non-smooth at terminator crossings, so hill-climb instead of assuming
    // a parabola.
    let mut step = (maximum_angle_rad - minimum_angle_rad) / SAMPLES as f64;
    for _ in 0..6 {
        step *= 0.5;
        let mut improved = false;
        for candidate in [best_angle - step, best_angle + step] {
            let candidate = candidate.clamp(minimum_angle_rad, maximum_angle_rad);
            let value = tracking_objective(array, candidate, sources, weights_w_m2);
            if value > best_value {
                best_value = value;
                best_angle = candidate;
                improved = true;
            }
        }
        if !improved && step < 1.0e-9 {
            break;
        }
    }
    // Hold position unless the gain exceeds numerical noise scaled by the
    // incident scale; avoids wasteful eclipse hunting.
    if best_value <= current_value + 1.0e-9 * (1.0 + total_weight) {
        current_angle_rad
    } else {
        best_angle
    }
}

fn allocate_by_priority(requests: &[f64], priorities: &[PowerPriority], supply_w: f64) -> Vec<f64> {
    let mut allocation = vec![0.0; requests.len()];
    let mut remaining = supply_w.max(0.0);
    for priority in PowerPriority::ORDERED {
        let group_requested: f64 = requests
            .iter()
            .zip(priorities)
            .filter(|(_, item_priority)| **item_priority == priority)
            .map(|(power, _)| *power)
            .sum();
        if group_requested <= 0.0 || remaining <= 0.0 {
            continue;
        }
        let fraction = (remaining / group_requested).clamp(0.0, 1.0);
        for (index, (request, item_priority)) in requests.iter().zip(priorities).enumerate() {
            if *item_priority == priority {
                allocation[index] = request * fraction;
            }
        }
        remaining = (remaining - group_requested * fraction).max(0.0);
    }
    allocation
}

fn share_with_caps(requested_w: f64, caps: &[f64]) -> Vec<f64> {
    let available: f64 = caps.iter().sum();
    if available <= 0.0 || requested_w <= 0.0 {
        return vec![0.0; caps.len()];
    }
    let fraction = (requested_w / available).clamp(0.0, 1.0);
    caps.iter().map(|cap| cap * fraction).collect()
}

fn finite_sum(values: &[f64], label: &str) -> Result<f64, ElectricalPowerError> {
    let sum: f64 = values.iter().sum();
    if sum.is_finite() {
        Ok(sum)
    } else {
        Err(ElectricalPowerError::InvalidCommand(format!(
            "{label} overflowed"
        )))
    }
}

fn cuboid_inertia(mass_kg: f64, dimensions: DVec3) -> DMat3 {
    DMat3::from_diagonal(DVec3::new(
        mass_kg * (dimensions.y.powi(2) + dimensions.z.powi(2)) / 12.0,
        mass_kg * (dimensions.x.powi(2) + dimensions.z.powi(2)) / 12.0,
        mass_kg * (dimensions.x.powi(2) + dimensions.y.powi(2)) / 12.0,
    ))
}

fn parallel_axis(mass_kg: f64, center_m: DVec3) -> DMat3 {
    (DMat3::IDENTITY * center_m.length_squared()
        - DMat3::from_cols(
            center_m * center_m.x,
            center_m * center_m.y,
            center_m * center_m.z,
        ))
        * mass_kg
}

fn require_positive(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite and positive"
        )));
    }
    Ok(())
}

fn require_non_negative(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || value < 0.0 {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite and non-negative"
        )));
    }
    Ok(())
}

fn require_unit_interval(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite and in [0, 1]"
        )));
    }
    Ok(())
}

fn require_efficiency(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || value <= 0.0 || value > 1.0 {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite and in (0, 1]"
        )));
    }
    Ok(())
}

fn validate_finite_vector(value: DVec3, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite"
        )));
    }
    Ok(())
}

fn validate_positive_vector(value: DVec3, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || value.min_element() <= 0.0 {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite and positive on every axis"
        )));
    }
    Ok(())
}

fn validate_unit_vector(value: DVec3, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || (value.length() - 1.0).abs() > 1.0e-9 {
        return Err(ElectricalPowerError::InvalidSpec(format!(
            "{label} must be finite and unit length"
        )));
    }
    Ok(())
}

fn require_positive_command(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(ElectricalPowerError::InvalidCommand(format!(
            "{label} must be finite and positive"
        )));
    }
    Ok(())
}

fn require_non_negative_command(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || value < 0.0 {
        return Err(ElectricalPowerError::InvalidCommand(format!(
            "{label} must be finite and non-negative"
        )));
    }
    Ok(())
}

fn require_unit_interval_command(value: f64, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(ElectricalPowerError::InvalidCommand(format!(
            "{label} must be finite and in [0, 1]"
        )));
    }
    Ok(())
}

fn validate_unit_vector_command(value: DVec3, label: &str) -> Result<(), ElectricalPowerError> {
    if !value.is_finite() || (value.length() - 1.0).abs() > 1.0e-9 {
        return Err(ElectricalPowerError::InvalidCommand(format!(
            "{label} must be finite and unit length"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn battery() -> BatterySpec {
        BatterySpec {
            name: "battery".into(),
            capacity_j: 1_000.0,
            initial_charge_fraction: 0.5,
            maximum_charge_power_w: 400.0,
            maximum_discharge_power_w: 500.0,
            charge_efficiency: 0.8,
            discharge_efficiency: 0.9,
            specific_energy_j_kg: 500.0,
            dimensions_body_m: DVec3::splat(1.0),
            position_body_m: DVec3::ZERO,
        }
    }

    fn capacitor() -> UltracapacitorSpec {
        UltracapacitorSpec {
            name: "ultracap".into(),
            capacity_j: 200.0,
            initial_charge_fraction: 0.5,
            maximum_charge_power_w: 5_000.0,
            maximum_discharge_power_w: 8_000.0,
            charge_efficiency: 0.97,
            discharge_efficiency: 0.97,
            specific_energy_j_kg: 20_000.0,
            dimensions_body_m: DVec3::splat(0.5),
            position_body_m: DVec3::ZERO,
        }
    }

    fn consumer(name: &str, rated_power_w: f64, priority: PowerPriority) -> PowerConsumerSpec {
        PowerConsumerSpec {
            name: name.into(),
            rated_power_w,
            priority,
        }
    }

    fn flux(irradiance_w_m2: f64, direction: DVec3) -> SolarFluxSource {
        SolarFluxSource::new(irradiance_w_m2, direction, 1.0).expect("valid flux")
    }

    fn array(deployment: SolarArrayDeployment) -> SolarArraySpec {
        SolarArraySpec {
            name: "array".into(),
            cell_count_x: 10,
            cell_count_y: 10,
            cell_size_x_m: 0.1,
            cell_size_y_m: 0.1,
            cell_efficiency: 0.25,
            cell_areal_density_kg_m2: 4.0,
            support_areal_density_kg_m2: 1.0,
            panel_u_axis_body: DVec3::X,
            panel_v_axis_body: DVec3::Y,
            position_body_m: DVec3::ZERO,
            deployment,
            tracking: SolarArrayTracking::Fixed,
        }
    }

    fn tracking_array() -> SolarArraySpec {
        SolarArraySpec {
            name: "tracking-array".into(),
            cell_count_x: 10,
            cell_count_y: 10,
            cell_size_x_m: 0.1,
            cell_size_y_m: 0.1,
            cell_efficiency: 0.25,
            cell_areal_density_kg_m2: 4.0,
            support_areal_density_kg_m2: 1.0,
            panel_u_axis_body: DVec3::X,
            panel_v_axis_body: DVec3::Y,
            position_body_m: DVec3::ZERO,
            deployment: SolarArrayDeployment::Fixed,
            tracking: SolarArrayTracking::SingleAxis {
                rotation_axis_body: DVec3::X,
                minimum_angle_rad: -std::f64::consts::FRAC_PI_2,
                maximum_angle_rad: std::f64::consts::FRAC_PI_2,
                slew_rate_rad_s: 0.5,
                actuator_power_w: 60.0,
                initial_angle_rad: 0.0,
            },
        }
    }

    fn reactor() -> ReactorSpec {
        ReactorSpec {
            name: "reactor".into(),
            rated_thermal_power_w: 1_000.0,
            electric_efficiency: 0.4,
            radiator_capacity_w: 1_500.0,
            initial_fuel_mass_kg: 0.01,
            fuel_specific_energy_j_kg: 1.0e8,
            dry_mass_kg: 50.0,
            dimensions_body_m: DVec3::new(1.0, 2.0, 3.0),
            position_body_m: DVec3::ZERO,
        }
    }

    fn fuel_cell() -> FuelCellSpec {
        FuelCellSpec {
            name: "hydrogen-cell".into(),
            rated_electrical_power_w: 1_000.0,
            electrical_efficiency: 0.5,
            dry_mass_kg: 20.0,
            dimensions_body_m: DVec3::new(0.5, 0.5, 0.8),
            position_body_m: DVec3::ZERO,
            feed_port_name: None,
        }
    }

    #[test]
    fn fuel_cell_obeys_reactant_inventory_and_closes_water_heat_balance() {
        let system = ElectricalPowerSystem {
            fuel_cells: vec![fuel_cell()],
            consumers: vec![consumer(
                "life-support",
                1_000.0,
                PowerPriority::LifeSupport,
            )],
            ..ElectricalPowerSystem::default()
        };
        let state = system.initial_state().expect("fuel-cell power state");
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.consumer_power_w = vec![500.0];
        let (_, report) = system
            .advance_with_fuel_cell_inventory(&state, &command, 1.0, 8.0)
            .expect("inventory-backed cell step");
        let cell = &report.fuel_cells[0];
        let expected_hydrogen_flow =
            500.0 / (fuel_cell().electrical_efficiency * FUEL_CELL_HYDROGEN_LHV_J_KG);
        assert_eq!(report.total_delivered_power_w, 500.0);
        assert!((cell.hydrogen_flow_kg_s - expected_hydrogen_flow).abs() < 1.0e-18);
        assert!((cell.oxygen_flow_kg_s / cell.hydrogen_flow_kg_s - 8.0).abs() < 1.0e-12);
        assert!((cell.water_production_kg_s - 9.0 * cell.hydrogen_flow_kg_s).abs() < 1.0e-18);
        assert!(
            (cell.chemical_power_w - cell.electrical_power_w - cell.waste_heat_w).abs() < 1.0e-12
        );

        let hydrogen_limited = system
            .advance_with_fuel_cell_inventory(&state, &command, expected_hydrogen_flow * 0.5, 8.0)
            .expect("reactant-limited cell step");
        assert!((hydrogen_limited.1.fuel_cells[0].electrical_power_w - 250.0).abs() < 1.0e-10);
        assert_eq!(hydrogen_limited.1.unserved_power_w, 250.0);
        assert!(
            system
                .advance_with_fuel_cell_inventory(&state, &command, -1.0, 1.0)
                .is_err()
        );
    }

    #[test]
    fn physically_supplied_apu_power_enters_the_shared_bus_and_reports_spill() {
        let system = ElectricalPowerSystem {
            consumers: vec![consumer(
                "essential-load",
                1_000.0,
                PowerPriority::LifeSupport,
            )],
            ..ElectricalPowerSystem::default()
        };
        let state = system.initial_state().expect("bus state");
        let mut command = ElectricalPowerCommand::idle_for(&system, 0.1);
        command.consumer_power_w = vec![300.0];
        command.auxiliary_generation_power_w = 500.0;
        let (_, report) = system.advance(&state, &command).expect("APU bus input");
        assert_eq!(report.auxiliary_generation_power_w, 500.0);
        assert_eq!(report.auxiliary_to_load_power_w, 300.0);
        assert_eq!(report.spilled_auxiliary_power_w, 200.0);
        assert_eq!(report.unserved_power_w, 0.0);

        command.auxiliary_generation_power_w = f64::NAN;
        assert!(system.advance(&state, &command).is_err());
    }

    #[test]
    fn solar_cell_area_incidence_and_visibility_set_output() {
        let system = ElectricalPowerSystem {
            batteries: vec![],
            ultracapacitors: vec![],
            solar_arrays: vec![array(SolarArrayDeployment::Fixed)],
            reactors: vec![],
            fuel_cells: vec![],
            consumers: vec![consumer("avionics", 1_000.0, PowerPriority::FlightControl)],
        };
        let state = system.initial_state().expect("initial state");
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.solar_flux = vec![flux(1_000.0, DVec3::Z)];
        command.consumer_power_w = vec![250.0];
        let (_, aligned) = system.advance(&state, &command).expect("aligned panel");
        assert!((aligned.solar_available_power_w - 250.0).abs() < 1.0e-12);
        assert_eq!(aligned.total_delivered_power_w, 250.0);

        command.solar_flux = vec![flux(1_000.0, DVec3::X)];
        let (_, edge_on) = system.advance(&state, &command).expect("edge-on panel");
        assert_eq!(edge_on.solar_available_power_w, 0.0);
        assert_eq!(edge_on.unserved_power_w, 250.0);

        command.solar_flux = vec![SolarFluxSource::new(1_000.0, DVec3::Z, 0.25).unwrap()];
        let (_, eclipsed) = system.advance(&state, &command).expect("eclipse");
        assert!((eclipsed.solar_available_power_w - 62.5).abs() < 1.0e-12);
    }

    #[test]
    fn inverse_square_stellar_flux_and_panel_mass_are_parametric() {
        let near =
            SolarFluxSource::from_luminosity(1.0e26, 1.0e11, DVec3::Z, 1.0).expect("near flux");
        let far =
            SolarFluxSource::from_luminosity(1.0e26, 2.0e11, DVec3::Z, 1.0).expect("far flux");
        assert!((near.irradiance_w_m2 / far.irradiance_w_m2 - 4.0).abs() < 1.0e-12);
        let panel = array(SolarArrayDeployment::Fixed);
        assert!((panel.mass_kg() - 5.0).abs() < 1.0e-12);
        assert_eq!(panel.inertia_body_kg_m2().z_axis.z, 10.0 / 12.0);
    }

    #[test]
    fn priority_sheds_utilities_before_flight_control_and_shares_equal_tier() {
        let mut source = reactor();
        source.rated_thermal_power_w = 300.0;
        source.radiator_capacity_w = 450.0;
        let system = ElectricalPowerSystem {
            batteries: vec![],
            ultracapacitors: vec![],
            solar_arrays: vec![],
            reactors: vec![source],
            fuel_cells: vec![],
            consumers: vec![
                consumer("life-support", 40.0, PowerPriority::LifeSupport),
                consumer("avionics-a", 50.0, PowerPriority::FlightControl),
                consumer("avionics-b", 30.0, PowerPriority::FlightControl),
                consumer("lights", 100.0, PowerPriority::Utility),
            ],
        };
        let state = system.initial_state().unwrap();
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.consumer_power_w = vec![40.0, 50.0, 30.0, 100.0];
        // A 120 W reactor supplies life support plus a proportional 80 W to
        // equal-priority avionics; no bus capacity is left for utility.
        command.reactor_power_fraction = vec![1.0];
        let (_, report) = system.advance(&state, &command).expect("dispatch");
        assert!((report.consumer_allocations[0].supplied_power_w - 40.0).abs() < 1.0e-12);
        assert!((report.consumer_allocations[1].supplied_power_w - 50.0).abs() < 1.0e-12);
        assert!((report.consumer_allocations[2].supplied_power_w - 30.0).abs() < 1.0e-12);
        assert_eq!(report.consumer_allocations[3].supplied_power_w, 0.0);
        assert_eq!(report.unserved_power_w, 100.0);
    }

    #[test]
    fn battery_charge_and_discharge_obey_energy_efficiency_and_power_caps() {
        let system = ElectricalPowerSystem {
            batteries: vec![battery()],
            ultracapacitors: vec![],
            solar_arrays: vec![array(SolarArrayDeployment::Fixed)],
            reactors: vec![],
            fuel_cells: vec![],
            consumers: vec![consumer("load", 1_000.0, PowerPriority::Utility)],
        };
        let initial = system.initial_state().unwrap();
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.consumer_power_w = vec![600.0];
        let (discharged, report) = system.advance(&initial, &command).expect("battery step");
        assert_eq!(report.total_delivered_power_w, 450.0);
        assert_eq!(report.battery_discharge_power_w, 450.0);
        assert_eq!(discharged.battery_energy_j[0], 0.0);
        assert_eq!(report.unserved_power_w, 150.0);

        let empty = ElectricalPowerState {
            battery_energy_j: vec![0.0],
            ..initial
        };
        command.consumer_power_w = vec![0.0];
        command.solar_flux = vec![flux(2_000.0, DVec3::Z)];
        let (charged, report) = system.advance(&empty, &command).expect("charge step");
        assert_eq!(report.battery_charge_power_w, 400.0);
        assert!((charged.battery_energy_j[0] - 320.0).abs() < 1.0e-12);
    }

    #[test]
    fn reactor_output_closes_thermal_balance_and_fuel_inventory() {
        let source = reactor();
        assert_eq!(source.maximum_electrical_power_w(), 400.0);
        let system = ElectricalPowerSystem {
            batteries: vec![],
            ultracapacitors: vec![],
            solar_arrays: vec![],
            reactors: vec![source],
            fuel_cells: vec![],
            consumers: vec![consumer("life-support", 250.0, PowerPriority::LifeSupport)],
        };
        let state = system.initial_state().unwrap();
        let mut command = ElectricalPowerCommand::idle_for(&system, 2.0);
        command.consumer_power_w = vec![250.0];
        let (next, report) = system.advance(&state, &command).expect("reactor step");
        let reactor = &report.reactors[0];
        assert_eq!(reactor.electrical_power_w, 250.0);
        assert_eq!(reactor.thermal_power_w, 625.0);
        assert_eq!(reactor.waste_heat_w, 375.0);
        assert!((reactor.fuel_consumed_kg - 1.25e-5).abs() < 1.0e-16);
        assert!((next.reactor_fuel_mass_kg[0] - (0.01 - 1.25e-5)).abs() < 1.0e-16);
        assert!(report.reactor_waste_heat_w <= 1_500.0);
    }

    #[test]
    fn foldable_array_motion_is_rate_and_bus_power_limited() {
        let system = ElectricalPowerSystem {
            batteries: vec![],
            ultracapacitors: vec![],
            solar_arrays: vec![array(SolarArrayDeployment::Foldable {
                deployment_rate_per_s: 0.25,
                actuator_power_w: 100.0,
                initial_fraction: 0.0,
            })],
            reactors: vec![reactor()],
            fuel_cells: vec![],
            consumers: vec![consumer(
                "life-support",
                1_000.0,
                PowerPriority::LifeSupport,
            )],
        };
        let state = system.initial_state().unwrap();
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.consumer_power_w = vec![400.0];
        command.solar_deployment_targets = vec![Some(1.0)];
        command.reactor_power_fraction = vec![0.5];
        let (next, report) = system.advance(&state, &command).expect("deployment");
        // Reactor supplies 200 W; the higher-priority life-support load gets
        // 200 W and the lower-priority deployment motor remains unpowered.
        assert_eq!(report.consumer_allocations[0].supplied_power_w, 200.0);
        assert_eq!(next.solar_array_deployed_fraction[0], 0.0);

        command.consumer_power_w = vec![100.0];
        let (next, report) = system
            .advance(&state, &command)
            .expect("deploy with headroom");
        assert_eq!(report.solar_arrays[0].deployment_power_w, 100.0);
        assert_eq!(next.solar_array_deployed_fraction[0], 0.25);
    }

    #[test]
    fn ultracapacitor_burst_discharge_and_charge_share_the_bus() {
        let system = ElectricalPowerSystem {
            batteries: vec![battery()],
            ultracapacitors: vec![capacitor()],
            solar_arrays: vec![],
            reactors: vec![],
            fuel_cells: vec![],
            consumers: vec![consumer("pulse-load", 9_000.0, PowerPriority::Utility)],
        };
        assert!((capacitor().mass_kg() - 0.01).abs() < 1.0e-12);
        let initial = system.initial_state().unwrap();
        assert_eq!(initial.capacitor_energy_j[0], 100.0);
        // 10 ms pulse: 500 W battery + 8000 W ultracapacitor cover most of
        // the 9000 W request; the shortfall is shed, not invented.
        let mut command = ElectricalPowerCommand::idle_for(&system, 0.01);
        command.consumer_power_w = vec![9_000.0];
        let (next, report) = system.advance(&initial, &command).expect("pulse step");
        assert_eq!(report.total_delivered_power_w, 8_500.0);
        assert_eq!(report.unserved_power_w, 500.0);
        assert_eq!(report.battery_discharge_power_w, 500.0);
        assert_eq!(report.capacitor_discharge_power_w, 8_000.0);
        assert!((next.capacitor_energy_j[0] - (100.0 - 80.0 / 0.97)).abs() < 1.0e-9);
        assert!((next.battery_energy_j[0] - (500.0 - 5.0 / 0.9)).abs() < 1.0e-9);

        // Recharge from a reactor surplus splits across both stores.
        let mut charge = ElectricalPowerCommand::idle_for(&system, 1.0);
        charge.reactor_power_fraction = vec![1.0];
        let system = ElectricalPowerSystem {
            reactors: vec![reactor()],
            consumers: vec![consumer("trickle", 50.0, PowerPriority::Utility)],
            batteries: vec![battery()],
            ultracapacitors: vec![capacitor()],
            solar_arrays: vec![],
            fuel_cells: vec![],
        };
        let empty = ElectricalPowerState {
            battery_energy_j: vec![0.0],
            capacitor_energy_j: vec![0.0],
            reactor_fuel_mass_kg: vec![0.01],
            solar_array_deployed_fraction: vec![],
            solar_array_tracking_angle_rad: vec![],
        };
        let (charged, report) = system.advance(&empty, &charge).expect("charge step");
        assert!(report.capacitor_charge_power_w > 0.0);
        assert!(report.battery_charge_power_w > 0.0);
        assert!(charged.capacitor_energy_j[0] > 0.0);
        assert!(charged.battery_energy_j[0] > 0.0);
    }

    #[test]
    fn occluders_dim_sources_by_disc_overlap() {
        // Total eclipse by a larger disc: nothing reaches the cells.
        let blocked = SolarFluxSource::from_luminosity_with_occluders(
            1.0e26,
            7.0e8,
            1.5e11,
            DVec3::Z,
            1.0,
            vec![SolarOccluder::new(DVec3::Z, 0.02).unwrap()],
        )
        .expect("eclipsed source");
        assert_eq!(blocked.effective_visibility(), 0.0);
        assert_eq!(blocked.effective_irradiance_w_m2(), 0.0);

        // Clear sky beside the occluder: full illumination.
        let clear = SolarFluxSource::from_luminosity_with_occluders(
            1.0e26,
            7.0e8,
            1.5e11,
            DVec3::Z,
            1.0,
            vec![SolarOccluder::new(DVec3::X, 0.02).unwrap()],
        )
        .expect("clear source");
        assert_eq!(clear.effective_visibility(), 1.0);

        // Annular transit: a small craft blocks (ro/rl)^2 of the disc.
        let light_radius = 0.02;
        let mut annular = SolarFluxSource::new(1_000.0, DVec3::Z, 1.0).unwrap();
        annular.light_angular_radius_rad = light_radius;
        annular.occluders = vec![SolarOccluder::new(DVec3::Z, 0.01).unwrap()];
        assert!((annular.effective_visibility() - 0.75).abs() < 1.0e-9);

        // Coincident shadows cover the same area only once.
        let mut duplicate = annular.clone();
        duplicate
            .occluders
            .push(SolarOccluder::new(DVec3::Z, 0.01).unwrap());
        assert!((duplicate.effective_visibility() - 0.75).abs() < 1.0e-9);

        // Partially overlapping discs subtract their shared lens only once.
        let offset_rad: f64 = 0.005;
        let offset_direction = DVec3::new(offset_rad.sin(), 0.0, offset_rad.cos());
        let mut partial_overlap = annular.clone();
        partial_overlap.occluders = vec![
            SolarOccluder::new(DVec3::Z, 0.01).unwrap(),
            SolarOccluder::new(offset_direction, 0.01).unwrap(),
        ];
        let shared_area = circle_overlap_area(0.01, 0.01, offset_rad);
        let expected_visibility = 1.0
            - (2.0 * std::f64::consts::PI * 0.01_f64.powi(2) - shared_area)
                / (std::f64::consts::PI * light_radius.powi(2));
        assert!((partial_overlap.effective_visibility() - expected_visibility).abs() < 1.0e-9);

        // Separated discs inside the stellar limb contribute disjoint areas.
        let disjoint_radius = 0.005;
        let disjoint_offset: f64 = 0.01;
        let mut disjoint = annular.clone();
        disjoint.occluders = vec![
            SolarOccluder::new(
                DVec3::new(disjoint_offset.sin(), 0.0, disjoint_offset.cos()),
                disjoint_radius,
            )
            .unwrap(),
            SolarOccluder::new(
                DVec3::new(-disjoint_offset.sin(), 0.0, disjoint_offset.cos()),
                disjoint_radius,
            )
            .unwrap(),
        ];
        let expected_disjoint_visibility =
            1.0 - 2.0 * disjoint_radius.powi(2) / light_radius.powi(2);
        assert!((disjoint.effective_visibility() - expected_disjoint_visibility).abs() < 1.0e-9);

        // A near-tangent contact must not classify a whole boundary arc as
        // eclipsed when another disc sends the calculation through the
        // multi-occluder union path.
        let tangent_separation = light_radius + 0.01 - 1.0e-12;
        let mut tangent = annular.clone();
        tangent.occluders = vec![
            SolarOccluder::new(
                DVec3::new(0.0, -tangent_separation.sin(), tangent_separation.cos()),
                0.01,
            )
            .unwrap(),
            SolarOccluder::new(DVec3::Z, 0.002).unwrap(),
        ];
        let expected_tangent_visibility = 1.0 - 0.002_f64.powi(2) / light_radius.powi(2);
        assert!((tangent.effective_visibility() - expected_tangent_visibility).abs() < 1.0e-9);

        // Occluders without a stellar disc fail closed instead of silently
        // passing full sun.
        let mut no_disc = SolarFluxSource::new(1_000.0, DVec3::Z, 1.0).unwrap();
        no_disc.occluders = vec![SolarOccluder::new(DVec3::Z, 0.01).unwrap()];
        let system = ElectricalPowerSystem {
            batteries: vec![],
            ultracapacitors: vec![],
            solar_arrays: vec![array(SolarArrayDeployment::Fixed)],
            reactors: vec![],
            fuel_cells: vec![],
            consumers: vec![],
        };
        let state = system.initial_state().unwrap();
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.solar_flux = vec![no_disc];
        assert!(system.advance(&state, &command).is_err());
    }

    #[test]
    fn tracking_array_turns_toward_the_strongest_of_three_suns() {
        // Panel reference normal +Z, slew axis +X: positive rotation tips
        // the normal toward -Y, negative toward +Y. Three suns: weak +Z,
        // strong +Y tilted 30 degrees off +Z, medium -Y. The optimum tips
        // toward +Y at about -30 degrees.
        let tilted = (DVec3::Y * 0.5 + DVec3::Z * 0.866_025_403_784_438_6).normalize();
        let system = ElectricalPowerSystem {
            batteries: vec![],
            ultracapacitors: vec![],
            solar_arrays: vec![tracking_array()],
            reactors: vec![reactor()],
            fuel_cells: vec![],
            consumers: vec![consumer("avionics", 2_000.0, PowerPriority::FlightControl)],
        };
        let state = system.initial_state().unwrap();
        assert_eq!(state.solar_array_tracking_angle_rad[0], 0.0);
        let mut command = ElectricalPowerCommand::idle_for(&system, 2.0);
        command.solar_flux = vec![
            flux(400.0, DVec3::Z),
            SolarFluxSource::new(1_600.0, tilted, 1.0).unwrap(),
            flux(700.0, -DVec3::Y),
        ];
        command.consumer_power_w = vec![300.0];
        command.solar_tracking_auto = vec![true];
        command.reactor_power_fraction = vec![0.0];
        let (next, report) = system.advance(&state, &command).expect("track step");
        // Slew rate 0.5 rad/s over 2 s reaches the optimum, and the
        // reduced consumer load leaves bus headroom for the motor. The
        // optimum (~-24 deg) balances the strong tilted sun against the
        // weaker zenith sun rather than facing either exactly.
        let angle = next.solar_array_tracking_angle_rad[0];
        assert!((-0.55..=-0.30).contains(&angle), "angle was {angle}");
        assert!(report.solar_arrays[0].tracking_power_w > 0.0);
        assert_eq!(
            report.solar_arrays[0].tracking_target_rad,
            next.solar_array_tracking_angle_rad[0]
        );
        // Facing the tilted sun beats the reference pose.
        let faced = tracking_objective(
            &tracking_array(),
            angle,
            &command.solar_flux,
            &[400.0, 1_600.0, 700.0],
        );
        let reference = tracking_objective(
            &tracking_array(),
            0.0,
            &command.solar_flux,
            &[400.0, 1_600.0, 700.0],
        );
        assert!(faced > reference);

        // Total eclipse: the drive holds instead of hunting.
        let mut dark = command.clone();
        dark.solar_flux = vec![SolarFluxSource::new(1_600.0, tilted, 0.0).unwrap()];
        let (held, _) = system.advance(&next, &dark).expect("eclipse hold");
        assert_eq!(
            held.solar_array_tracking_angle_rad[0],
            next.solar_array_tracking_angle_rad[0]
        );
    }

    #[test]
    fn tracking_optimum_matches_brute_force_within_envelope() {
        // Three fixed suns; compare the 72-sample search against a 3600-step
        // brute force. The fast optimum must recover >= 99.5% of the brute
        // projected irradiance (absolute error envelope, not relative near
        // zero: the scale here is ~kW/m^2).
        let suns = [
            (1_200.0, DVec3::new(0.3, 0.8, 0.5).normalize()),
            (900.0, DVec3::new(-0.7, 0.2, 0.6).normalize()),
            (500.0, DVec3::new(0.1, -0.9, 0.4).normalize()),
        ];
        let sources: Vec<SolarFluxSource> = suns
            .iter()
            .map(|(power, dir)| SolarFluxSource::new(*power, *dir, 1.0).unwrap())
            .collect();
        let weights: Vec<f64> = suns.iter().map(|(power, _)| *power).collect();
        let panel = tracking_array();
        let fast = best_tracking_angle(&panel, 0.0, &sources, &weights);
        let fast_value = tracking_objective(&panel, fast, &sources, &weights);
        let mut brute_best = 0.0_f64;
        for sample in 0..=3600 {
            let angle =
                -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * sample as f64 / 3600.0;
            brute_best = brute_best.max(tracking_objective(&panel, angle, &sources, &weights));
        }
        assert!(brute_best > 100.0);
        assert!(
            fast_value >= 0.995 * brute_best,
            "fast {fast_value} vs brute {brute_best}"
        );
    }

    #[test]
    fn invalid_step_is_atomic_and_luminosity_inputs_reject_garbage() {
        let system = ElectricalPowerSystem {
            batteries: vec![battery()],
            ..ElectricalPowerSystem::default()
        };
        let state = system.initial_state().unwrap();
        let mut command = ElectricalPowerCommand::idle_for(&system, 1.0);
        command.dt_s = 0.0;
        assert!(system.advance(&state, &command).is_err());
        assert_eq!(state.battery_energy_j[0], 500.0);
        assert!(SolarFluxSource::from_luminosity(1.0, 0.0, DVec3::Z, 1.0).is_err());
    }
}
