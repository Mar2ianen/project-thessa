//! Engine, tank, jet, electric, fusion, and drive asset compilation.

use super::*;

mod advanced_propulsion_assets;
pub(super) use advanced_propulsion_assets::*;

/// One procedural engine mount from the source vehicle asset.
///
/// Example TOML (liquid):
///
/// ```text
/// [[engines]]
/// name = "main"
/// kind = "liquid"
/// mount_position_body_m = [-3.0, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// propellant = "lox-methane"
/// cycle = "gas-generator"
/// chamber_pressure_mpa = 12.0
/// throat_radius_m = 0.15
/// expansion_ratio = 35.0
/// nozzle_length_m = 1.8
/// contour = "bell"
/// material = "nickel-superalloy"
/// cooling = "regenerative"
/// gimbal_range_rad = 0.09
/// restartable = true
/// ```
///
/// Solid motors use `kind = "solid"` with grain fields
/// (`outer_radius_m`, `core_radius_m`, `segment_length_m`, `segments`,
/// optional APCP ballistics overrides, `ignition_shots`) instead of the
/// chamber/cycle fields. Optional `grain_geometry` selects star or finocyl
/// port burnback; omitted geometry is circular BATES:
///
/// ```text
/// grain_geometry = { kind = "star", tip_count = 6, tip_radius_m = 0.30 }
/// grain_geometry = { kind = "finocyl", fin_count = 8, fin_tip_radius_m = 0.32, fin_width_rad = 0.24 }
/// ```
#[derive(Debug, Deserialize)]
pub(crate) struct EngineAsset {
    name: String,
    kind: EngineKind,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    // Liquid fields.
    #[serde(default)]
    propellant: Option<Propellant>,
    #[serde(default)]
    cycle: Option<EngineCycle>,
    #[serde(default)]
    chamber_pressure_mpa: Option<f64>,
    #[serde(default)]
    throat_radius_m: Option<f64>,
    #[serde(default)]
    expansion_ratio: Option<f64>,
    #[serde(default)]
    nozzle_length_m: Option<f64>,
    #[serde(default)]
    contour: Option<NozzleContour>,
    #[serde(default)]
    material: Option<MaterialAsset>,
    #[serde(default)]
    cooling: Option<CoolingMode>,
    #[serde(default)]
    mixture_ratio: Option<f64>,
    #[serde(default)]
    gimbal_range_rad: f64,
    #[serde(default)]
    min_throttle: Option<f64>,
    #[serde(default = "default_true")]
    restartable: bool,
    // Solid grain fields.
    #[serde(default)]
    outer_radius_m: Option<f64>,
    #[serde(default)]
    core_radius_m: Option<f64>,
    #[serde(default)]
    segment_length_m: Option<f64>,
    #[serde(default)]
    segments: Option<u32>,
    #[serde(default)]
    segment_core_radii_m: Option<Vec<f64>>,
    #[serde(default)]
    grain_geometry: SolidGrainGeometry,
    #[serde(default)]
    burn_rate_coeff: Option<f64>,
    #[serde(default)]
    burn_rate_exponent: Option<f64>,
    #[serde(default = "default_one_shot")]
    ignition_shots: u32,
    // Nuclear thermal fields.
    #[serde(default)]
    fluid: Option<NtrFluid>,
    #[serde(default)]
    core_temp_k: Option<f64>,
    #[serde(default)]
    core_power_mw: Option<f64>,
    #[serde(default)]
    reactor_specific_mass_kg_per_mw: Option<f64>,
    #[serde(default)]
    startup_tau_s: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum EngineKind {
    Liquid,
    Solid,
    Nuclear,
}
#[derive(Debug, Deserialize)]
#[serde(untagged)]
pub(crate) enum MaterialAsset {
    Preset(String),
    Custom(ChamberMaterial),
}

impl EngineAsset {
    pub(crate) fn bake(self) -> Result<EngineMount, Box<dyn Error>> {
        let compiled = match self.kind {
            EngineKind::Liquid => CompiledEngine::Liquid(self.liquid_spec()?.compile()?),
            EngineKind::Solid => CompiledEngine::Solid(self.solid_spec()?.compile()?),
            EngineKind::Nuclear => {
                let (engine, supplement) = self.nuclear_spec()?.compile()?;
                println!(
                    "engine {} (ntr): {:.0} MWt at {:.0} K, reactor {:.0} kg, startup {:.0} s",
                    self.name,
                    supplement.core_power_mw,
                    supplement.core_temp_k,
                    supplement.reactor_mass_kg,
                    supplement.startup_tau_s,
                );
                CompiledEngine::Liquid(engine)
            }
        };
        Ok(EngineMount {
            name: self.name.clone(),
            engine: compiled,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }

    fn material(&self) -> Result<ChamberMaterial, Box<dyn Error>> {
        let asset = self.material.as_ref().ok_or("engine needs material")?;
        match asset {
            MaterialAsset::Custom(material) => Ok(*material),
            MaterialAsset::Preset(name) => match name.as_str() {
                "regen-alloy" => Ok(ChamberMaterial::regen_alloy()),
                "nickel-superalloy" => Ok(ChamberMaterial::nickel_superalloy()),
                "radiative-niobium" => Ok(ChamberMaterial::radiative_niobium()),
                "ablative" => Ok(ChamberMaterial::ablative()),
                unknown => Err(format!("unknown material preset {unknown}").into()),
            },
        }
    }

    fn liquid_spec(&self) -> Result<LiquidEngineSpec, Box<dyn Error>> {
        Ok(LiquidEngineSpec {
            name: self.name.clone(),
            propellant: self.propellant.ok_or("liquid engine needs propellant")?,
            cycle: self.cycle.ok_or("liquid engine needs cycle")?,
            chamber_pressure_pa: required_mpa(self.chamber_pressure_mpa, "chamber_pressure_mpa")?,
            throat_radius_m: self
                .throat_radius_m
                .ok_or("liquid engine needs throat_radius_m")?,
            expansion_ratio: self
                .expansion_ratio
                .ok_or("liquid engine needs expansion_ratio")?,
            nozzle_length_m: self
                .nozzle_length_m
                .ok_or("liquid engine needs nozzle_length_m")?,
            contour: self.contour.unwrap_or(NozzleContour::Bell),
            chamber_material: self.material()?,
            cooling: self.cooling.unwrap_or(CoolingMode::Regenerative),
            mixture_ratio: self.mixture_ratio,
            characteristic_length_m: None,
            gimbal_range_rad: self.gimbal_range_rad,
            min_throttle: self.min_throttle,
            restartable: self.restartable,
        })
    }

    fn solid_spec(&self) -> Result<SolidMotorSpec, Box<dyn Error>> {
        let (default_a, default_n) = SolidMotorSpec::apcp_ballistics();
        Ok(SolidMotorSpec {
            name: self.name.clone(),
            propellant: self.propellant.unwrap_or(Propellant::SolidApcp),
            outer_radius_m: self
                .outer_radius_m
                .ok_or("solid motor needs outer_radius_m")?,
            core_radius_m: self
                .core_radius_m
                .ok_or("solid motor needs core_radius_m")?,
            grain_geometry: self.grain_geometry,
            segment_length_m: self
                .segment_length_m
                .ok_or("solid motor needs segment_length_m")?,
            segments: self.segments.ok_or("solid motor needs segments")?,
            burn_rate_coeff: self.burn_rate_coeff.unwrap_or(default_a),
            burn_rate_exponent: self.burn_rate_exponent.unwrap_or(default_n),
            throat_radius_m: self
                .throat_radius_m
                .ok_or("solid motor needs throat_radius_m")?,
            expansion_ratio: self
                .expansion_ratio
                .ok_or("solid motor needs expansion_ratio")?,
            nozzle_length_m: self
                .nozzle_length_m
                .ok_or("solid motor needs nozzle_length_m")?,
            contour: self.contour.unwrap_or(NozzleContour::Conical),
            casing_material: self.material()?,
            inhibited_ends: true,
            segment_core_radii_m: self.segment_core_radii_m.clone(),
            gimbal_range_rad: self.gimbal_range_rad,
            ignition_shots: self.ignition_shots,
        })
    }

    fn nuclear_spec(&self) -> Result<NuclearThermalSpec, Box<dyn Error>> {
        Ok(NuclearThermalSpec {
            name: self.name.clone(),
            fluid: self.fluid.ok_or("nuclear engine needs fluid")?,
            core_temp_k: self.core_temp_k.ok_or("nuclear engine needs core_temp_k")?,
            core_power_mw: self
                .core_power_mw
                .ok_or("nuclear engine needs core_power_mw")?,
            reactor_specific_mass_kg_per_mw: self.reactor_specific_mass_kg_per_mw,
            throat_radius_m: self
                .throat_radius_m
                .ok_or("nuclear engine needs throat_radius_m")?,
            expansion_ratio: self
                .expansion_ratio
                .ok_or("nuclear engine needs expansion_ratio")?,
            nozzle_length_m: self
                .nozzle_length_m
                .ok_or("nuclear engine needs nozzle_length_m")?,
            contour: self.contour.unwrap_or(NozzleContour::Bell),
            material: self.material()?,
            cooling: self.cooling.unwrap_or(CoolingMode::Regenerative),
            gimbal_range_rad: self.gimbal_range_rad,
            startup_tau_s: self.startup_tau_s,
            min_throttle: self.min_throttle,
        })
    }
}

fn required_mpa(value: Option<f64>, name: &str) -> Result<f64, Box<dyn Error>> {
    match value {
        Some(mpa) if mpa.is_finite() && mpa > 0.0 => Ok(mpa * 1.0e6),
        _ => Err(format!("liquid engine needs {name} in MPa").into()),
    }
}

/// One propellant tank from the source vehicle asset.
///
/// Example TOML:
///
/// ```text
/// [[tanks]]
/// name = "methane-tank"
/// shape = "cylinder"
/// diameter_m = 1.3
/// length_m = 3.0
/// pressure_mpa = 0.5
/// material = "regen-alloy"
/// position_body_m = [1.0, 0.0, 0.0]
/// propellant = "lox-methane"
/// ```
#[derive(Debug, Deserialize)]
pub(crate) struct TankAsset {
    name: String,
    shape: TankShapeAsset,
    diameter_m: f64,
    #[serde(default)]
    length_m: Option<f64>,
    pressure_mpa: f64,
    material: MaterialAsset,
    #[serde(default = "mount_position_default")]
    position_body_m: [f64; 3],
    /// Bulk density source: named propellant or an explicit value (one is
    /// required so tank fill mass stays physical).
    #[serde(default)]
    propellant: Option<Propellant>,
    #[serde(default)]
    bulk_density_kg_m3: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum TankShapeAsset {
    Sphere,
    Cylinder,
}

impl TankAsset {
    pub(crate) fn bake(self) -> Result<TankMount, Box<dyn Error>> {
        if self.name.trim().is_empty() {
            return Err("tank needs a name".into());
        }
        let shape = match self.shape {
            TankShapeAsset::Sphere => TankShape::Sphere {
                diameter_m: self.diameter_m,
            },
            TankShapeAsset::Cylinder => TankShape::Cylinder {
                diameter_m: self.diameter_m,
                length_m: self.length_m.ok_or("cylindrical tanks need length_m")?,
            },
        };
        if !self.pressure_mpa.is_finite() || self.pressure_mpa <= 0.0 {
            return Err("tank pressure_mpa must be finite and > 0".into());
        }
        let density = match (self.propellant, self.bulk_density_kg_m3) {
            (_, Some(density)) if density.is_finite() && density > 0.0 => density,
            (Some(propellant), None) => propellant.thermo().bulk_density_kg_m3,
            _ => {
                return Err(
                    format!("tank {} needs propellant or bulk_density_kg_m3", self.name).into(),
                );
            }
        };
        let spec = TankSpec {
            shape,
            pressure_pa: self.pressure_mpa * 1.0e6,
            material: match &self.material {
                MaterialAsset::Custom(material) => *material,
                MaterialAsset::Preset(name) => match name.as_str() {
                    "regen-alloy" => ChamberMaterial::regen_alloy(),
                    "nickel-superalloy" => ChamberMaterial::nickel_superalloy(),
                    "radiative-niobium" => ChamberMaterial::radiative_niobium(),
                    "ablative" => ChamberMaterial::ablative(),
                    unknown => {
                        return Err(format!("unknown material preset {unknown}").into());
                    }
                },
            },
        };
        let tank = spec.compile(density)?;
        let intrinsic_inertia_body_kg_m2 =
            shape.intrinsic_inertia_body_kg_m2(tank.dry_mass_kg, tank.full_propellant_kg)?;
        let initial_propellant_kg = tank.full_propellant_kg;
        Ok(TankMount {
            name: self.name,
            tank,
            position_body_m: self.position_body_m,
            intrinsic_inertia_body_kg_m2,
            initial_propellant_kg: Some(initial_propellant_kg),
            resource: self.propellant.map(TankResource::Pair).unwrap_or_default(),
        })
    }
}

/// One chamber of a multi-chamber system from the source vehicle asset.
///
/// Example TOML:
///
/// ```text
/// [[systems]]
/// name = "quad"
/// propellant = "lox-rp1"
/// cycle = "gas-generator"
/// chamber_pressure_mpa = 9.7
/// material = "nickel-superalloy"
/// cooling = "regenerative"
///
/// [[systems.chambers]]
/// name = "a"
/// throat_radius_m = 0.134
/// expansion_ratio = 16.0
/// nozzle_length_m = 1.5
/// contour = "bell"
/// position_body_m = [-3.0, 0.0, 0.5]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// gimbal_range_rad = 0.09
/// ```
#[derive(Debug, Deserialize)]
pub(crate) struct ChamberAsset {
    name: String,
    throat_radius_m: f64,
    expansion_ratio: f64,
    nozzle_length_m: f64,
    #[serde(default = "bell_contour")]
    contour: NozzleContour,
    #[serde(default = "mount_position_default")]
    position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    #[serde(default)]
    gimbal_range_rad: f64,
}

fn bell_contour() -> NozzleContour {
    NozzleContour::Bell
}

impl ChamberAsset {
    pub(crate) fn bake(self) -> Result<ChamberSpec, Box<dyn Error>> {
        Ok(ChamberSpec {
            name: self.name,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            nozzle_length_m: self.nozzle_length_m,
            contour: self.contour,
            position_body_m: self.position_body_m,
            thrust_axis_body: self.thrust_axis_body,
            gimbal_range_rad: self.gimbal_range_rad,
        })
    }
}

/// One multi-chamber propulsion system: shared feed plus chamber list.
#[derive(Debug, Deserialize)]
pub(crate) struct SystemAsset {
    name: String,
    propellant: Propellant,
    cycle: EngineCycle,
    chamber_pressure_mpa: f64,
    #[serde(default)]
    mixture_ratio: Option<f64>,
    material: MaterialAsset,
    #[serde(default = "regen_cooling")]
    cooling: CoolingMode,
    #[serde(default)]
    characteristic_length_m: Option<f64>,
    #[serde(default)]
    min_throttle: Option<f64>,
    #[serde(default = "default_true")]
    restartable: bool,
    chambers: Vec<ChamberAsset>,
}

fn regen_cooling() -> CoolingMode {
    CoolingMode::Regenerative
}

impl SystemAsset {
    pub(crate) fn bake(self) -> Result<SystemMount, Box<dyn Error>> {
        if !self.chamber_pressure_mpa.is_finite() || self.chamber_pressure_mpa <= 0.0 {
            return Err("system needs chamber_pressure_mpa in MPa".into());
        }
        let material = match &self.material {
            MaterialAsset::Custom(material) => *material,
            MaterialAsset::Preset(name) => match name.as_str() {
                "regen-alloy" => ChamberMaterial::regen_alloy(),
                "nickel-superalloy" => ChamberMaterial::nickel_superalloy(),
                "radiative-niobium" => ChamberMaterial::radiative_niobium(),
                "ablative" => ChamberMaterial::ablative(),
                unknown => return Err(format!("unknown material preset {unknown}").into()),
            },
        };
        let chambers = self
            .chambers
            .into_iter()
            .map(ChamberAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let spec = PropulsionSystemSpec {
            name: self.name.clone(),
            propellant: self.propellant,
            cycle: self.cycle,
            chamber_pressure_pa: self.chamber_pressure_mpa * 1.0e6,
            mixture_ratio: self.mixture_ratio,
            chamber_material: material,
            cooling: self.cooling,
            characteristic_length_m: self.characteristic_length_m,
            min_throttle: self.min_throttle,
            restartable: self.restartable,
            chambers,
        };
        Ok(SystemMount {
            name: self.name,
            system: spec.compile()?,
        })
    }
}

/// One air-breathing jet or ESTOC from the source vehicle asset.
///
/// Example TOML (turbojet):
///
/// ```text
/// [[jets]]
/// name = "cruise-jet"
/// kind = "jet"
/// mount_position_body_m = [2.0, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// fuel = "kerosene"
/// intake_area_m2 = 0.5
/// intake = "pitot"
/// compressor_ratio = 8.0
/// turbine_inlet_temp_k = 1400.0
/// material = "nickel-superalloy"
/// ```
///
/// Example TOML (ESTOC adds the rocket block):
///
/// ```text
/// [[jets]]
/// name = "estoc-1"
/// kind = "estoc"
/// mount_position_body_m = [0.0, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// fuel = "kerosene"
/// intake_area_m2 = 0.9
/// intake = "pitot"
/// compressor_ratio = 12.0
/// turbine_inlet_temp_k = 1500.0
/// material = "nickel-superalloy"
/// rocket_chamber_pressure_mpa = 7.0
/// rocket_throat_radius_m = 0.09
/// ```
///
/// Optional shaft topology block (section 8.1): starter, generator, and
/// light-off/self-sustain thresholds. Omitted = inert default
/// (starterless, windmill/relight only):
///
/// ```text
/// [jets.shaft]
/// light_off_n = 0.15
/// self_sustain_n = 0.10
///
/// [jets.shaft.starter]
/// kind = "electric"       # "none" | "electric" | "pneumatic" | "rocket-bootstrap"
/// power_w = 200000.0
/// charge_j = 20000000.0
/// mass_kg = 12.0
///
/// [jets.shaft.generator]
/// fitted = true
/// power_w = 50000.0
/// efficiency = 0.92
/// cut_in_spool_n = 0.5
/// mass_kg = 25.0
/// ```
#[derive(Debug, Deserialize)]
pub(crate) struct JetAsset {
    name: String,
    kind: JetKind,
    /// Airbreathing cycle topology; omitted assets keep the legacy inference
    /// from bypass ratio (turbofan if positive, turbojet otherwise).
    #[serde(default)]
    cycle: Option<AirCycle>,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    #[serde(default)]
    gimbal_range_rad: f64,
    fuel: JetFuel,
    intake_area_m2: f64,
    #[serde(default = "pitot_intake")]
    intake: IntakeKind,
    #[serde(default = "default_compressor_ratio")]
    compressor_ratio: f64,
    #[serde(default)]
    bypass_ratio: f64,
    #[serde(default = "default_fan_ratio")]
    fan_pressure_ratio: f64,
    turbine_inlet_temp_k: f64,
    #[serde(default)]
    afterburner: bool,
    #[serde(default)]
    reheat_temp_k: f64,
    material: MaterialAsset,
    #[serde(default = "default_spool_tau")]
    spool_tau_s: f64,
    /// Shaft topology block (`[jets.shaft]`): starter, generator,
    /// light-off/self-sustain. Inert default when omitted.
    #[serde(default)]
    shaft: ShaftSpec,
    // ESTOC-only rocket block.
    #[serde(default)]
    bulk_fuel: Option<JetFuel>,
    #[serde(default)]
    boost_coolant_fuel: Option<JetFuel>,
    #[serde(default)]
    precooler: Option<EstocPrecoolerSpec>,
    #[serde(default)]
    ejector: Option<EstocEjectorSpec>,
    #[serde(default)]
    rocket_chamber_pressure_mpa: Option<f64>,
    #[serde(default)]
    rocket_throat_radius_m: Option<f64>,
    #[serde(default)]
    oxidizer_fuel_ratio: Option<f64>,
    #[serde(default)]
    switch_mach_hi: Option<f64>,
    #[serde(default)]
    switch_mach_lo: Option<f64>,
    #[serde(default)]
    transition_tau_s: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum JetKind {
    Jet,
    Estoc,
}

fn pitot_intake() -> IntakeKind {
    IntakeKind::Pitot
}

fn default_compressor_ratio() -> f64 {
    8.0
}

fn default_fan_ratio() -> f64 {
    1.6
}

fn default_spool_tau() -> f64 {
    5.0
}

impl JetAsset {
    fn air_spec(&self) -> Result<AirbreathingSpec, Box<dyn Error>> {
        Ok(AirbreathingSpec {
            name: self.name.clone(),
            cycle: self.cycle.unwrap_or({
                if self.bypass_ratio > 0.0 {
                    AirCycle::Turbofan
                } else {
                    AirCycle::Turbojet
                }
            }),
            fuel: self.fuel,
            intake_area_m2: self.intake_area_m2,
            intake: self.intake,
            compressor_ratio: self.compressor_ratio,
            bypass_ratio: self.bypass_ratio,
            fan_pressure_ratio: self.fan_pressure_ratio,
            turbine_inlet_temp_k: self.turbine_inlet_temp_k,
            afterburner: self.afterburner,
            reheat_temp_k: self.reheat_temp_k,
            turbine_material: self.material()?,
            spool_tau_s: self.spool_tau_s,
            shaft: self.shaft.clone(),
        })
    }

    fn material(&self) -> Result<ChamberMaterial, Box<dyn Error>> {
        match &self.material {
            MaterialAsset::Custom(material) => Ok(*material),
            MaterialAsset::Preset(name) => match name.as_str() {
                "regen-alloy" => Ok(ChamberMaterial::regen_alloy()),
                "nickel-superalloy" => Ok(ChamberMaterial::nickel_superalloy()),
                "radiative-niobium" => Ok(ChamberMaterial::radiative_niobium()),
                "ablative" => Ok(ChamberMaterial::ablative()),
                unknown => Err(format!("unknown material preset {unknown}").into()),
            },
        }
    }

    pub(crate) fn bake(self) -> Result<JetMount, Box<dyn Error>> {
        let engine = match self.kind {
            JetKind::Jet => CompiledJet::Air(Box::new(self.air_spec()?.compile()?)),
            JetKind::Estoc => {
                let spec = EstocSpec {
                    name: self.name.clone(),
                    air: self.air_spec()?,
                    bulk_fuel: self.bulk_fuel,
                    boost_coolant_fuel: self.boost_coolant_fuel,
                    precooler: self.precooler,
                    ejector: self.ejector,
                    rocket_chamber_pressure_pa: required_mpa(
                        self.rocket_chamber_pressure_mpa,
                        "rocket_chamber_pressure_mpa",
                    )?,
                    rocket_throat_radius_m: self
                        .rocket_throat_radius_m
                        .ok_or("estoc needs rocket_throat_radius_m")?,
                    oxidizer_fuel_ratio: self.oxidizer_fuel_ratio,
                    switch_mach_hi: self.switch_mach_hi,
                    switch_mach_lo: self.switch_mach_lo,
                    transition_tau_s: self.transition_tau_s,
                };
                let compiled = spec.compile()?;
                println!(
                    "jet {} (estoc): rocket {:.0} kN vac, shared expansion {:.1}x",
                    self.name,
                    compiled.rocket_thrust_vac_n / 1000.0,
                    compiled.rocket_expansion_ratio,
                );
                CompiledJet::Estoc(Box::new(compiled))
            }
        };
        Ok(JetMount {
            name: self.name,
            engine,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
            gimbal_range_rad: self.gimbal_range_rad,
        })
    }
}

/// One body-local collision primitive from the source vehicle asset.
///
/// Example TOML:
///
/// ```text
/// [[collision_parts]]
/// shape = "capsule"
/// position_body_m = [0.0, 0.0, 0.0]
/// orientation_body_xyzw = [0.0, 0.0, 0.0, 1.0]
/// axis = "x"
/// half_segment_m = 2.0
/// radius_m = 0.5
/// friction = 0.7
/// restitution = 0.0
/// ```
#[derive(Debug, Deserialize)]
pub(crate) struct CollisionPartAsset {
    #[serde(default)]
    position_body_m: [f64; 3],
    #[serde(default = "identity_quaternion")]
    orientation_body_xyzw: [f64; 4],
    #[serde(default = "default_friction")]
    friction: f64,
    #[serde(default)]
    restitution: f64,
    #[serde(flatten)]
    shape: CollisionShapeAsset,
}

impl CollisionPartAsset {
    pub(crate) fn bake(self) -> Result<CollisionPart, Box<dyn Error>> {
        let [x, y, z, w] = self.orientation_body_xyzw;
        Ok(CollisionPart::new(
            vector(self.position_body_m),
            DQuat::from_xyzw(x, y, z, w),
            self.shape.bake(),
            CollisionMaterial::new(self.friction, self.restitution)?,
        )?)
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "shape", rename_all = "kebab-case")]
pub(crate) enum CollisionShapeAsset {
    Sphere {
        radius_m: f64,
    },
    Cuboid {
        half_extents_m: [f64; 3],
    },
    Capsule {
        axis: CollisionAxisAsset,
        half_segment_m: f64,
        radius_m: f64,
    },
}

impl CollisionShapeAsset {
    pub(crate) fn bake(self) -> CollisionShape {
        match self {
            Self::Sphere { radius_m } => CollisionShape::Sphere { radius_m },
            Self::Cuboid { half_extents_m } => CollisionShape::Cuboid {
                half_extents_m: vector(half_extents_m),
            },
            Self::Capsule {
                axis,
                half_segment_m,
                radius_m,
            } => CollisionShape::Capsule {
                axis: axis.into(),
                half_segment_m,
                radius_m,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum CollisionAxisAsset {
    X,
    Y,
    Z,
}

impl From<CollisionAxisAsset> for CollisionAxis {
    fn from(value: CollisionAxisAsset) -> Self {
        match value {
            CollisionAxisAsset::X => Self::X,
            CollisionAxisAsset::Y => Self::Y,
            CollisionAxisAsset::Z => Self::Z,
        }
    }
}
