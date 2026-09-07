use std::{collections::BTreeMap, error::Error, fmt};

use serde::{Deserialize, Serialize};

use crate::{
    AU_M, BakedBody, BakedEphemeris, BodyId, DAY_S, EARTH_MASS_KG, EphemerisError, G,
    JUPITER_MASS_KG, KeplerOrbit, SOLAR_MASS_KG,
};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct SystemConfig {
    pub meta: SystemMeta,
    pub star: Vec<StarConfig>,
    pub orbit: OrbitConfig,
    pub planet: Vec<CelestialConfig>,
    pub moon: Vec<CelestialConfig>,
    pub minor_body: Vec<CelestialConfig>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct SystemMeta {
    pub epoch_name: String,
    pub status: String,
    pub runtime_body_motion: String,
    pub vehicle_gravity: String,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct OrbitConfig {
    pub bc_inner: Option<BinaryOrbitConfig>,
    pub a_bc_outer: Option<BinaryOrbitConfig>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct BinaryOrbitConfig {
    pub primary: Option<String>,
    pub secondary: Option<String>,
    pub components: Vec<String>,
    pub semi_major_axis_au_relative: Option<f64>,
    pub eccentricity: Option<f64>,
    pub period_days_design: Option<f64>,
    pub mutual_inclination_outer_deg: Option<f64>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct StarConfig {
    pub id: String,
    pub mass_solar: f64,
    pub radius_solar: f64,
    pub temperature_k: f64,
    pub luminosity_solar: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct CelestialConfig {
    pub id: String,
    pub host: Option<String>,
    pub kind: Option<String>,
    pub semi_major_axis_au: Option<f64>,
    pub semi_major_axis_km: Option<f64>,
    pub period_days_design: Option<f64>,
    pub period_hours_design: Option<f64>,
    pub eccentricity: Option<f64>,
    pub inclination_deg: Option<f64>,
    pub mean_longitude_offset_deg: Option<f64>,
    pub mass_solar: Option<f64>,
    pub mass_jupiter: Option<f64>,
    pub mass_earth: Option<f64>,
    pub radius_km: Option<f64>,
    pub density_kg_m3: Option<f64>,
    pub rotation_period_hours: Option<f64>,
    pub axial_tilt_deg: Option<f64>,
    pub tidal_lock: bool,
}

#[derive(Debug, Clone)]
struct RawBody {
    name: String,
    mu: f64,
    radius_m: f64,
    gravity_source: bool,
    orbit: Option<RawOrbit>,
    design_period_s: Option<f64>,
    rotation_period_s: Option<f64>,
    tidal_lock: bool,
    axial_tilt_rad: f64,
}

#[derive(Debug, Clone)]
struct RawOrbit {
    parent_name: String,
    semi_major_axis_m: f64,
    eccentricity: f64,
    inclination_rad: f64,
    mean_anomaly_at_epoch_rad: f64,
    /// Synthetic barycentre entries already include the orbiting component's
    /// mass; ordinary body entries derive `mu_host + mu_body` later.
    central_mu: Option<f64>,
    mean_motion_rad_s: Option<f64>,
}

impl SystemConfig {
    /// Turn design-target TOML into a deterministic analytic ephemeris.
    ///
    /// This is intentionally a conservative baker: absent phase angles default
    /// to zero, and design periods are retained for diagnostics while the
    /// actual mean motion is derived from `mu` and semi-major axis.
    pub fn bake(&self) -> Result<BakedEphemeris, SystemSpecError> {
        let mut bodies = Vec::new();
        let mut ids = BTreeMap::<String, BodyId>::new();

        let stellar_mu: f64 = self
            .star
            .iter()
            .map(|star| star.mass_solar * SOLAR_MASS_KG * G)
            .sum();
        if self.star.is_empty() || !stellar_mu.is_finite() || stellar_mu <= 0.0 {
            return Err(SystemSpecError::Invalid(
                "at least one star with positive mass is required".into(),
            ));
        }
        push_raw_body(
            &mut bodies,
            &mut ids,
            RawBody {
                name: "system_barycenter".into(),
                mu: stellar_mu,
                radius_m: 0.0,
                gravity_source: false,
                orbit: None,
                design_period_s: None,
                rotation_period_s: None,
                tidal_lock: false,
                axial_tilt_rad: 0.0,
            },
        )?;
        for star in &self.star {
            if star.id.is_empty()
                || !star.mass_solar.is_finite()
                || star.mass_solar <= 0.0
                || !star.radius_solar.is_finite()
                || star.radius_solar < 0.0
            {
                return Err(SystemSpecError::Invalid(format!(
                    "invalid star {:?}",
                    star.id
                )));
            }
            push_raw_body(
                &mut bodies,
                &mut ids,
                RawBody {
                    name: star.id.clone(),
                    mu: star.mass_solar * SOLAR_MASS_KG * G,
                    radius_m: star.radius_solar * 695_700_000.0,
                    gravity_source: true,
                    orbit: None,
                    design_period_s: None,
                    rotation_period_s: None,
                    tidal_lock: false,
                    axial_tilt_rad: 0.0,
                },
            )?;
        }

        let inner = self.orbit.bc_inner.as_ref();
        let outer = self.orbit.a_bc_outer.as_ref();
        if let Some(inner) = inner {
            let inner_components = if inner.components.len() == 2 {
                [inner.components[0].clone(), inner.components[1].clone()]
            } else if let (Some(primary), Some(secondary)) = (&inner.primary, &inner.secondary) {
                [primary.clone(), secondary.clone()]
            } else {
                return Err(SystemSpecError::Invalid(
                    "orbit.bc_inner needs components or primary/secondary".into(),
                ));
            };
            let first = *ids
                .get(&inner_components[0])
                .ok_or_else(|| SystemSpecError::UnknownHost(inner_components[0].clone()))?;
            let second = *ids
                .get(&inner_components[1])
                .ok_or_else(|| SystemSpecError::UnknownHost(inner_components[1].clone()))?;
            let bc_mu = bodies[first.index()].mu + bodies[second.index()].mu;
            let bc_id = push_raw_body(
                &mut bodies,
                &mut ids,
                RawBody {
                    name: "bc_barycenter".into(),
                    mu: bc_mu,
                    radius_m: 0.0,
                    gravity_source: false,
                    orbit: None,
                    design_period_s: None,
                    rotation_period_s: None,
                    tidal_lock: false,
                    axial_tilt_rad: 0.0,
                },
            )?;
            let relative_a = required_positive(
                inner.semi_major_axis_au_relative,
                "orbit.bc_inner.semi_major_axis_au_relative",
            )? * AU_M;
            let eccentricity = inner.eccentricity.unwrap_or(0.0);
            let inclination = inner
                .mutual_inclination_outer_deg
                .unwrap_or(0.0)
                .to_radians();
            let first_a = relative_a * bodies[second.index()].mu / bc_mu;
            let second_a = relative_a * bodies[first.index()].mu / bc_mu;
            let inner_mean_motion = (bc_mu / relative_a.powi(3)).sqrt();
            set_orbit(
                &mut bodies[first.index()],
                RawOrbit {
                    parent_name: "bc_barycenter".into(),
                    semi_major_axis_m: first_a,
                    eccentricity,
                    inclination_rad: inclination,
                    mean_anomaly_at_epoch_rad: 0.0,
                    central_mu: Some(bc_mu),
                    mean_motion_rad_s: Some(inner_mean_motion),
                },
                inner.period_days_design.map(|days| days * DAY_S),
            );
            set_orbit(
                &mut bodies[second.index()],
                RawOrbit {
                    parent_name: "bc_barycenter".into(),
                    semi_major_axis_m: second_a,
                    eccentricity,
                    inclination_rad: inclination,
                    mean_anomaly_at_epoch_rad: std::f64::consts::PI,
                    central_mu: Some(bc_mu),
                    mean_motion_rad_s: Some(inner_mean_motion),
                },
                inner.period_days_design.map(|days| days * DAY_S),
            );
            if let Some(outer) = outer {
                if outer.components.len() != 2
                    || !outer
                        .components
                        .iter()
                        .any(|component| component == "bc_barycenter")
                {
                    return Err(SystemSpecError::Invalid(
                        "orbit.a_bc_outer.components must include bc_barycenter and Asterion A"
                            .into(),
                    ));
                }
                let a_name = outer
                    .components
                    .iter()
                    .find(|component| component.as_str() != "bc_barycenter")
                    .ok_or_else(|| SystemSpecError::Invalid("missing outer A component".into()))?;
                let a_id = *ids
                    .get(a_name)
                    .ok_or_else(|| SystemSpecError::UnknownHost(a_name.clone()))?;
                let outer_a = required_positive(
                    outer.semi_major_axis_au_relative,
                    "orbit.a_bc_outer.semi_major_axis_au_relative",
                )? * AU_M;
                let outer_mu = bodies[a_id.index()].mu + bodies[bc_id.index()].mu;
                let a_a = outer_a * bodies[bc_id.index()].mu / outer_mu;
                let bc_a = outer_a * bodies[a_id.index()].mu / outer_mu;
                let outer_mean_motion = (stellar_mu / outer_a.powi(3)).sqrt();
                set_orbit(
                    &mut bodies[a_id.index()],
                    RawOrbit {
                        parent_name: "system_barycenter".into(),
                        semi_major_axis_m: a_a,
                        eccentricity: outer.eccentricity.unwrap_or(0.0),
                        inclination_rad: 0.0,
                        mean_anomaly_at_epoch_rad: 0.0,
                        central_mu: Some(stellar_mu),
                        mean_motion_rad_s: Some(outer_mean_motion),
                    },
                    outer.period_days_design.map(|days| days * DAY_S),
                );
                set_orbit(
                    &mut bodies[bc_id.index()],
                    RawOrbit {
                        parent_name: "system_barycenter".into(),
                        semi_major_axis_m: bc_a,
                        eccentricity: outer.eccentricity.unwrap_or(0.0),
                        inclination_rad: 0.0,
                        mean_anomaly_at_epoch_rad: std::f64::consts::PI,
                        central_mu: Some(stellar_mu),
                        mean_motion_rad_s: Some(outer_mean_motion),
                    },
                    outer.period_days_design.map(|days| days * DAY_S),
                );
            }
        } else if outer.is_some() {
            return Err(SystemSpecError::Invalid(
                "orbit.a_bc_outer requires orbit.bc_inner".into(),
            ));
        }

        for body in self.planet.iter().chain(&self.moon).chain(&self.minor_body) {
            add_config_body(&mut bodies, &mut ids, body)?;
        }

        let mu_by_id: Vec<f64> = bodies.iter().map(|body| body.mu).collect();
        let mut baked = Vec::with_capacity(bodies.len());
        for raw in bodies {
            let id = ids[&raw.name];
            let (parent, orbit) = if let Some(orbit) = raw.orbit {
                let parent = *ids
                    .get(&orbit.parent_name)
                    .ok_or_else(|| SystemSpecError::UnknownHost(orbit.parent_name.clone()))?;
                let central_mu = orbit
                    .central_mu
                    .unwrap_or(mu_by_id[parent.index()] + raw.mu);
                let kepler = KeplerOrbit::new(
                    central_mu,
                    orbit.semi_major_axis_m,
                    orbit.eccentricity,
                    orbit.inclination_rad,
                    0.0,
                    0.0,
                    orbit.mean_anomaly_at_epoch_rad,
                )?;
                let kepler = match orbit.mean_motion_rad_s {
                    Some(mean_motion) => kepler.with_mean_motion(mean_motion)?,
                    None => kepler,
                };
                (Some(parent), Some(kepler))
            } else {
                (None, None)
            };
            baked.push(BakedBody {
                id,
                name: raw.name,
                mu: raw.mu,
                radius_m: raw.radius_m,
                parent,
                orbit,
                design_period_s: raw.design_period_s,
                rotation_period_s: raw.rotation_period_s,
                tidal_lock: raw.tidal_lock,
                axial_tilt_rad: raw.axial_tilt_rad,
                gravity_source: raw.gravity_source,
            });
        }
        Ok(BakedEphemeris::new(
            if self.meta.epoch_name.is_empty() {
                "UNNAMED_EPOCH"
            } else {
                &self.meta.epoch_name
            },
            baked,
        )?)
    }
}

fn push_raw_body(
    bodies: &mut Vec<RawBody>,
    ids: &mut BTreeMap<String, BodyId>,
    body: RawBody,
) -> Result<BodyId, SystemSpecError> {
    if ids.contains_key(&body.name) {
        return Err(SystemSpecError::DuplicateId(body.name));
    }
    let id = BodyId(bodies.len() as u32);
    ids.insert(body.name.clone(), id);
    bodies.push(body);
    Ok(id)
}

fn add_config_body(
    bodies: &mut Vec<RawBody>,
    ids: &mut BTreeMap<String, BodyId>,
    config: &CelestialConfig,
) -> Result<BodyId, SystemSpecError> {
    if config.id.is_empty() {
        return Err(SystemSpecError::Invalid("body id must not be empty".into()));
    }
    let mu = config_mu(config)?;
    let radius_m = config.radius_km.unwrap_or(0.0) * 1_000.0;
    if !radius_m.is_finite() || radius_m < 0.0 {
        return Err(SystemSpecError::Invalid(format!(
            "body {} has invalid radius",
            config.id
        )));
    }
    let parent_name = config
        .host
        .clone()
        .unwrap_or_else(|| "system_barycenter".into());
    let semi_major_axis_m = config
        .semi_major_axis_au
        .map(|value| value * AU_M)
        .or_else(|| config.semi_major_axis_km.map(|value| value * 1_000.0));
    let orbit = semi_major_axis_m.map(|semi_major_axis_m| RawOrbit {
        central_mu: lagrange_central_mu(config, &parent_name, bodies, ids),
        parent_name,
        semi_major_axis_m,
        eccentricity: config.eccentricity.unwrap_or(0.0),
        inclination_rad: config.inclination_deg.unwrap_or(0.0).to_radians(),
        mean_anomaly_at_epoch_rad: config.mean_longitude_offset_deg.unwrap_or(0.0).to_radians(),
        mean_motion_rad_s: None,
    });
    let design_period_s = config
        .period_days_design
        .map(|days| days * DAY_S)
        .or_else(|| config.period_hours_design.map(|hours| hours * 3_600.0));
    let rotation_period_s = match config.rotation_period_hours {
        Some(hours) if hours.is_finite() && hours > 0.0 => Some(hours * 3_600.0),
        Some(_) => {
            return Err(SystemSpecError::Invalid(format!(
                "body {} has invalid rotation_period_hours",
                config.id
            )));
        }
        None => None,
    };
    let axial_tilt_rad = config.axial_tilt_deg.unwrap_or(0.0).to_radians();
    if !axial_tilt_rad.is_finite() {
        return Err(SystemSpecError::Invalid(format!(
            "body {} has invalid axial_tilt_deg",
            config.id
        )));
    }
    if orbit.is_none() && config.host.is_some() {
        return Err(SystemSpecError::Invalid(format!(
            "body {} has a host but no semi-major axis",
            config.id
        )));
    }
    if ids.contains_key(&config.id) {
        return Err(SystemSpecError::DuplicateId(config.id.clone()));
    }
    let id = BodyId(bodies.len() as u32);
    ids.insert(config.id.clone(), id);
    bodies.push(RawBody {
        name: config.id.clone(),
        mu,
        radius_m,
        gravity_source: true,
        orbit,
        design_period_s,
        rotation_period_s,
        tidal_lock: config.tidal_lock,
        axial_tilt_rad,
    });
    Ok(id)
}

fn lagrange_central_mu(
    config: &CelestialConfig,
    parent_name: &str,
    bodies: &[RawBody],
    ids: &BTreeMap<String, BodyId>,
) -> Option<f64> {
    let companion_name = config.kind.as_deref()?.strip_suffix("_l4_coorbital")?;
    let parent_id = *ids.get(parent_name)?;
    let companion_id = *ids.get(companion_name)?;
    Some(bodies[parent_id.index()].mu + bodies[companion_id.index()].mu)
}

fn set_orbit(body: &mut RawBody, orbit: RawOrbit, design_period_s: Option<f64>) {
    body.orbit = Some(orbit);
    body.design_period_s = design_period_s;
}

fn config_mu(config: &CelestialConfig) -> Result<f64, SystemSpecError> {
    let mass_kg = if let Some(mass) = config.mass_solar {
        mass * SOLAR_MASS_KG
    } else if let Some(mass) = config.mass_jupiter {
        mass * JUPITER_MASS_KG
    } else if let Some(mass) = config.mass_earth {
        mass * EARTH_MASS_KG
    } else if let (Some(radius_km), Some(density)) = (config.radius_km, config.density_kg_m3) {
        let radius_m = radius_km * 1_000.0;
        (4.0 / 3.0) * std::f64::consts::PI * radius_m.powi(3) * density
    } else {
        return Err(SystemSpecError::Invalid(format!(
            "body {} needs mass_earth/mass_jupiter/mass_solar or radius+density",
            config.id
        )));
    };
    if !mass_kg.is_finite() || mass_kg <= 0.0 {
        return Err(SystemSpecError::Invalid(format!(
            "body {} has invalid mass",
            config.id
        )));
    }
    Ok(mass_kg * G)
}

fn required_positive(value: Option<f64>, field: &str) -> Result<f64, SystemSpecError> {
    let value = value.ok_or_else(|| SystemSpecError::Invalid(format!("missing {field}")))?;
    if !value.is_finite() || value <= 0.0 {
        return Err(SystemSpecError::Invalid(format!(
            "{field} must be positive"
        )));
    }
    Ok(value)
}

#[derive(Debug, Clone, PartialEq)]
pub enum SystemSpecError {
    DuplicateId(String),
    UnknownHost(String),
    Invalid(String),
    Ephemeris(EphemerisError),
}

impl fmt::Display for SystemSpecError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateId(id) => write!(formatter, "duplicate body id {id}"),
            Self::UnknownHost(host) => write!(formatter, "unknown host {host}"),
            Self::Invalid(message) => write!(formatter, "invalid system config: {message}"),
            Self::Ephemeris(error) => error.fmt(formatter),
        }
    }
}

impl Error for SystemSpecError {}

impl From<EphemerisError> for SystemSpecError {
    fn from(error: EphemerisError) -> Self {
        Self::Ephemeris(error)
    }
}
