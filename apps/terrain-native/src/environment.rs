//! Native environment radiance: shared single scattering and auroral emission
//! onto the renderer's direction grid.
//!
//! This module owns the *policy* (which resolved graphics settings map to grid
//! resolution, integration steps and refresh cadence) and the *physical input*
//! (the body's own optics plus the current stellar lights, both from existing
//! owners). Renderers only sample the resulting field; none of them invents
//! sky, ambient or ground radiance.
use glam::DVec3;
use thessa_atmosphere::{AtmosphereOptics, CelestialLight, nitrogen_oxygen_optics, sky_radiance};
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_render_core::environment::EnvironmentRadiance;

/// Luminous scale the native presentation uses for a W/m^2 input, shared with
/// the celestial lighting assembly so sky radiance and direct light colours
/// stay in one photometric system.
pub const LUX_PER_W_M2: f64 = 93.0;

/// Design temperature and fallback composition of the flight authority's
/// atmosphere config, so rendering and flight physics read one atmosphere.
const DESIGN_TEMPERATURE_K: f64 = 288.15;

/// Visual cache displacement threshold, not a globally proven error bound.
const POSITION_EPSILON_M: f64 = 250.0;
/// Re-evaluate after any contributing source turns this much (~0.11 deg).
const STAR_EPSILON_RAD: f64 = 0.002;
/// Relative threshold for each source's weighted RGB irradiance.
const IRRADIANCE_EPSILON: f64 = 0.01;
/// Bounded visual refresh policy in simulation time, not a proven temporal
/// error envelope. Pausing reuses the field; forward/backward warp both wake it.
const AURORA_REFRESH_SIM_S: f64 = 0.5;

/// Atmosphere scalars read from the frozen body provider by the caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyAtmosphere {
    pub surface_pressure_pa: f64,
    pub gas_constant_j_kg_k: f64,
}

/// Resolved evaluation budget for the field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnvironmentBudget {
    pub width: u32,
    pub height: u32,
    pub view_steps: u32,
    pub sun_steps: u32,
    /// Minimum number of frames between two evaluations, bounding the worst
    /// case on a fast camera; a change of inputs is still required.
    pub max_interval_frames: u64,
    /// Visual medium multiplier from the atmosphere settings.
    pub density_scale: f32,
    pub multi_star: bool,
    pub airglow: bool,
    pub scattering: bool,
    pub aurora_steps: u32,
    pub aurora_intensity: f32,
    pub aurora_animate: bool,
}

impl EnvironmentBudget {
    /// Disabling scattering, airglow AND aurora resolves to `None`: no grid or
    /// evaluation, retaining the isotropic `ambient_lux / PI` fallback. Emission
    /// toggles are independent of scattering, like the shared optical model.
    pub fn resolve(settings: &ResolvedGraphicsSettings) -> Option<Self> {
        if !settings.atmosphere_enabled && !settings.airglow && !settings.aurora_shell {
            return None;
        }
        let steps = settings.atmosphere_steps.clamp(4, 32);
        let (width, height, view_steps, sun_steps, max_interval_frames) =
            match settings.atmosphere_quality {
                Quality::Low => (8, 4, 8.min(steps), 4.min(steps), 8),
                Quality::Medium => (16, 8, 12.min(steps), 6.min(steps), 4),
                Quality::High => (32, 16, steps, (steps / 3).clamp(2, 8), 2),
            };
        Some(Self {
            width,
            height,
            view_steps,
            sun_steps,
            max_interval_frames,
            density_scale: settings.atmosphere_density_scale,
            multi_star: settings.multi_star,
            airglow: settings.airglow,
            scattering: settings.atmosphere_enabled,
            aurora_steps: if !settings.aurora_shell || settings.aurora_intensity == 0.0 {
                0
            } else {
                match settings.aurora_quality {
                    thessa_graphics::AuroraQuality::Low => 8,
                    thessa_graphics::AuroraQuality::High => 32,
                }
            },
            aurora_intensity: settings.aurora_intensity,
            aurora_animate: settings.aurora_animate,
        })
    }
}

/// Evaluation accounting for enabled updates, including pre-stream ticks;
/// this is not necessarily the number of displayed frames.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnvironmentStats {
    pub enabled: bool,
    pub evaluated: u64,
    pub skipped: u64,
    pub width: u32,
    pub height: u32,
    pub view_steps: u32,
    pub sun_steps: u32,
    pub ambient_lux: [f32; 3],
    pub storage_bytes: u64,
    pub last_evaluation_ms: f64,
    pub aurora_steps: u32,
}

/// Sky radiance field plus its bounded refresh policy.
pub struct EnvironmentField {
    optics: AtmosphereOptics,
    budget: Option<EnvironmentBudget>,
    grid: Option<EnvironmentRadiance>,
    last_position_m: DVec3,
    last_lights: Vec<CelestialLight>,
    frames_since_evaluation: u64,
    evaluated: u64,
    skipped: u64,
    last_evaluation_ms: f64,
    revision: u64,
    last_time_s: f64,
}

impl EnvironmentField {
    /// Build the shared optics from the body's own data. `None` when the body
    /// has no atmosphere provider: an airless body has no sky, and the native
    /// path must not invent one.
    pub fn new(
        body_radius_m: f64,
        body_mu: f64,
        atmosphere: Option<BodyAtmosphere>,
    ) -> Option<Self> {
        let atmosphere = atmosphere?;
        let base = build_optics(body_radius_m, body_mu, atmosphere, 1.0)?;
        Some(Self {
            optics: base,
            budget: None,
            grid: None,
            last_position_m: DVec3::ZERO,
            last_lights: Vec::new(),
            frames_since_evaluation: u64::MAX,
            evaluated: 0,
            skipped: 0,
            last_evaluation_ms: 0.0,
            revision: 0,
            last_time_s: f64::NAN,
        })
    }

    /// Apply resolved settings. A changed budget discards the current grid so
    /// the next update evaluates immediately at the new resolution.
    pub fn set_budget(&mut self, budget: Option<EnvironmentBudget>) {
        if self.budget == budget {
            return;
        }
        self.budget = budget;
        self.grid = None;
        self.frames_since_evaluation = u64::MAX;
        self.revision += 1;
    }

    /// Evaluate when the inputs changed enough and the budget allows, then
    /// return the grid the renderers should sample.
    #[cfg(test)]
    pub fn update(
        &mut self,
        camera_body_m: DVec3,
        lights: &[CelestialLight],
    ) -> Option<&EnvironmentRadiance> {
        self.update_at(camera_body_m, lights, 0.0)
    }

    pub fn update_at(
        &mut self,
        camera_body_m: DVec3,
        lights: &[CelestialLight],
        sim_time_s: f64,
    ) -> Option<&EnvironmentRadiance> {
        let budget = self.budget?;
        if !sim_time_s.is_finite() || !camera_body_m.is_finite() {
            return None;
        }
        self.frames_since_evaluation = self.frames_since_evaluation.saturating_add(1);
        let lights = if budget.multi_star {
            lights
        } else {
            lights
                .iter()
                .max_by(|a, b| a.irradiance_w_m2.total_cmp(&b.irradiance_w_m2))
                .map_or(&[][..], std::slice::from_ref)
        };
        let moved = (camera_body_m - self.last_position_m).length() > POSITION_EPSILON_M;
        let animated = budget.aurora_steps > 0
            && budget.aurora_animate
            && (sim_time_s - self.last_time_s).abs() >= AURORA_REFRESH_SIM_S;
        let changed =
            self.grid.is_none() || moved || animated || lights_changed(&self.last_lights, lights);
        if !changed || self.frames_since_evaluation < budget.max_interval_frames {
            self.skipped += 1;
            return self.grid.as_ref();
        }
        let started = std::time::Instant::now();
        self.grid = self.evaluate_at(camera_body_m, lights, sim_time_s);
        self.last_evaluation_ms = started.elapsed().as_secs_f64() * 1000.0;
        self.last_position_m = camera_body_m;
        self.last_time_s = sim_time_s;
        self.last_lights.clear();
        self.last_lights.extend_from_slice(lights);
        self.frames_since_evaluation = 0;
        self.evaluated += 1;
        self.revision = self.revision.wrapping_add(1);
        self.grid.as_ref()
    }

    /// Evaluate every texel centre with the shared single-scattering model.
    /// Texel radiance is converted to the presentation's photometric scale
    /// exactly like the direct stellar colours.
    ///
    /// Texels are independent, so the grid is filled row-parallel: the result
    /// is identical to a sequential fill. The refresh interval bounds work
    /// frequency, not evaluation latency; timings are reported separately.
    #[cfg(test)]
    fn evaluate(
        &mut self,
        camera_body_m: DVec3,
        lights: &[CelestialLight],
    ) -> Option<EnvironmentRadiance> {
        self.evaluate_at(camera_body_m, lights, 0.0)
    }

    fn evaluate_at(
        &self,
        camera_body_m: DVec3,
        lights: &[CelestialLight],
        sim_time_s: f64,
    ) -> Option<EnvironmentRadiance> {
        use rayon::prelude::*;
        let budget = self.budget?;
        let mut optics = self.optics.clone();
        for channel in 0..3 {
            let scale = if budget.scattering {
                f64::from(budget.density_scale)
            } else {
                0.0
            };
            optics.rayleigh.beta_rgb[channel] *= scale;
            optics.mie.beta_rgb[channel] *= scale;
        }
        if !budget.airglow {
            optics.emission.airglow_rgb = [0.0; 3];
        }
        let columns = budget.width as usize;
        let mut texels = vec![[0.0_f32; 3]; columns * budget.height as usize];
        texels
            .par_chunks_mut(columns)
            .enumerate()
            .for_each(|(row, row_texels)| {
                let row = row as u32;
                let elevation = ((f64::from(row) + 0.5) / f64::from(budget.height) - 0.5)
                    * std::f64::consts::PI;
                for (column, texel) in row_texels.iter_mut().enumerate() {
                    let azimuth = ((column as f64 + 0.5) / f64::from(budget.width) - 0.5)
                        * std::f64::consts::TAU;
                    let direction = DVec3::new(
                        elevation.cos() * azimuth.cos(),
                        elevation.sin(),
                        elevation.cos() * azimuth.sin(),
                    );
                    let sky = sky_radiance(
                        &optics,
                        camera_body_m,
                        direction,
                        lights,
                        budget.view_steps,
                        budget.sun_steps,
                    );
                    *texel = sky.radiance_rgb.map(|c| (c * LUX_PER_W_M2) as f32);
                }
            });
        let up = camera_body_m.normalize_or_zero().as_vec3().to_array();
        let base = EnvironmentRadiance::with_up(budget.width, budget.height, texels, up)?;
        if budget.aurora_steps == 0 {
            return Some(base);
        }
        // Same analytic model as shared CPU queries, not a second painted
        // shell. The magnetic axis is the legacy Thessa aligned-dipole proxy
        // expressed in native body axes (+Y); no MHD claim is made.
        let gain = f64::from(budget.aurora_intensity) * 1e-6;
        optics.emission.aurora = Some(thessa_atmosphere::AuroraParams {
            magnetic_pole: DVec3::Y,
            oval_latitude_rad: 67.0f64.to_radians(),
            oval_width_rad: 3.0f64.to_radians(),
            altitude_min_m: 90_000.0,
            altitude_max_m: 250_000.0,
            activity: 0.8,
            color_bottom_rgb: [0.2, 1.0, 0.35].map(|c| c * gain),
            color_top_rgb: [0.9, 0.25, 0.3].map(|c| c * gain),
        });
        let (width, height) = if budget.aurora_steps <= 8 {
            (64, 32)
        } else {
            (128, 64)
        };
        let time = if budget.aurora_animate {
            sim_time_s
        } else {
            0.0
        };
        let mut output = vec![[0.0; 3]; width * height];
        output
            .par_chunks_mut(width)
            .enumerate()
            .for_each(|(row, pixels)| {
                let elevation = ((row as f64 + 0.5) / height as f64 - 0.5) * std::f64::consts::PI;
                for (column, pixel) in pixels.iter_mut().enumerate() {
                    let azimuth =
                        ((column as f64 + 0.5) / width as f64 - 0.5) * std::f64::consts::TAU;
                    let direction = DVec3::new(
                        elevation.cos() * azimuth.cos(),
                        elevation.sin(),
                        elevation.cos() * azimuth.sin(),
                    );
                    let base = base.sample(direction.as_vec3().to_array());
                    let emission = thessa_atmosphere::aurora_radiance(
                        &optics,
                        camera_body_m,
                        direction,
                        time,
                        budget.aurora_steps,
                    );
                    *pixel = std::array::from_fn(|c| base[c] + (emission[c] * LUX_PER_W_M2) as f32);
                }
            });
        EnvironmentRadiance::with_up(width as u32, height as u32, output, up)
    }

    /// Revision changes on evaluation or budget invalidation, including disable.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn grid(&self) -> Option<&EnvironmentRadiance> {
        self.grid.as_ref()
    }

    pub fn stats(&self) -> EnvironmentStats {
        let (budget, grid) = (self.budget, self.grid.as_ref());
        EnvironmentStats {
            enabled: budget.is_some(),
            evaluated: self.evaluated,
            skipped: self.skipped,
            width: grid.map_or(0, EnvironmentRadiance::width),
            height: grid.map_or(0, EnvironmentRadiance::height),
            view_steps: budget.map_or(0, |b| b.view_steps),
            sun_steps: budget.map_or(0, |b| b.sun_steps),
            ambient_lux: grid.map_or([0.0; 3], EnvironmentRadiance::ambient_lux),
            storage_bytes: grid.map_or(0, EnvironmentRadiance::storage_bytes),
            last_evaluation_ms: self.last_evaluation_ms,
            aurora_steps: budget.map_or(0, |b| b.aurora_steps),
        }
    }
}

fn build_optics(
    body_radius_m: f64,
    body_mu: f64,
    atmosphere: BodyAtmosphere,
    density_scale: f64,
) -> Option<AtmosphereOptics> {
    if !body_radius_m.is_finite()
        || body_radius_m <= 0.0
        || !body_mu.is_finite()
        || body_mu <= 0.0
        || !atmosphere.gas_constant_j_kg_k.is_finite()
        || atmosphere.gas_constant_j_kg_k <= 0.0
        || !atmosphere.surface_pressure_pa.is_finite()
        || atmosphere.surface_pressure_pa < 0.0
    {
        return None;
    }
    let gas_constant = atmosphere.gas_constant_j_kg_k;
    let gravity = body_mu / (body_radius_m * body_radius_m);
    let scale_height_m = gas_constant * DESIGN_TEMPERATURE_K / gravity;
    if !scale_height_m.is_finite() || scale_height_m <= 0.0 {
        return None;
    }
    let mut optics = nitrogen_oxygen_optics(
        body_radius_m,
        atmosphere.surface_pressure_pa.max(0.0),
        scale_height_m,
    )
    .ok()?;
    // The settings' density scale is a visual optical-depth multiplier of the
    // scattering medium only: absorption is a separate physical term.
    let scale = density_scale.clamp(0.0, 4.0);
    for channel in 0..3 {
        optics.rayleigh.beta_rgb[channel] *= scale;
        optics.mie.beta_rgb[channel] *= scale;
    }
    optics.validate().ok()?;
    Some(optics)
}

/// Compare each source, not just the brightest source or the RGB sum: a
/// secondary sun can move while the primary and total irradiance stay fixed.
fn lights_changed(before: &[CelestialLight], after: &[CelestialLight]) -> bool {
    before.len() != after.len()
        || before.iter().zip(after).any(|(a, b)| {
            a.direction_to_star.angle_between(b.direction_to_star) > STAR_EPSILON_RAD
                || a.weighted_rgb()
                    .into_iter()
                    .zip(b.weighted_rgb())
                    .any(|(x, y)| (x - y).abs() > IRRADIANCE_EPSILON * x.abs().max(1e-6))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "benchmark: native target-size aurora grids, latency and representation error"]
    fn aurora_grid_cost_and_error() {
        let lat = 67.0f64.to_radians();
        let camera = DVec3::new(lat.cos(), lat.sin(), 0.0) * 3_201_000.0;
        let mut settings = settings(Quality::Low, 8);
        settings.atmosphere_enabled = false;
        settings.aurora_shell = true;
        settings.aurora_intensity = 1.0;
        settings.aurora_animate = false;
        let mut field =
            EnvironmentField::new(3_200_000.0, 5.02e13, Some(thessa_atmosphere())).unwrap();
        let mut reference_budget = EnvironmentBudget::resolve(&settings).unwrap();
        reference_budget.aurora_steps = 256;
        field.set_budget(Some(reference_budget));
        let reference = field.evaluate_at(camera, &[], 0.0).unwrap();
        for quality in [
            thessa_graphics::AuroraQuality::Low,
            thessa_graphics::AuroraQuality::High,
        ] {
            settings.aurora_quality = quality;
            field.set_budget(EnvironmentBudget::resolve(&settings));
            let grid = field.evaluate_at(camera, &[], 0.0).unwrap();
            let mut samples = Vec::new();
            for _ in 0..7 {
                let start = std::time::Instant::now();
                std::hint::black_box(field.evaluate_at(camera, &[], 0.0).unwrap());
                samples.push(start.elapsed().as_secs_f64() * 1000.0);
            }
            samples.sort_by(f64::total_cmp);
            let mut max_error = 0.0f32;
            for index in 0..256 {
                let y = (index as f64 + 0.5) / 128.0 - 1.0;
                let phi = index as f64 * 2.399963229728653;
                let r = (1.0 - y * y).sqrt();
                let direction = [(r * phi.cos()) as f32, y as f32, (r * phi.sin()) as f32];
                for (a, b) in grid
                    .sample(direction)
                    .into_iter()
                    .zip(reference.sample(direction))
                {
                    max_error = max_error.max((a - b).abs());
                }
            }
            println!(
                "aurora_grid quality={quality:?} grid={}x{} p50_ms={:.6} p95_ms={:.6} max_abs_error_lux_sr_vs_256_step_128x64={max_error}",
                grid.width(),
                grid.height(),
                samples[3],
                samples[6]
            );
            assert!(max_error.is_finite());
        }
    }

    #[test]
    #[ignore = "hardware: actual native auroral publication into the shared sky background"]
    fn aurora_field_reaches_native_sky_and_disabled_restores_fallback() {
        use thessa_rcbt_core::{
            HeightPage,
            tile_anchor::{TileAnchor, TileKey},
        };
        use thessa_render_wgpu::{
            Backend,
            environment::EnvironmentBinding,
            terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer},
            view::ViewParams,
        };
        let backend = Backend::bring_up().unwrap();
        let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
        let mut terrain = TerrainRenderer::new(
            &backend,
            &[TerrainPatch {
                anchor: TileAnchor::new(TileKey::new(0, 0, 0, 0).unwrap(), 3_200_000.0).unwrap(),
                page: &page,
            }],
            OffscreenTarget::FORMAT,
        )
        .unwrap();
        terrain.publish_partition(&backend, &[]).unwrap();
        let target = OffscreenTarget::new(&backend, 129, 65).unwrap();
        let lat = 67.0f64.to_radians();
        let up = DVec3::new(lat.cos(), lat.sin(), 0.0);
        let camera = up * 3_201_000.0;
        let view = ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(camera, camera + up, DVec3::Z)
                .inverse(),
            world_position_m: camera,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                1.0,
                129.0 / 65.0,
                0.1,
                1e7,
            ),
            exposure: 0.1,
        };
        terrain.update_view(&backend, &view).unwrap();
        let mut binding = EnvironmentBinding::new(&backend.device);
        let mut frames = Vec::new();
        for mode in [
            None,
            Some(thessa_graphics::AuroraQuality::Low),
            Some(thessa_graphics::AuroraQuality::High),
            None,
        ] {
            let mut settings = settings(Quality::Low, 8);
            settings.atmosphere_enabled = false;
            settings.aurora_shell = mode.is_some();
            settings.aurora_quality = mode.unwrap_or_default();
            settings.aurora_intensity = 1.0;
            let mut field =
                EnvironmentField::new(3_200_000.0, 5.02e13, Some(thessa_atmosphere())).unwrap();
            field.set_budget(EnvironmentBudget::resolve(&settings));
            let start = std::time::Instant::now();
            field.update_at(camera, &[], 0.0);
            println!(
                "aurora_native mode={mode:?} evaluation_ms={:.6} stats={:?}",
                start.elapsed().as_secs_f64() * 1000.0,
                field.stats()
            );
            binding.update(&backend, field.grid());
            terrain.share_environment(&backend, Some(&binding));
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_scene(&mut encoder, &target.color, &target.depth, None);
            backend.queue.submit([encoder.finish()]);
            frames.push(target.read_rgba8(&backend).unwrap());
            assert_eq!(terrain.sky_enabled(), mode.is_some());
        }
        assert_eq!(
            frames[0], frames[3],
            "disabled release must restore exact fallback"
        );
        for pixels in &frames[1..3] {
            assert!(
                pixels
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .any(|p| p[1] > p[0] + 5 && p[1] > p[2] + 5),
                "exercise visible green emission, not an empty sky"
            );
            assert_ne!(pixels, &frames[0]);
        }
        let directory =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/sky-layers");
        std::fs::create_dir_all(&directory).unwrap();
        for (name, pixels) in ["off", "low", "high", "off-again"].into_iter().zip(frames) {
            let file = std::fs::File::create(directory.join(format!("aurora-{name}.png"))).unwrap();
            let mut encoder = png::Encoder::new(file, 129, 65);
            encoder.set_color(png::ColorType::Rgba);
            encoder.set_depth(png::BitDepth::Eight);
            encoder
                .write_header()
                .unwrap()
                .write_image_data(&pixels)
                .unwrap();
        }
    }

    #[test]
    fn aurora_is_independent_cadenced_and_disabled_without_residency() {
        let mut settings = settings(Quality::Low, 8);
        settings.atmosphere_enabled = false;
        settings.airglow = false;
        assert!(EnvironmentBudget::resolve(&settings).is_none());
        settings.aurora_shell = true;
        settings.aurora_intensity = 1.0;
        settings.aurora_animate = false;
        let budget = EnvironmentBudget::resolve(&settings).unwrap();
        assert_eq!(budget.aurora_steps, 8);
        let mut field =
            EnvironmentField::new(3_200_000.0, 5.02e13, Some(thessa_atmosphere())).unwrap();
        let lat = 67.0f64.to_radians();
        let camera = DVec3::new(lat.cos(), lat.sin(), 0.0) * 3_201_000.0;
        field.set_budget(Some(budget));
        let grid = field.update_at(camera, &[], 0.0).unwrap();
        assert_eq!((grid.width(), grid.height()), (64, 32));
        assert!(
            grid.texels().iter().flatten().any(|&c| c > 0.1),
            "must exercise real emission, not roundoff"
        );
        let evaluated = field.stats().evaluated;
        for _ in 0..16 {
            field.update_at(camera, &[], 10.0);
        }
        assert_eq!(
            field.stats().evaluated,
            evaluated,
            "animate=false ignores time"
        );
        let mut animated = budget;
        animated.aurora_animate = true;
        field.set_budget(Some(animated));
        field.update_at(camera, &[], 10.0);
        let evaluated = field.stats().evaluated;
        for _ in 0..16 {
            field.update_at(camera, &[], 11.0);
        }
        assert_eq!(
            field.stats().evaluated,
            evaluated + 1,
            "time change wakes once, paused state reuses"
        );
        field.set_budget(None);
        assert!(field.update_at(camera, &[], 20.0).is_none());
        assert_eq!(field.stats().storage_bytes, 0);
        assert_eq!(field.stats().aurora_steps, 0);
        settings.aurora_quality = thessa_graphics::AuroraQuality::High;
        assert_eq!(
            EnvironmentBudget::resolve(&settings).unwrap().aurora_steps,
            32
        );
    }

    fn settings(quality: Quality, steps: u32) -> ResolvedGraphicsSettings {
        ResolvedGraphicsSettings {
            atmosphere_enabled: true,
            atmosphere_quality: quality,
            atmosphere_steps: steps,
            atmosphere_density_scale: 1.0,
            multi_star: true,
            airglow: false,
            aurora_shell: false,
            aurora_intensity: 0.0,
            ..Default::default()
        }
    }

    fn thessa_atmosphere() -> BodyAtmosphere {
        BodyAtmosphere {
            surface_pressure_pa: 120_000.0,
            gas_constant_j_kg_k: 287.052_87,
        }
    }

    /// Budgets are bounded per quality tier and cost nothing when disabled.
    #[test]
    fn budgets_scale_with_quality_and_disable_cleanly() {
        let low = EnvironmentBudget::resolve(&settings(Quality::Low, 24)).unwrap();
        let medium = EnvironmentBudget::resolve(&settings(Quality::Medium, 24)).unwrap();
        let high = EnvironmentBudget::resolve(&settings(Quality::High, 24)).unwrap();
        assert!(low.width * low.height < medium.width * medium.height);
        assert!(medium.width * medium.height < high.width * high.height);
        assert!(low.view_steps <= medium.view_steps && medium.view_steps <= high.view_steps);
        assert!(low.max_interval_frames >= high.max_interval_frames);
        let mut off = settings(Quality::High, 24);
        off.atmosphere_enabled = false;
        assert!(EnvironmentBudget::resolve(&off).is_none());
    }

    /// An airless body has no sky: the field must not be constructed at all.
    #[test]
    fn airless_bodies_have_no_field() {
        assert!(EnvironmentField::new(3_200_000.0, 3.986_004e14, None).is_none());
        assert!(
            EnvironmentField::new(3_200_000.0, 3.986_004e14, Some(thessa_atmosphere())).is_some()
        );
        // Scale height follows the supplied body gravity even below 0.1 m/s²;
        // there is no hidden gravity floor changing the atmosphere.
        let atmosphere = thessa_atmosphere();
        let optics = build_optics(3_200_000.0, 5.12e11, atmosphere, 1.0).unwrap();
        let expected = atmosphere.gas_constant_j_kg_k * DESIGN_TEMPERATURE_K / 0.05;
        assert!((optics.rayleigh.scale_height_m / expected - 1.0).abs() < 1e-12);
    }

    /// A disabled budget allocates nothing and returns no grid, while an
    /// enabled one evaluates only when the inputs change.
    #[test]
    fn evaluation_is_cadenced_by_input_change() {
        let mut field =
            EnvironmentField::new(3_200_000.0, 3.986_004e14, Some(thessa_atmosphere())).unwrap();
        let camera = DVec3::new(0.0, 3_200_100.0, 0.0);
        let light = CelestialLight {
            direction_to_star: DVec3::Y,
            irradiance_w_m2: 1_300.0,
            color_rgb: [1.0, 1.0, 1.0],
            angular_radius_rad: 0.005,
            visibility: 1.0,
        };
        assert!(field.update(camera, std::slice::from_ref(&light)).is_none());
        field.set_budget(EnvironmentBudget::resolve(&settings(Quality::High, 12)));
        let grid = field.update(camera, std::slice::from_ref(&light)).unwrap();
        assert_eq!((grid.width(), grid.height()), (32, 16));
        assert!(grid.ambient_lux().iter().all(|c| *c > 0.0));
        let first = field.stats().evaluated;
        for _ in 0..8 {
            assert!(field.update(camera, std::slice::from_ref(&light)).is_some());
        }
        // Unchanged inputs re-use the field instead of re-integrating it.
        assert_eq!(field.stats().evaluated, first);
        assert!(field.stats().skipped >= 8);
        // A horizon-scale camera move invalidates it.
        let moved = field.update(camera + DVec3::new(0.0, 0.0, 5_000.0), &[light]);
        assert!(moved.is_some());
        assert!(field.stats().evaluated > first);
    }

    /// A daylit sky above the horizon must be brighter than the same shell at
    /// night, and the integrated ambient must follow.
    #[test]
    fn day_is_brighter_than_night() {
        let mut seen = Vec::new();
        for visibility in [0.0_f64, 1.0] {
            let mut field =
                EnvironmentField::new(3_200_000.0, 3.986_004e14, Some(thessa_atmosphere()))
                    .unwrap();
            field.set_budget(EnvironmentBudget::resolve(&settings(Quality::Medium, 12)));
            let light = CelestialLight {
                direction_to_star: DVec3::new(0.0, 1.0, 0.0),
                irradiance_w_m2: 1_300.0,
                color_rgb: [1.0, 0.95, 0.9],
                angular_radius_rad: 0.005,
                visibility,
            };
            // A camera at the surface looking up: -Y is down in body axes.
            let camera = DVec3::new(0.0, 3_200_100.0, 0.0);
            let grid = field.update(camera, &[light]).unwrap().clone();
            seen.push(grid.ambient_lux()[2]);
        }
        assert!(seen[1] > seen[0] * 10.0, "day {seen:?}");
    }

    /// The visual medium multiplier changes the optics, not the physics data.
    #[test]
    fn density_scale_scales_scattering_only() {
        let one = build_optics(3_200_000.0, 3.986_004e14, thessa_atmosphere(), 1.0).unwrap();
        let half = build_optics(3_200_000.0, 3.986_004e14, thessa_atmosphere(), 0.5).unwrap();
        let none = build_optics(3_200_000.0, 3.986_004e14, thessa_atmosphere(), 0.0).unwrap();
        assert!((half.rayleigh.beta_rgb[0] - one.rayleigh.beta_rgb[0] * 0.5).abs() < 1e-12);
        assert_eq!(none.rayleigh.beta_rgb, [0.0; 3]);
        assert_eq!(one.absorption.len(), none.absorption.len());
    }

    #[test]
    fn budget_change_invalidates_immediately_and_disabled_does_no_work() {
        let mut field =
            EnvironmentField::new(3_200_000.0, 5.02e13, Some(thessa_atmosphere())).unwrap();
        let camera = DVec3::Y * 3_200_100.0;
        let lights = [CelestialLight {
            direction_to_star: DVec3::Y,
            irradiance_w_m2: 1_000.0,
            color_rgb: [1.0; 3],
            angular_radius_rad: 0.005,
            visibility: 1.0,
        }];
        let budget = EnvironmentBudget::resolve(&settings(Quality::Low, 12));
        field.set_budget(budget);
        assert!(field.update(camera, &lights).unwrap().ambient_lux()[0] > 1.0);
        let before = field.revision();
        let mut zero = budget.unwrap();
        zero.density_scale = 0.0;
        field.set_budget(Some(zero));
        assert!(field.revision() > before);
        assert_eq!(
            field.update(camera, &lights).unwrap().ambient_lux(),
            [0.0; 3]
        );
        let evaluated = field.stats().evaluated;
        field.set_budget(None);
        for _ in 0..8 {
            assert!(field.update(camera, &lights).is_none());
        }
        assert_eq!(field.stats().evaluated, evaluated);
        assert_eq!(field.stats().storage_bytes, 0);
    }

    #[test]
    fn secondary_source_changes_invalidate_even_with_fixed_total_irradiance() {
        let a = CelestialLight {
            direction_to_star: DVec3::Y,
            irradiance_w_m2: 1_000.0,
            color_rgb: [1.0; 3],
            angular_radius_rad: 0.005,
            visibility: 1.0,
        };
        let mut b = a.clone();
        b.irradiance_w_m2 = 10.0;
        b.direction_to_star = DVec3::X;
        let before = [a.clone(), b.clone()];
        b.direction_to_star = DVec3::Z;
        assert!(lights_changed(&before, &[a.clone(), b]));
        assert!(!lights_changed(&before, &before));
        assert!(lights_changed(&before, &[a]));
    }

    #[test]
    fn airglow_is_independent_of_scattering_and_obeys_resolved_setting() {
        let mut field =
            EnvironmentField::new(3_200_000.0, 5.02e13, Some(thessa_atmosphere())).unwrap();
        let mut budget = EnvironmentBudget::resolve(&settings(Quality::Low, 12)).unwrap();
        budget.density_scale = 0.0;
        budget.airglow = true;
        field.set_budget(Some(budget));
        let camera = DVec3::Y * 3_200_100.0;
        assert!(field.update(camera, &[]).unwrap().ambient_lux()[1] > 0.0);
        budget.airglow = false;
        field.set_budget(Some(budget));
        assert_eq!(field.update(camera, &[]).unwrap().ambient_lux(), [0.0; 3]);
    }

    /// Reproducible same-process CPU cost and interpolation diagnostic. No
    /// timing assertion: reports latency rather than assuming a frame budget.
    #[test]
    #[ignore = "benchmark: sky-grid evaluation and interpolation envelope"]
    fn sky_grid_cost_and_error() {
        let camera = DVec3::Y * 3_201_000.0;
        let lights = [CelestialLight {
            direction_to_star: DVec3::new(0.6, 0.8, 0.0),
            irradiance_w_m2: 1_000.0,
            color_rgb: [1.0; 3],
            angular_radius_rad: 0.005,
            visibility: 1.0,
        }];
        let mut optics = build_optics(3_200_000.0, 5.02e13, thessa_atmosphere(), 1.0).unwrap();
        optics.emission.airglow_rgb = [0.0; 3];
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut field =
                EnvironmentField::new(3_200_000.0, 5.02e13, Some(thessa_atmosphere())).unwrap();
            field.set_budget(EnvironmentBudget::resolve(&settings(quality, 24)));
            field.update(camera, &lights); // warm Rayon and model
            let mut samples = Vec::new();
            for _ in 0..21 {
                let start = std::time::Instant::now();
                std::hint::black_box(field.evaluate(camera, &lights).unwrap());
                samples.push(start.elapsed().as_secs_f64() * 1000.0);
            }
            samples.sort_by(f64::total_cmp);
            let mut max_error = 0.0_f64;
            let mut sum_error = 0.0_f64;
            // Upper-hemisphere directions, including near-horizon rays.
            for index in 0..128 {
                let y = (f64::from(index) + 0.5) / 128.0;
                let phi = f64::from(index) * 2.399_963_229_728_653;
                let r = (1.0 - y * y).sqrt();
                let direction = DVec3::new(r * phi.cos(), y, r * phi.sin());
                let reference =
                    sky_radiance(&optics, camera, direction, &lights, 128, 64).radiance_rgb;
                let sampled = field.grid().unwrap().sample(direction.as_vec3().to_array());
                for c in 0..3 {
                    // Absolute radiance error normalized by direct stellar E/pi,
                    // not a relative error divided by a near-black sky texel.
                    let error = (f64::from(sampled[c]) / LUX_PER_W_M2 - reference[c]).abs()
                        / (lights[0].irradiance_w_m2 / std::f64::consts::PI);
                    max_error = max_error.max(error);
                    sum_error += error;
                }
            }
            println!(
                "sky_grid quality={quality:?} median_ms={:.3} p95_ms={:.3} max_error_vs_direct_e_pi={max_error:.6} mean_error={:.6}",
                samples[10],
                samples[19],
                sum_error / 384.0
            );
            assert!(
                max_error < 0.25,
                "fixture absolute error envelope, {quality:?}: {max_error}"
            );
        }
    }
}
