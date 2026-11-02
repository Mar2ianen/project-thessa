//! Native consumers of accepted compiled-nozzle observations. The legacy
//! scalar X-15 retains an explicit provisional visualization deck.
//! Nothing here writes authoritative state, forces, clocks, or mesh poses.
use glam::{DQuat, DVec3};
use std::sync::Arc;
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_plume_core::{
    AxialProfile, ExhaustFamily, PlumeEnvironment, PlumeSource, RigidTransform,
    profile::build_axial_profile,
};
use thessa_render_core::mesh::MeshInstance;
use thessa_render_wgpu::plume::{PlumeFrame, ProfileStation, VolumeDraw};

#[derive(Clone, Copy)]
pub struct Input {
    pub throttle: f64,
    pub engine_active: bool,
    pub sim_time_s: f64,
    /// Ambient air minus COM velocity in native scene axes (m/s).
    pub air_velocity_scene_mps: DVec3,
    pub angular_velocity_scene_rps: DVec3,
    pub air_rotation_scene_rps: DVec3,
}

/// SI craft-space station (nose +Y); deliberately not scaled a second time
/// by the mesh asset normalization. Replaced by compiled nozzle data later.
const NOZZLE_M: DVec3 = DVec3::new(0.0, -7.5, 0.0);

pub fn environment(sample: Option<[f64; 3]>) -> PlumeEnvironment {
    let [pressure_pa, density_kg_m3, temperature_k] = sample
        .filter(|s| s.iter().all(|v| v.is_finite() && *v >= 0.0))
        .unwrap_or([0.0; 3]);
    PlumeEnvironment {
        pressure_pa,
        density_kg_m3,
        temperature_k,
        // Same provisional oxygen assumption as the retained producer.
        // Secondary combustion is not implemented in this slice.
        oxygen_fraction: if density_kg_m3 > 0.0 { 0.21 } else { 0.0 },
        flow_velocity_local_mps: [0.0; 3],
    }
}

fn source(throttle: f64) -> PlumeSource {
    PlumeSource {
        nozzle_to_vehicle: RigidTransform::IDENTITY,
        exit_radius_m: 0.55,
        mass_flow_kg_s: 480.0,
        exhaust_velocity_mps: 3550.0,
        exit_pressure_pa: 120_000.0,
        exit_temperature_k: 1850.0,
        exit_mach: 3.5,
        gamma: 1.2,
        throttle,
        exhaust: ExhaustFamily::Methalox,
    }
}

struct Field {
    key: (i64, i64),
    source: PlumeSource,
    profile: AxialProfile,
    environment: PlumeEnvironment,
    physical: bool,
    gpu_profile: Arc<[ProfileStation]>,
}

#[derive(Default)]
pub struct Cache {
    field: Option<Field>,
}

impl Cache {
    pub fn clear(&mut self) {
        self.field = None;
    }

    pub fn frame(
        &mut self,
        input: Input,
        env: PlumeEnvironment,
        instance: &MeshInstance,
        camera_m: DVec3,
        settings: &ResolvedGraphicsSettings,
    ) -> Option<PlumeFrame> {
        if !settings.plume_enabled || !input.throttle.is_finite() || !input.sim_time_s.is_finite() {
            self.clear();
            return None;
        }
        if !input.air_velocity_scene_mps.is_finite()
            || !input.angular_velocity_scene_rps.is_finite()
            || !input.air_rotation_scene_rps.is_finite()
        {
            self.clear();
            return None;
        }
        let amount = if input.engine_active {
            input.throttle.clamp(0.0, 1.0)
        } else {
            0.0
        };
        if amount == 0.0 {
            self.clear();
            return None;
        }
        // 2% throttle bands and logarithmic ~2% pressure bands with a 50 Pa
        // floor. Unlike p/(.02*p+50), this does not saturate near 50 buckets
        // and accidentally freeze atmospheric profiles over wide ranges.
        let key = (
            (amount * 50.0).round().max(1.0) as i64,
            (env.pressure_pa / 50.0).ln_1p().div_euclid(1.02_f64.ln()) as i64,
        );
        if self
            .field
            .as_ref()
            .is_none_or(|f| f.physical || f.key != key)
        {
            let source = source(amount);
            let profile = build_axial_profile(&source, &env, 48).ok()?;
            let gpu_profile = profile
                .stations
                .iter()
                .map(|s| {
                    [
                        s.z_m as f32,
                        s.radius_m as f32,
                        s.extinction_per_m as f32,
                        s.shock as f32,
                        s.emission_rgb[0] as f32,
                        s.emission_rgb[1] as f32,
                        s.emission_rgb[2] as f32,
                        0.0,
                    ]
                })
                .collect();
            self.field = Some(Field {
                key,
                source,
                profile,
                environment: env,
                physical: false,
                gpu_profile,
            });
        }
        instance.camera_relative_matrix(camera_m)?;
        let craft = instance.rotation
            * thessa_flight_authority::presentation::x15_asset_to_craft_rotation()
                .as_dquat()
                .normalize()
                .conjugate();
        draw_field(
            self.field.as_ref()?,
            input,
            env,
            Placement {
                position_m: instance.position_m,
                rotation: craft,
                nozzle_m: NOZZLE_M,
                nozzle_rotation: DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            },
            camera_m,
            settings,
        )
    }

    /// Actual accepted flow has already been throttled by resource allocation.
    /// Missing legacy outlets stay absent; no scalar-X15 deck is substituted.
    #[allow(clippy::too_many_arguments)]
    pub fn accepted_frame(
        &mut self,
        nozzle: &thessa_sim_core::RocketNozzleObservation,
        pose: thessa_flight_authority::presentation::FlightPose,
        frame: thessa_render_core::BodyFrame,
        input: Input,
        env: PlumeEnvironment,
        camera_m: DVec3,
        settings: &ResolvedGraphicsSettings,
    ) -> Option<PlumeFrame> {
        if !settings.plume_enabled
            || !input.sim_time_s.is_finite()
            || !input.air_velocity_scene_mps.is_finite()
            || !input.angular_velocity_scene_rps.is_finite()
            || !input.air_rotation_scene_rps.is_finite()
        {
            self.clear();
            return None;
        }
        let Some(outlet) = nozzle.outlet else {
            self.clear();
            return None;
        };
        let Some(state) = nozzle.state else {
            self.clear();
            return None;
        };
        if state.mass_flow_kg_s <= 0.0 || nozzle.active_fraction <= 0.0 {
            self.clear();
            return None;
        }
        let axis = DVec3::from_array(outlet.exhaust_axis_body);
        if !axis.is_finite() || (axis.length_squared() - 1.0).abs() > 2e-9 {
            self.clear();
            return None;
        }
        let nozzle_rotation = DQuat::from_rotation_arc(DVec3::Z, axis);
        use thessa_sim_core::Propellant;
        let exhaust = match state.propellant {
            Propellant::LoxRp1 => ExhaustFamily::Kerolox,
            Propellant::LoxMethane => ExhaustFamily::Methalox,
            Propellant::LoxHydrogen => ExhaustFamily::Hydrolox,
            Propellant::NtoMmh | Propellant::MonopropHydrazine => ExhaustFamily::Hypergolic,
            Propellant::ColdGasNitrogen | Propellant::ColdGasHelium => ExhaustFamily::ColdGas,
            Propellant::SolidApcp => ExhaustFamily::Solid,
        };
        let source = PlumeSource {
            nozzle_to_vehicle: RigidTransform {
                translation_m: outlet.position_body_m,
                rotation_xyzw: nozzle_rotation.to_array(),
            },
            exit_radius_m: outlet.radius_m,
            mass_flow_kg_s: state.mass_flow_kg_s,
            exhaust_velocity_mps: state.exhaust_velocity_mps,
            exit_pressure_pa: state.exit_pressure_pa,
            exit_temperature_k: state.exit_temp_k,
            exit_mach: state.exit_mach,
            gamma: nozzle.gamma,
            throttle: 1.0,
            exhaust,
        };
        let optical_source = PlumeSource {
            nozzle_to_vehicle: RigidTransform::IDENTITY,
            ..source
        };
        if self.field.as_ref().is_none_or(|f| {
            let previous = PlumeSource {
                nozzle_to_vehicle: RigidTransform::IDENTITY,
                ..f.source
            };
            !f.physical || previous != optical_source || f.environment != env
        }) {
            let profile = build_axial_profile(&source, &env, 48).ok()?;
            let gpu_profile = profile
                .stations
                .iter()
                .map(|s| {
                    [
                        s.z_m as f32,
                        s.radius_m as f32,
                        s.extinction_per_m as f32,
                        s.shock as f32,
                        s.emission_rgb[0] as f32,
                        s.emission_rgb[1] as f32,
                        s.emission_rgb[2] as f32,
                        0.0,
                    ]
                })
                .collect();
            self.field = Some(Field {
                key: (0, 0),
                source,
                profile,
                gpu_profile,
                environment: env,
                physical: true,
            });
        }
        // A COM rebase changes placement, not the optical field or uploads.
        self.field.as_mut()?.source = source;
        draw_field(
            self.field.as_ref()?,
            input,
            env,
            Placement {
                position_m: frame.render_to_body_point(pose.render_position_m()),
                rotation: frame.rotation().inverse()
                    * DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2)
                    * pose.orientation_body_to_inertial(),
                nozzle_m: DVec3::from_array(outlet.position_body_m),
                nozzle_rotation,
            },
            camera_m,
            settings,
        )
    }
}

struct Placement {
    position_m: DVec3,
    rotation: DQuat,
    nozzle_m: DVec3,
    nozzle_rotation: DQuat,
}

fn draw_field(
    field: &Field,
    input: Input,
    env: PlumeEnvironment,
    placement: Placement,
    camera_m: DVec3,
    settings: &ResolvedGraphicsSettings,
) -> Option<PlumeFrame> {
    if field.profile.is_empty() {
        return None;
    }
    let nozzle =
        ((placement.position_m - camera_m) + placement.rotation * placement.nozzle_m).as_vec3();
    let nozzle_rotation = placement.rotation * placement.nozzle_rotation;
    let exhaust = (nozzle_rotation * DVec3::Z).as_vec3().normalize();
    if !nozzle.is_finite() || !exhaust.is_finite() {
        return None;
    }
    let time = if settings.plume_flicker {
        input.sim_time_s as f32
    } else {
        0.0
    };
    let offset = placement.rotation * placement.nozzle_m;
    let mut motion_env = env;
    motion_env.flow_velocity_local_mps = (nozzle_rotation.conjugate()
        * (input.air_velocity_scene_mps
            + (input.air_rotation_scene_rps - input.angular_velocity_scene_rps).cross(offset)))
    .to_array();
    let angular = (nozzle_rotation.conjugate() * input.angular_velocity_scene_rps).to_array();
    let deformed = angular[0] != 0.0
        || angular[1] != 0.0
        || (motion_env.density_kg_m3 > 0.0
            && (motion_env.flow_velocity_local_mps[0] != 0.0
                || motion_env.flow_velocity_local_mps[1] != 0.0));
    let points = if deformed {
        thessa_plume_core::motion::centerline(
            &field.source,
            &motion_env,
            angular,
            field.profile.length_m,
            if settings.plume_quality == Quality::High {
                17
            } else {
                9
            },
        )
        .ok()?
    } else {
        Vec::new()
    };
    // The exact straight field remains the cheaper path below 0.1 mm
    // transverse displacement; this is a representation tolerance only.
    let bent = points
        .iter()
        .any(|p| p[0].hypot(p[1]) > 1e-4 || (p[2] - p[3]).abs() > 1e-4);
    let centerline = if bent {
        points
            .iter()
            .map(|p| {
                let center = (placement.position_m - camera_m)
                    + offset
                    + nozzle_rotation * DVec3::new(p[0], p[1], p[2]);
                [
                    center.x as f32,
                    center.y as f32,
                    center.z as f32,
                    p[3] as f32,
                ]
            })
            .collect::<Vec<_>>()
            .into()
    } else {
        Arc::from([])
    };
    Some(PlumeFrame::Volume(VolumeDraw {
        pixel_grid: match settings.plume_quality {
            Quality::Low => 1,
            Quality::Medium => 4,
            Quality::High => 8,
        },
        radial_reference: (nozzle_rotation * DVec3::X).as_vec3().to_array(),
        centerline,
        origin_len: nozzle.extend(field.profile.length_m as f32).to_array(),
        axis_radius: exhaust
            .extend(
                field
                    .profile
                    .stations
                    .iter()
                    .map(|s| s.radius_m)
                    .fold(0.0, f64::max) as f32
                    * 1.6,
            )
            .to_array(),
        animation: [
            match settings.plume_quality {
                Quality::Low => 6.0,
                Quality::Medium => 8.0,
                Quality::High => 14.0,
            },
            time,
            if settings.plume_flicker { 0.4 } else { 0.0 },
            field.source.exhaust_velocity_mps as f32 * 0.1,
        ],
        shock_cells: [
            field.profile.cells.spacing_m as f32,
            field.profile.cells.radius_m as f32,
            field.profile.cells.compression as f32,
            field.profile.cells.phase as f32,
        ],
        diamonds: settings.plume_diamonds,
        tail_dissolve: true,
        profile: field.gpu_profile.clone(),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;
    use glam::Vec3;

    fn instance(position_m: DVec3, craft: DQuat, scale: f64) -> MeshInstance {
        MeshInstance {
            position_m,
            rotation: craft
                * thessa_flight_authority::presentation::x15_asset_to_craft_rotation()
                    .as_dquat()
                    .normalize(),
            scale,
        }
    }

    fn settings() -> ResolvedGraphicsSettings {
        ResolvedGraphicsSettings::default()
    }

    fn input() -> Input {
        Input {
            throttle: 0.5,
            engine_active: true,
            sim_time_s: 12.0,
            air_velocity_scene_mps: DVec3::ZERO,
            angular_velocity_scene_rps: DVec3::ZERO,
            air_rotation_scene_rps: DVec3::ZERO,
        }
    }

    #[test]
    fn cache_cadence_tracks_pressure_without_high_pressure_saturation() {
        let mut cache = Cache::default();
        let instance = instance(DVec3::ZERO, DQuat::IDENTITY, 1.0);
        let camera = DVec3::new(0.0, 0.0, 100.0);
        let env = environment(Some([100_000.0, 1.2, 280.0]));
        cache
            .frame(input(), env, &instance, camera, &settings())
            .unwrap();
        let first = cache.field.as_ref().unwrap().profile.stations.as_ptr();
        let mut jitter = env;
        jitter.pressure_pa += 0.001;
        cache
            .frame(input(), jitter, &instance, camera, &settings())
            .unwrap();
        assert_eq!(
            cache.field.as_ref().unwrap().profile.stations.as_ptr(),
            first
        );
        let next = environment(Some([50_000.0, 0.6, 260.0]));
        cache
            .frame(input(), next, &instance, camera, &settings())
            .unwrap();
        assert_ne!(
            cache.field.as_ref().unwrap().key.1,
            (env.pressure_pa / 50.0).ln_1p().div_euclid(1.02_f64.ln()) as i64
        );
        let mut off = input();
        off.engine_active = false;
        assert!(
            cache
                .frame(off, env, &instance, camera, &settings())
                .is_none()
        );
        assert!(cache.field.is_none());
        let mut disabled = settings();
        disabled.plume_enabled = false;
        assert!(
            cache
                .frame(input(), env, &instance, camera, &disabled)
                .is_none()
        );
        assert!(cache.field.is_none());
    }

    #[test]
    fn nozzle_is_in_physical_metres_and_camera_relative_even_at_astronomical_origins() {
        let mut cache = Cache::default();
        let craft = DQuat::from_rotation_x(0.6) * DQuat::from_rotation_z(1.1);
        for translation in [DVec3::ZERO, DVec3::new(1e12, -2e12, 3e12)] {
            let camera = translation + DVec3::new(0.0, 0.0, 100.0);
            for scale in [1.0, 15.45 / 16.77] {
                let mesh = instance(translation, craft, scale);
                let before = mesh;
                let Some(PlumeFrame::Volume(draw)) =
                    cache.frame(input(), environment(None), &mesh, camera, &settings())
                else {
                    panic!("expected volume");
                };
                let expected = (translation - camera + craft * NOZZLE_M).as_vec3();
                assert!(
                    Vec3::from_array(draw.origin_len[..3].try_into().unwrap()).distance(expected)
                        < 1e-4
                );
                assert!(
                    Vec3::from_array(draw.axis_radius[..3].try_into().unwrap())
                        .distance((craft * DVec3::NEG_Y).as_vec3())
                        < 1e-5
                );
                assert_eq!(mesh, before);
            }
        }
    }

    #[test]
    fn quality_and_animation_change_representation_not_the_field() {
        let mut cache = Cache::default();
        let mesh = instance(DVec3::ZERO, DQuat::IDENTITY, 1.0);
        let camera = DVec3::new(0.0, 0.0, 100.0);
        let env = environment(None);
        let mut settings = settings();
        settings.plume_flicker = false;
        let Some(PlumeFrame::Volume(a)) = cache.frame(input(), env, &mesh, camera, &settings)
        else {
            panic!()
        };
        let key = cache.field.as_ref().unwrap().key;
        let profile = cache.field.as_ref().unwrap().profile.clone();
        settings.plume_quality = Quality::High;
        settings.plume_diamonds = false;
        let Some(PlumeFrame::Volume(b)) = cache.frame(input(), env, &mesh, camera, &settings)
        else {
            panic!()
        };
        assert_eq!((a.animation[0], b.animation[0]), (8.0, 14.0));
        assert_eq!((a.pixel_grid, b.pixel_grid), (4, 8));
        assert_eq!(a.animation[1], 0.0);
        assert!(!b.diamonds);
        settings.plume_quality = Quality::Low;
        let Some(PlumeFrame::Volume(low)) = cache.frame(input(), env, &mesh, camera, &settings)
        else {
            panic!("all native qualities must consume the optical field")
        };
        assert_eq!((low.animation[0], low.pixel_grid), (6.0, 1));
        assert!(Arc::ptr_eq(&a.profile, &low.profile));
        assert_eq!(low.origin_len, a.origin_len);
        assert_eq!(low.axis_radius, a.axis_radius);
        assert_eq!(low.shock_cells, a.shock_cells);
        assert_eq!(cache.field.as_ref().unwrap().key, key);
        assert_eq!(cache.field.as_ref().unwrap().profile, profile);
        assert_eq!(environment(Some([f64::NAN, 1.0, 280.0])), environment(None));
        assert_eq!(environment(None).pressure_pa, 0.0);
    }

    #[test]
    fn motion_tracks_air_and_attitude_without_rebuilding_optics_or_following_camera() {
        let craft = DQuat::from_rotation_x(0.4) * DQuat::from_rotation_z(0.7);
        let env = environment(Some([101325.0, 1.2, 280.0]));
        let mut settings = settings();
        settings.plume_flicker = false;
        let mut moving = input();
        moving.air_velocity_scene_mps = DVec3::new(180.0, -90.0, 30.0);
        moving.angular_velocity_scene_rps = DVec3::new(0.5, -0.7, 0.2);
        let mut cache = Cache::default();
        let mut baseline = None;
        for shift in [DVec3::ZERO, DVec3::new(1e12, -2e12, 3e12)] {
            let mesh = instance(shift, craft, 1.0);
            let camera = shift + DVec3::new(2.0, 3.0, 100.0);
            let Some(PlumeFrame::Volume(draw)) = cache.frame(moving, env, &mesh, camera, &settings)
            else {
                panic!()
            };
            assert_eq!(draw.centerline.len(), 9);
            assert_eq!(draw.centerline[0][..3], draw.origin_len[..3]);
            assert_eq!(
                draw.animation[1], 0.0,
                "motion does not require cosmetic flicker"
            );
            if let Some(ref old) = baseline {
                assert_eq!(&draw.centerline, old);
            }
            baseline = Some(draw.centerline);
        }
        let mesh = instance(DVec3::ZERO, craft, 1.0);
        let camera = DVec3::new(2.0, 3.0, 100.0);
        let profile = cache.field.as_ref().unwrap().gpu_profile.clone();
        settings.plume_quality = Quality::High;
        let Some(PlumeFrame::Volume(high)) = cache.frame(moving, env, &mesh, camera, &settings)
        else {
            panic!()
        };
        assert_eq!(high.centerline.len(), 17);
        for (a, b) in baseline
            .unwrap()
            .iter()
            .zip(high.centerline.iter().step_by(2))
        {
            assert_eq!(a, b);
        }
        assert!(Arc::ptr_eq(&profile, &high.profile));
        let Some(PlumeFrame::Volume(rest)) = cache.frame(input(), env, &mesh, camera, &settings)
        else {
            panic!()
        };
        assert!(rest.centerline.is_empty());
        assert!(Arc::ptr_eq(&profile, &rest.profile));
        moving.air_velocity_scene_mps = DVec3::NAN;
        assert!(cache.frame(moving, env, &mesh, camera, &settings).is_none());
        assert!(cache.field.is_none());
    }

    #[test]
    #[ignore = "hardware: actual cached native producer orbit/motion HDR preview; new THESSA_BENCH_OUTPUT required"]
    fn plume_motion_gallery() {
        motion_gallery(false);
    }

    #[test]
    #[ignore = "hardware: close native shock-cell orbit at declared 30 kPa ambient"]
    fn plume_shock_gallery() {
        motion_gallery(true);
    }

    fn cache_window(env: &PlumeEnvironment) -> f64 {
        let profile = build_axial_profile(&source(1.0), env, 48).unwrap();
        (profile.cells.spacing_m * 2.5).min(profile.length_m)
    }

    fn motion_gallery(close: bool) {
        use thessa_render_wgpu::{
            Backend, plume::PlumeRenderer, terrain::OffscreenTarget, view::ViewParams,
        };
        let out = std::path::PathBuf::from(
            std::env::var_os("THESSA_BENCH_OUTPUT").expect("new output required"),
        );
        std::fs::create_dir(&out).unwrap();
        let backend = Backend::bring_up().unwrap();
        let depth = OffscreenTarget::new(&backend, 640, 360).unwrap();
        let hdr = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("native-plume-motion-preview"),
            size: wgpu::Extent3d {
                width: 640,
                height: 360,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba16Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color = hdr.create_view(&Default::default());
        let mut renderer =
            PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
        let exposure = thessa_render_core::display::manual_exposure(13.0);
        let settings = ResolvedGraphicsSettings {
            plume_quality: Quality::High,
            ..settings()
        };
        let pressure = if close { 30_000.0 } else { 1000.0 };
        let env = environment(Some([pressure, pressure / (287.05 * 288.15), 288.15]));
        let mut cache = Cache::default();
        let length = build_axial_profile(&source(1.0), &env, 48)
            .unwrap()
            .length_m;
        let window = if close { cache_window(&env) } else { length };
        let mut records = Vec::new();
        for index in 0..32 {
            let time = f64::from(index) / 20.0;
            let craft = DQuat::from_rotation_z(0.35 * time);
            let position = DVec3::X * (120.0 * time);
            let mesh = instance(position, craft, 1.0);
            let target = position + craft * (NOZZLE_M + DVec3::NEG_Y * (0.45 * window));
            let yaw = time * 1.4;
            let camera = target + DVec3::new(yaw.sin(), 0.0, yaw.cos()) * (window * 1.25);
            let view = ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(camera, target, DVec3::Y)
                    .inverse(),
                world_position_m: camera,
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    0.9,
                    640.0 / 360.0,
                    0.1,
                    1e6,
                ),
                exposure,
            };
            let input = Input {
                throttle: 1.0,
                engine_active: true,
                sim_time_s: time,
                air_velocity_scene_mps: -DVec3::X * 120.0,
                angular_velocity_scene_rps: DVec3::Z * 0.35,
                air_rotation_scene_rps: DVec3::ZERO,
            };
            let frame = cache.frame(input, env, &mesh, camera, &settings).unwrap();
            let PlumeFrame::Volume(ref draw) = frame else {
                panic!("volume required")
            };
            assert_eq!(draw.centerline.len(), 17);
            records.push(
                serde_json::json!({"time_s":time,"craft_position_m":position.to_array(),
                "craft_rotation_xyzw":craft.to_array(),"camera_m":camera.to_array(),
                "world_from_view":view.world_from_view.to_cols_array(),"origin_len":draw.origin_len,
                "centerline":&*draw.centerline,"radial_reference":draw.radial_reference,
                "pixel_filter":"adaptive-box-v1","pixel_grid":draw.pixel_grid,
                    "axis_radius":draw.axis_radius,"animation":draw.animation,"diamonds":draw.diamonds,
                    "shock_cells":draw.shock_cells,"tail_dissolve":draw.tail_dissolve,"clip_from_view":view.clip_from_view.to_cols_array()}),
            );
            renderer.update(&backend, &view, Some(frame)).unwrap();
            if index > 0 {
                assert_eq!(renderer.stats().profile_upload_bytes, 0);
            }
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &color,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &depth.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
            }
            renderer.encode(&mut encoder, &color, &depth.depth);
            backend.queue.submit([encoder.finish()]);
            let raw = thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap();
            assert!(
                raw.as_chunks::<2>()
                    .0
                    .iter()
                    .all(|b| u16::from_le_bytes(*b) & 0x7c00 != 0x7c00)
            );
            std::fs::write(out.join(format!("frame-{index:02}.rgba16f")), raw).unwrap();
        }
        std::fs::write(out.join("scene.json"),serde_json::to_vec_pretty(&serde_json::json!({
            "scope":"Actual native Cache/PlumeRenderer; prescribed kinematic preview, not calibrated firing or authority replay",
            "extent":[640,360],"frames":records,"settings":settings,"exposure_gain":exposure,
            "emission_units":"RGB cd/m^3; graybody reference, authored hue",
            "profile":&*cache.field.as_ref().unwrap().gpu_profile,
            "ambient_pressure_pa":pressure,"view_window_m":window,"ambient_density_kg_m3":env.density_kg_m3,
            "air_velocity_scene_mps":[-120.0,0.0,0.0],"angular_velocity_scene_rps":[0.0,0.0,0.35]
        })).unwrap()).unwrap();
        eprintln!(
            "native_motion_gallery adapter={:?} frames=32 extent=640x360 profile_uploads=1",
            backend.adapter_name
        );
    }

    #[test]
    #[ignore = "hardware: actual native qualities at day/night exposure; new output required"]
    fn plume_quality_exposure_reference() {
        use thessa_render_wgpu::{
            Backend, plume::PlumeRenderer, terrain::OffscreenTarget, view::ViewParams,
        };
        let out = std::path::PathBuf::from(std::env::var_os("THESSA_BENCH_OUTPUT").unwrap());
        std::fs::create_dir(&out).unwrap();
        let backend = Backend::bring_up().unwrap();
        let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
        let hdr = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("native-quality-exposure"),
            size: wgpu::Extent3d {
                width: 129,
                height: 129,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba16Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color = hdr.create_view(&Default::default());
        let mut renderer =
            PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
        let mesh = instance(DVec3::ZERO, DQuat::IDENTITY, 1.0);
        let mut records = Vec::new();
        for pressure in [101325.0, 1000.0, 0.0] {
            let env = environment(Some([pressure, pressure / (287.05 * 288.15), 288.15]));
            let profile = build_axial_profile(&source(input().throttle), &env, 48).unwrap();
            let window = (2.5 * profile.cells.spacing_m)
                .max(8.0 * source(1.0).exit_radius_m)
                .min(profile.length_m);
            let aim = NOZZLE_M + DVec3::NEG_Y * window * 0.4;
            let camera = aim + DVec3::Z * window * 1.5;
            let mut cache = Cache::default();
            let mut published = None;
            for quality in [Quality::Low, Quality::Medium, Quality::High] {
                let settings = ResolvedGraphicsSettings {
                    plume_quality: quality,
                    plume_flicker: false,
                    ..settings()
                };
                for ev in [13.0, 5.0] {
                    let view = ViewParams {
                        world_from_view: glam::dcamera::rh::view::look_at_mat4(
                            camera,
                            aim,
                            DVec3::Y,
                        )
                        .inverse(),
                        world_position_m: camera,
                        clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                            0.9, 1.0, 0.1, 1e6,
                        ),
                        exposure: thessa_render_core::display::manual_exposure(ev),
                    };
                    let frame = cache.frame(input(), env, &mesh, camera, &settings).unwrap();
                    let PlumeFrame::Volume(ref draw) = frame else {
                        panic!("native qualities share volume optics")
                    };
                    if let Some(ref p) = published {
                        assert!(Arc::ptr_eq(p, &draw.profile));
                    } else {
                        published = Some(draw.profile.clone());
                    }
                    let name = format!("p{pressure:.0}-{quality:?}-ev{ev:.0}");
                    records.push(serde_json::json!({"name":name,"pressure_pa":pressure,"quality":quality,
                        "ev100":ev,"exposure_gain":view.exposure,"extent":[129,129],"origin_len":draw.origin_len,
                        "axis_radius":draw.axis_radius,"profile":&*draw.profile,"shock_cells":draw.shock_cells,
                        "centerline":&*draw.centerline,"steps":draw.animation[0],"pixel_grid":draw.pixel_grid,
                        "camera_m":camera.to_array(),"world_from_view":view.world_from_view.to_cols_array(),
                        "clip_from_view":view.clip_from_view.to_cols_array()}));
                    renderer.update(&backend, &view, Some(frame)).unwrap();
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    {
                        let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &color,
                                depth_slice: None,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &target.depth,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Clear(1.0),
                                        store: wgpu::StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            ..Default::default()
                        });
                    }
                    renderer.encode(&mut encoder, &color, &target.depth);
                    backend.queue.submit([encoder.finish()]);
                    let pixels = thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap();
                    assert!(
                        pixels
                            .as_chunks::<2>()
                            .0
                            .iter()
                            .all(|b| u16::from_le_bytes(*b) & 0x7c00 != 0x7c00)
                    );
                    assert!(
                        pixels
                            .as_chunks::<8>()
                            .0
                            .iter()
                            .any(|p| u16::from_le_bytes([p[4], p[5]]) & 0x7fff > 0)
                    );
                    std::fs::write(out.join(format!("{name}.rgba16f")), pixels).unwrap();
                }
            }
        }
        std::fs::write(
            out.join("scenes.json"),
            serde_json::to_vec_pretty(&records).unwrap(),
        )
        .unwrap();
        eprintln!(
            "native_quality_exposure adapter={:?} records={} field=shared exposure=13/5",
            backend.adapter_name,
            records.len()
        );
    }

    #[test]
    #[ignore = "diagnostic: one-source cached frame extraction versus profile rebuild CPU cost"]
    fn plume_producer_cost() {
        let mesh = instance(DVec3::ZERO, DQuat::IDENTITY, 15.45 / 16.77);
        let camera = DVec3::new(0.0, 0.0, 100.0);
        let env = environment(Some([101_325.0, 1.2, 280.0]));
        let settings = settings();
        let mut cache = Cache::default();
        let count = 20_000;
        let start = std::time::Instant::now();
        for _ in 0..count {
            std::hint::black_box(cache.frame(input(), env, &mesh, camera, &settings));
        }
        let cached_us = start.elapsed().as_secs_f64() * 1e6 / count as f64;
        let start = std::time::Instant::now();
        for _ in 0..count {
            cache.clear();
            std::hint::black_box(cache.frame(input(), env, &mesh, camera, &settings));
        }
        let rebuild_us = start.elapsed().as_secs_f64() * 1e6 / count as f64;
        eprintln!(
            "native_plume_cpu sources=1 stations=48 iterations={count} cached_us={cached_us:.6} rebuild_us={rebuild_us:.6}"
        );
    }
}

#[cfg(test)]
mod accepted_tests {
    use super::*;
    use thessa_sim_core::{
        ChamberMaterial, CompiledEngine, CoolingMode, EngineCycle, EngineMount, LiquidEngineSpec,
        NozzleContour, Propellant, RocketNozzleId, RocketNozzleObservation, SimTime,
    };

    #[test]
    fn accepted_flow_and_body_outlet_ignore_pilot_command_and_reuse_field_on_com_rebase() {
        let liquid = LiquidEngineSpec {
            name: "accepted-native".into(),
            propellant: Propellant::LoxMethane,
            cycle: EngineCycle::GasGenerator,
            chamber_pressure_pa: 9.7e6,
            throat_radius_m: 0.08,
            expansion_ratio: 18.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Bell,
            chamber_material: ChamberMaterial::nickel_superalloy(),
            cooling: CoolingMode::Regenerative,
            mixture_ratio: None,
            characteristic_length_m: None,
            gimbal_range_rad: 0.0,
            min_throttle: None,
            restartable: true,
        }
        .compile()
        .unwrap();
        let gamma = liquid.gamma;
        let mount = EngineMount {
            name: "actual".into(),
            engine: CompiledEngine::Liquid(liquid),
            position_body_m: [-2.0, 1.0, 0.0],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        let point = mount.engine.operating_point(0.5, 100000.0, 0.0).unwrap();
        let mut nozzle = RocketNozzleObservation {
            id: RocketNozzleId::Engine(0),
            outlet: mount.nozzle_outlet(),
            gamma,
            state: Some(mount.engine.plume_state(&point)),
            active_fraction: 1.0,
        };
        let orientation = DQuat::from_rotation_y(0.73) * DQuat::from_rotation_z(-1.21);
        let position = DVec3::new(1e8, 2e8, -3e8);
        let pose = thessa_flight_authority::presentation::FlightPose::new(
            SimTime(12.0),
            position,
            orientation,
        )
        .unwrap();
        let frame = thessa_render_core::BodyFrame::new(
            DVec3::splat(1e12),
            DQuat::from_rotation_y(0.67),
            DVec3::splat(1e12),
        )
        .unwrap();
        let camera =
            frame.render_to_body_point(pose.render_position_m()) + DVec3::new(10.0, 20.0, 30.0);
        let settings = ResolvedGraphicsSettings {
            plume_enabled: true,
            ..Default::default()
        };
        let env = environment(Some([100000.0, 1.2, 290.0]));
        let mut input = Input {
            throttle: 0.1,
            engine_active: false,
            sim_time_s: 12.0,
            air_velocity_scene_mps: DVec3::ZERO,
            angular_velocity_scene_rps: DVec3::ZERO,
            air_rotation_scene_rps: DVec3::ZERO,
        };
        let mut cache = Cache::default();
        let PlumeFrame::Volume(first) = cache
            .accepted_frame(&nozzle, pose, frame, input, env, camera, &settings)
            .unwrap()
        else {
            panic!()
        };
        let source = cache.field.as_ref().unwrap().source;
        assert_eq!(source.throttle, 1.0);
        assert_eq!(source.mass_flow_kg_s, point.mass_flow_kg_s);
        assert_eq!(source.gamma, gamma);
        let rotation = frame.rotation().inverse()
            * DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2)
            * orientation;
        let expected = frame.render_to_body_point(pose.render_position_m()) - camera
            + rotation * DVec3::from_array(nozzle.outlet.unwrap().position_body_m);
        assert!((glam::Vec3::from_slice(&first.origin_len).as_dvec3() - expected).length() < 1e-5);
        input.throttle = 1.0;
        input.engine_active = true;
        let PlumeFrame::Volume(second) = cache
            .accepted_frame(&nozzle, pose, frame, input, env, camera, &settings)
            .unwrap()
        else {
            panic!()
        };
        assert_eq!(
            first, second,
            "pilot command must not re-throttle accepted flow"
        );
        let shift = DVec3::new(0.125, -0.25, 0.5);
        let outlet = nozzle.outlet.as_mut().unwrap();
        outlet.position_body_m = (DVec3::from_array(outlet.position_body_m) + shift).to_array();
        let rebased = thessa_flight_authority::presentation::FlightPose::new(
            SimTime(12.0),
            position - orientation * shift,
            orientation,
        )
        .unwrap();
        let PlumeFrame::Volume(third) = cache
            .accepted_frame(&nozzle, rebased, frame, input, env, camera, &settings)
            .unwrap()
        else {
            panic!()
        };
        assert!(Arc::ptr_eq(&first.profile, &third.profile));
        assert!(
            (glam::Vec3::from_slice(&first.origin_len) - glam::Vec3::from_slice(&third.origin_len))
                .length()
                < 1e-5
        );
        nozzle.outlet = None;
        assert!(
            cache
                .accepted_frame(&nozzle, pose, frame, input, env, camera, &settings)
                .is_none()
        );
        assert!(cache.field.is_none());
    }
}
