//! Direct native port of the starter X-15 plume producer. This provisional
//! methalox deck is a visualization fixture, not calibrated propulsion data.
//! Nothing here writes authoritative state, forces, clocks, or mesh poses.
use glam::{DQuat, DVec3, Mat4};
use std::sync::Arc;
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_plume_core::{
    AxialProfile, ExhaustFamily, PlumeEnvironment, PlumeSource, RigidTransform,
    profile::build_axial_profile,
};
use thessa_render_core::mesh::MeshInstance;
use thessa_render_wgpu::plume::{ConeDraw, PlumeFrame, ProfileStation, VolumeDraw};

#[derive(Clone, Copy)]
pub struct Input {
    pub throttle: f64,
    pub engine_active: bool,
    pub sim_time_s: f64,
    /// Ambient air minus COM velocity, inertial-aligned render axes (m/s).
    pub air_velocity_render_mps: DVec3,
    pub angular_velocity_render_rps: DVec3,
    pub air_rotation_render_rps: DVec3,
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
        throttle,
        exhaust: ExhaustFamily::Methalox,
    }
}

struct Field {
    key: (i64, i64),
    source: PlumeSource,
    profile: AxialProfile,
    gpu_profile: Arc<[ProfileStation]>,
}

#[derive(Default)]
pub struct Cache {
    field: Option<Field>,
    impostor: Option<(bool, Arc<[u8]>)>,
}

impl Cache {
    pub fn clear(&mut self) {
        self.field = None;
        self.impostor = None;
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
        if !input.air_velocity_render_mps.is_finite()
            || !input.angular_velocity_render_rps.is_finite()
            || !input.air_rotation_render_rps.is_finite()
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
        if self.field.as_ref().is_none_or(|f| f.key != key) {
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
                gpu_profile,
            });
        }
        let field = self.field.as_ref()?;
        if field.profile.is_empty() {
            return None;
        }
        // Match the mesh's body-axis, camera-relative convention. Remove
        // asset->craft rotation but not the craft's presentation rotation.
        instance.camera_relative_matrix(camera_m)?;
        let craft = instance.rotation
            * thessa_flight_authority::presentation::x15_asset_to_craft_rotation()
                .as_dquat()
                .normalize()
                .conjugate();
        let nozzle = ((instance.position_m - camera_m) + craft * NOZZLE_M).as_vec3();
        let exhaust = (craft * DVec3::NEG_Y).as_vec3().normalize();
        if !nozzle.is_finite() || !exhaust.is_finite() {
            return None;
        }
        let time = if settings.plume_flicker {
            input.sim_time_s as f32
        } else {
            0.0
        };
        if settings.plume_quality == Quality::Low {
            if self
                .impostor
                .as_ref()
                .is_none_or(|(diamonds, _)| *diamonds != settings.plume_diamonds)
            {
                self.impostor = Some((
                    settings.plume_diamonds,
                    impostor_bake(settings.plume_diamonds).into(),
                ));
            }
            let flick = if settings.plume_flicker {
                flicker(input.sim_time_s, amount)
            } else {
                1.0
            };
            let len = ((2.0 + 6.5 * amount) * flick) as f32;
            let girth = ((0.35 + 0.65 * amount) * flick.clamp(0.8, 1.2)) as f32;
            let x = (craft * DVec3::X).as_vec3();
            return Some(PlumeFrame::Cone(ConeDraw {
                texture: self.impostor.as_ref()?.1.clone(),
                model: Mat4::from_cols(
                    (x * (girth * 1.1)).extend(0.0),
                    (exhaust * len).extend(0.0),
                    (x.cross(exhaust) * (girth * 1.1)).extend(0.0),
                    (nozzle + exhaust * (len * 0.5)).extend(1.0),
                )
                .to_cols_array(),
            }));
        }
        let nozzle_rotation = craft * DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let offset = craft * NOZZLE_M;
        let mut motion_env = env;
        motion_env.flow_velocity_local_mps = (nozzle_rotation.conjugate()
            * (input.air_velocity_render_mps
                + (input.air_rotation_render_rps - input.angular_velocity_render_rps)
                    .cross(offset)))
        .to_array();
        let angular = (nozzle_rotation.conjugate() * input.angular_velocity_render_rps).to_array();
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
                    let center = (instance.position_m - camera_m)
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
                if settings.plume_quality == Quality::High {
                    14.0
                } else {
                    8.0
                },
                time,
                if settings.plume_flicker { 0.4 } else { 0.0 },
                field.source.exhaust_velocity_mps as f32 * 0.1,
            ],
            diamonds: settings.plume_diamonds,
            tail_dissolve: true,
            profile: field.gpu_profile.clone(),
        }))
    }
}

fn flicker(time: f64, throttle: f64) -> f64 {
    1.0 + 0.10 * (time * 37.0).sin() * throttle + 0.06 * (time * 61.0 + 1.3).sin() * throttle
        - 0.04 * throttle
}

/// Direct GPL-to-GPL port of the retained game's 32x256 fallback bake.
fn impostor_bake(diamonds: bool) -> Vec<u8> {
    let mut px = Vec::with_capacity(32 * 256 * 4);
    for y in 0..256 {
        let v = y as f64 / 255.0;
        let width = (1.0 - v * 0.75).max(0.15);
        for x in 0..32 {
            let u = (x as f64 + 0.5) / 32.0 * 2.0 - 1.0;
            let r = (u.abs() / width).min(1.0);
            let body = (1.0 - r * r).max(0.0);
            let mut bright = body * (-v * 3.2).exp();
            if diamonds {
                let cells = (0.5 + 0.5 * (v * 5.0 * std::f64::consts::TAU).sin()).powi(2);
                bright *= 0.55 + 0.45 * cells * (-v * 2.0).exp();
            }
            let core = (bright * 1.6).min(1.0);
            for c in [
                (0.35 + core).min(1.0),
                0.85 * core + 0.15 * body,
                0.55 * core * core,
                (bright * 1.25).clamp(0.0, 1.0),
            ] {
                px.push((c.clamp(0.0, 1.0) * 255.0) as u8);
            }
        }
    }
    px
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
            air_velocity_render_mps: DVec3::ZERO,
            angular_velocity_render_rps: DVec3::ZERO,
            air_rotation_render_rps: DVec3::ZERO,
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
        assert_eq!(a.animation[1], 0.0);
        assert!(!b.diamonds);
        settings.plume_quality = Quality::Low;
        assert!(matches!(
            cache.frame(input(), env, &mesh, camera, &settings),
            Some(PlumeFrame::Cone(_))
        ));
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
        moving.air_velocity_render_mps = DVec3::new(180.0, -90.0, 30.0);
        moving.angular_velocity_render_rps = DVec3::new(0.5, -0.7, 0.2);
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
        moving.air_velocity_render_mps = DVec3::NAN;
        assert!(cache.frame(moving, env, &mesh, camera, &settings).is_none());
        assert!(cache.field.is_none());
    }

    #[test]
    fn low_bake_has_symmetric_transparent_edges_and_a_dissolving_tail() {
        for diamonds in [false, true] {
            let texture = impostor_bake(diamonds);
            assert_eq!(texture.len(), 32 * 256 * 4);
            let alpha = |x, y| texture[((y * 32 + x) * 4 + 3) as usize];
            assert!(alpha(16, 0) > alpha(16, 255) * 4);
            for y in 0..256 {
                assert_eq!(alpha(0, y), alpha(31, y));
            }
        }
    }

    #[test]
    #[ignore = "hardware: actual cached native producer orbit/motion HDR preview; new THESSA_BENCH_OUTPUT required"]
    fn plume_motion_gallery() {
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
        let settings = ResolvedGraphicsSettings {
            plume_quality: Quality::High,
            ..settings()
        };
        let env = environment(Some([1000.0, 1000.0 / (287.05 * 288.15), 288.15]));
        let mut cache = Cache::default();
        let length = build_axial_profile(&source(1.0), &env, 48)
            .unwrap()
            .length_m;
        let mut records = Vec::new();
        for index in 0..32 {
            let time = f64::from(index) / 20.0;
            let craft = DQuat::from_rotation_z(0.35 * time);
            let position = DVec3::X * (120.0 * time);
            let mesh = instance(position, craft, 1.0);
            let target = position + craft * (NOZZLE_M + DVec3::NEG_Y * (0.45 * length));
            let yaw = time * 1.4;
            let camera = target + DVec3::new(yaw.sin(), 0.0, yaw.cos()) * (length * 1.25);
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
                exposure: 1.0,
            };
            let input = Input {
                throttle: 1.0,
                engine_active: true,
                sim_time_s: time,
                air_velocity_render_mps: -DVec3::X * 120.0,
                angular_velocity_render_rps: DVec3::Z * 0.35,
                air_rotation_render_rps: DVec3::ZERO,
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
                    "axis_radius":draw.axis_radius,"animation":draw.animation,"diamonds":draw.diamonds,
                    "tail_dissolve":draw.tail_dissolve,"clip_from_view":view.clip_from_view.to_cols_array()}),
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
            "extent":[640,360],"frames":records,"settings":settings,"exposure_gain":1.0,
            "profile":&*cache.field.as_ref().unwrap().gpu_profile,
            "ambient_pressure_pa":1000.0,"ambient_density_kg_m3":env.density_kg_m3,
            "air_velocity_render_mps":[-120.0,0.0,0.0],"angular_velocity_render_rps":[0.0,0.0,0.35]
        })).unwrap()).unwrap();
        eprintln!(
            "native_motion_gallery adapter={:?} frames=32 extent=640x360 profile_uploads=1",
            backend.adapter_name
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
