//! Direct game-owned port of the retained two-craft docking fixture. Rapier
//! solves every pose; DockingSession alone owns the capture/pressure states.
//! This isolated vacuum experiment is not a second production flight solver.
use glam::{DMat3, DQuat, DVec3};
use thessa_collision::{
    CollisionBodyId, CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench,
};
use thessa_render_core::mesh::{Material, MeshAsset, MeshInstance, Primitive, Vertex};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, DockingKinematics,
    DockingPortSpec, DockingPortState, DockingSession, RigidBodyProperties, RigidBodyState,
    SimTime,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
pub const STEP_S: f64 = 1.0 / 120.0;
const PORT_X_M: f64 = 1.4;
const LOAD_N: f64 = 350.0;

#[derive(Clone, Debug, PartialEq)]
pub struct Snapshot {
    pub time: SimTime,
    pub state: DockingPortState,
    pub bodies: [RigidBodyState; 2],
    pub port_gap_m: f64,
    pub load_time_s: f64,
    pub joints: usize,
    pub joint_force_n: Option<f64>,
    pub contacts: usize,
    pub captures: u32,
    pub finished: bool,
}

pub struct Demo {
    world: CollisionWorld,
    bodies: [CollisionBodyId; 2],
    ports: [DockingPortSpec; 2],
    session: DockingSession,
    joint: Option<thessa_collision::JointId>,
    tick: u64,
    debt_s: f64,
    load_ticks: u32,
    finished: bool,
    captures: u32,
    impact: Option<f64>,
    pub transitions: Vec<(SimTime, DockingPortState)>,
    /// Craft-local source ranges reuse the existing heterogeneous mesh draw.
    pub ranges: Vec<std::ops::Range<u32>>,
}

impl Demo {
    pub fn new() -> Result<Self> {
        let ports = [
            DockingPortSpec::d1("demo-craft-a-port", DVec3::X * PORT_X_M, DQuat::IDENTITY)?,
            DockingPortSpec::d1(
                "demo-craft-b-port",
                DVec3::NEG_X * PORT_X_M,
                DQuat::IDENTITY,
            )?,
        ];
        let mut session = DockingSession::new(ports[0].clone(), ports[1].clone(), 0.5)?;
        session.begin_soft_capture(0.04)?;
        let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;
        let properties =
            RigidBodyProperties::new(300.0, DMat3::from_diagonal(DVec3::splat(400.0)))?;
        let mut insert = |sign| -> Result<CollisionBodyId> {
            Ok(world.insert_dynamic_body(
                RigidBodyState::new(
                    DVec3::new(-1.5 * sign, 0.0, 0.0),
                    DVec3::new(0.02 * sign, 0.0, 0.0),
                    DQuat::IDENTITY,
                    DVec3::ZERO,
                )?,
                properties,
                &geometry(PORT_X_M * sign)?,
                DynamicBodyConfig {
                    full_ccd: true,
                    can_sleep: false,
                },
            )?)
        };
        let bodies = [insert(1.0)?, insert(-1.0)?];
        Ok(Self {
            world,
            bodies,
            ports,
            session,
            joint: None,
            tick: 0,
            debt_s: 0.0,
            load_ticks: 0,
            finished: false,
            captures: 0,
            impact: None,
            transitions: vec![(SimTime::EPOCH, DockingPortState::SoftCapture)],
            ranges: vec![],
        })
    }

    /// Bounded host admission; simulation time comes only from fixed solver
    /// steps. Render cadence cannot change dt or the deterministic replay.
    pub fn advance(&mut self, seconds: f64) -> Result<()> {
        if !seconds.is_finite() || !(0.0..=0.05).contains(&seconds) {
            return Err("docking elapsed must be finite and within 0..=0.05 s".into());
        }
        if self.finished {
            return Ok(());
        }
        self.debt_s += seconds;
        while self.debt_s + 1e-12 >= STEP_S && !self.finished {
            self.step()?;
            self.debt_s = (self.debt_s - STEP_S).max(0.0);
        }
        Ok(())
    }

    pub fn seek(&mut self, seconds: f64) -> Result<()> {
        if !seconds.is_finite() || !(0.0..=30.0).contains(&seconds) || self.tick != 0 {
            return Err("docking seek requires a fresh fixture and finite 0..=30 seconds".into());
        }
        for _ in 0..(seconds / STEP_S).floor() as u64 {
            if self.finished {
                break;
            }
            self.step()?;
        }
        // Replay/seek is silent: do not deliver old impacts to a new listener.
        self.impact = None;
        Ok(())
    }

    fn transition(&mut self) {
        self.transitions
            .push((SimTime(self.tick as f64 * STEP_S), self.session.state));
    }

    fn step(&mut self) -> Result<()> {
        let force = if self.session.state == DockingPortState::PressureEqualized {
            self.load_ticks += 1;
            DVec3::X * LOAD_N
        } else {
            DVec3::ZERO
        };
        self.world.step(
            STEP_S,
            [
                (
                    self.bodies[0],
                    ExternalWrench {
                        force_inertial_n: force,
                        torque_inertial_nm: DVec3::ZERO,
                    },
                ),
                (self.bodies[1], ExternalWrench::ZERO),
            ],
        )?;
        self.tick += 1;
        let states = self.states()?;
        let kinematics =
            DockingKinematics::between(states[0], &self.ports[0], states[1], &self.ports[1])?;
        if self.session.state == DockingPortState::SoftCapture
            && kinematics.relative_position_m().length() <= 0.025
            && self.session.align(kinematics).is_ok()
        {
            self.transition();
            self.session.hard_dock()?;
            self.joint = Some(self.world.attach_fixed_joint(
                self.bodies[0],
                self.bodies[1],
                self.ports[0].local_position_m,
                self.ports[0].local_orientation,
                self.ports[1].local_position_m,
                self.ports[1].local_orientation,
            )?);
            self.captures += 1;
            self.impact = Some(kinematics.relative_velocity_mps().length());
            self.transition();
            self.session.engage_outer_structure()?;
            self.transition();
        }
        if self.session.state == DockingPortState::OuterStructureEngaged {
            self.session.advance_pressure_equalization(STEP_S)?;
            if self.session.state == DockingPortState::PressureEqualized {
                self.transition();
            }
        }
        if self.session.state == DockingPortState::PressureEqualized && self.load_ticks >= 240 {
            self.world
                .remove_joint(self.joint.take().ok_or("missing docking fixture joint")?)?;
            self.session.undock()?;
            self.transition();
            self.finished = true;
        }
        Ok(())
    }

    fn states(&self) -> Result<[RigidBodyState; 2]> {
        Ok([
            self.world.body_state(self.bodies[0])?,
            self.world.body_state(self.bodies[1])?,
        ])
    }

    pub fn snapshot(&self) -> Result<Snapshot> {
        let bodies = self.states()?;
        let port = |i: usize| {
            bodies[i].position_inertial_m
                + bodies[i].orientation_body_to_inertial * self.ports[i].local_position_m
        };
        Ok(Snapshot {
            time: SimTime(self.tick as f64 * STEP_S),
            state: self.session.state,
            port_gap_m: (port(1) - port(0)).length(),
            bodies,
            load_time_s: f64::from(self.load_ticks) * STEP_S,
            joints: self.world.joint_count(),
            joint_force_n: self
                .world
                .joint_loads(STEP_S)?
                .first()
                .map(|load| load.force_n),
            contacts: self.world.touching_contact_pair_count(),
            captures: self.captures,
            finished: self.finished,
        })
    }

    pub fn take_impact(&mut self) -> Option<f64> {
        self.impact.take()
    }

    /// Place the isolated solved world near a body-space display anchor. This
    /// transform never enters CollisionWorld; the vacuum fixture has no gravity.
    pub fn instances(&self, anchor: MeshInstance) -> Result<[MeshInstance; 2]> {
        Ok(self.states()?.map(|s| MeshInstance {
            position_m: anchor.position_m + anchor.rotation * s.position_inertial_m,
            rotation: (anchor.rotation * s.orientation_body_to_inertial).normalize(),
            scale: 1.0,
        }))
    }

    /// Retain the authored D1 CAD surfaces and both colored craft/proxy parts,
    /// using the same mesh pipeline as all other native rigid render sources.
    pub fn source(&mut self) -> Result<MeshAsset> {
        let mut source = thessa_render_assets::decode_glb_rest_pose(
            include_bytes!(
                "../../../assets/models/thessa-d1-docking-port/thessa_d1_docking_port.glb"
            ),
            Default::default(),
        )?;
        let cad = std::mem::take(&mut source.primitives);
        self.ranges.clear();
        for (i, color) in [[0.10, 0.22, 0.42, 1.0], [0.42, 0.16, 0.08, 1.0]]
            .into_iter()
            .enumerate()
        {
            let sign = if i == 0 { 1.0 } else { -1.0 };
            let material = source.materials.len();
            source.materials.push(Material {
                base_color: color,
                metallic: 0.75,
                roughness: 0.28,
                ..Default::default()
            });
            source.materials.push(Material {
                base_color: [0.92, 0.58, 0.12, 1.0],
                metallic: 0.55,
                roughness: 0.22,
                ..Default::default()
            });
            let begin = source.primitives.len();
            source
                .primitives
                .push(cuboid(DVec3::ZERO, DVec3::new(0.55, 0.45, 0.45), material));
            source.primitives.push(cuboid(
                DVec3::X * (PORT_X_M * sign),
                DVec3::new(0.09, 0.625, 0.625),
                material + 1,
            ));
            // The exporter converts STEP +Z to glTF +Y. Map that actual GLB
            // mating normal to +/-X, not the legacy adapter's assumed +Z.
            let rotation = DQuat::from_rotation_z(-sign * std::f64::consts::FRAC_PI_2);
            for primitive in &cad {
                let mut p = primitive.clone();
                for v in &mut p.vertices {
                    v.position = (DVec3::X * (PORT_X_M * sign)
                        + rotation * glam::Vec3::from(v.position).as_dvec3())
                    .as_vec3()
                    .to_array();
                    v.normal = (rotation * glam::Vec3::from(v.normal).as_dvec3())
                        .as_vec3()
                        .to_array();
                }
                source.primitives.push(p);
            }
            self.ranges
                .extend((begin..source.primitives.len()).map(|_| i as u32..i as u32 + 1));
        }
        source.validate()?;
        Ok(source)
    }
}

pub fn geometry(port_x: f64) -> Result<CollisionGeometry> {
    let mut parts = vec![CollisionPart::new(
        DVec3::ZERO,
        DQuat::IDENTITY,
        CollisionShape::Cuboid {
            half_extents_m: DVec3::new(0.55, 0.45, 0.45),
        },
        CollisionMaterial::default(),
    )?];
    for i in 0..8 {
        let angle = f64::from(i) * std::f64::consts::TAU / 8.0;
        parts.push(CollisionPart::new(
            DVec3::new(
                port_x - port_x.signum() * 0.01,
                0.55 * angle.cos(),
                0.55 * angle.sin(),
            ),
            DQuat::from_rotation_x(angle),
            CollisionShape::Cuboid {
                half_extents_m: DVec3::new(0.01, 0.12, 0.12),
            },
            CollisionMaterial::default(),
        )?);
    }
    Ok(CollisionGeometry::new(parts)?)
}

#[cfg(feature = "hud")]
pub fn solved_shell(snapshot: &Snapshot) -> Result<CollisionGeometry> {
    let center =
        (snapshot.bodies[0].position_inertial_m + snapshot.bodies[1].position_inertial_m) * 0.5;
    let mut parts = Vec::new();
    for (i, state) in snapshot.bodies.iter().enumerate() {
        for mut part in geometry(if i == 0 { PORT_X_M } else { -PORT_X_M })?.parts {
            part.local_position_m = state.position_inertial_m - center
                + state.orientation_body_to_inertial * part.local_position_m;
            part.local_orientation =
                (state.orientation_body_to_inertial * part.local_orientation).normalize();
            parts.push(part);
        }
    }
    Ok(CollisionGeometry::new(parts)?)
}

fn cuboid(center: DVec3, half: DVec3, material: usize) -> Primitive {
    let mut vertices = Vec::with_capacity(24);
    let mut indices = Vec::with_capacity(36);
    for n in [
        DVec3::X,
        -DVec3::X,
        DVec3::Y,
        -DVec3::Y,
        DVec3::Z,
        -DVec3::Z,
    ] {
        let a = n.any_orthonormal_vector();
        let b = n.cross(a);
        let begin = vertices.len() as u32;
        for (u, v) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            vertices.push(Vertex {
                position: (center + (n + a * u + b * v) * half).as_vec3().to_array(),
                normal: n.as_vec3().to_array(),
                uv: [((u + 1.0) * 0.5) as f32, ((v + 1.0) * 0.5) as f32],
            });
        }
        indices.extend([0, 1, 2, 0, 2, 3].map(|v| v + begin));
    }
    Primitive {
        vertices,
        indices,
        material,
    }
}

impl Snapshot {
    pub fn telemetry(&self) -> String {
        format!(
            "native_docking t={:.6} state={:?} gap_m={:.6} load_s={:.6} joints={} contacts={} captures={} finished={} a_vx={:.6} b_vx={:.6}",
            self.time.0,
            self.state,
            self.port_gap_m,
            self.load_time_s,
            self.joints,
            self.contacts,
            self.captures,
            self.finished,
            self.bodies[0].velocity_inertial_mps.x,
            self.bodies[1].velocity_inertial_mps.x
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_replay_captures_transfers_real_load_and_removes_the_joint_once() {
        let mut demo = Demo::new().unwrap();
        assert!((demo.snapshot().unwrap().port_gap_m - 0.2).abs() < 1e-12);
        let mut impacts = Vec::new();
        for _ in 0..2000 {
            demo.advance(STEP_S).unwrap();
            if let Some(speed) = demo.take_impact() {
                impacts.push(speed);
            }
            if demo.finished {
                break;
            }
        }
        let s = demo.snapshot().unwrap();
        assert!(s.finished, "{}", s.telemetry());
        assert_eq!(s.state, DockingPortState::Free);
        assert_eq!((s.joints, s.captures, impacts.len()), (0, 1, 1));
        assert!(impacts[0] > 0.0 && impacts[0] < 0.05);
        assert_eq!(s.load_time_s, 2.0);
        assert!(
            s.bodies[1].velocity_inertial_mps.x > 0.9,
            "partner must receive actual joint load"
        );
        assert!(
            (s.bodies[0].velocity_inertial_mps.x - s.bodies[1].velocity_inertial_mps.x).abs()
                < 0.001
        );
        assert!(s.port_gap_m < 0.025);
        let momentum = s
            .bodies
            .iter()
            .map(|s| s.velocity_inertial_mps.x * 300.0)
            .sum::<f64>();
        assert!(
            (momentum - LOAD_N * 2.0).abs() < 1e-5,
            "joint must conserve net momentum: {momentum}"
        );
        eprintln!("{} transitions={:?}", s.telemetry(), demo.transitions);
        assert_eq!(
            demo.transitions.iter().map(|(_, s)| *s).collect::<Vec<_>>(),
            [
                DockingPortState::SoftCapture,
                DockingPortState::Aligned,
                DockingPortState::HardDock,
                DockingPortState::OuterStructureEngaged,
                DockingPortState::PressureEqualized,
                DockingPortState::Free
            ]
        );
        demo.advance(0.05).unwrap();
        assert_eq!(s, demo.snapshot().unwrap());
    }

    #[test]
    fn admission_chunking_seek_and_render_extraction_do_not_change_solved_state() {
        let mut a = Demo::new().unwrap();
        let mut b = Demo::new().unwrap();
        for _ in 0..300 {
            a.advance(0.01).unwrap();
        }
        for _ in 0..60 {
            b.advance(0.05).unwrap();
        }
        assert_eq!(a.snapshot().unwrap(), b.snapshot().unwrap());
        let before = a.snapshot().unwrap();
        for dt in [f64::NAN, -1.0, f64::INFINITY, 0.06] {
            assert!(a.advance(dt).is_err());
        }
        assert_eq!(a.snapshot().unwrap(), before);
        let anchor = MeshInstance {
            position_m: DVec3::new(1e12, -2e12, 3e12),
            rotation: DQuat::from_rotation_y(0.4),
            scale: 1.0,
        };
        let instances = a.instances(anchor).unwrap();
        assert!(
            instances
                .iter()
                .all(|i| i.camera_relative_matrix(anchor.position_m).is_some())
        );
        assert_eq!(a.snapshot().unwrap(), before);
        let mut seek = Demo::new().unwrap();
        seek.seek(3.0).unwrap();
        assert_eq!(seek.snapshot().unwrap(), before);
        assert!(seek.take_impact().is_none());
        let mut loaded = Demo::new().unwrap();
        loaded.seek(5.5).unwrap();
        let force = loaded.snapshot().unwrap().joint_force_n.unwrap();
        assert!(
            (force - LOAD_N * 0.5).abs() < 0.001,
            "the solved joint must transfer half the external load to B: {force}"
        );
    }

    #[test]
    fn retained_cad_and_proxy_sources_have_valid_materials_and_two_body_ranges() {
        let bytes = include_bytes!(
            "../../../assets/models/thessa-d1-docking-port/thessa_d1_docking_port.glb"
        );
        assert!(
            thessa_render_assets::decode_glb(bytes, Default::default()).is_err(),
            "ordinary loading cannot silently discard animations"
        );
        assert!(
            thessa_render_assets::decode_glb_rest_pose(
                bytes,
                thessa_render_assets::Limits {
                    vertices: 1,
                    ..Default::default()
                }
            )
            .is_err()
        );
        let mut demo = Demo::new().unwrap();
        let source = demo.source().unwrap();
        source.validate().unwrap();
        assert_eq!(source.primitives.len(), demo.ranges.len());
        assert!(
            source.primitives.len() > 4,
            "CAD surfaces cannot be silently dropped"
        );
        assert!(demo.ranges.contains(&(0..1)) && demo.ranges.contains(&(1..2)));
        for body in [0, 1] {
            let mut min = glam::Vec3::splat(f32::INFINITY);
            let mut max = glam::Vec3::splat(f32::NEG_INFINITY);
            // Two diagnostic cuboids followed by the actual authored surfaces.
            for (index, primitive) in source.primitives.iter().enumerate() {
                if demo.ranges[index].start as usize != body {
                    continue;
                }
                let first = demo
                    .ranges
                    .iter()
                    .position(|r| r.start as usize == body)
                    .unwrap();
                if index < first + 2 {
                    continue;
                }
                for vertex in &primitive.vertices {
                    let p = glam::Vec3::from(vertex.position);
                    min = min.min(p);
                    max = max.max(p);
                }
            }
            let extent = max - min;
            assert!(
                extent.x < 0.4 && extent.y > 1.2 && extent.z > 1.2,
                "actual CAD thin axis must face the mating corridor: {extent}"
            );
        }
    }

    #[test]
    #[ignore = "diagnostic: isolated two-body 120 Hz docking replay CPU cost, no renderer or audio"]
    fn docking_replay_cost() {
        let mut samples = Vec::new();
        for _ in 0..9 {
            let mut demo = Demo::new().unwrap();
            let started = std::time::Instant::now();
            for _ in 0..1000 {
                demo.advance(STEP_S).unwrap();
            }
            samples.push(started.elapsed().as_secs_f64() * 1000.0);
            assert!(demo.finished);
            std::hint::black_box(demo.snapshot().unwrap());
        }
        samples.sort_by(f64::total_cmp);
        eprintln!(
            "native_docking_cpu bodies=2 colliders=18 steps_requested=1000 trials=9 median_ms={:.6} max_ms={:.6}; completed fixtures freeze, not an isolated per-tick solver benchmark",
            samples[4], samples[8]
        );
    }
}
