//! Vehicle mounts: installed engines with their stations, thrust axes, and
//! gimbal authority pairs for the flight allocator.

use serde::{Deserialize, Serialize};

use super::{CompiledEngine, PropulsionError};

/// One gimbal control effector for the flight allocator. A normalized
/// command in [-1, 1] spans the gimbal range about `gimbal_axis`
/// (right-hand rule); force/moment scale linearly with the command.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GimbalEffector {
    /// Unit gimbal rotation axis in body axes.
    pub gimbal_axis: [f64; 3],
    /// Force per unit command (N, body axes).
    pub force_per_command_n: [f64; 3],
    /// Moment about the body origin per unit command (N·m, body axes).
    pub moment_per_command_nm: [f64; 3],
}

/// One engine installed on a vehicle: compiled data plus its mount station
/// and thrust axis. Mass aggregates into the vehicle budget at bake time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EngineMount {
    pub name: String,
    pub engine: CompiledEngine,
    /// Mount (throat) station in vehicle body metres.
    pub position_body_m: [f64; 3],
    /// Unit thrust direction in vehicle body axes (usually +X).
    pub thrust_axis_body: [f64; 3],
}

impl EngineMount {
    /// Validate mount data (NaN fails closed; axis must be unit).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "mount position must be finite".into(),
            ));
        }
        let axis = self.thrust_axis_body;
        if axis.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "thrust axis must be finite".into(),
            ));
        }
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if (norm - 1.0).abs() > 1e-9 {
            return Err(PropulsionError::InvalidSpec(
                "thrust axis must be unit length".into(),
            ));
        }
        Ok(())
    }

    /// Thrust vector in body axes at this mount's command.
    pub fn thrust_vector_body_n(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<[f64; 3], PropulsionError> {
        self.validate()?;
        let point = self
            .engine
            .operating_point(throttle, ambient_pa, burn_time_s)?;
        Ok([
            self.thrust_axis_body[0] * point.thrust_n,
            self.thrust_axis_body[1] * point.thrust_n,
            self.thrust_axis_body[2] * point.thrust_n,
        ])
    }

    /// Gimbal range of the installed engine (rad, 0 = fixed).
    pub fn gimbal_range_rad(&self) -> f64 {
        match &self.engine {
            CompiledEngine::Liquid(engine) => engine.gimbal_range_rad,
            CompiledEngine::Solid(engine) => engine.gimbal_range_rad,
        }
    }

    /// Gimbal control authority for the flight allocator: moment per gimbal
    /// radian about the two body axes perpendicular to the thrust axis, at
    /// an operating point. Moments are about the body origin (the allocator
    /// translates to the CG); the two effectors share the gimbal range, so
    /// the allocator must coordinate them. Fixed mounts return zero
    /// moments (axes still well-defined).
    pub fn gimbal_authority(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<[GimbalEffector; 2], PropulsionError> {
        use glam::DVec3;
        self.validate()?;
        let thrust_n =
            DVec3::from_array(self.thrust_vector_body_n(throttle, ambient_pa, burn_time_s)?);
        let axis = DVec3::from_array(self.thrust_axis_body);
        let reference = if axis.x.abs() < 0.9 {
            DVec3::X
        } else {
            DVec3::Y
        };
        let gimbal_a = axis.cross(reference).normalize();
        let gimbal_b = axis.cross(gimbal_a).normalize();
        let position = DVec3::from_array(self.position_body_m);
        let mut effectors = Vec::with_capacity(2);
        for gimbal_axis in [gimbal_a, gimbal_b] {
            // dF/ddelta = gimbal_axis x F; moment = r x dF/ddelta.
            let force_per_rad = gimbal_axis.cross(thrust_n);
            let moment_per_rad = position.cross(force_per_rad);
            effectors.push(GimbalEffector {
                gimbal_axis: gimbal_axis.to_array(),
                force_per_command_n: (force_per_rad * self.gimbal_range_rad()).to_array(),
                moment_per_command_nm: (moment_per_rad * self.gimbal_range_rad()).to_array(),
            });
        }
        Ok([effectors[0], effectors[1]])
    }
}

#[cfg(test)]
mod tests {
    use super::super::liquid::merlin_like;
    use super::*;

    #[test]
    fn gimbal_authority_matches_lever_arm() {
        // +X thrust at x = -3 m with 1 MN: gimbaling must produce ~3 MN·m
        // per radian about the transverse axes and ~0 about the thrust
        // axis; per-command values scale by the installed range.
        let mount = EngineMount {
            name: "lever probe".into(),
            engine: CompiledEngine::Liquid(
                super::super::liquid::LiquidEngineSpec {
                    name: "lever".into(),
                    gimbal_range_rad: 0.1,
                    ..merlin_like()
                }
                .compile()
                .expect("compile"),
            ),
            position_body_m: [-3.0, 0.0, 0.0],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        let authority = mount.gimbal_authority(1.0, 0.0, 0.0).expect("authority");
        let full = mount
            .engine
            .operating_point(1.0, 0.0, 0.0)
            .expect("point")
            .thrust_n;
        for effector in authority {
            let moment = glam::DVec3::from_array(effector.moment_per_command_nm);
            // Per radian (divide the range back out), transverse only.
            let per_rad = moment / 0.1;
            assert!(per_rad.x.abs() < full * 0.01, "no roll authority expected");
            assert!(
                (per_rad.length() - full * 3.0).abs() / (full * 3.0) < 1e-9,
                "moment must equal thrust times lever arm"
            );
        }
    }
}
