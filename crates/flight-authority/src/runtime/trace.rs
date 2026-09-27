//! Optional low-overhead CSV trace writer for reproducing flight states.

use std::{
    fs::{File, create_dir_all},
    io::{BufWriter, Write},
    path::Path,
};

use super::*;

/// Low-overhead CSV recorder for reproducing bad live-flight states outside
/// the renderer. It is enabled only by the executable; unit tests remain
pub struct FlightTraceWriter {
    writer: BufWriter<File>,
    samples_since_flush: u32,
}

impl FlightTraceWriter {
    pub(super) fn create(path: &Path) -> Option<Self> {
        if let Some(parent) = path.parent() {
            create_dir_all(parent).ok()?;
        }
        let file = File::create(path).ok()?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "t_s,altitude_m,relative_speed_mps,vertical_speed_mps,mach,aoa_deg,q_pa,\
             pos_x_m,pos_y_m,pos_z_m,vel_x_mps,vel_y_mps,vel_z_mps,\
             quat_x,quat_y,quat_z,quat_w,omega_x_rps,omega_y_rps,omega_z_rps,\
             pitch_cmd,yaw_cmd,roll_cmd,throttle,engine,sas,rcs,\
             force_x_n,force_y_n,force_z_n,moment_x_nm,moment_y_nm,moment_z_nm,\
             accel_x_mps2,accel_y_mps2,accel_z_mps2,control_mode,surface_pitch,surface_yaw,surface_roll,actuator_saturated,target_quat_x,target_quat_y,target_quat_z,target_quat_w"
        )
        .ok()?;
        Some(Self {
            writer,
            samples_since_flush: 0,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn record(
        &mut self,
        time_s: f64,
        altitude_m: f64,
        relative_velocity_inertial_mps: DVec3,
        radial_up: DVec3,
        air_velocity_body_mps: DVec3,
        state: RigidBodyState,
        controls: DVec3,
        throttle: f64,
        engine_active: bool,
        sas_enabled: bool,
        rcs_enabled: bool,
        forces: &FlightForces,
        mode: ControlMode,
        surfaces: DVec3,
        saturated: bool,
        target: DQuat,
    ) {
        let aoa_deg = conventional_angle_of_attack_deg(air_velocity_body_mps);
        let vertical_speed_mps = relative_velocity_inertial_mps.dot(radial_up);
        let q = forces.aero.dynamic_pressure_pa;
        let p = state.position_inertial_m;
        let v = state.velocity_inertial_mps;
        let qrot = state.orientation_body_to_inertial;
        let omega = state.angular_velocity_body_rps;
        let force = forces.total_force_body_n;
        let moment = forces.total_moment_body_nm;
        let accel = forces.acceleration_inertial_mps2;
        let _ = writeln!(
            self.writer,
            "{time_s:.6},{altitude_m:.6},{:.6},{vertical_speed_mps:.6},{:.6},{aoa_deg:.6},{q:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.9},\
             {:.9},{:.9},{:.9},{:.6},{:.6},{:.6},{throttle:.6},{},{},{},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{},{:.9},{:.9},{:.9},{:.9}",
            relative_velocity_inertial_mps.length(),
            forces.aero.mach,
            p.x,
            p.y,
            p.z,
            v.x,
            v.y,
            v.z,
            qrot.x,
            qrot.y,
            qrot.z,
            qrot.w,
            omega.x,
            omega.y,
            omega.z,
            controls.x,
            controls.y,
            controls.z,
            engine_active as u8,
            sas_enabled as u8,
            rcs_enabled as u8,
            force.x,
            force.y,
            force.z,
            moment.x,
            moment.y,
            moment.z,
            accel.x,
            accel.y,
            accel.z,
            mode.label(),
            surfaces.x,
            surfaces.y,
            surfaces.z,
            saturated as u8,
            target.x,
            target.y,
            target.z,
            target.w,
        );
        self.samples_since_flush += 1;
        if self.samples_since_flush >= 30 {
            let _ = self.writer.flush();
            self.samples_since_flush = 0;
        }
    }
}

pub fn conventional_angle_of_attack_deg(air_velocity_body: DVec3) -> f64 {
    // The reusable aero contract stores +Z as up and defines its coefficient
    // alpha from the velocity vector. Pilot HUDs conventionally report
    // positive AoA when the nose is above the velocity vector, hence -w/u.
    (-air_velocity_body.z)
        .atan2(air_velocity_body.x)
        .to_degrees()
}
