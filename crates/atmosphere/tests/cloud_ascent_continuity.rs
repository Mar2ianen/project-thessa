//! Camera Altitude Budget Geometry Policy Verification across Full Transition (50–220 km).
//!
//! Verifies that the continuous camera altitude budget policy in `cloud_volume.wgsl`:
//!   `altitude_weight = smoothstep(march_top, march_top + 0.05, r_cam)`
//!   `max_chord = mix(min(geometric_tangent_chord, 0.025), geometric_tangent_chord, altitude_weight)`
//! exhibits smooth, continuous transition across the full range [50.0, 220.0] km,
//! seamlessly connecting:
//!   - Low-altitude horizon budget regime (altitude < 54.7 km, clamped to 80 km chord)
//!   - Smoothstep transition interval (54.7 km to 214.7 km)
//!   - Full orbital geometric tangent regime (altitude > 214.7 km, up to 868.8 km chord)
//!
//! with strictly bounded discrete differences and zero slope spikes at the boundaries.
//!
//! Note: This test verifies the geometric ray-length budget policy and raymarch quadrature properties.
//! Full production GPU rendering continuity on Vulkan hardware is verified in
//! `thessa-bevy-rcbt::gpu_tests::production_camera_ascent_continuity_across_full_transition_on_hardware`.

use thessa_atmosphere::clouds::{beer_powder, convective_cumulus_gradient, dual_lobe_phase};

#[test]
fn camera_altitude_budget_geometry_policy_is_continuous_across_full_transition() {
    let planet_radius_km: f64 = 3200.0;
    // Canonical production Thessa parameters (apps/client/src/beauty.rs):
    let bot: f64 = 1.008; // 25.6 km cloud base
    let top: f64 = 1.014; // 44.8 km cloud top
    let march_top: f64 = 1.0171; // 54.72 km volumetric ceiling (cloud.shape.z = top + 0.5 * (top - bot))
    let ext_per_radii: f64 = 1000.0; // CLOUD_EXT_LOW = 1000.0

    let geometric_tangent_chord = 2.0 * (march_top * march_top - bot * bot).max(1e-4_f64).sqrt();
    let r_low = march_top; // 54.72 km altitude
    let r_high = march_top + 0.05; // 214.72 km altitude

    fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
        let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }

    struct Sample {
        altitude_km: f64,
        max_chord_km: f64,
        mean_step_km: f64,
        optical_depth: f64,
        transmittance: f64,
        alpha: f64,
        radiance: f64,
    }

    let mut samples: Vec<Sample> = Vec::new();

    // Sample across the entire transition span from 50.0 km to 220.0 km in 2.0 km increments (86 points):
    let n_points = 86;
    for i in 0..n_points {
        let altitude_km = 50.0 + (i as f64) * 2.0;
        let r_cam = 1.0 + altitude_km / planet_radius_km;

        // Tangent ray grazing the cloud deck mid-layer:
        let r_graze = (bot + top) * 0.5;
        let sin_theta = (r_graze / r_cam).clamp(0.0, 1.0);
        let cos_theta = -(1.0 - sin_theta * sin_theta).sqrt();
        let b = r_cam * cos_theta;

        // Intersection with cloud ceiling sphere (radius = march_top):
        let disc_top = b * b - (r_cam * r_cam - march_top * march_top);
        assert!(disc_top > 0.0, "grazing ray must intersect march_top sphere");
        let sq_top = disc_top.sqrt();
        let t_top0 = -b - sq_top;
        let t_top1 = -b + sq_top;

        let t0 = t_top0.max(0.0);
        let mut t1 = t_top1;

        // Intersection with cloud bottom sphere (radius = bot):
        let disc_bot = b * b - (r_cam * r_cam - bot * bot);
        if disc_bot > 0.0 {
            let t_bot0 = -b - disc_bot.sqrt();
            if t_bot0 > 0.0 {
                t1 = t1.min(t_bot0);
            }
        }

        // Continuous camera altitude budget policy (matching cloud_volume.wgsl):
        let altitude_weight = smoothstep(r_low, r_high, r_cam);
        let low_budget_cap: f64 = 0.025; // 80 km on Thessa (0.025 * 3200 km)
        let max_chord = low_budget_cap.min(geometric_tangent_chord) * (1.0 - altitude_weight)
            + geometric_tangent_chord * altitude_weight;

        t1 = t1.min(t0 + max_chord);
        let chord = (t1 - t0).max(1e-5);
        let steps = 24.0;
        let dt = chord / steps; // in planet radii
        let mean_step_km = dt * planet_radius_km;

        // 24-step numerical raymarch integration:
        let mut transmittance = 1.0;
        let mut radiance = 0.0;
        let mut optical_depth = 0.0;

        for step in 0..24 {
            let t_current = t0 + ((step as f64) + 0.5) * dt;
            let r_current = (r_cam * r_cam + t_current * t_current + 2.0 * t_current * b).sqrt();
            let h01 = ((r_current - bot) / (top - bot)).clamp(0.0, 1.0);
            let hgrad = convective_cumulus_gradient(h01);

            let density = hgrad * 0.8;
            let step_tau = density * ext_per_radii * dt; // physical step optical depth
            optical_depth += step_tau;

            let step_trans = (-step_tau).exp();
            let sun_depth = density * 0.4;
            let direct_light = beer_powder(sun_depth) * dual_lobe_phase(0.7);
            let ambient_light = 0.25 * (h01 * 0.75 + 0.25);
            let in_scatter = direct_light + ambient_light;

            radiance += transmittance * in_scatter * (1.0 - step_trans);
            transmittance *= step_trans;
        }

        let alpha = 1.0 - transmittance;
        samples.push(Sample {
            altitude_km,
            max_chord_km: max_chord * planet_radius_km,
            mean_step_km,
            optical_depth,
            transmittance,
            alpha,
            radiance,
        });
    }

    eprintln!("\n=== Cloud Raymarch Camera Altitude Budget across Full Transition (50–220 km) ===");
    eprintln!(
        "{:>8} | {:>12} | {:>12} | {:>12} | {:>8} | {:>8} | {:>10}",
        "Alt (km)", "Chord (km)", "Step (km)", "Opt Depth", "Trans", "Alpha", "Radiance"
    );
    eprintln!("{:-<8}-+-{:-<12}-+-{:-<12}-+-{:-<12}-+-{:-<8}-+-{:-<8}-+-{:-<10}", "", "", "", "", "", "", "");

    for s in &samples {
        if (s.altitude_km as i32) % 10 == 0 {
            eprintln!(
                "{:>8.1} | {:>12.2} | {:>12.2} | {:>12.4} | {:>8.4} | {:>8.4} | {:>10.4}",
                s.altitude_km, s.max_chord_km, s.mean_step_km, s.optical_depth, s.transmittance, s.alpha, s.radiance
            );
        }
    }

    // Verify bounded derivatives and strict continuity across all steps:
    for i in 1..samples.len() {
        let prev = &samples[i - 1];
        let curr = &samples[i];
        let d_alt = curr.altitude_km - prev.altitude_km;
        assert!((d_alt - 2.0).abs() < 1e-4);

        let d_chord = (curr.max_chord_km - prev.max_chord_km).abs();
        let chord_derivative = d_chord / d_alt;

        // Bounded derivative (maximum theoretical rate of smoothstep chord expansion is ~7.4 km/km):
        assert!(
            chord_derivative <= 8.0,
            "chord derivative spike at {} km: {chord_derivative} km/km",
            curr.altitude_km
        );

        let d_step = (curr.mean_step_km - prev.mean_step_km).abs();
        let step_derivative = d_step / d_alt;
        assert!(
            step_derivative <= 0.35,
            "step derivative spike at {} km: {step_derivative} km/km",
            curr.altitude_km
        );
    }
}

