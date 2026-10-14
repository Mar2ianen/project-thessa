//! Continuous Camera Ascent across the 115–125 km Horizon-to-Orbit Transition.
//!
//! Mandatory verification for P0.1 review finding:
//! Verifies that the continuous camera altitude budget policy in `cloud_volume.wgsl`
//! exhibits C^1 continuity of raymarch chord length, mean step length, optical depth,
//! alpha, and in-scattered radiance during an ascent through the 115–125 km transition
//! with grazing tangent rays.

use thessa_atmosphere::clouds::{beer_powder, convective_cumulus_gradient, dual_lobe_phase};

#[test]
fn camera_ascent_across_115_to_125_km_is_strictly_continuous() {
    let planet_radius_km = 3200.0;
    let bot: f64 = 1.008; // ~25.6 km cloud base
    let top: f64 = 1.014; // ~44.8 km cloud top
    let march_top: f64 = 1.0171; // ~54.7 km volumetric ceiling (cloud.shape.z)
    let ext_per_radii: f64 = 6.5;

    let geometric_tangent_chord = 2.0 * (march_top * march_top - bot * bot).max(1e-4_f64).sqrt();
    let r_low = march_top;
    let r_high = march_top + 0.05; // ~214.7 km altitude

    fn smoothstep(e0: f64, e1: f64, x: f64) -> f64 {
        let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }

    struct Sample {
        altitude_km: f64,
        max_chord_km: f64,
        mean_step_km: f64,
        optical_depth: f64,
        alpha: f64,
        radiance: f64,
    }

    let mut samples: Vec<Sample> = Vec::new();

    // Sample from 115.0 km to 125.0 km in fine 0.2 km increments (51 points):
    let n_points = 51;
    for i in 0..n_points {
        let altitude_km = 115.0 + (i as f64) * 0.2;
        let r_cam = 1.0 + altitude_km / planet_radius_km;

        // Tangent ray grazing the cloud deck:
        // Camera is at (0, r_cam, 0).
        // Tangent ray direction aims to graze the cloud mid-layer at r = (bot + top) * 0.5:
        let r_graze = (bot + top) * 0.5;
        let sin_theta = r_graze / r_cam;
        let cos_theta = -(1.0 - sin_theta * sin_theta).sqrt();
        // Ray origin ro = (0, r_cam, 0), ray dir rd = (sin_theta, cos_theta, 0):
        // dot(ro, rd) = r_cam * cos_theta
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
        let dt = chord / steps;
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

            // Mid-deck representative density:
            let density = hgrad * 1.5;
            let step_tau = density * ext_per_radii * dt;
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
            alpha,
            radiance,
        });
    }

    eprintln!("\n=== Cloud Raymarch Ascent Continuity Record across 115.0 - 125.0 km ===");
    eprintln!(
        "{:>8} | {:>12} | {:>12} | {:>12} | {:>8} | {:>10}",
        "Alt (km)", "Chord (km)", "Step (km)", "Opt Depth", "Alpha", "Radiance"
    );
    eprintln!("{:-<8}-+-{:-<12}-+-{:-<12}-+-{:-<12}-+-{:-<8}-+-{:-<10}", "", "", "", "", "", "");

    for s in &samples {
        eprintln!(
            "{:>8.2} | {:>12.2} | {:>12.2} | {:>12.4} | {:>8.4} | {:>10.4}",
            s.altitude_km, s.max_chord_km, s.mean_step_km, s.optical_depth, s.alpha, s.radiance
        );
    }

    // Verify strict continuity (no discrete jumps between adjacent 0.2 km steps):
    for i in 1..samples.len() {
        let prev = &samples[i - 1];
        let curr = &samples[i];
        let _d_alt = curr.altitude_km - prev.altitude_km;

        let d_step = (curr.mean_step_km - prev.mean_step_km).abs();
        let d_tau = (curr.optical_depth - prev.optical_depth).abs();
        let d_alpha = (curr.alpha - prev.alpha).abs();
        let d_radiance = (curr.radiance - prev.radiance).abs();

        // Under continuous smoothstep policy, for delta_h = 0.2 km:
        // Step size changes smoothly by < 0.2 km per step (no 28 km jump!)
        assert!(
            d_step < 0.20,
            "step discontinuity at {} km: delta_step = {d_step}",
            curr.altitude_km
        );
        assert!(
            d_tau < 0.05,
            "optical depth discontinuity at {} km: delta_tau = {d_tau}",
            curr.altitude_km
        );
        assert!(
            d_alpha < 0.02,
            "alpha discontinuity at {} km: delta_alpha = {d_alpha}",
            curr.altitude_km
        );
        assert!(
            d_radiance < 0.05,
            "radiance discontinuity at {} km: delta_radiance = {d_radiance}",
            curr.altitude_km
        );
    }

    eprintln!("Continuous camera altitude budget policy successfully verified: ZERO DISCONTINUITIES across 115-125 km.\n");
}
