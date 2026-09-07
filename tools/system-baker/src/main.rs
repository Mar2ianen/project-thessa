use std::{env, error::Error, fs, path::PathBuf};

use thessa_sim_core::{SimTime, SystemConfig};

fn main() -> Result<(), Box<dyn Error>> {
    let options = Options::parse(env::args().skip(1))?;
    if options.help {
        print_help();
        return Ok(());
    }

    let source = fs::read_to_string(&options.input)?;
    let config: SystemConfig = toml::from_str(&source)?;
    let ephemeris = config.bake()?;
    for body in &ephemeris.bodies {
        ephemeris.body_state(body.id, SimTime(options.sample_seconds))?;
    }

    println!("epoch: {}", ephemeris.epoch_name);
    println!("bodies: {}", ephemeris.bodies.len());
    println!("gravity sources: {}", ephemeris.gravity_sources().count());
    println!("sample time: {:.3} s", options.sample_seconds);
    for body in &ephemeris.bodies {
        let state = ephemeris.body_state(body.id, SimTime(options.sample_seconds))?;
        let orbit = body
            .orbit
            .map(|orbit| {
                format!(
                    "a={:.3e} m, P={:.3e} s",
                    orbit.semi_major_axis_m,
                    orbit.period_s()
                )
            })
            .unwrap_or_else(|| "fixed".into());
        println!(
            "  {:>20}  mu={:.6e}  pos=({:.6e}, {:.6e}, {:.6e})  {orbit}",
            body.name,
            body.mu,
            state.position_inertial.x,
            state.position_inertial.y,
            state.position_inertial.z,
        );
    }

    if let Some(output) = options.output {
        let json = serde_json::to_string_pretty(&ephemeris)?;
        fs::write(&output, format!("{json}\n"))?;
        println!("wrote: {}", output.display());
    }
    Ok(())
}

struct Options {
    input: PathBuf,
    output: Option<PathBuf>,
    sample_seconds: f64,
    help: bool,
}

impl Options {
    fn parse(arguments: impl Iterator<Item = String>) -> Result<Self, Box<dyn Error>> {
        let mut input = PathBuf::from("data/system.toml");
        let mut output = None;
        let mut sample_seconds: f64 = 0.0;
        let mut help = false;
        let mut arguments = arguments.peekable();
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--input" => input = PathBuf::from(required_value(&mut arguments, "--input")?),
                "--output" => {
                    output = Some(PathBuf::from(required_value(&mut arguments, "--output")?))
                }
                "--sample-seconds" => {
                    sample_seconds = required_value(&mut arguments, "--sample-seconds")?.parse()?;
                }
                "--help" | "-h" => help = true,
                unknown => return Err(format!("unknown argument {unknown}; use --help").into()),
            }
        }
        if !sample_seconds.is_finite() {
            return Err("--sample-seconds must be finite".into());
        }
        Ok(Self {
            input,
            output,
            sample_seconds,
            help,
        })
    }
}

fn required_value(
    arguments: &mut impl Iterator<Item = String>,
    option: &str,
) -> Result<String, Box<dyn Error>> {
    arguments
        .next()
        .ok_or_else(|| format!("{option} requires a value").into())
}

fn print_help() {
    println!(
        "Usage: thessa-system-baker [--input data/system.toml] [--output data/system.baked.json] [--sample-seconds N]"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn design_system_bakes_to_stable_runtime_body_set() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("design system TOML should parse");
        let ephemeris = config.bake().expect("design system should bake");
        assert_eq!(ephemeris.bodies.len(), 24);
        assert_eq!(ephemeris.gravity_sources().count(), 22);
        let thessa = ephemeris.body_id("thessa").expect("Thessa body");
        let state = ephemeris
            .body_state(thessa, SimTime::EPOCH)
            .expect("Thessa epoch state");
        assert!(state.position_inertial.is_finite());
        assert!(state.velocity_inertial.is_finite());

        let nereid = ephemeris.body_id("nereid").expect("Nereid body");
        let borea = ephemeris.body_id("borea").expect("Borea body");
        let halo = ephemeris.body_id("halo").expect("Halo body");
        assert_eq!(
            ephemeris
                .body(halo)
                .expect("Halo descriptor")
                .orbit
                .expect("Halo orbit")
                .central_mu,
            ephemeris.body(nereid).expect("Nereid descriptor").mu
                + ephemeris.body(borea).expect("Borea descriptor").mu
        );
    }
}
