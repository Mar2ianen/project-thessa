//! Thessa rocky-planet worldgen dev library (MIT, dev-only).
//!
//! Authored macrostructure + deterministic geology/biome logic + explicit
//! landmark generators + procedural physical-scale detail.
//! All scales in metres; gores are normalized fractions.

pub mod bake;
pub mod biomes;
pub mod client_export;
pub mod climate;
pub mod erosion;
pub mod features;
pub mod field;
pub mod geothermal;
pub mod gores;
pub mod height;
pub mod hydro;
pub mod landmarks;
pub mod manifest;
pub mod minerals;
pub mod png_min;
pub mod preview;
pub mod rng;
pub mod scatter;
pub mod spec_recipe;
pub mod sphere;
pub mod system_body;
pub mod tectonics;
pub mod terrain;
pub mod terrain_fields;
