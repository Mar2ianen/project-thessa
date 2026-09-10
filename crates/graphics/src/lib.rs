//! Requested/resolved graphics settings for Project Thessa.
//!
//! Implements spec `docs/14_VISUAL_ATMOSPHERE.md` sections 15-18:
//! TOML-first settings, presets as bulk writes, requested vs resolved
//! separation, ray-tracing quality modes, and GUI metadata groundwork.
//!
//! The crate depends only on `std` + `serde`/`toml`: resolution inputs are
//! plain data so the client, perf captures, and the future GUI share one
//! model without pulling GPU APIs into settings.

#![forbid(unsafe_code)]

mod metadata;
mod resolve;
mod settings;

pub use metadata::{SettingKind, SettingMeta, describe};
pub use resolve::{Capabilities, ResolvedBackend, ResolvedGraphicsSettings, ResolvedRayTracing};
pub use settings::{
    AuroraQuality, BackendRequest, CloudSettings, ConfigError, DebugSettings, Preset, Quality,
    RayTracingRequest, RaytracingParticipation, RendererSettings, RequestedGraphics,
    UpperAtmosphereSettings,
};
