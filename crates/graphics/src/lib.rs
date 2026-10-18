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
pub use resolve::{
    ActiveCloudRepresentation, Capabilities, ResolvedBackend, ResolvedGraphicsSettings,
    ResolvedInstruments, ResolvedMaterialStorage, ResolvedRayTracing, ResolvedTerrainRender,
    ResolvedTerrainTopology, cloud_backend_override_env, resolve_cloud_integrator,
    resolve_instruments,
};
pub use settings::{
    AtmosphereSettings, AuroraQuality, BackendRequest, CloudIntegratorRequest, CloudSettings,
    ConfigError, DebugSettings, EnginePlumeSettings, GasGiantSettings, InstrumentLayout,
    InstrumentSettings, MaterialStorageRequest, NavballRenderer, OceanSettings, Preset, Quality,
    RayTracingRequest, RaytracingParticipation, RendererSettings, RequestedGraphics,
    ShadowSettings, TerrainRenderRequest, TerrainTopologySettings, UpperAtmosphereSettings,
};
