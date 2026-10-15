//! Isolated `unsafe` consent for wgpu experimental device features.
//!
//! wgpu 30 gates every `EXPERIMENTAL_*` feature behind a consent token
//! (`ExperimentalFeatures::enabled`, itself `unsafe`) acknowledging that
//! work-in-progress implementations may contain UB bugs and asking users to
//! report them upstream. Every other Thessa engine crate is
//! `forbid(unsafe_code)`, so the single consent call lives here — one
//! audited site — instead of being sprinkled across backends.
//!
//! Callers must still check the adapter first ([`supported_experimental`])
//! and request only exposed bits; consent is not a substitute for
//! capability probing.

/// Adapter-exposed subset of the experimental features Thessa backends may
/// negotiate (mesh shaders ×3, ray query). Pure query, no consent needed.
pub fn supported_experimental(adapter: &wgpu::Adapter) -> wgpu::Features {
    adapter.features()
        & (wgpu::Features::EXPERIMENTAL_MESH_SHADER
            | wgpu::Features::EXPERIMENTAL_MESH_SHADER_MULTIVIEW
            | wgpu::Features::EXPERIMENTAL_MESH_SHADER_POINTS
            | wgpu::Features::EXPERIMENTAL_RAY_QUERY)
}

/// The wgpu experimental-features consent token.
///
/// # Safety contract (mirrors wgpu's own)
///
/// Experimental implementations may contain UB bugs hit through otherwise
/// safe calls. Thessa's side of the contract: request only
/// adapter-exposed bits (see [`supported_experimental`]), drive them
/// through wgpu's safe API surface only, and treat any GPU-side anomaly
/// as a backend bug to reproduce and report upstream instead of working
/// around it in engine code.
pub fn experimental_consent() -> wgpu::ExperimentalFeatures {
    // SAFETY: justified above; single call site by design.
    unsafe { wgpu::ExperimentalFeatures::enabled() }
}
