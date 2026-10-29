# ADR-0012 — Incremental engine boundaries and reference scenes

Status: accepted; R1/R2 implemented, later milestones staged.

## Context

The renderer and numerical owners already exist in this Cargo workspace.
A separate engine repository or general app framework adds migration cost
without removing concrete coupling. Collision imports existing vehicle types
through the simulation facade and unnecessarily requires celestial code.

## Decision

Retain the workspace/crate names. Use the real dependency/source ownership audit
in [doc 88](../88_ENGINE_BOUNDARIES_AND_REFERENCE_SCENES.md). Enforce valid
boundaries now; stage only exact existing facade edges. Reuse canonical owners
before extracting primitives. Preserve sim-core compatibility reexports,
numerics, licensing, capability resolution and fallbacks.

Keep neutral camera/mesh/material/instance data in render-core, decoding in
render-assets, GPU implementation in render-wgpu/rcbt-wgpu, settings policy in
graphics, and game worldgen/residency/application scheduling outside them.
Consolidate demonstrated duplication only. Ship R1–R5 as separate useful,
verified and documented commits. Extend capture after R1–R4, then B01 generated
PBR geometry and bounded B02 vegetation. Assets need individual license audits,
pinned revisions or original hashes, explicit conversion semantics and external
caches for large files. Never discard unsupported material semantics silently.

## Consequences

Resolved graph checks traverse runtime/build and relevant root dev edges,
without inheriting downstream dev tooling. Existing staged roots cannot hide
new forbidden paths. Source/API review and external license audits remain.
No renderer rewrite, BikeSim, new engine repository, universal app framework,
bulk asset ingestion or initial cross-engine adapters. Report physical,
subjective, timing and temporal acceptance independently; screenshots need
comparable content, exposure, color management and settings.
