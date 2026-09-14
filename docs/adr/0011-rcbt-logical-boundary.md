# ADR 0011 — RCBT logical boundary

Status: accepted and implemented in part, 2026-09-14.

## Decision

Keep adaptive binary-tree topology independent from terrain semantics and
graphics APIs.

- `thessa-rcbt-core` owns logical nodes, split/merge validation, frame
  planning, compact height pages, serialization, and backend contracts.
- `thessa-bevy-rcbt` owns only Bevy resources and the `PostUpdate` commit
  boundary. It accepts domain candidates and returns bounded updates plus a
  leaf snapshot.
- Terrain code owns cube-face addressing, `PlanetField` sampling, material
  policy, and the CPU fallback.
- `thessa-rcbt-wgpu` owns wgpu handles and WGSL dispatch.
- `thessa-rcbt-ref` remains a test oracle. `thessa-rcbt-ffi` is an optional
  topology backend: it may be selected by a client/tool/runtime build, but it
  is never required by the server or by `rcbt-core`.

## Rationale

CBT is a visual representation, not authoritative physics. Keeping this
boundary means a server can answer surface queries without a GPU, a client can
retain a safe CPU path, and another renderer can consume the same topology
contracts without importing Bevy or wgpu into simulation code.

## Consequences

The first integrated client milestone intentionally runs both the established
CPU terrain renderer and CBT scheduling. GPU draw-list extraction is a follow-up
implementation, gated by parity, error, and benchmark evidence rather than by
the existence of a compute shader.
