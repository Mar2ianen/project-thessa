# ADR-0011 — RCBT logical and backend boundary

Status: accepted.

## Decision

The first RCBT implementation is split into three packages:

- `thessa-rcbt-core`: pure Rust node addressing, observable topology,
  deterministic update planning, serialization, and backend-neutral contracts;
- `thessa-rcbt-ref`: an independent, portable observable-semantics oracle for
  differential tests. The pinned upstream `libcbt` revision remains a reference
  and benchmark target, not a runtime dependency;
- `thessa-rcbt-wgpu`: the portable compute adapter, owning all wgpu handles and
  WGSL kernels.

`PlanetField`, cube-sphere addressing, baked height pages, and Bevy extraction
remain adapters outside `thessa-rcbt-core`. The current CPU tile renderer is a
fallback until topology, seams, height error, and frame metrics reach parity.

The initial core uses a sorted leaf set deliberately. Packed bitplanes and
parallel mutation are optimization candidates, not observable API contracts;
they require workload benchmarks before adoption.

## Consequences

- Dedicated/headless server builds do not pull in Bevy, wgpu, or a GPU.
- A native backend can implement the same `CbtBackend` contract later.
- The wgpu prototype can validate dispatch and shader portability before it is
  allowed to replace the existing terrain path.
- Logical topology snapshots are stable across internal representation changes.
