# Decision index

This is the short ADR index. When a decision changes, do not silently rewrite
the old decision: add a new ADR and mark the old one `superseded`.

| ID | Decision | Status |
| --- | --- | --- |
| ADR-0001 | Celestial bodies use baked deterministic ephemerides; ships use full multi-body test-particle gravity | accepted |
| ADR-0002 | Bevy is the client/app shell; `sim-core` has no Bevy dependency | accepted |
| ADR-0004 | Parametric vehicle designs compile to multiple physical representations | proposed |
| ADR-0005 | x86_64 release baseline is AVX2 with an optional AVX-512 target | proposed |
| ADR-0006 | Physics ray queries have a CPU/BVH canonical path; hardware RT is not required | proposed |
| ADR-0007 | MIT engine/reusable crates and GPL game/application crates are separated | accepted |
| ADR-0008 | Renderer boundary is cross-platform Bevy/wgpu, not a DirectX API | accepted |
| ADR-0009 | Autopilot is a composable typed graph with event-driven waits | accepted |
| ADR-0010 | Realtime aero uses a bounded reduced-order model with offline reference solvers | accepted |
| ADR-0011 | RCBT topology is backend-neutral; Bevy, terrain, and wgpu stay adapters | accepted |

See [`docs/adr/`](docs/adr/).
