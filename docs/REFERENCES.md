# References / fact-check notes

Accessed: 2026-09-07 unless stated otherwise.

Эти источники подтверждают **внешние технические факты**. Числа fictional system являются design values и вычислены отдельно.

## Bevy / Rust ecosystem

1. **Bevy 0.19 release** — current 0.19 line, renderer/task/WASM changes, GPU-driven renderer, Solari status.  
   https://bevy.org/news/bevy-0-19/

2. **Bevy 0.18 -> 0.19 migration guide** — 0.19 has breaking changes; pinning version is intentional.  
   https://bevy.org/learn/migration-guides/0-18-to-0-19/

3. **Bevy WebGPU examples** — official examples running in browser via WASM + WebGPU.  
   https://bevy.org/examples-webgpu/

4. **Bevy ComputeTaskPool** — CPU-intensive work that must complete for next frame.  
   https://docs.rs/bevy/latest/bevy/tasks/struct.ComputeTaskPool.html

5. **Bevy AsyncComputeTaskPool** — CPU-intensive work that may span frames.  
   https://docs.rs/bevy/latest/bevy/tasks/struct.AsyncComputeTaskPool.html

6. **Tokio `spawn_blocking` docs** — large CPU-bound workloads may be better served by Rayon/dedicated pool.  
   https://docs.rs/tokio/latest/tokio/task/fn.spawn_blocking.html

7. **Rayon 1.12** — data-parallel work-stealing library.  
   https://docs.rs/crate/rayon/latest

8. **Lightyear 0.29** — Bevy 0.19 support, server-authoritative replication, client prediction, interpolation, interest management, WASM/WebTransport.  
   https://docs.rs/crate/lightyear/latest

9. **Avian 0.7** — Bevy 0.19 compatibility, rigid bodies/collision, f32/f64 modes.  
   https://docs.rs/crate/avian3d/latest

10. **Parry 0.30 / parry3d-f64** — standalone geometry/collision query library with f64 variant.  
    https://docs.rs/crate/parry3d/latest  
    https://docs.rs/crate/parry3d-f64/latest

11. **avian_fdm 0.2** — current zone-based 6-DoF Bevy/Avian FDM; documented limitations include no compressibility, aeroelasticity, fuel burn, autopilot or physical detachment; LGPL-3.0-or-later.  
    https://docs.rs/avian_fdm/latest/avian_fdm/  
    https://docs.rs/crate/avian_fdm/latest/source/README.md

12. **nyx-space 2.5.1** — high-fidelity astrodynamics including multibody dynamics, spherical harmonics, finite burns, visibility/eclipses; core AGPLv3.  
    https://rustdoc.nyxspace.com/nyx_space/  
    https://docs.rs/crate/nyx-space/latest/source/README.md

## Orbital / celestial design references

13. **Holman & Wiegert (1999), Long-Term Stability of Planets in Binary Systems** — empirical S-type/P-type critical semimajor-axis fits used only as first-pass sanity checks.  
    https://ui.adsabs.harvard.edu/abs/1999AJ....117..621H/abstract

14. **NASA Europa facts** — Io/Europa/Ganymede 4:2:1 Laplace resonance and tidal lock context.  
    https://science.nasa.gov/jupiter/jupiter-moons/europa/europa-facts/

15. **Kollmeier & Raymond, “Can Moons Have Moons?”** — long-lived submoon constraints; cites low-e prograde stability fraction around ~0.4895 Hill radius and emphasizes tidal survival.  
    https://academic.oup.com/mnrasl/article/483/1/L80/5195537

## Notes on fictional calculations

- Stellar/planetary orbital periods use Kepler's third law with design masses.
- Surface gravity/escape velocity use Newtonian point/spherical mass formulas.
- Nereid moon chain semimajor axes are chosen to give exact nominal 40/80/160/320/640 h Kepler periods around a 0.95 MJ host.
- Hohmann times/phase angles in the system document are ideal two-body reference values around Nereid, not route guarantees.
- Holman–Wiegert numbers are screening estimates, not proof of stability. Canonical system must pass offline integration.


## Autopilot UX references

- MechJeb2 source/module inventory: https://github.com/MuMech/MechJeb2
- MechJeb localization/module names expose Ascent Guidance, Maneuver Planner, Landing Guidance, Docking/Rendezvous and SmartASS-like helpers; used only as UX/functionality reference, not source dependency.

## Cross-platform rendering

- Bevy + WebGPU / wgpu backend overview: https://bevy.org/news/bevy-webgpu/
- Bevy 0.19 release: https://bevy.org/news/bevy-0-19/
