# 01 — Celestial system

## Status

**Implemented design-target dataset.** `data/system.toml` and its baked output
are deterministic prototype inputs, not final astronomical canon. Runtime body
motion uses baked analytic ephemerides; ships receive simultaneous multi-body
gravity.

## 1.1. Runtime policy

- `thessa-system-baker` validates the parent graph and writes the reproducible
  `data/system.baked.json` descriptor.
- `BakedEphemeris` evaluates body states at `SimTime` without integrating the
  mutual dynamics of stars, planets, or moons.
- Ships, stations, debris, and temporary objects use the sum of gravity from
  relevant physical sources.
- `system_barycenter` and `bc_barycenter` are coordinate anchors; they do not
  add a second copy of their children’s gravity.
- SOI labels may help UI or optimization, but SOI switching is not physics.

The data file intentionally declares `status = "design-target-not-canonical"`.
Numbers in this document must therefore be treated as reproducible scenario
values, not claims about a real system.

## 1.2. Stars and the binary branch

The design contains Asterion A, B, and C. B and C form an inner binary with a
relative semi-major axis of `0.220 AU`, eccentricity `0.040`, and a design
period of `24.430692 d`. The A–BC outer orbit uses a relative semi-major axis
of `45 AU`, eccentricity `0.060`, and a design period of `61635.9375 d`.

The binary is coplanar with the outer design orbit so the prototype can test
eclipses and multi-source lighting. B/C is represented by its physical bodies
and a barycentric anchor, not by a fake replacement gravity source.

## 1.3. Planets around Asterion A

| Body | Role in the design | Key values |
| --- | --- | --- |
| Khepri | hot inner rocky world | `0.220 AU`, `4400 km`, `0.04 bar` |
| Nereid | gas giant and main moon system | `0.780 AU`, `68000 km`, rings `85000–140000 km` |
| Orthea | large rocky world | `2.150 AU`, `9000 km`, `2.8 bar` |
| Vesper | outer gas giant | `3.800 AU`, `36000 km`, `0.12 Mj` |

## 1.4. Nereid system

The primary gameplay start is Thessa, a tidally locked moon of Nereid. The
design moon chain is:

| Body | Semi-major axis | Design period | Notes |
| --- | ---: | ---: | --- |
| Pyra | `398358 km` | `40 h` | sulfur/rocky inner moon |
| Thessa | `632354 km` | `80 h` | start world, `3200 km`, `1.20 bar` |
| Pelagos | `1003799 km` | `160 h` | wet rocky moon, `1.7 bar` |
| Auron | `1593432 km` | `320 h` | small resource moon |
| Borea | `2529415 km` | `640 h` | cold moon, `0.55 bar` |

Additional Nereid bodies are Nix, a submoon of Borea; Halo, a proposed Borea
L4 co-orbital; and Cinder, an irregular retrograde moon. Halo is still a
diagnostic design approximation: the current ephemeris inputs do not make a
fully eccentric, phase-consistent L4 solution.

Thessa’s current design target is radius `3200 km`, surface gravity about
`4.903325 m/s²`, atmosphere pressure `1.20 bar`, and an `80 h` rotation/orbit.
The atmosphere composition is provisional `N2/O2/Ar/CO2`.

## 1.5. Orthea and Vesper systems

Orthea has Koro, Mira, and Dey in the current design input. Vesper has Skadi
and Mote. Their resource lists and atmosphere values are data-driven and are
available to the baker; they are not yet connected to a complete factory or
logistics gameplay loop.

## 1.6. Lighting and eclipses

The visual atmosphere crate uses body positions, star temperatures, angular
radii, irradiance, and eclipse visibility to build shared optical inputs. The
client renders those inputs through Bevy atmosphere/raster paths. This is a
visual subsystem; it does not change authoritative gravity or pressure.

The Thessa design starts near a Nereid/Asterion eclipse geometry. Exact eclipse
durations and climate effects remain scenario targets because orbital-plane
and atmosphere canon are not locked.

## 1.7. Resource families

The data model currently uses resource families such as iron ore,
nickel/cobalt/refractory material, bulk silicates, water ice, nitrogen
volatiles, carbon feedstock, sulfur, fissile ore, and hydrogen/helium
atmosphere. These are design inputs only; the factory/resource simulation is
not implemented in the current runtime.

## 1.8. Validation before canon lock

Before calling this system canonical, validate:

- long-horizon stability of the selected analytic segments;
- phase and eccentricity consistency for the Nereid chain;
- Halo’s co-orbital behavior with real eccentric/inclined inputs;
- J2/Jn and body-fixed rotation data for bodies where they matter;
- atmosphere composition, weather, and climate parameters;
- resource placement and gameplay throughput;
- reference-frame and unit contracts against generated descriptors.

Useful commands:

```bash
cargo run -p thessa-system-baker -- \
  --input data/system.toml --output data/system.baked.json
cargo test -p thessa-sim-core
```
