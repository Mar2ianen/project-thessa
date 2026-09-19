# Project Thessa — Atmosphere Model & Tracking Table

Status: partial implementation. A-system/Janus/Mora rows match `data/system.toml`
pressures; BC-subsystem rows added from the 02B bake; TBD cells stay future.

> Companion to `02_WORLD_ATLAS.md`.
>
> **Canonical atmospheric inputs are composition + pressure profile + temperature profile.** Density is derived from them and must not be stored as an independent simulation truth.

## 1. Data rule

For a well-mixed ideal-gas layer, reference density is approximately

`rho = p * M / (R * T)`

where `M` is mean molar mass. Real runtime atmospheres may deviate through condensation, chemistry, non-ideal behavior and vertical temperature structure.

Each atmosphere should eventually define:

- surface/reference pressure;
- molar fractions;
- reference temperature or climate profile;
- derived reference density;
- scale-height / vertical-profile model;
- condensable species;
- aerosol/haze model;
- biome/local overrides where required.

## 2. Current atmosphere table

`rho_ref` values are **illustrative design estimates**, not locked canon, until temperature/composition are fixed.

| Body / region | Pressure | Working bulk composition | Reference thermal state | Approx. rho_ref | Notes |
|---|---:|---|---|---:|---|
| **Khepri — global datum** | ~0.06–0.08 bar | CO₂ dominant; SO₂, Ar, N₂ | extremely regional | derive per biome | global atmosphere intentionally poor at heat redistribution |
| **Khepri — cold basin floor** | up to ~0.8 bar | CO₂-rich, enhanced condensable/volcanic species | ~190–220 K target | ~1.9–2.2 kg/m³ for CO₂-rich gas | local gas sea, not a separate sealed atmosphere |
| **Nereid** | profile, no surface datum | H₂/He; CH₄/NH₃/H₂O traces | altitude-dependent | profile only | use pressure-level reference radii instead of surface density |
| **Pyra** | near vacuum | local SO₂ exosphere | strongly regional | negligible | transient volcanic exospheres |
| **Thessa** | ~1.20 bar | ~76% N₂, ~21% O₂, ~3% Ar/CO₂/H₂O/trace | ~280–292 K | ~1.4–1.5 kg/m³ | composition provisional until biosphere canon lock |
| **Pelagos** | ~1.7 bar | N₂-rich; CO₂/H₂O/Ar | warm/humid | ~1.8–2.2 kg/m³ | strong humidity and weather variation |
| **Auron** | ~0.006 bar | CO₂/Ar | cold, strongly diurnal | ~0.01–0.02 kg/m³ | near-exosphere / thin-atmosphere regime |
| **Borea** | ~0.55 bar | N₂/CH₄/Ar; minor NH₃/hydrocarbons | cryogenic | ~1.5–2.2 kg/m³ | condensation/seasonality important |
| **Nix** | none | — | — | 0 | — |
| **Halo** | none | — | — | 0 | — |
| **Cinder** | none | — | — | 0 | — |
| **Orthea** | ~2–3 bar | N₂ + substantial CO₂ + Ar | ~200–230 K target | roughly ~4–6 kg/m³ | exact CO₂ partial pressure must be climate-driven |
| **Mira** | ~0.08 bar N₂/CO₂ provisional | N₂/CO₂ | cold | TBD | reopened from previous moon concept; exact target still TBD |
| **Dey** | none | — | — | 0 | — |
| **Vesper** | profile, no surface datum | H₂/He/CH₄; deeper NH₃/H₂O chemistry | altitude-dependent | profile only | pressure-level atmosphere model |
| **Skadi** | trace | N₂/CH₄ | cryogenic | TBD | mass and atmosphere both reopened |
| **Mote** | none | — | — | 0 | — |
| **Janus** | ~1.4 bar | N₂/CO₂/Ar, H₂O variable | ~260–290 K | ~1.7–2.0 kg/m³ | stronger UV-driven chemistry under B |
| **janus_inner** | none | — | — | 0 | impact-derived concept |
| **Mora** | ~0.15–0.35 bar | residual N₂/CO₂; localized H₂O | dry/cool, local brine microclimates | TBD | dying-ocean / evaporite moon |
| **janus_haze** | ~1.75 bar | N₂ dominant; CH₄, minor CO₂, organic haze | warm relative to Titan | TBD | Titan-like atmospheric architecture, not Titan surface thermodynamics |
| **bc_i** | TBD | likely mineral-vapor / trace volatile atmosphere | extremely hot | TBD | composition should emerge from surface vapor equilibrium |
| **bc_outer** | TBD (H₂/He/CH₄ provisional, no pressure) | H₂/He/CH₄ | cold | TBD | planet class working pick, pressures open |
| **bc_outer_retro** | TBD (N₂/CH₄ provisional, no pressure) | N₂/CH₄ | cold | TBD | pressures open |
| **bc_outer_corona / ridge / rubble** | none | — | — | 0 | airless |
| **bc_outer_plume** | transient H₂O provisional | H₂O | cold | TBD | cryovolcanic transient |

## 3. Khepri local-atmosphere rule

Khepri requires a local pressure/temperature field rather than one global `surface_pressure` number.

At minimum distinguish:

1. hot illuminated highlands / plains;
2. terminator regions;
3. cold anti-A hemisphere;
4. giant basin rim;
5. basin gas-sea floor;
6. volcanic cells inside the basin.

The ~25 km topographic depth naturally produces a large hydrostatic pressure increase. Cold dense gas additionally strengthens the basin trap, while volcanic heat produces local buoyant plumes without globally mixing the reservoir.

## 4. Biome coupling

Biome definitions should be allowed to modify atmospheric boundary conditions without creating independent fake atmospheres.

Examples:

- Khepri volcanic oasis: local `T`, SO₂ source and aerosol source;
- Pelagos ocean: H₂O flux, cloud nucleation and humidity;
- Borea volatile cap: N₂/CH₄ condensation/sublimation source;
- Mora brine basin: local H₂O vapor and salt aerosol;
- Orthea seasonal cold trap: CO₂/H₂O deposition and pressure variation if climate model supports it.

The atmosphere solver remains global; biomes provide boundary/source terms.
