# Project Thessa — World Atlas

Status: partial implementation. §§1–4 and §6.2 describe baked working values
(`data/system.toml`); §5.1 and §7 are superseded by `02B_BC_SUBSYSTEM.md`
(working values below are kept for history, normative numbers live in 02B).

> Working design document. This file tracks **body identity, bulk composition, atmosphere, climate, biomes, resources, and origin notes**. Orbital mechanics and long-horizon stability remain in `01_CELESTIAL_SYSTEM.md`.
>
> Values marked **working** are design targets, not locked canon.

## 0. System-level formation hypothesis

### Asterion architecture

The current leading formation hypothesis is that **Asterion B–C formed as a binary and later captured Asterion A**, or that the present wide A–BC hierarchy emerged from an early few-body interaction rather than quiet in-situ formation.

This is intentionally not hard canon yet. It is useful because it naturally allows:

- chemically and dynamically distinct planetary populations around A and around BC;
- different system ages / apparent evolutionary histories;
- truncated and rearranged protoplanetary disks;
- captured or strongly migrated planets and moons;
- a plausible reason for the A and BC subsystems to have noticeably different architectures.

The playable stellar neighborhood is not intended to stop at the Asterion system. For Epstein-class propulsion, the long-term world-design target is a **local volume roughly 6–10 light-years in radius**, containing several distinct stellar systems and enough travel time that interstellar logistics still matter.

---

# 1. Asterion A subsystem

## 1.1. Khepri

**Identity:** tidally locked hot rocky world with an extreme day/night thermal gradient and a deep cold-trap basin containing a local dense gas sea.

| Parameter | Working value |
|---|---:|
| radius | ~3000 km |
| diameter | ~6000 km |
| mass | ~0.080 M⊕ |
| surface gravity | ~0.36 g |
| escape velocity | ~4.6 km/s |
| semi-major axis | 0.220 AU |
| orbital / rotation period | 41.62 d, synchronous |
| global atmosphere | thin; target ~0.06–0.08 bar near reference datum |
| basin-floor pressure | up to ~0.8 bar |

### Atmosphere

Working bulk composition:

- CO₂ dominant;
- SO₂ as an important volcanic / chemical component;
- Ar and N₂ minor components;
- mineral and sulfur aerosols regionally important.

The global atmosphere is too thin to efficiently erase the permanent day/night temperature contrast.

The anti-Asterion-A hemisphere is not fully dark: Asterion B–C provide weak external illumination and of order **10 W/m²** of irradiance across the wide A–BC orbit. This is visually important but thermally minor compared with direct irradiation from A.

### Cold-trap impact basin

A giant ancient impact basin lies on the cold hemisphere.

- depth: ~25 km;
- very high basin walls;
- bottom atmosphere can reach ~0.8 bar;
- stable thermal inversion strongly suppresses exchange with the upper global atmosphere;
- cold dense gas pools in the basin as a true **local gas sea**;
- upper haze / aerosol layers can visually mark the transition into the dense basin air.

The floor is not uniformly dead. Deep fractures occasionally feed **localized lava fields and volcanic vents** through the otherwise cold basin floor. These create small convective weather cells inside the gas sea: heated plumes, local fog clearing, sulfur chemistry, and sharp horizontal atmospheric gradients.

### Biomes

- **Substellar melt/high-temperature terrain** — hottest illuminated regions, refractory surfaces, thermal cracking.
- **Hot sulfur plains** — sulfur-rich deposits, dust and chemically aggressive near-surface air.
- **Terminator highlands** — strongest long-lived thermal gradients and mechanically active terrain.
- **Cold-side regolith** — cryogenic / frost-bearing surface under weak BC illumination.
- **Basin walls** — steep, cold, wind-sheltered cliffs with condensate deposits.
- **Gas-sea floor** — dense cold atmosphere, fog and sediment-like aerosol deposition.
- **Volcanic oases** — rare lava-fed warm regions inside the basin with strong local convection.

### Resources

Sulfur, iron, nickel-group metals, refractory ores, silicates, trace fissile material. Khepri is primarily a high-temperature materials and specialty-mining world.

---

## 1.2. Nereid

**Identity:** warm gas giant and the main moon-system hub of Asterion A.

| Parameter | Working value |
|---|---:|
| radius | 68,000 km |
| mass | 0.95 MJ |
| cloud-top gravity | ~2.65 g |
| escape velocity | ~59.5 km/s |
| atmosphere | H₂/He-dominated |
| rings | ~85,000–140,000 km from center |

### Atmosphere

Bulk:

- H₂ dominant;
- He secondary;
- CH₄, NH₃ and H₂O as trace / cloud-forming species;
- photochemical hazes and cloud decks vary strongly with altitude.

### Atmospheric biomes / operational layers

- high hazes;
- upper methane/ammonia cloud belts;
- storm bands;
- deep high-pressure atmosphere;
- auroral / magnetospheric regions;
- ring-shadow zones.

### Rings

Ice, silicate dust and darker rocky material. Rings are both scenery and an industrial environment.

---

# 2. Nereid moon system

## 2.1. Pyra

**Identity:** dense, metal-rich, volcanically and tidally heated inner moon.

| Parameter | Working value |
|---|---:|
| radius | 1050 km |
| diameter | 2100 km |
| mass | 0.006 M⊕ |
| gravity | ~0.22 g |
| atmosphere | near-vacuum; local SO₂ exospheres |

### Composition

Very high bulk density is intentional: large metallic fraction, iron-rich interior, refractory mantle / crust.

### Biomes

- active volcanic provinces;
- sulfur plains;
- cooled lava seas;
- tectonic fracture fields;
- metal-rich cratered highlands;
- fresh ejecta blankets.

---

## 2.2. Thessa

**Identity:** old captured planetary body, now a large moon of Nereid and the primary starting world.

| Parameter | Working value |
|---|---:|
| radius | 3200 km |
| diameter | 6400 km |
| mass | ~0.126 M⊕ |
| gravity | ~0.50 g |
| atmosphere | ~1.20 bar |
| state | tidally locked to Nereid |

### Origin hypothesis

Thessa did **not necessarily form with Nereid**. Working hypothesis:

- formed as a small independent planet, possibly in another stellar system;
- was ejected and spent time as a rogue / wandering world;
- retained subsurface liquid water and potentially life beneath an ice shell;
- was captured during the early evolution of Nereid while a massive circumplanetary disk and strong multi-body interactions still existed;
- migrated inward/outward through that disk;
- helped establish the current resonant architecture of Nereid's regular moons.

This origin also permits Thessa's biosphere to be significantly older than the current Asterion configuration.

### Atmosphere

Working composition:

- N₂ ~73.5–73.7%;
- O₂ 25.0%;
- Ar ~1.0–1.2%;
- CO₂ ~0.3%;
- H₂O variable, with O₃ and other photochemical species at trace levels;
- total pressure ~1.20 bar.

These composition percentages are molar/volume fractions. For propulsion and
other mass-flow calculations the runtime atmosphere API must derive species
mass fractions from the same canonical composition rather than reuse the
numeric molar percentage directly.

Composition remains provisional until biosphere canon is locked.

### Biomes

- temperate oceans;
- archipelagos;
- humid coastal forests / analogous high-productivity biomes if complex life is retained;
- cool continental interiors;
- high mountain plateaus;
- glacial / polar terrain;
- large tidal / coastal environments shaped by Nereid;
- ancient impact and cryogenic-era terrains preserving pre-capture geology.

### Geological / biological fingerprints of capture

Possible canon clues:

- isotope ratios inconsistent with the other Nereid moons;
- volatile inventory unlike circum-Nereid material;
- unusually old crustal minerals;
- biosphere divergence times older than the Asterion system's current stellar-age constraints;
- remnants of a globally frozen rogue-planet epoch.

---

## 2.3. Pelagos

**Identity:** ocean-dominated humid moon and water/chemical industry hub.

| Parameter | Working value |
|---|---:|
| radius | 2500 km |
| diameter | 5000 km |
| mass | 0.055 M⊕ |
| gravity | ~0.36 g |
| atmosphere | ~1.7 bar |

### Atmosphere

Working:

- N₂ dominant;
- substantial H₂O vapor regionally;
- CO₂;
- Ar and minor gases.

### Biomes

- global / near-global ocean;
- volcanic island arcs;
- shallow carbonate / salt shelves;
- storm belts;
- warm archipelagos;
- polar sea-ice fields;
- deep brine basins.

### Resources

Bulk water, dissolved salts, lithium-bearing brines, deuterium feedstock, carbon and nitrogen chemistry.

---

## 2.4. Auron

**Identity:** dry metal-rich industrial moon.

| Parameter | Working value |
|---|---:|
| radius | 1700 km |
| diameter | 3400 km |
| mass | 0.020 M⊕ |
| gravity | ~0.28 g |
| atmosphere | ~0.006 bar |

### Atmosphere

Thin CO₂/Ar-dominated exosphere-like atmosphere with strong day/night variability.

### Biomes

- bare metallic highlands;
- oxidized dust basins;
- large exposed ore provinces;
- crater fields;
- tectonic scarps;
- permanently shadowed cold traps.

### Resources

Aluminium ores, titanium, nickel/cobalt, uranium/thorium family, quartz and bulk silicates.

---

## 2.5. Borea

**Identity:** cold volatile-rich outer moon with its own submoon.

| Parameter | Working value |
|---|---:|
| radius | 2100 km |
| diameter | 4200 km |
| mass | 0.022 M⊕ |
| gravity | ~0.20 g |
| atmosphere | ~0.55 bar |

### Atmosphere

Working:

- N₂ dominant;
- CH₄ significant;
- Ar;
- NH₃ / hydrocarbons / seasonal condensates as minor components.

### Biomes

- high-albedo nitrogen/water ice plains;
- methane frost terrain;
- dark tholin-rich regions;
- cryovolcanic provinces;
- fractured ice highlands;
- seasonal volatile caps.

### Resources

Nitrogen volatiles, methane/carbon feedstock, water ice, ammonia compounds.

---

## 2.6. Nix

**Identity:** tiny icy submoon and natural orbital propellant source.

| Parameter | Working value |
|---|---:|
| radius | 12.5 km |
| diameter | 25 km |
| density | ~2.2 g/cm³ |
| atmosphere | none |

### Surface / biomes

- dirty water-ice regolith;
- exposed fresh ice in young craters;
- boulder fields;
- almost entirely low-gravity traversal terrain.

---

## 2.7. Halo

**Identity:** small icy/dusty co-orbital body near the Borea L4 region.

| Parameter | Working value |
|---|---:|
| radius | ~30 km |
| diameter | ~60 km |
| atmosphere | none |

Irregular shape preferred. Mostly a navigation / co-orbital dynamics object rather than a progression-critical resource body.

---

## 2.8. Cinder

**Identity:** unmistakably non-spherical fragment of a differentiated parent body.

### Shape

Working target: strongly irregular shard, approximately **180 × 110 × 70 km** rather than a hydrostatic moon.

Possible morphology:

- one broad fracture face exposing metallic interior;
- remnants of darker differentiated crust on the opposite side;
- large fault planes;
- complex non-principal-axis / tumbling rotation if dynamically acceptable.

### Composition

Dense Fe/Ni-rich material with platinum-group / refractory inclusions. Cinder may be the surviving fragment of a violently disrupted larger body rather than a primordial asteroid.

### Biomes

- exposed core material;
- broken crust remnants;
- fracture cliffs;
- regolith pockets in local potential lows;
- fresh impact scars.

---

# 3. Orthea–Mira double planet

## 3.1. Orthea

**Identity:** cold heavy terrestrial component of a mutually tidally locked double planet.

| Parameter | Working value |
|---|---:|
| radius | ~8200 km |
| diameter | ~16,400 km |
| mass | ~1.8 M⊕ |
| gravity | ~1.09 g |
| atmosphere | dense, working ~2–3 bar |

### Atmosphere

Working family:

- N₂ major component;
- CO₂ climatically important;
- Ar minor;
- H₂O low but regionally significant.

Exact partial pressures remain open because they directly control whether the intended ~200–230 K climate is viable.

### Biomes

- cold high-pressure plains;
- CO₂ / water frost regions;
- rocky continental highlands;
- glacial valleys;
- buried ice provinces;
- seasonal condensation basins;
- ring-shadow climate bands if ring opacity is substantial.

### Rings — Koro remnants

Koro is no longer retained as a stable standalone moon in the current concept.

Working history:

- a small asteroid/comet or temporary satellite was captured by the Orthea–Mira pair;
- repeated perturbations lowered its periapsis;
- it crossed the disruptive region around Orthea and was tidally / collisionally fragmented;
- the surviving debris forms a **young ring system**.

The rings should look less dynamically pristine than Saturn's: clumps, gaps, shepherd fragments, precession and perturbation by Mira are desirable.

---

## 3.2. Mira

**Identity:** large icy-rocky companion of Orthea; both bodies are mutually tidally locked.

| Parameter | Working value |
|---|---:|
| radius | ~4100 km |
| diameter | ~8200 km |
| mass | ~0.22 M⊕ |
| gravity | ~0.53 g |
| atmosphere | thin; exact target TBD |
| separation from Orthea | ~190,000 km working |

The system barycenter lies outside Orthea, so Orthea–Mira is intentionally presented as a **true double-planet-like pair**, not simply a planet and ordinary moon.

### Biomes

- ice-rock plains;
- large impact basins;
- tectonically fractured regions caused by early mutual tidal evolution;
- volatile-rich cold traps;
- dark equatorial / low-albedo deposits;
- possible ancient cryovolcanic terrain.

---

## 3.3. Dey

**Identity:** small distant dirty-ice body associated with the Orthea–Mira system.

Working redesign: move from a simple Orthea satellite to a more distant **circumbinary** orbit around the pair if long-term integrations support it.

| Parameter | Working value |
|---|---:|
| radius | ~10 km |
| diameter | ~20 km |
| atmosphere | none |

Biomes: dirty ice, dark regolith, impact-exposed clean ice.

---

# 4. Vesper subsystem

## 4.1. Vesper

**Identity:** distant ice giant; currently the major world most in need of a stronger unique environmental hook.

| Parameter | Current design value |
|---|---:|
| radius | 36,000 km |
| mass | 0.12 MJ |
| atmosphere | H₂/He/CH₄ |

### Working atmosphere

- H₂ dominant;
- He;
- CH₄;
- deeper H₂O/NH₃-bearing chemistry;
- photochemical haze.

### Candidate identity hooks

Not canon yet:

- unusually high internal heat flux;
- extreme storm belts;
- strong tilted magnetic field;
- auroral environment affecting moons and upper-atmosphere industry.

---

## 4.2. Skadi

**Identity:** cold volatile-rich moon of Vesper.

| Parameter | Revised working value |
|---|---:|
| radius | 720 km |
| diameter | 1440 km |
| mass | ~0.00045–0.00060 M⊕ |
| gravity | ~0.035–0.047 g |
| atmosphere | trace N₂/CH₄ |

The previous 0.002 M⊕ target implied an implausibly high density for an icy/volatile-rich body; mass is intentionally reopened.

### Biomes

- water-ice plains;
- dark hydrocarbon deposits;
- nitrogen/methane frost pockets;
- old cratered terrain;
- possible cryovolcanic resurfacing.

---

## 4.3. Mote

**Identity:** tiny dirty-ice moon / depot body.

| Parameter | Working value |
|---|---:|
| radius | 25 km |
| diameter | 50 km |
| atmosphere | none |

---

# 5. Asterion B–C subsystem

BC should be a **full planetary subsystem**, not a late-game backdrop with one planet.

> Superseded by `02B_BC_SUBSYSTEM.md` + `data/system.toml`: the BC architecture
> is now 3 planets (`bc_i`, Janus, `bc_outer`) + 8 moons. The paragraphs below
> record the pre-02B design direction and are kept for history.

The exact planet count is open, but the current design direction is:

1. a hot inner circumbinary rocky world near the stable inner region;
2. Janus at 4.4 AU as the major temperate/heavy terrestrial world;
3. at least one colder outer planet or sub-Neptune / giant with its own moon system.

This leaves room for additional minor planets, captured bodies and debris populations.

## 5.1. Inner circumbinary world — BC-I (was TBD)

> Resolved as S-type circum-B remnant `bc_i`: 0.075 AU, 2000 km, 0.042 M⊕,
> synchronous (`system.toml`, `02B §1.1`). The ~0.8–1.0 AU P-type family below
> is the superseded pre-02B direction.

**Identity target:** intensely irradiated rocky / refractory world under two moving suns.

Working orbit family: ~0.8–1.0 AU, subject to stability and climate checks.

Potential biomes:

- glassy impact plains;
- refractory highlands;
- metallic lava / melt remnants if temperatures permit;
- extreme eclipse-driven thermal cycling;
- mineral-vapor exosphere.

---

## 5.2. Janus

**Identity:** massive circumbinary terrestrial world and the central logistics anchor of the BC subsystem.

| Parameter | Current / working value |
|---|---:|
| radius | 9800 km |
| diameter | 19,600 km |
| mass | 3.2 M⊕ |
| gravity | ~1.35 g |
| atmosphere | ~1.4 bar working |
| orbit | 4.4 AU around BC barycenter |

### Atmosphere

Working:

- N₂ major;
- CO₂ climatically important;
- Ar minor;
- H₂O variable;
- stronger UV-driven photochemistry than on Asterion-A worlds.

### Biomes

- high-gravity rocky continents;
- large sedimentary basins;
- cold / temperate highlands depending final greenhouse state;
- strong binary-shadow / eclipse illumination cycles;
- chemically weathered lowlands;
- possible shallow seas if retained in final climate model.

The central identity is not resource richness: it is the **binary sky and the logistics network built around a heavy surface world**.

---

# 6. Janus moon system

## 6.1. Inner moon — janus_inner (was TBD)

**Identity:** small close irregular / heavily fractured moon, possibly impact-derived.

Working size: **~150–300 km radius**; baked pick 225 km at 80 000 km
(`system.toml`, `02B §2.2`).

### Biomes

- fault scarps;
- fractured blocks;
- impact melt plains;
- exposed deep material;
- regolith ponds.

Role: first cheap off-world base, mass-driver site and navigation landmark near Janus.

---

## 6.2. Mora — evaporite / dying-ocean moon

**Identity:** former mini-ocean world, now mostly desiccated into giant salt basins with rare surviving hypersaline lakes and subsurface brines.

| Parameter | Revised working value |
|---|---:|
| radius | ~1800–2000 km |
| mass | ~0.015–0.020 M⊕ |
| gravity | ~0.19–0.20 g |
| atmosphere | residual ~0.15–0.35 bar working |

### Atmosphere

Likely N₂/CO₂-dominated residual atmosphere with H₂O vapor strongly localized around surviving brines. Trace chlorine/sulfur-bearing chemistry may be regionally important but should not dominate globally without a specific source.

### Biomes

- vast white evaporite flats;
- red/brown mineral salt pans;
- collapsed former shorelines;
- hypersaline residual lakes;
- geothermal brine pools;
- subsurface aquifers / brines;
- salt karst / dissolution terrain;
- dry former seafloor sediment basins.

### Resources

Exceptionally rich in **Li, Na, K, Mg and Cl salts**, plus borates and other evaporite-associated industrial feedstocks. Mora is primarily a chemical-industry world rather than another generic metal mine.

---

## 6.3. Titan-like outer moon — janus_haze (was TBD)

**Identity:** thick-atmosphere organic moon; analogous to Titan in atmospheric architecture, not in surface temperature.

Baked pick: 2600 km, 0.025 M⊕, 1.75 bar, 1 150 000 km (`system.toml`, `02B §2.4`).
Original working target for history:

| Parameter | Working target |
|---|---:|
| radius | ~2500–2700 km |
| mass | ~0.02–0.03 M⊕ |
| gravity | ~0.13–0.17 g |
| atmosphere | ~1.5–2 bar |
| orbit | ~1.0–1.3 million km from Janus |

### Atmosphere

Working:

- N₂ dominant;
- CH₄ minor but photochemically important;
- CO₂ trace/minor;
- complex organic haze;
- H₂ and higher hydrocarbons as photochemical products.

### Biomes

- orange haze-dimmed plains;
- organic dune seas;
- water-ice bedrock mountains;
- impact basins;
- ammonia/water cryovolcanic provinces;
- dark hydrocarbon-rich sediment regions.

No stable Titan-style surface methane seas are assumed at the current irradiation level unless later climate modelling specifically supports them.

---

# 7. Outer BC planet — BC-Outer (was TBD)

**Identity target:** colder giant / sub-Neptune giving BC a second major moon system and a different formation history from Nereid.

> Resolved working values: 11.5 AU, 27 M⊕, 31 500 km, range 10–13 AU /
> 20–35 M⊕ / 28 000–35 000 km (`system.toml`, `02B §3.1`). Preferred
> architecture below is kept as design guidance.

Preferred architecture:

- fewer neat resonances than Nereid;
- more captured irregular satellites;
- at least one strongly inclined or retrograde large moon if dynamically plausible;
- volatile-rich cold environments;
- visibly different ring/moon chemistry from the Asterion-A subsystem.

Exact planet mass, radius and orbit: see resolved values above (`02B §3.1`); moon-system details in `02B §§4.1–4.5`.

---

# 8. Design rules for future worlds

1. Every major world should have a **one-sentence physical identity** that is not just its temperature or resource table.
2. Avoid repeating “rocky super-Earth with N₂/CO₂ atmosphere” unless the gameplay and geological history are radically different.
3. Atmosphere entries should eventually specify **surface pressure + molar fractions**, not only a list of gases.
4. Bulk mass/radius pairs must be checked for implied density before canon lock.
5. Biomes should emerge from climate, geology and orbital history rather than being painted on solely for visual variety.
6. Moon systems should tell formation stories: regular in-situ moons, captured objects, resonance migration, shattered bodies, co-orbitals and submoons should not all look dynamically interchangeable.
7. Not every body needs to be progression-critical. Some exist for navigation, science, visual identity or orbital-dynamics gameplay.
8. Asterion A and BC should feel like **different planetary families**, especially if the capture hypothesis becomes canon.
9. The wider 6–10 ly playable neighborhood should introduce new stellar architectures instead of duplicating the starting system at larger scale.
