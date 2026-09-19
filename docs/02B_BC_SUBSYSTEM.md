# Project Thessa — Asterion BC Subsystem

Status: implemented reference. Working ranges contain the baked
`data/system.toml` midpoints; orbits are working slots, not stability-proven.

> Working world-design supplement for the Asterion B–C subsystem. Values are provisional unless explicitly marked otherwise. Orbital integrations must be run before canon lock.

## 0. Design identity

Asterion BC should not feel like a smaller copy of the A subsystem. Its current design language is:

- a dynamically old close binary with a later-captured wide A component;
- strong spectral/illumination effects from the hot B primary;
- at least one surviving S-type planet close to B;
- a temperate circumbinary terrestrial system around Janus;
- a distant cold giant with a moon family shaped by capture, disruption and reaccretion rather than a clean regular resonant chain.

The BC subsystem therefore contains both **circumstellar (S-type)** and **circumbinary (P-type)** planets.

---

# 1. Inner surviving world around Asterion B

## 1.1. BC-I — stripped planetary remnant

**Identity:** an atmosphere-free, iron/refractory-rich remnant of a once larger planet, surviving on a very tight orbit around Asterion B.

This world is intentionally not a generic Mercury analogue. It is the exposed deep interior of a planet that lost most of its silicate mantle during one or more catastrophic early impacts and subsequent thermal erosion.

| Parameter | Working value |
|---|---:|
| host | Asterion B |
| orbital type | S-type circum-B |
| semi-major axis | ~0.07–0.08 AU |
| orbital period | ~4.8–5.8 d |
| radius | ~2000 km |
| diameter | ~4000 km |
| bulk density | ~7–8 g/cm³ |
| mass | ~0.04 M⊕ |
| surface gravity | ~0.43 g |
| escape velocity | ~4.1 km/s |
| atmosphere | none / transient metal-silicate exosphere only |
| rotation | probably synchronous or near-synchronous |

### Thermal environment

At ~0.07–0.08 AU from B the incident stellar flux is thousands of Earth solar constants. The surface survives only because the body is composed largely of refractory material and has an unusually high effective reflectivity.

The exact Bond albedo is deliberately not locked yet; it must be solved together with the surface mineralogy and the desired substellar temperature. The design target is:

- most of the bright refractory surface remains solid;
- the substellar region can contain local melt lakes or thin molten seams;
- the night side is much colder because there is essentially no atmosphere to redistribute heat;
- the terminator contains extreme thermal-stress terrain.

### Composition

Working bulk composition:

- Fe/Ni-rich interior and exposed core-derived material;
- refractory silicates;
- Al/Ca-rich high-temperature phases;
- titanium-bearing minerals;
- very low volatile inventory;
- local impact-delivered incompatible elements.

A visually bright refractory crust is preferred over a uniformly dark iron ball.

### Biomes

- **substellar refractory plains** — brightest, hottest terrain;
- **melt scars / metal-silicate lakes** — small local partially molten areas;
- **thermal fracture belts** — extreme expansion/contraction terrain near the terminator;
- **night-side shattered highlands** — cold, heavily fractured exposed interior;
- **giant impact faces** — broad excavated regions where deep differentiated layers are visible;
- **metal frost / condensate cold traps** — trace vaporized material recondensed on the night side.

### Origin

Possible history:

1. BC-I formed as a larger rocky world close to B while the binary system was young.
2. Major impacts stripped much of its original mantle.
3. Stellar heating removed essentially all remaining volatiles.
4. The surviving refractory remnant tidally evolved into its current close orbit.

Its existence is a fossil of the violent early BC subsystem.

---

# 2. Janus system

## 2.1. Janus

Janus remains the primary temperate circumbinary terrestrial world of BC.

Current working values from the main system design remain approximately:

| Parameter | Working value |
|---|---:|
| host | BC barycenter |
| semi-major axis | 4.4 AU |
| mass | ~3.2 M⊕ |
| radius | ~9800 km |
| surface gravity | ~1.35 g |
| atmosphere | ~1.4 bar, N₂/CO₂/Ar family |

Its defining environmental signature is the moving double-star sky rather than exotic bulk composition.

## 2.2. Janus inner fragment moon

**Identity:** small close irregular moon, probably impact-derived.

| Parameter | Working value |
|---|---:|
| mean radius | ~150–300 km |
| shape | irregular / non-hydrostatic |
| atmosphere | none |
| composition | silicate + metal-rich impact debris |

Biomes:

- fracture scarps;
- boulder seas;
- exposed impact-melt sheets;
- regolith ponds in local potential lows.

Primary gameplay role: cheap first off-world industrial site in the Janus system.

## 2.3. Mora — desiccated brine world

**Identity:** former shallow ocean moon, now dominated by salt flats and residual hypersaline lakes.

| Parameter | Working value |
|---|---:|
| radius | ~1800–2000 km |
| mass | ~0.015–0.020 M⊕ |
| gravity | ~0.19–0.20 g |
| atmosphere | ~0.15–0.35 bar working |
| state | tidally locked to Janus |

### Atmosphere

Working family:

- N₂ dominant;
- CO₂ significant;
- H₂O variable and locally enhanced;
- trace sulfur/chlorine-bearing chemistry possible near active brines.

### Surface history

Mora once carried a shallow global or near-global saline ocean. Water loss and long-term climate evolution reduced it to isolated deep basins, groundwater and hypersaline remnants.

### Biomes

- continental-scale white evaporite flats;
- red/brown salt crust provinces;
- polygonal desiccation terrain;
- residual hypersaline lakes;
- seasonal or geothermal brine pools;
- buried brine aquifers;
- old wave-cut terraces and former coastlines.

### Resources

Mora is a chemical-processing world rather than a generic metal mine:

- lithium salts;
- sodium salts;
- potassium salts;
- magnesium salts;
- chlorine compounds;
- borates;
- water/brine feedstock.

## 2.4. Janus outer haze moon

**Identity:** a large Titan-like atmospheric moon in morphology and photochemistry, but warmer than Titan and not dependent on surface methane seas.

| Parameter | Working value |
|---|---:|
| radius | ~2500–2700 km |
| mass | ~0.02–0.03 M⊕ |
| atmosphere | ~1.5–2 bar |
| bulk atmosphere | N₂-dominant |

### Atmosphere

Working composition:

- N₂ dominant;
- CH₄ minor but photochemically important;
- CO₂ minor;
- complex hydrocarbon/organic haze;
- trace H₂ and heavier photochemical products.

### Biomes

- orange/brown haze-covered plains;
- organic dune fields;
- water-ice bedrock highlands;
- cryovolcanic provinces;
- impact basins with dark organic sediments;
- possible ammonia-water cryomagma regions.

No large stable methane seas are assumed at the current thermal target.

---

# 3. Outer BC giant

## 3.1. BC-Outer — cold Neptune/sub-Neptune

**Identity:** distant ice giant / sub-Neptune orbiting the BC barycenter, with a moon system dominated by capture and collisional history.

The exact orbit is open. A working range of **~10–13 AU** is preferred: far enough to be genuinely cold and spatially separate from Janus, but still inside the approximate outer dynamically safe region imposed by Asterion A.

| Parameter | Working value |
|---|---:|
| semi-major axis | ~10–13 AU |
| mass | ~20–35 M⊕ |
| radius | ~28,000–35,000 km |
| atmosphere | H₂/He with CH₄ and deeper H₂O/NH₃ chemistry |
| rings | likely faint, dusty/icy, collision-fed |

### Atmospheric identity

- H₂/He bulk envelope;
- methane-bearing upper atmosphere;
- pale blue/gray appearance rather than a direct Neptune copy;
- high-altitude photochemical haze;
- strong internal heat flux is allowed but not required;
- weather should include long-lived dark vortices and narrow bright convective clouds.

---

# 4. BC-Outer moon system

The moon system deliberately borrows **physical archetypes** from memorable Solar System moons without cloning their exact chemistry or maps.

Its formation history should be messy. A large captured retrograde moon can have destroyed or destabilized much of an older regular satellite system; inner moons may therefore be second-generation bodies reaccreted from debris.

## 4.1. Major retrograde captured moon — Triton archetype

**Identity:** largest moon of BC-Outer, captured onto a retrograde orbit and subsequently circularized.

| Parameter | Working value |
|---|---:|
| radius | ~1400–1800 km |
| orbit | retrograde |
| atmosphere | thin-to-moderate volatile atmosphere, exact pressure TBD |
| composition | water ice + silicates + volatile-rich outer layers |

Unlike Triton, surface nitrogen ice is not automatically assumed because BC-Outer is likely warmer than Neptune's real environment.

Preferred distinguishing features:

- young resurfaced plains;
- volatile frost caps in the coldest regions;
- active or recently active cryovolcanic vents;
- cantaloupe / cellular terrain;
- dark plume deposits;
- evidence that its capture catastrophically rearranged the entire moon system.

## 4.2. Patchwork tectonic moon — Miranda archetype

**Identity:** small-to-medium icy moon with absurdly varied geology for its size.

| Parameter | Working value |
|---|---:|
| radius | ~300–500 km |
| atmosphere | none |
| composition | mixed water ice / silicate rock |

Biomes:

- giant corona-like provinces;
- deep fault canyons;
- abrupt boundaries between young and ancient terrain;
- high cliffs;
- chaotic reassembled / partially melted regions.

Its geology may come from repeated tidal-resonance episodes after the captured retrograde moon rearranged the system.

## 4.3. Two-tone ridge moon — Iapetus archetype

**Identity:** outer regular or marginally irregular icy moon with a strong leading/trailing hemispheric contrast and a huge equatorial ridge.

| Parameter | Working value |
|---|---:|
| radius | ~600–900 km |
| atmosphere | none |
| composition | porous water ice + dark carbonaceous contaminants |

Biomes:

- extremely bright ice hemisphere;
- dark externally contaminated hemisphere;
- thermal-migration transition zones;
- continuous or broken equatorial ridge several kilometres high;
- giant ancient impact basins.

The ridge can be the reaccreted remnant of a temporary equatorial debris ring around the moon.

## 4.4. Chaotic rubble moon — Hyperion archetype

**Identity:** irregular porous outer moon in chaotic rotation.

| Parameter | Working value |
|---|---:|
| dimensions | ~200–350 km characteristic size |
| density | very low; high macroporosity |
| atmosphere | none |
| rotation | chaotic / tumbling |

Biomes:

- sponge-like deep craters;
- dark deposits in crater floors;
- exposed bright ice walls;
- kilometre-scale void-rich rubble terrain.

Gameplay distinction: landing and surface operations occur on an object whose orientation cannot be treated like a conventional stable moon.

## 4.5. Active plume moon — Enceladus archetype

**Identity:** small second-generation icy moon with strong tidal heating and active jets.

| Parameter | Working value |
|---|---:|
| radius | ~250–400 km |
| atmosphere | transient plume atmosphere |
| composition | water ice + salts + silicates |

Biomes:

- young bright ice plains;
- polar fracture bands;
- geyser fields;
- plume-fall snow deposits;
- older cratered terrain away from the active pole.

A subsurface saline ocean or regional water layer is preferred. The moon can feed a faint icy ring around BC-Outer.

---

# 5. Formation narrative for BC-Outer moons

Working sequence:

1. BC-Outer originally formed a conventional regular satellite family.
2. A large outer icy body was captured onto a retrograde orbit.
3. Capture/circularization destabilized or destroyed much of the primordial moon system.
4. A debris disk formed around the planet.
5. Several inner moons reaccreted from that debris.
6. Later resonant migration produced episodes of strong tidal heating, creating the Miranda-like and Enceladus-like bodies.
7. Some distant fragments survived as irregular moons; one porous survivor remains in chaotic rotation.

This gives the outer giant a visibly archaeological moon system: every moon is evidence of the same ancient catastrophe rather than an unrelated gimmick.
