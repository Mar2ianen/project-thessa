# Thessa v0.2 — World Design

## 1. Identity

Thessa is the habitable-ish starting moon of the gas giant Nereid. It should not look like
"Earth with shuffled continents". Its visual identity comes from four coupled facts:

1. low gravity (`~0.5 g`) with a dense, vertically extended atmosphere;
2. synchronous rotation around Nereid and regular eclipses by the giant;
3. moderate tidal/geothermal activity maintained by the resonant moon system;
4. a cool K-star primary plus a distant hot blue-white secondary star.

The intended climate is **cool, oceanic, windy and geologically alive**, with large glaciers,
temperate lowlands, dry continental interiors, volcanic provinces, giant impact structures and
high cloud decks.

Target global mean surface temperature is approximately **276–280 K**, not Earth-like 288 K.

---

## 2. Proposed stellar environment

### Asterion A — proposed v0.2

- mass: `0.840 M_sun`
- radius: `0.850 R_sun`
- effective temperature: `5150 K`
- luminosity from Stefan-Boltzmann scaling: `0.4579 L_sun`

Nereid remains at `0.780 AU`.

Derived:
- stellar flux at Nereid/Thessa before eclipses: `1024.3 W/m²`
- Nereid orbital period: `274.54 d`

This is intentionally only a small increase in stellar mass from the current design, but recovers
roughly 9% luminosity.

### Asterion B

Keep the current approximate `15.8 L_sun` hot secondary at ~45 AU from the A system.
Its direct flux near Thessa is around `10.6 W/m²`.

B is not a major heat source, but is:
- visually important;
- disproportionately important in UV/photochemistry because it is hot;
- capable of keeping the sky dimly illuminated during many eclipses of Asterion A.

Asterion C is mainly a visual/astronomical secondary for Thessa's climate.

---

## 3. Bulk body

- mean radius: `3200 km`
- mass: `0.1259652 M_earth`
- surface gravity: `4.9033 m/s²`
- escape velocity: `5.602 km/s`
- circular surface orbital speed: `3.961 km/s`
- orbital semimajor axis around Nereid: `632354 km`
- sidereal period: `80 h`
- eccentricity: `0.003`
- synchronous rotation: yes

### Equilibrium figure

Thessa is not perfectly spherical. Render/terrain datum may eventually be a triaxial ellipsoid:
- longest axis points approximately toward/away from Nereid;
- intermediate axis lies in the orbital plane;
- shortest axis is the spin axis.

Expected global deviation is only a few kilometres across axes, so it should be physically present
but visually subtle. Terrain relief can exceed the equilibrium-figure distortion.

Do **not** exaggerate Thessa into a visible egg.

---

## 4. Atmosphere

### Proposed dry composition

Use this as a design target, not a biochemical simulation requirement:

- `N2 = 73.5%`
- `O2 = 25.0%`
- `Ar = 1.2%`
- `CO2 = 0.3%`
- variable `H2O`
- trace photochemical species including `O3`, possibly `N2O`

Surface pressure: `1.20 bar`.

Derived at `278 K`:
- mean molar mass: `29.201 g/mol`
- approximate gamma: `1.4015`
- speed of sound: `333.1 m/s`
- surface density: `1.516 kg/m³`
- pressure scale height: `16.1 km`
- dry adiabatic lapse rate: `4.93 K/km`

The composition is intentionally close enough to terrestrial air that aircraft do not inherit a
wildly different speed of sound.

### Oxygen choice

At 1.2 bar and 25% O2:

`pO2 ≈ 0.30 bar`.

This should make combustion noticeably oxygen-rich compared with modern Earth, while still
remaining a broadly terrestrial-style atmosphere. It also makes wildfire and industrial fire
hazards part of the world's character.

### Carbon dioxide

`0.3% = 3000 ppm`, not "very little". At 1.2 bar this is `~3.6 mbar pCO2`.
Combined with low gravity, the atmospheric column is large, so this can provide meaningful
greenhouse forcing without making CO2 a bulk gas.

Do not try to tune climate by adding tens of percent heavy greenhouse gas; preserving familiar
air-breathing flight behavior is a design goal.

### Ozone / photochemical layer

Do not model this as a separate bulk atmosphere.

Use a vertically enhanced trace-chemistry layer:
- broad stratospheric O3 maximum;
- stronger/vertically broader than modern Earth's as an artistic/scientific design choice;
- Asterion B UV can help motivate unusual photochemistry;
- it may affect upper-atmosphere heating and UV shielding.

Below the homopause, bulk gases remain well mixed.

---

## 5. Clouds

High clouds are a major visual and climatic feature of Thessa.

Low gravity and large scale height allow a vertically extended weather system. Suggested design:

- ordinary low/mid clouds in weather systems;
- strong storm anvils reaching roughly `18–22 km`;
- persistent thin high ice clouds / cirrus-like decks around `20–30 km`;
- occasional extreme convective tops higher than that.

High clouds are useful because they:
- strengthen longwave greenhouse trapping;
- create a distinctive layered limb from orbit;
- make the atmosphere look deep;
- visually soften the otherwise cool global climate.

Do not make the entire globe a uniform opaque cloud ball. Major surface landmarks must remain
readable from orbit through clear regions and variable cloud cover.

---

## 6. Eclipses and Nereid-facing asymmetry

Nereid radius: `68000 km`.
From Thessa it spans roughly 12 degrees of sky.

A central eclipse of Asterion A lasts on the order of **2.6–2.8 hours**. The whole moon can spend
roughly **2.5–2.6 hours** inside deep central shadow in favorable geometry.

Exact annual eclipse frequency and duration should depend on the final definition of Thessa's
orbital plane relative to Nereid's equator/orbit. Do not bake a single annual-average eclipse
number until that geometry is explicit.

Climate consequence:
- the Nereid-facing hemisphere repeatedly loses heating near local midday;
- the same hemisphere receives Nereid IR and reflected light at other phases;
- its diurnal temperature range should tend to be smaller and its climate more maritime/cloudy;
- the anti-Nereid hemisphere should have stronger continental day/night thermal swings.

This asymmetry should influence the macro-biome layout.

---

## 7. Tidal / geothermal activity

Use a global mean tidal heat target around:

`~0.10–0.20 W/m²`

A useful nominal value is `0.146 W/m²`, corresponding to effective
`k2/Q ~ 0.008` in the simple eccentricity-tide estimate.

This is **not** a major direct climate heater. Its main purpose is geology.

Do not distribute it uniformly as visible terrain. Concentrate expression into several
geothermal provinces:

- 2–4 major active rift / volcanic regions;
- 4–10 secondary geothermal fields;
- many minor hydrothermal/fumarolic areas;
- large inactive ancient volcanic provinces.

Local heat flux may be orders of magnitude above the global mean.

Worldgen consequences:
- basalt plains;
- calderas;
- shield volcanoes;
- rift valleys;
- hot springs;
- geothermal lakes;
- sulfur-rich fields;
- submarine hydrothermal provinces;
- local snow-free geothermal ground inside cold regions.

---

## 8. Climate geography

Use four macro driver fields, even if they are only heuristic masks:

1. `eclipse_exposure`
2. `nereid_radiative_influence`
3. `tidal_geothermal_activity`
4. `continentality`

Additional normal drivers:
- latitude / seasonal insolation;
- elevation;
- moisture availability;
- distance to ocean;
- prevailing circulation proxy;
- slope / aspect;
- geology.

### Desired hemispheric tendency

**Nereid-facing hemisphere**
- more oceanic;
- somewhat cooler midday climate;
- wetter/cloudier;
- extensive storm tracks;
- lower snow line in mountain belts;
- major tidal flats and estuaries are welcome.

**Anti-Nereid hemisphere**
- larger old continental landmass;
- broader plateaus;
- stronger hot-day/cold-night contrast;
- more deserts, salt basins and canyon country;
- still contains rivers/coasts where moisture permits.

**Leading/trailing transition longitudes**
- good places for mountain arcs;
- active weather boundaries;
- island arcs / volcanic chains.

This is a design bias, not a hard rule.

---

## 9. Surface / geology language

Thessa must remain recognizable when the global map is shrunk to 480×270.

The planet should contain several enormous features deliberately, not only noise.

Required macro-scale feature classes:
- ocean basins;
- old continents / cratons;
- mountain arcs;
- giant impact basins;
- large cratered provinces;
- volcanic plateaus;
- shield-volcano provinces;
- rift/canyon systems;
- large sedimentary plains;
- polar and high-altitude glaciation;
- archipelagos;
- tidal flats / shallow inland seas.

### Landmark targets

Generate at least:
- 1 giant ancient impact basin: `~1200–2000 km` class;
- 1 major mountain arc: `~1000–1800 km` long;
- 1 visually obvious rift/canyon system: `~600–1400 km`;
- 1 dark volcanic province: `~400–900 km`;
- 2–4 major shield/caldera complexes;
- 5–12 very large recognizable craters;
- 1 large dry plateau / salt-basin complex;
- 1 highly glaciated mountain/coastal region;
- multiple archipelagos.

Do not evenly distribute all features.

---

## 10. Relief

Low gravity permits dramatic relief, but atmosphere/water also drive erosion.

Suggested macro range:
- deep ocean trenches / basins: down to roughly `-8 km` datum;
- ordinary continents: `0–3 km`;
- major plateaus: `2–5 km`;
- big mountain systems: peaks commonly `5–8 km`;
- exceptional peaks can reach roughly `9–12 km`;
- giant canyons can be several kilometres deep.

Avoid covering the entire planet with 8–12 km mountains. Extreme relief should create landmarks.

---

## 11. Biome design

Do not make biome IDs a salt-and-pepper texture.

Use large coherent regions plus local sub-biomes.

Suggested primary biome families:

### Ocean / coast
- deep_ocean
- shallow_sea
- cold_ocean
- coastal_shelf
- tidal_flat
- rocky_coast
- beach
- archipelago

### Wet / temperate lowlands
- cool_maritime_plain
- temperate_grassland
- wetland
- river_delta
- temperate_forest
- cool_forest

### Dry interiors
- steppe
- cold_desert
- sand_desert
- stony_desert
- salt_flat
- dry_basin
- badlands

### Highlands
- rolling_highlands
- rocky_plateau
- alpine_meadow
- alpine_barren
- mountain_ridge
- escarpment
- canyon_province

### Cold
- seasonal_snow
- permanent_snow
- glacier
- ice_cap
- periglacial_barren

### Volcanic / geothermal
- basalt_plain
- volcanic_field
- fresh_lava
- caldera
- fumarole_field
- geothermal_wetland
- sulfur_field

### Impact
- crater_floor
- crater_rim
- ejecta_plain
- ancient_impact_basin
- cratered_highlands

A location may have separate:
- primary biome;
- geology;
- feature tags.

Example:

```text
biome = alpine_barren
geology = basaltic
features = [caldera, glacier_margin, fumaroles]
```

---

## 12. Generator philosophy

The generator should use explicit large features and causal masks.

Good:

```text
impact basin
  -> elevation depression + rim
  -> drainage changes
  -> geology exposure
  -> climate basin effect
  -> biome response
```

Bad:

```text
noise1 -> height
noise2 -> biome color
noise3 -> minerals
```

GPT Image may help with:
- macro art direction;
- albedo/color style;
- proposing broad region shapes.

It must not be trusted to independently generate seven pixel-perfect physical truth maps.
