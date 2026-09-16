# Project Thessa — Far Asterion Companion

> Working design document for the ultra-wide substellar companion / first-expansion target. The object is intentionally much farther from the ABC hierarchy than any ordinary planet: of order **0.5–1.0 light-year** from the Asterion barycenter.

## 1. Core concept

The preferred concept is not an ordinary fourth star formed quietly with Asterion. It is a **former flyby object that became only weakly bound** during the early dynamical history of the system or the dissolution of the birth cluster.

Working preference: a **brown dwarf** rather than an ordinary red dwarf.

Reasons:

- it makes the target visually and thermally distinct from a normal stellar system;
- it is luminous enough in the infrared to shape very close satellites while remaining extremely faint at interstellar distances;
- it can plausibly carry a compact satellite system even though it lies far outside the ordinary planetary architecture of Asterion;
- its weak binding to ABC makes the system feel like the first step into interstellar space rather than merely another outer planet;
- it is a natural first expansion target before true multi-light-year travel.

An ultra-faint late-M red dwarf remains an alternate option if later gameplay requires more local stellar power.

---

## 2. Orbit around Asterion ABC

Working wide orbit:

| Parameter | Working target |
|---|---:|
| host | Asterion ABC barycenter |
| object class | captured brown dwarf / ultra-wide companion |
| mass | ~25–45 MJ |
| radius | ~1 RJ |
| semi-major axis | ~0.7 ly (~44,000 AU) |
| eccentricity | ~0.3 |
| periapsis | ~0.49 ly |
| apoapsis | ~0.91 ly |
| orbital period | ~5 Myr order of magnitude |

This orbit is deliberately fragile on Galactic timescales. It should be treated as a weakly bound outer companion whose orbit can be perturbed by stellar encounters and the Galactic tide.

At 0.5–1 ly the ABC stars are visually a bright compact group but provide negligible thermal input to local worlds. The brown dwarf's own infrared luminosity dominates the environment of its close satellites.

---

## 3. Brown dwarf physical target

Working range rather than hard canon:

- mass: ~25–45 MJ;
- radius: near Jupiter-sized;
- effective temperature: roughly late-L / T-dwarf regime depending age and final mass;
- spectrum: strongly infrared, methane / water absorption important if cool enough;
- no fusion-like stellar photosphere; luminosity is residual cooling plus contraction;
- strong magnetic / auroral activity is allowed if supported by the final rotation and atmospheric model.

The object should be visually almost invisible at normal human-visible-light exposure except for reflected/scattered light from nearby infrastructure and occasional auroral emission. In thermal / IR views it becomes the dominant object in the sky.

---

## 4. Origin hypothesis

Preferred history:

1. the brown dwarf formed independently, probably as a low-mass member of the same young stellar environment or a nearby one;
2. it carried a compact primordial satellite / debris system;
3. a low-velocity encounter with the evolving Asterion multiple system or cluster potential placed it onto a very wide weakly bound orbit;
4. the capture event stripped its most distant original companions and disturbed the survivors;
5. subsequent encounters allowed it to retain a mixture of **primordial close satellites, captured minor worlds, and collision fragments**.

This mixed origin is important: the local bodies should not look like a clean regular moon chain.

---

## 5. Satellite-system design rule

The far companion is a good place for bodies that would be awkward elsewhere because almost every surviving object can have a capture / scattering history.

The system should contain only a handful of authored major bodies, each with a different dynamical signature:

- one close, heavily tidally processed rocky body;
- one large volatile-rich captured dwarf world with active internal heating;
- one irregular binary/contact object;
- one distant weakly bound captured body on a strongly inclined or retrograde orbit;
- optional debris arcs / dust torus from stripped former satellites.

Avoid a neat Galilean-like resonance chain. The visual language should be **survivors of capture**, not orderly in-situ formation.

---

## 6. Candidate major bodies

Names are intentionally not locked.

### 6.1. Close refractory moon

Working identity: dense rocky/metal-rich satellite on a close orbit, tidally locked to the brown dwarf.

Suggested scale:

- radius: ~500–900 km;
- atmosphere: none;
- surface: dark refractory plains, metal-rich scarps, old melt provinces;
- heating: brown-dwarf infrared irradiation plus tidal dissipation;
- resource role: refractory metals and compact early outpost.

This body should feel warm despite being nearly a light-year from the stars.

### 6.2. Large captured volatile world

Working identity: former free minor planet / dwarf planet captured by the brown dwarf.

Suggested scale:

- radius: ~1500–2300 km;
- orbit: eccentric and moderately inclined;
- bulk composition: ice + rock, differentiated;
- atmosphere: tenuous N2/CH4/CO/Ar family, strongly seasonal or partly collapsed depending final thermal model;
- interior: subsurface ocean or deep brine layer maintained primarily by tidal heating;
- biomes: ancient dark ice, young cryovolcanic resurfacing, fracture provinces, evaporite / salt-rich regions around old vent systems.

This is the natural science centerpiece of the first expansion target.

### 6.3. Irregular binary / contact body

Working identity: two captured rubble piles or a contact binary surviving from a disrupted outer population.

Suggested scale:

- characteristic dimensions: tens to low hundreds of km;
- strongly irregular shapes;
- complex spin state;
- low density / high porosity;
- compositionally heterogeneous surfaces.

Gameplay: low-gravity construction, mining and navigation around a genuinely non-spherical multi-lobed body.

### 6.4. Distant retrograde survivor

Working identity: very loosely bound captured body on a high-inclination or retrograde orbit.

Suggested scale:

- radius: ~200–600 km;
- distance from brown dwarf: many AU to tens of AU, final value from stability integration;
- volatile-rich or carbon-rich composition;
- no atmosphere or only transient sublimation exosphere;
- potentially the remnant of the population stripped during brown-dwarf capture.

This should be dynamically validation-sensitive and may be omitted if long-horizon integrations reject it.

---

## 7. Gameplay role — first expansion target

This object is intentionally positioned between ordinary interplanetary and full interstellar gameplay.

At **0.5–1.0 ly** it is:

- far outside the Asterion planetary system in every practical logistics sense;
- reachable with mature conventional fusion propulsion before Epstein-class drives are available;
- close enough that the first off-system expedition can still be built around decades rather than centuries;
- a natural location for the first autonomous industrial colony, fusion-fuel depot, observatory and interstellar shipyard;
- an ideal proving ground for reliability, closed-loop life support, long-duration reactor operation and high-velocity navigation.

It should not be merely a stepping stone. Its captured-body system needs enough unique geology and resources to justify permanent settlement.

---

## 8. Travel-design target

Do not hard-code travel time until engine models are final.

For scale only, an ideal symmetric acceleration/deceleration trajectory over ~0.7 ly gives approximate Newtonian travel times:

| constant acceleration | ideal travel time | midpoint speed |
|---:|---:|---:|
| 0.001 g | ~52 y | ~0.027 c |
| 0.003 g | ~30 y | ~0.047 c |
| 0.01 g | ~16.5 y | ~0.085 c |

Real fusion craft will generally coast, carry finite reaction mass and operate below those ideal duty cycles, so these are **scale references only**, not balance targets.

The first-expansion propulsion benchmark should therefore explicitly include a non-Epstein fusion ship capable of reaching this companion in a useful gameplay timescale.

---

## 9. Canon-lock questions

Before locking the object:

1. choose brown dwarf vs ultra-faint late-M dwarf;
2. choose mass/age/Teff from one consistent substellar evolution grid;
3. integrate the ~0.5–1 ly outer orbit under ABC gravity plus Galactic tide / stellar-encounter assumptions;
4. determine which local captured satellites survive the original capture event;
5. build a local irradiation model using the brown dwarf's evolving IR luminosity;
6. define the first-generation fusion transport envelope and confirm that this target is reachable before Epstein-class propulsion;
7. decide whether the far companion is permanently bound or on a metastable orbit likely to escape on Gyr timescales.
