# Project Thessa — Nearby Brown-Dwarf Interloper

Status: design target. Deliberately unimplemented: unbound cloud member,
not baked (`data/system.toml` carries only a comment).

> Working design document for the first true off-system expansion target. The object is **not an ultra-wide Asterion companion**. It is a separate substellar system moving through the same dense stellar cloud, currently passing of order **0.5–1.0 light-year** from Asterion.

## 1. Core concept

Preferred identity: a **free / cluster-member brown dwarf** rather than an ordinary red dwarf.

The important distinction is dynamical:

- it does not orbit the Asterion ABC barycenter;
- its current proximity is set by the phase-space structure of the local stellar cloud and its own trajectory;
- it can remain a nearby target for the game timescale without requiring an implausibly fragile ~30,000–60,000 AU binary;
- its local satellite system is bound to the brown dwarf itself.

This makes it the first real expansion beyond Asterion rather than an absurdly wide fourth component.

A very faint late-M dwarf remains an alternate if later gameplay needs more local stellar power, but a brown dwarf is preferred because it makes the environment visibly and thermally unlike an ordinary star system.

---

## 2. Current encounter geometry

Working target:

| Parameter | Working target |
|---|---:|
| relation to Asterion | unbound / independent cloud member |
| object class | brown dwarf system |
| mass | ~25–45 MJ |
| radius | ~1 RJ |
| present distance from Asterion | ~0.5–1.0 ly |
| preferred current distance | ~0.6–0.8 ly |
| relative velocity | TBD from stellar-cloud model |
| closest approach | TBD |
| encounter duration | long compared with normal campaign logistics, but not an orbital period |

Do **not** assign a Keplerian orbit around Asterion.

The final trajectory must be generated as part of the local stellar-cloud phase-space model. The object may be approaching, near closest passage, or receding at game epoch depending on what creates the best navigation and expansion gameplay.

At this distance the Asterion ABC stars are visually a compact bright group but thermally irrelevant to the local worlds. The brown dwarf's own infrared luminosity and internal heat dominate its close satellite environment.

---

## 3. Why the old ultra-wide orbit was rejected

The previous concept placed this object on a ~0.7 ly (~44,000 AU) orbit around Asterion. That does not fit the intended **dense stellar-cloud environment**.

Design rule going forward:

- objects at ~0.5–1 ly are independent cloud members / flyby systems unless a dedicated N-body model proves otherwise;
- any genuinely bound outer companion of Asterion must be dramatically closer and should be treated as outer-system content rather than the first interstellar expansion target;
- the local stellar environment must participate in long-horizon dynamics instead of assuming an isolated field-star system.

---

## 4. Brown dwarf physical target

Working range rather than hard canon:

- mass: ~25–45 MJ;
- radius: near Jupiter-sized;
- effective temperature: late-L / T-dwarf regime depending age and final mass;
- spectrum: strongly infrared, with H2O / CH4 absorption if cool enough;
- luminosity: residual cooling and contraction rather than sustained hydrogen fusion;
- strong magnetic / auroral activity is allowed if supported by the final rotation and atmosphere model.

The brown dwarf should be extremely faint visually at interstellar distance. In IR navigation / science views it becomes an obvious local primary.

---

## 5. Origin and local-system history

The brown dwarf formed independently of Asterion and belongs dynamically to the same broader stellar cloud.

Its own local bodies can have a mixed history:

- primordial close satellites formed in a circum-substellar disk;
- captured minor bodies acquired during earlier passages through the cloud;
- collision fragments from destabilized satellites;
- outer bodies stripped by old encounters.

The present system should therefore **not** look like a pristine Galilean resonance chain.

---

## 6. Satellite-system design rule

Keep only a handful of authored major bodies, each with a different dynamical and environmental signature:

- one close, heavily tidally processed rocky/refractory body;
- one large volatile-rich captured dwarf world with active internal heating;
- one irregular binary/contact object;
- optional debris arcs / dust torus from disrupted satellites;
- at most one weakly bound outer survivor, only if local stellar-cloud integrations show that it survives repeated encounters.

The outermost satellites must be validated against perturbations from passing stars. The cloud environment is part of the system design, not background decoration.

---

## 7. Candidate major bodies

Names are intentionally not locked.

### 7.1. Close refractory moon

- radius: ~500–900 km;
- atmosphere: none;
- dense rocky/metal-rich composition;
- tidally locked;
- dark refractory plains, metal-rich scarps, old melt provinces;
- heating from brown-dwarf IR plus tidal dissipation.

### 7.2. Large volatile world

- radius: ~1500–2300 km;
- eccentric / inclined orbit;
- differentiated ice + rock body;
- tenuous N2 / CH4 / CO / Ar atmosphere if the thermal model permits;
- subsurface ocean or brine layer maintained mainly by tidal heating;
- dark ancient ice, young cryovolcanic terrain, fractures and salt-rich deposits.

This is the main science / settlement world of the first expansion target.

### 7.3. Irregular binary or contact object

- dimensions: tens to low hundreds of km;
- strongly non-spherical;
- complex spin state;
- porous rubble-pile composition;
- heterogeneous surface materials.

Gameplay focus: navigation, anchoring, mining and construction in a genuinely non-spherical low-gravity environment.

### 7.4. Outer survivor — optional

If long-horizon cloud integrations permit it:

- radius: ~100–400 km;
- strongly inclined or retrograde orbit;
- volatile-rich or carbon-rich;
- may be a remnant of a once larger outer satellite population.

This body is **not canon until survival is demonstrated** under repeated stellar perturbations.

---

## 8. Gameplay role — first expansion target

The brown dwarf is the bridge between planetary and full interstellar gameplay.

At ~0.5–1.0 ly current separation it is:

- genuinely outside Asterion rather than an outer planet or companion;
- reachable with mature non-Epstein fusion propulsion;
- close enough for a first long-duration expedition without requiring the full endgame torch-drive economy;
- a natural first autonomous colony, fusion-fuel depot, observatory and interstellar shipyard;
- a practical tutorial for stellar-cloud navigation, moving targets and long-horizon trajectory planning.

Unlike a bound target, its ephemeris matters: launch windows and future closest-approach geometry can change strategic value over decades or centuries.

---

## 9. Travel-design target

Do not hard-code travel times until engine models are final.

Benchmark this target using its **actual relative state vector**, not only scalar distance.

Required propulsion cases:

- mature pulsed-fusion craft;
- high-Isp low-thrust cargo fusion craft;
- early torch drive;
- Epstein-class craft as a later comparison.

The design intent remains that the first expedition is possible before Epstein-class propulsion, while regular high-throughput logistics becomes much easier later.

---

## 10. Canon-lock questions

1. choose brown dwarf mass / age / Teff from one consistent evolution model;
2. define the stellar cloud's density, velocity dispersion and age;
3. generate a self-consistent current state vector and closest-approach history relative to Asterion;
4. integrate the brown dwarf's local satellite system under repeated stellar perturbations;
5. choose which outer satellites survive and which historical bodies were stripped;
6. build the local IR irradiation / magnetic-environment model;
7. benchmark non-Epstein travel to the moving target;
8. ensure the target remains nearby for a useful gameplay era without pretending it is permanently bound to Asterion.
