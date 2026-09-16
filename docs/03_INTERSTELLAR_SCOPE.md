# Project Thessa — Interstellar Scope

> Working design target for the scale of the playable stellar neighborhood. This is deliberately separate from per-system celestial-body data in `02_WORLD_ATLAS.md`.

## 1. Playable volume

Current target: a **roughly 6–10 light-year radius** around the starting Asterion system.

The neighborhood is intentionally a **stellar cloud / overdense local association**, not a Solar-neighborhood clone with multi-light-year empty gaps between every useful target.

The exact stellar number density, velocity dispersion and age remain open and must be locked together because they control:

- encounter frequency;
- survival of wide binaries and outer planetary systems;
- how often rogue planets / brown dwarfs pass within sub-light-year distances;
- long-horizon navigation and future system geometry;
- whether apparently nearby targets remain nearby over centuries or millennia.

The useful content unit is a **stellar system / encounter region**, with enough nearby structure that interstellar expansion begins before Epstein-class propulsion but still retains real logistics cost.

---

## 2. Content-density target

Preferred structure:

- Asterion as the starting triple system and densest early/mid-game planetary environment;
- at least one independent brown dwarf / rogue substellar system currently passing within ~0.5–1.0 ly as the first off-system expansion target;
- several stellar systems inside the 6–10 ly playable radius, deliberately closer on average than the real Solar neighborhood;
- dynamically meaningful flybys, weak associations and former encounter partners;
- rogue planets, brown dwarfs and stripped bodies where they add routing, science or resource gameplay;
- no assumption that every nearby object is gravitationally bound to Asterion.

Do **not** solve content density by inventing ultra-wide companions at tens of thousands of AU. In a dense cloud those are dynamically soft and should normally be treated as independent moving objects instead.

---

## 3. Dynamical-environment rule

The stellar cloud is part of the simulation / authored ephemeris model.

Long-horizon design must account for:

- relative stellar velocities;
- historical and future close approaches;
- perturbation / stripping of very wide companions;
- exchange interactions and capture events where appropriate;
- secular evolution of outer comet / debris reservoirs;
- moving interstellar destinations rather than fixed points on a static galaxy map.

Working qualitative rule:

- ordinary planetary systems: robust;
- companions at tens to hundreds of AU: common design space;
- ~10^3 AU-scale companions: environment-dependent and validation-sensitive;
- >10^4 AU pairs: do not assume long-term survival in the intended cluster-like environment.

Exact limits come from the final cloud density and velocity dispersion rather than a universal hard cutoff.

---

## 4. System differentiation rule

New stellar systems should not be reskins of Asterion.

Useful contrasts include:

- compact red-dwarf systems;
- old metal-poor systems;
- young active stars with debris disks;
- white-dwarf remnants;
- close binaries with circumbinary planets;
- dynamically processed systems with missing outer planets;
- systems showing giant-planet migration / scattering;
- systems dominated by small airless bodies;
- free brown dwarfs and rogue planets;
- systems currently undergoing or recovering from close stellar encounters.

---

## 5. Travel-model requirement

The final playable radius should be locked only after propulsion models produce useful travel-time curves for representative craft classes.

Required benchmark separations / encounters:

- 0.05–0.1 ly;
- 0.5–1.0 ly first off-system brown-dwarf encounter;
- 1 ly;
- 3 ly;
- 6 ly;
- 10 ly.

For each, benchmark at least:

- early fusion craft;
- mature high-performance fusion / torch drive;
- Epstein-class endgame craft;
- cargo-optimized low-acceleration freighter.

Benchmarks must include:

- actual relative target velocity;
- finite acceleration / thrust power;
- propellant fraction;
- waste heat;
- flip / deceleration;
- coasting where appropriate;
- changing arrival geometry.

Scalar `distance / cruise_speed` is not sufficient for moving targets in the cloud.

---

## 6. First expansion layer — independent brown-dwarf encounter

The current preferred first interstellar target is an **independent brown dwarf system** passing of order **0.5–1.0 ly** from Asterion.

It is not on a multi-ten-thousand-AU Asterion orbit.

Gameplay role:

- first destination genuinely outside the Asterion hierarchy;
- reachable by mature conventional fusion propulsion before Epstein-class drives;
- forces autonomous industry, reliability and moving-target trajectory planning;
- gives the player a stepping stone into the cloud without requiring a 4+ ly Solar-neighborhood-style first jump;
- can have its own compact moon / captured-body system while remaining dynamically independent.

Detailed body design lives in `02C_FAR_COMPANION.md` even though the historical filename still says `FAR_COMPANION`; the document itself now treats it as an interloper / independent cloud member.

---

## 7. Asterion formation implication

Working hypothesis: **Asterion B–C formed as the original close binary, while A joined later through an early dynamical interaction / capture-like event in the same broader stellar environment.**

This is more natural in the current cloud concept than in an isolated field environment, but still requires an explicit N-body history that preserves:

- the current ~45 AU A–BC hierarchy;
- the compact B–C binary;
- surviving planetary systems around A and around BC;
- plausible disk truncation / migration histories;
- the desired captured-body history of Thessa.

The same cloud that makes Asterion's assembly plausible must also be reflected in nearby systems: close encounters and stripped / exchanged bodies should leave visible architectural fingerprints.

---

## 8. Canon-lock questions

1. choose stellar-cloud number / mass density;
2. choose velocity dispersion and age;
3. generate a local 6–10 ly phase-space realization rather than only static star positions;
4. verify the survival of Asterion and other wide multiples inside that realization;
5. pick the brown-dwarf first-expansion encounter from the same realization;
6. benchmark travel against actual state vectors;
7. confirm that system density gives interesting routing without turning the sky into an unrealistically packed globular-cluster core;
8. lock the playable radius only after propulsion and cloud dynamics work together.
