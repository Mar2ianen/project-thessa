# Project Thessa — Interstellar Scope

> Working design target for the scale of the playable stellar neighborhood. This is deliberately separate from per-system celestial-body data in `02_WORLD_ATLAS.md`.

## 1. Playable volume

Current target: a **roughly 6–10 light-year radius** around the starting Asterion system.

The intent is not to fill a sphere uniformly. The useful content unit is a **stellar system / encounter region**, with large interstellar distances retaining real logistics cost.

This scale is chosen for gameplay built around very high-performance fusion propulsion in the broad **Epstein-class** design space:

- interplanetary travel inside a system should become routine before interstellar travel does;
- neighboring stars should be reachable without turning the game into a generation-ship simulator;
- interstellar routes should still be long enough to make propellant, maintenance, reliability, fleet scheduling and infrastructure matter;
- several distinct stellar architectures can exist without requiring hundreds of shallow procedural systems.

## 2. Content density target

The 6–10 ly radius is a **maximum useful neighborhood**, not a promise that every point contains authored content.

Preferred structure:

- Asterion as the starting triple system and densest early/mid-game environment;
- the ultra-wide Asterion companion at ~0.5–1.0 ly as the first expansion target;
- several nearby stellar systems with deliberately different formation histories;
- sparse minor stars / brown dwarfs / rogue bodies where they create useful routing or science gameplay;
- optional deep-space objects and infrastructure nodes between major systems;
- no requirement that every nearby star host an Earth-like world or a large planet family.

## 3. System differentiation rule

New stellar systems should not be reskins of Asterion.

Useful contrasts include:

- compact red-dwarf systems;
- old metal-poor systems;
- young active stars with debris disks;
- white-dwarf remnants;
- close binaries with only circumbinary planets;
- wide binaries whose planetary systems evolved mostly independently;
- systems with giant-planet migration / scattering signatures;
- systems dominated by small airless bodies;
- rogue planets or former system members on interstellar trajectories.

## 4. Travel-model requirement

The final radius should be locked only after the propulsion model gives a useful travel-time curve for representative craft classes.

Required benchmark table before canon lock:

- 0.1 ly;
- 0.5–1.0 ly first-expansion companion;
- 1 ly;
- 3 ly;
- 6 ly;
- 10 ly.

For each distance, benchmark at least:

- early fusion craft;
- mature high-performance torch drive;
- Epstein-class endgame craft;
- cargo-optimized low-acceleration freighter.

The benchmark must include acceleration limits, propellant fraction, waste heat and realistic flip/deceleration rather than only ideal constant-acceleration travel time.

The **0.5–1.0 ly far companion is intentionally a pre-Epstein target**: a mature non-Epstein fusion craft must be capable of reaching it in a strategically useful timescale, even if the trip is measured in decades.

## 5. Asterion formation implication

Working hypothesis: **Asterion B–C formed as the original close binary, while A joined later through an early dynamical interaction / capture-like event in a young stellar environment.**

This should be treated as a formation hypothesis until an N-body history is constructed that simultaneously preserves:

- the current ~45 AU A–BC hierarchy;
- the compact B–C binary;
- surviving planetary systems around A and around BC;
- plausible disk truncation / migration histories;
- the desired captured-body history of Thessa.

The hypothesis is valuable because it gives a physical reason for the A and BC planetary families to differ strongly in composition and architecture.

## 6. Ultra-wide Asterion companion — first expansion layer

A working additional component lies on an orbit of order **0.5–1.0 ly** around the Asterion ABC barycenter. Preferred identity: a **captured brown dwarf**; an ultra-faint late-M dwarf remains an alternate.

This object is intentionally much farther away than any ordinary Asterion planet, so the logistics transition is real:

- planetary-system craft cannot casually reach it;
- mature fusion ships can;
- Epstein-class propulsion is not required;
- the route introduces long-duration reliability, autonomous maintenance, closed-loop life support and fleet scheduling before true multi-light-year expansion.

The nominal working orbit is ~0.7 ly semimajor axis with enough eccentricity to range roughly between 0.5 and 0.9 ly. At this scale the orbital period is of order several million years, so the object is effectively stationary on gameplay timescales while still being only weakly bound dynamically.

Its detailed captured-body system and physical model live in `02C_FAR_COMPANION.md`.
