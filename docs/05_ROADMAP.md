# 05 — Roadmap: from equations to game

Status: dependency order, not a calendar (M0–M1/M3–M4/M6 partial prototypes;
M4 stdlib minimum partially shipped; M5 gravity-assist exact-revalidation
prototype exists, canonical ephemeris and UX still future).

This is a dependency order, not a calendar. A milestone is not complete until
its state contract, tests, error evidence, and benchmark exist.

## M0 — Numerical kernel — implemented prototype

- `SimTime`, frame-labelled `f64` state, and deterministic baked ephemerides;
- point-mass multi-body gravity and ordered batch evaluation;
- adaptive Dormand–Prince, velocity-Verlet, on-rails caches, gravity cohorts,
  and bounded affine propagation;
- atmosphere, panel aero, rigid-body flight, contacts, and SIMD helpers;
- system baker, validation harnesses, and physics regression suite.

Remaining M0 work includes higher-fidelity ephemeris fitting, body harmonics,
hyperbolic/parabolic segments, and broader reference-vector coverage.

## M1 — Controllable vehicle and flight lab — partial/implemented prototype

Implemented: serializable vehicle definitions, 6-DoF starter vehicle, control
surfaces, actuator dynamics, RCS/propulsion demand, authority runtime, Bevy
pilot HUD, server snapshots, reset path, and flight traces.

Remaining: complete staging, richer propulsion catalogs, full contact/wheels,
vehicle editor, and production asset workflow.

## M2 — Aero, spaceplane, thermal, and structure — partial

Implemented: local panel aero, atmosphere rotation, stall/transonic/supersonic
reduced-order branches, coefficient tables, control laws, and actuator limits.

Remaining:

- expanded wing/flap/spoiler/grid-fin geometry;
- wake/occlusion compiler;
- structural graph and fracture into multiple bodies;
- thermal graph, entry heating, and material strength coupling;
- water contact and buoyancy;
- high-fidelity offline reference tables.

## M3 — Thessa surface slice — partial

Implemented: rocky world generator, deterministic geology/climate/landmark
fields, client texture export, terrain streaming, obstacle reports, pilot
render origin, atmosphere visuals, basic water raster effects, and
authoritative streamed terrain contact via `thessa-collision`
(Rapier: static trimesh, kinematic terrain, fixed joints, contact
activation hysteresis, load evidence).

Remaining: player movement, resource nodes, construction, power,
storage, save/load, and a first factory loop.

## M4 — Surface logistics and automation — partial

Implemented: typed event-driven graph IR, sequence/parallel/wait/failure paths,
server-owned continuations, QuickJS sandbox, typed guidance, typed maneuver
plans, server execution, and obstacle/site declarations.

Remaining: trucks/trains/aircraft logistics, physical stations and cargo,
rest of the guidance standard library past the shipped `Ascent`/`LandAt`/
`ExecuteManeuver`/`Rendezvous`-approach minimum, reusable route certification,
alarms, resource events, and factory integration.

## M5 — Nereid system gameplay — partial prototype

Implemented: broad chain/flyby survey, B-plane targeting, variational
midcourse correction, and L0–L2 mission-replay fixtures in CI. Remaining:

- canonical ephemeris version and long-horizon system validation;
- system map and transfer-window UX;
- orbital depots, resource differentiation, and reusable routes;
- remaining gravity-assist UX (planner core exists; map/window/assist UX future);
- eclipse/planetshine gameplay and additional moon content.

## M6 — Production multiplayer — partial foundation

The authoritative server, validated commands/snapshots, TCP/stdio transport,
and shared warp vote policy exist as a prototype. Future work includes:

- authentication and permissions;
- prediction/interpolation for remote craft;
- interest management and fleet replication;
- persistent server saves;
- transport/replication decision and packaging;
- native cross-platform and WASM/WebGPU smoke coverage.

## M7 — Nuclear age — future

Fission power, nuclear thermal and electric propulsion, radiators, cryogenics,
maintenance, and the Orthea/Vesper content layer.

## M8 — Fusion industrialization — future

Isotope separation, breeding chains, pulsed fusion, D–He3, high-power thermal
systems, and late-game torch-class propulsion.

## M9 — BC endgame — future

Long-distance A–BC flight, Janus/Mora content, binary-star lighting and
eclipse gameplay, circumbinary planning, and long-haul automation.

## Do not build early

- full weather CFD or FEM;
- a planet-formation simulator or procedural galaxy;
- FTL;
- photorealistic rendering before the causal loop is proven;
- hundreds of raw resource types or a market simulator;
- complex NPC civilization systems.

First prove a mass-scale physical logistics loop with honest error bounds.
