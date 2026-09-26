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

Remaining M0 work includes higher-fidelity ephemeris fitting, harmonics
beyond degree 2 (J2/C22 evaluation landed, gated), hyperbolic/parabolic
segments, and broader reference-vector coverage.

## M1 — Controllable vehicle and flight lab — partial/implemented prototype

Implemented: serializable vehicle definitions and procedural body baking, 6-DoF
starter vehicle, control surfaces and body strips, actuator dynamics,
RCS/reaction-wheel/propulsion demand, retractable wheel chassis and fold-out
legs, deployable parachutes, ordered vehicle-part commands, authority runtime,
Bevy pilot HUD, server snapshots, reset path, and flight traces.

Remaining: stage-definition resolution and action-group routing, per-engine
control allocation, wheel interactions with dynamic bodies, steering/anti-skid,
vehicle editor, and production asset workflow. The current articulated wheel
runtime, its boundaries and acceptance tests are documented in
[`details/05_PROCEDURAL_LANDING_GEAR.md`](details/05_PROCEDURAL_LANDING_GEAR.md).

## M2 — Aero, spaceplane, thermal, and structure — partial

Implemented: procedural wing/body compilation, local panel and fuselage-strip
aero, composition-aware bulk atmosphere properties, atmosphere rotation,
stall/transonic/supersonic reduced-order branches, coefficient tables, control
laws, and actuator limits.

Remaining:

- general flap/spoiler/grid-fin aerodynamic models beyond the current incidence
  controls and hinged fuselage-strip actuators;
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

## M7 — Nuclear age — future gameplay/content

Nuclear-thermal and electric propulsion models exist in the engineering
backend. Fission power, radiators, cryogenics, resource/thermal integration,
maintenance gameplay, and the Orthea/Vesper content layer remain future work.

## M8 — Fusion industrialization — future gameplay/content

Continuous and pulsed fusion propulsion models exist in the engineering
backend. Isotope separation, breeding chains, D–He3 resource chains, high-power
thermal systems, and late-game torch gameplay remain future work.

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
