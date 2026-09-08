# 03 — Физический движок

## 3.1. Цель

Не «максимально точный aerospace FEM/CFD package», а единая физическая модель, в которой из тех же primitive systems естественно получаются:

- обычная ракета;
- многоступенчатый reusable launcher;
- Starship-like belly-flop vehicle;
- spaceplane / lifting body;
- самолёт;
- ground vehicle;
- орбитальная станция;
- weird player-built craft.

Движок не должен заранее знать классы `rocket` / `plane` / `spaceplane`. Он знает массы, геометрию, поля, материалы, шарниры, жидкости и исполнительные органы.

---

## 3.2. Уровни state

### Celestial state

Canonical body state приходит из baked ephemeris:

```rust
BodyState {
    position_inertial: DVec3,
    velocity_inertial: DVec3,
    orientation: DQuat,
    angular_velocity: DVec3,
}
```

### Vehicle cluster state

Минимум:

```rust
RigidClusterState {
    position: DVec3,
    velocity: DVec3,
    orientation: DQuat,
    angular_velocity: DVec3,
    mass: f64,
    center_of_mass_local: DVec3,
    inertia_body: DMat3,
}
```

`f64` authoritative. Render transform строится относительно локального/floating origin и переводится в `f32` только на стороне renderer.

---

## 3.3. Frames / координаты

Поддержать явные frames:

- global inertial / system barycentric;
- star-centric;
- planet/moon-centered inertial;
- body-fixed rotating;
- local tangent ENU/NED-like;
- vehicle body;
- render-local floating origin.

Frame transform — first-class API, не разбросанные `position -= planet_pos` по коду.

Для атмосферного полёта `AtmosphereConfig` выдаёт детерминированные
`temperature/pressure/density/speed_of_sound/viscosity` по высоте; его sample
конвертируется в `AeroEnvironment` в vehicle body frame. Это позволяет
считать один и тот же aircraft state на sea level, в стратосфере и в
сверхзвуковом flight corridor без ручного пересчёта `Mach`.

---

## 3.4. Гравитация

Для vehicle/test-particle:

\[
\mathbf a = \sum_i \mathbf a_i + \mathbf a_{harmonics} + \mathbf a_{other}
\]

Point-mass term:

\[
\mathbf a_i = \mu_i\frac{\mathbf r_i-\mathbf r}{|\mathbf r_i-\mathbf r|^3}
\]

### Requirements

- все релевантные stars/planets/moons могут действовать одновременно;
- ship-ship gravity по умолчанию не считается;
- SOI — только UI/optimization concept;
- contribution culling допускается по bounded error threshold;
- J2 обязателен для тел, где он meaningful;
- API позволяет J3/J4 и general spherical harmonics;
- body-fixed harmonic coefficients rotating with body.

### Что это даёт

Без специального gameplay-кода появляются:

- L1–L5 regions;
- halo/Lissajous-like trajectories;
- nodal/apsidal precession;
- sun-synchronous orbits;
- frozen/unstable low orbits;
- multi-moon gravity-assist routes.

---

## 3.5. Celestial ephemerides

Runtime body motion не интегрируется.

### Offline pipeline

```text
design orbital targets
→ high-accuracy n-body integration
→ resonance capture/relaxation if needed
→ long-horizon validation
→ fit deterministic ephemeris representation
→ versioned game data
```

Возможные representations:

1. Kepler elements + periodic/resonant corrections;
2. Chebyshev polynomial segments;
3. Hermite/spline state segments с bounded error.

Критерий — deterministic cross-platform evaluation и быстрый random access по `SimTime`.

---

## 3.6. Integrators

Один integrator на всё — плохая цель.

### Free flight / orbital

Нужен adaptive high-order ODE path. Кандидаты для собственного/референсного решения:

- Dormand–Prince / DOP853-like;
- adaptive RK для thrust/aero transition;
- symplectic mode для долгих conservative coast tests, если даёт пользу.

### Atmosphere / active control

- bounded adaptive step;
- guidance/control evaluation rate может отличаться от integration substeps;
- high-rate near max-Q, rapid attitude changes, landing.

### Contact regime

- отдельный local contact solver fixed/substepped;
- не заставлять orbital integrator решать колёса и constraints.

---

## 3.7. Vehicle design compilation

Параметрический design компилируется в несколько независимых representations:

```text
VehicleDesign
├── RenderMeshSet
├── CollisionModel
├── AeroModel
├── StructuralGraph
├── ThermalGraph
├── ActuatorGraph
├── FluidGraph
└── MassModel
```

Это ключевая optimization boundary: свобода редактора не равна количеству rigid bodies.

---

## 3.8. Structural model

### Базовая модель

Graph/beam network:

```text
node: structural station / mass attachment / hinge / engine mount
edge: beam/shell connection with stiffness + strength + thermal state
```

Считаемые quantities по edge/node по мере fidelity:

- axial load;
- shear;
- bending moment;
- torsion;
- elastic deflection;
- temperature-dependent stiffness/yield;
- accumulated fatigue/damage.

### Rigid cluster optimization

Пока connected structure ведёт себя близко к rigid body, translational/rotational integration идёт одним cluster state. Structural solver всё равно знает внутренние loads.

При failure:

```text
edge breaks
→ connected-components(structural graph)
→ recompute mass/CoM/inertia/aero/thermal connectivity
→ create N rigid clusters
```

Это сохраняет реальные разрушения без KSP-like `hundreds of rigid bodies + joints` в штатном полёте.

### Aeroelasticity

Для больших wings/flaps/long bodies structural deformation должен менять aero geometry хотя бы на reduced-order level:

`aero load -> beam twist/bend -> local AoA -> aero load`.

Полный FEM не требуется для baseline.

---

## 3.9. Articulated systems / hinges

Не все control surfaces обязаны быть отдельными generic rigid bodies.

`HingeDOF` содержит:

- axis;
- angle/limits;
- angular rate;
- actuator torque/speed curve;
- inertia of moving element;
- friction/backlash optional;
- thermal state;
- structural attachment.

Для large body flaps moving mass/CoM/inertia учитывается. Аэродинамический hinge moment может физически не дать actuator достичь commanded angle.

---

## 3.10. Aerodynamics

### Panel/zone model

Каждой aero zone известны:

- position/orientation;
- area/reference dimensions;
- local normal/tangent;
- shape/profile class;
- control deflection;
- material/surface temperature;
- coefficient model.

Локальная скорость:

\[
\mathbf v_{local}=\mathbf v_{vehicle}-\mathbf v_{air}+\boldsymbol\omega\times\mathbf r
\]

Динамическое давление:

\[
q=\frac{1}{2}\rho |\mathbf v_{local}|^2
\]

Сила каждой zone вычисляется локально и суммируется в force + moment about CoM.

### Coefficient domains

Baseline tables/functions должны допускать зависимости минимум от:

- angle of attack;
- sideslip;
- Mach;
- Reynolds where justified;
- control deflection;
- local flow exposure.

Нужна post-stall модель и transonic/supersonic behavior. Именно поэтому готовый `avian_fdm` нельзя принять как full solution v0.1: его текущий documented scope исключает compressibility/supersonic и aeroelasticity.

Первый realtime slice реализован в `thessa-sim-core::PanelAeroModel`: локальные
панели, `omega x r`, wind, dynamic pressure, Reynolds diagnostic, smooth
post-stall, transonic drag rise, supersonic trend и optional Mach/AoA
coefficient table. Он подключён к `evaluate_flight_forces` и
`integrate_rigid_body_step`/`integrate_rigid_body_duration`: translation,
quaternion attitude, gravity, rotating atmosphere и dynamic p/q/r damping
считаются в одном детерминированном 6-DoF state path. Полный контракт и
fidelity tiers зафиксированы в `docs/11_AERODYNAMICS.md`; внешние solvers
остаются validation-only.

### Occlusion / wake

Не CFD. Baseline:

- CPU BVH ray/cone queries against physics geometry;
- exposure estimate по incoming-flow directions;
- simple wake attenuation/deflection model;
- optional higher-fidelity panel interaction later.

Это позволяет не давать full aerodynamic force поверхности, закрытой корпусом.

Для finite-planform surfaces зона хранит не только площадь и chord, но также
фактический span, effective aspect ratio, sweep, body-interference factor и
center-of-pressure point. `PanelAeroModel` применяет Diederich correction к
2-D compressible lift slope перед расчётом силы; момент берётся относительно
center of pressure, а не автоматически относительно начала зоны. Это особенно
важно для низкоaspectных ракетных плавников: применение одного 2-D slope к
каждой панели завышает `CL_alpha` примерно вдвое.

---

## 3.11. Atmosphere

Каждое тело может задавать profile/model:

- density;
- pressure;
- temperature;
- composition;
- viscosity;
- speed of sound;
- wind/rotation;
- weather field optional.

Atmosphere вращается с телом, если design не задаёт другое. В runtime это
учитывается как `v_air = v_wind + ω_body × r_body`; поэтому relative air
velocity не равна inertial velocity.

### Weather fidelity

MVP: deterministic vertical profile + simple wind layers.

Later:

- large-scale weather cells;
- storms;
- spatial density/temperature variation;
- procedural but server-deterministic weather seeds.

---

## 3.12. Thermal model

Lumped thermal network:

\[
C_i\frac{dT_i}{dt}=\sum_j G_{ij}(T_j-T_i)+Q_{internal}+Q_{aero}+Q_{solar}+Q_{engine}-Q_{rad}
\]

Radiation:

\[
Q_{rad}=\epsilon\sigma A(T^4-T_{env}^4)
\]

### Thermal nodes

Автоматически генерируются из параметрической geometry:

- windward skin;
- leeward skin;
- internal structure;
- tank wall/contents;
- engine chamber/nozzle/mount;
- radiator surfaces;
- electronics/batteries where needed.

### Coupling

Температура влияет на:

- material strength/stiffness;
- actuator limits;
- tank pressure/boil-off;
- engine limits;
- battery/electronics capability;
- ablation/heat shield state.

---

## 3.13. Physics ray queries / radiation view factors

Hardware RT не является обязательным.

### Canonical server

CPU BVH / spatial acceleration structure для:

- aero occlusion;
- solar/stellar occlusion;
- thermal radiation visibility;
- plume impingement;
- lidar/radar sensors;
- terrain clearance.

### Optional GPU/native acceleration

На поддерживаемом native hardware можно использовать GPU ray tracing/compute для larger sample counts, preview/debug или local single-player acceleration, но authoritative server должен иметь CPU path и не требовать GPU.

Это важно и для dedicated server, и для deterministic behavior между AMD/Intel/NVIDIA/WebGPU clients.

---

## 3.14. Propulsion

Engine model отдаёт физические outputs:

- force vector;
- mass flow;
- heat flow;
- electrical/reactor demand;
- plume geometry;
- failure/limits state.

### Chemical

Nozzle/chamber/propellant parameterization; ambient pressure влияет на performance.

### Nuclear thermal

Reactor thermal power + propellant flow + nozzle; reactor heat/shielding/radiators matter.

### Electric/ion

Power-limited thrust; efficiency; propellant; radiator/power source coupling.

### Fusion

Pellet/injection/ignition/magnetic nozzle abstraction. Frequency/energy flow drive average thrust, но system remains parameterized machinery, not magic `TorchEngine` block.

---

## 3.15. Fluids / tanks

MVP:

- tank volume;
- propellant mass/density;
- pressure;
- temperature;
- outlets/feeds;
- boil-off;
- CoM changes with contents.

Later:

- reduced-order slosh model;
- ullage;
- pump/cavitation edge cases;
- cryogenic stratification if gameplay justifies.

Не делать full CFD tanks.

---

## 3.16. Control / FBW

Physics layer знает actuators. Guidance/FBW layer знает desired behavior.

Control allocator решает приближённую задачу:

\[
B(u)\approx \tau_{desired}
\]

с ограничениями:

- actuator limits;
- rates;
- hinge torque;
- fuel/energy cost;
- damaged/unavailable actuators.

Использовать numerical allocation / pseudo-inverse / constrained solve where useful. Никакого прямого `apply desired torque` кроме debug/test harness.

---

## 3.17. Contacts / wheels / debris

Вот здесь готовый rigid-body/collision engine полезен.

Нужны:

- landing gear contacts;
- wheels;
- player/world collisions;
- factory vehicles;
- docking hard-contact;
- debris;
- wreckage;
- local terrain contact.

Не использовать contact engine как источник orbital gravity/aero/thermal physics.

---

## 3.18. Что можно взять готовым

### Bevy 0.19.x — **да, shell**

Использовать:

- renderer/wgpu;
- assets;
- window/input;
- client ECS;
- UI/tooling;
- gizmos;
- Web/WASM build path.

Не использовать `Transform` как authoritative universe coordinates.

### Rayon — **да**

Fixed-size/data-parallel CPU work:

- gravity batches;
- aero zones;
- thermal edges;
- structural batches;
- trajectory candidate evaluation.

### Tokio — **да, но не для physics hot loop**

- networking;
- async persistence;
- metrics/admin APIs;
- channels/orchestration.

Tokio documentation сама рекомендует separate CPU-bound pool вроде Rayon для большого compute workload.

### Parry (`parry3d-f64`) — **сильный кандидат**

Геометрические queries/collision primitives, BVH-related spatial queries, mass properties и f64 path можно переиспользовать без принятия всего rigid-body engine.

### Avian 0.7 — **кандидат для local contact prototype**

Плюсы:

- Bevy 0.19 integration;
- f64 mode;
- collision/CCD/joints;
- modular ECS model.

Риск: не позволять Avian architecture вытечь в `sim-core`. Делать adapter/local physics bubble.

### `avian_fdm 0.2` — **reference/spike, не baseline dependency**

Очень близкая идея zone-based FDM: forces/moments на zones, Avian integration, damage effects. Но current scope не покрывает наши критичные вещи: supersonic/compressibility, aeroelasticity, fuel burn, autopilot, physical detachment. Лицензия LGPL-3.0-or-later требует отдельного решения.

### `nyx-space` — **validation/reference only по умолчанию**

Имеет multibody dynamics, spherical harmonics, finite burns, eclipse/visibility и high-fidelity orbit propagation. Очень полезен для cross-check numerical cases и offline tooling concepts. Но current core license — AGPLv3, поэтому нельзя тихо добавить как обычную dependency, если проект не принимает AGPL.

### Lightyear — **networking candidate**

Под Bevy 0.19 есть server-authoritative networking, prediction, interpolation, interest management и WASM/WebTransport support. Проверить на prototype перед hard commit.

### Bevy/wgpu — **render/compute adapter, не physical semantics**

Используем кроссплатформенный слой Bevy/wgpu. Физический движок не знает DirectX/DXR/Vulkan/Metal. GPU acceleration для ray queries/compute — optional accelerator через adapter; canonical server path остаётся CPU/BVH. Это позволяет Linux/Vulkan, macOS/Metal, Windows backend и WebGPU клиенту использовать один simulation contract.

### Bevy Tasks — **локально полезны**

- `ComputeTaskPool`: frame-bound client compute;
- `AsyncComputeTaskPool`: mesh/design compilation, preview trajectories, background client tasks.

Не делать их обязательной dependency simulation core.

---

## 3.19. SIMD / build targets

x86_64 proposal:

```text
native-avx2    baseline release
native-avx512  separate optimized release
```

Причина отдельного binary target: LLVM видит ISA globally и может auto-vectorize/inlining без runtime target-feature boundaries.

Hot kernels проектировать data-oriented:

```text
position_x[]
position_y[]
position_z[]
velocity_x[]
...
```

или AoSoA, если это лучше cache/locality.

AVX-512 — optimization, не semantic difference.

---

## 3.20. Simulation fidelity / rate

Разные systems не обязаны работать на одной частоте.

Пример baseline:

| System | Typical policy |
|---|---|
| free orbital coast | adaptive, large steps |
| powered vacuum flight | adaptive medium/high |
| atmospheric 6-DoF | 20–100+ Hz equivalent + substeps |
| active structural solve | coupled/adaptive |
| contacts | fixed/substepped local |
| factory | event/low-rate deterministic |
| script VM | event-driven |
| guidance | configurable 5–50+ Hz |
| rendering | independent client frame rate |

Warp multiplies simulated time, но fidelity переключается по error/active regime, а не по расстоянию от камеры.

---

## 3.21. Determinism

Цель — deterministic enough для authoritative server/replay, не обязательно bit-identical cross-ISA в каждом floating operation на первом prototype.

Обязательные шаги:

- deterministic event ordering;
- stable IDs;
- no hash iteration dependency;
- explicit random seeds;
- fixed content ephemerides;
- canonical server result;
- snapshot/replay regression tests.

Если AVX2/AVX-512 дают небольшие float divergence, клиентский prediction обязан уметь correction. Server build выбирает один target.

---

## 3.22. Validation suite

Минимум до gameplay production:

### Gravity/orbit

- two-body ellipse conservation;
- hyperbolic escape;
- restricted three-body Lagrange equilibrium checks;
- known J2 nodal precession;
- sun-synchronous target case;
- gravity-assist energy/frame sanity.

### 6-DoF

- torque-free rigid rotation;
- constant off-axis force;
- actuator reaction torque;
- changing mass/CoM.

### Aero

- symmetric craft zero side force at beta=0;
- lift/drag reference curves;
- stall;
- control surface sign/authority;
- occlusion test;
- belly-flop qualitative benchmark;
- lifting-body/spaceplane benchmark.

### Thermal

- two-node conduction analytic case;
- black-body cooling;
- equilibrium solar/radiative case;
- heat-shield/ablation regression.

### Structure

- cantilever beam reduced-order check;
- temperature-dependent failure;
- split into connected components conserves mass/momentum.

### Performance

Bench target batches:

- 1k / 5k / 10k vehicle gravity states;
- 100k / 1M aero zones;
- thermal graphs;
- contact active set;
- warp stress scenarios.
