# 21 — Terrain streaming throughput and representation split

Status: design baseline; invariants normative. Throughput phasing is archival:
GPU-indexed CBT is implemented opt-in (`terrain=gpu_indexed`); the CPU baking
description below applies to the fallback path only.

Статус: **design target / performance follow-up**.

Цель этой доки — увеличить скорость перемещения, при которой realtime terrain успевает корректно прогружаться, не ухудшая continuity и не превращая `PlanetField` в набор заранее запечённых растров.

Текущий vertical slice уже показывает важный результат: полностью процедурный terrain, включая geometry, albedo, roughness, normal maps, mip generation и upload preparation, успевает за аппаратом примерно до порядка **3 km/s** на текущей машине. Для first implementation это хороший baseline. Проблема не в том, что cube-sphere или canonical field сами по себе медленные; проблема в том, что client streaming frontier сейчас выполняет слишком много работы, часть которой вообще не обязана быть CPU tile-build work.

Критичное уточнение: **render streaming, authoritative surface queries и contact representation — разные consumers одной canonical поверхности**. Dedicated/headless server не обязан иметь GPU или render mesh, но поверхность не исчезает без наблюдателя: она нужна автоматике, terrain avoidance, посадкам и contact dynamics.

Основная target-схема:

```text
                         PlanetField
                canonical observer-independent field
                              |
             +----------------+----------------+
             |                |                |
             v                v                v
      server/query path   contact path      client renderer
      height/slope/etc.   local CPU patch   visual geometry
      automation/GNC      wheels/legs/body  + material
             |                |                |
             |                |                +--> GPU material/microdetail
             |                |                `--> optional GPU CBT/tessellation
             |                |
             `----------------+--> authoritative physics

No camera is required for the left or middle branches.
```

Для клиента цель остаётся простой:

```text
client streaming frontier = geometry bandwidth problem,
not per-tile texture baking problem
```

## 1. Что уже работает

Текущая система имеет правильные инварианты, их нельзя потерять ради скорости:

- canonical terrain — чистая функция physical direction + wavelength, а tile только cache/address unit;
- одна и та же physical point имеет одну и ту же поверхность независимо от camera, LOD, sampling order и наличия renderer;
- cube-sphere исключает equirectangular seam и pole pinching;
- coarse sample является frequency-prefix более fine sample, а не другой поверхностью;
- global subtraction выполняется в `f64`, tile-local geometry хранится в `f32`;
- parent удерживается до готовности replacement children, поэтому visual LOD transition не создаёт holes;
- coarse horizon cover и fine camera-weighted selection разделены;
- async workers не блокируют frame thread;
- cache bounded;
- authoritative geometry/semantics и renderer используют одну canonical физическую поверхность, а не независимые random fields;
- `FlightAuthority` уже способен обращаться к `PlanetField` без renderer; текущий terrain-impact guard является первым headless consumer canonical surface.

Эти свойства важнее конкретного implementation текущего renderer path.

Главное архитектурное правило:

```text
camera visibility may control render work,
but must never define whether physical terrain exists.
```

## 2. Где сейчас тратится client streaming budget

Один новый render tile сейчас означает существенно больше, чем quad mesh.

Упрощённо текущий worker выполняет:

```text
request TileKey
    |
    +--> build_tile()
    |      + height samples
    |      + normals
    |      + indices / skirts
    |
    +--> build_surface_texture_for_mesh()
    |      + per-texel height prefix
    |      + fine terrain sample
    |      + climate/material evaluation
    |      + residual height for normal map
    |      + grain noise
    |      + albedo
    |      + roughness
    |      + normal
    |
    +--> mesh_from_tile()
    |      + mesh assembly
    |      ` tangents
    |
    +--> surface_image() x3
           ` CPU mip generation / upload-ready images
```

На `L13+` texture tier сейчас 128 px plus apron. Следовательно один tile содержит порядка 17k material texels, каждый из которых выполняет несколько procedural evaluations. В flight streaming это легко становится существенно дороже самой mesh geometry.

Увеличение worker count может улучшить throughput на свободных cores, но оно не меняет асимптотику и быстро начинает конкурировать с flight/server/client work за CPU и memory bandwidth.

Этот bottleneck относится прежде всего к **client visual streaming**. Его нельзя лечить переносом authoritative surface на GPU: server и automation всё равно должны иметь CPU-доступ к canonical field.

## 3. Representation bandwidth и wasted resolution

Canonical terrain height сейчас имеет physical micro bands до порядка десятков метров. При этом near tiles могут иметь mesh vertex spacing и texture texel spacing существенно меньше этой длины волны.

Это само по себе не ошибка: mesh subdivision нужен для projection, curvature, skirts и будущих более коротких bands. Но после того, как geometry resolution становится finer, чем canonical height bandwidth, дальнейшее увеличение subdivisions **не добавляет новую форму поверхности**.

Нужно явно разделить spatial-frequency responsibilities:

```text
large / medium scale
    authoritative height field
    -> physics queries / contact base / render displacement

medium / near visual scale
    GPU procedural normal / roughness / color modulation

object scale
    deterministic scatter / rocks / debris / vegetation / structures

sub-object scale
    material normal / roughness only
```

Ориентир, не hard contract:

```text
> ~32 m          canonical geometric height
~2..32 m         GPU visual relief / detail normal, optional bounded displacement
~0.2..10 m       deterministic scatter / rocks / local features
< ~1 m           material normal / roughness / albedo microstructure
```

Если позже collision/gameplay требуют real geometry ниже 32 m, canonical field можно расширить дополнительными bands. Но renderer не должен требовать такого расширения только ради того, чтобы земля перестала выглядеть мыльной.

И наоборот: shader-only displacement ниже canonical cutoff не должен внезапно становиться препятствием для шасси. Если feature физически важен, он должен существовать в authoritative representation.

## 4. Target split: authoritative surface, contact materialization, render shading

### 4.1 CPU authoritative/query side

`PlanetField` остаётся CPU-доступным canonical источником поверхности для server/gameplay независимо от renderer.

Типовые consumers:

- height at direction / position;
- surface normal / slope estimate;
- terrain clearance;
- path/trajectory sampling;
- landing-zone evaluation;
- geology/biome/climate semantics, где они gameplay-relevant;
- deterministic placement крупных physical surface features.

Это **query representation**, а не render mesh. Во многих случаях автоматика может работать напрямую с field samples без материализации сетки.

Пример API-направления:

```rust
pub trait SurfaceQuery {
    fn height_m(&self, dir: DVec3, min_wavelength_m: f64) -> f64;
    fn normal(&self, dir: DVec3, min_wavelength_m: f64) -> DVec3;
    fn slope(&self, dir: DVec3, min_wavelength_m: f64) -> f64;
    fn clearance_along(&self, path: &SurfacePathQuery) -> ClearanceResult;
}
```

Не обязательно вводить именно такой trait сейчас; важна semantics: query fidelity задаётся физической задачей, а не camera LOD.

### 4.2 CPU contact representation

Для посадки одного `height_m(point)` недостаточно. Когда аппарат действительно взаимодействует с поверхностью, server должен уметь получить локальную геометрию/контактное представление для:

- нескольких landing legs / wheels;
- hull/body contacts;
- local surface normal;
- uneven terrain;
- penetration resolution;
- braking/friction;
- later suspension and rolling contacts.

Target lifecycle:

```text
craft approaches contact envelope
          |
          v
request local authoritative surface patch
          |
          v
PlanetField samples -> CPU contact patch / BVH / height patch
          |
          v
contact solver
          |
          v
craft leaves region -> patch may be evicted
```

Этот patch создаётся **из physics need**, не из observer/camera need. На пустом dedicated server полностью автоматическая посадка должна работать с той же physical surface, что и при подключённом клиенте.

Contact representation не обязано совпадать с client render triangulation. Обязано совпадать физическое поле высот/features в пределах заявленной error bound.

### 4.3 Automation without an observer

Автоматика является отдельным surface consumer и не должна зависеть от того, смотрит ли кто-то на аппарат.

Примеры:

```text
landing planner
    -> sample candidate zones
    -> slope / roughness / clearance queries
    -> choose approach corridor

terrain-following / avoidance
    -> sample look-ahead path
    -> derive clearance envelope

unobserved scripted landing
    -> query canonical field
    -> request contact patch near touchdown
    -> execute full authoritative contact dynamics
```

Это и есть правильная версия «оптимизации когда никто не видит»:

```text
no observer
    != no terrain
    != no physics

no observer
    -> no render workload
    -> automation/query workload remains if mission logic needs it
    -> contact workload appears only when physical interaction needs it
```

### 4.4 Client render geometry side

CPU client streaming должен в основном производить данные, которые нужны raster path и пока не вычисляются эффективнее на GPU:

- tile anchor;
- positions / displaced surface geometry;
- normals sufficient for base geometry;
- indices and skirts or replacement seam strategy;
- optional compact semantic weights needed by the material;

Новый render tile не должен по умолчанию создавать три уникальных CPU-generated texture assets.

### 4.5 GPU visual side

Surface shader получает continuous world/body-fixed coordinates и вычисляет visual detail непосредственно в пространстве планеты:

```text
planet/body direction
physical position / radius
base geometric normal
height / slope proxy
semantic climate inputs (if needed)
world seed / material parameters
```

На GPU должны переехать в первую очередь:

- regional/variation noise, используемый только для appearance;
- fine color grain;
- small-scale normal perturbation;
- roughness modulation;
- biome/material blending, если его входы доступны без дорогого authoritative query;
- near tiling / triplanar or spherical-coordinate detail layers.

Это не требует превращать мир в repeated texture wallpaper. Deterministic procedural noise может оставаться continuous в physical coordinates; просто consumer меняется с CPU image baker на shader.

## 5. Canonical field vs renderer field

Нужно сохранить чёткую границу:

```text
PlanetField authoritative outputs
    - physical height
    - climate / geology / biome semantics where gameplay cares
    - deterministic physical feature placement

SurfaceVisualField
    - cosmetic micro normal
    - cosmetic albedo grain
    - sub-grid roughness
    - shader-only blending detail
```

`SurfaceVisualField` может быть производным от того же seed и физических координат, но **не должен менять collision/flight terrain height**.

Это позволяет renderer быть сильно более высокочастотным, не заставляя terrain queries, contact solver и server повторять shader workload.

Если visual feature должен стать gameplay-relevant (например крупный валун или кратер), он перестаёт быть shader-only и получает deterministic object/geometry representation на authoritative side.

Нужно избегать скрытой третьей поверхности:

```text
BAD:
server terrain != automation terrain != visible terrain

GOOD:
one canonical physical field
    + task-specific representations/caches
    + cosmetic visual detail layered on top
```

## 6. GPU/client implementation options

Нет необходимости сразу выбирать одну технику для всего диапазона. Все варианты этого раздела — **renderer implementation**, не обязательная зависимость dedicated server.

### Option A — procedural WGSL

Портировать дешёвый deterministic noise и material functions в WGSL.

Плюсы:

- no per-tile texture generation/upload;
- continuous world-space coordinates;
- unlimited effective material resolution near camera;
- simple cache story.

Минусы:

- shader ALU cost;
- CPU/GPU bitwise identity не гарантируется;
- сложные climate/province calculations не стоит буквально дублировать на GPU.

Поэтому GPU path должен использовать только cosmetic subset или compact precomputed semantic inputs.

### Option B — shared material textures / arrays

Использовать небольшое число reusable detail textures / texture arrays, tiled/triplanar/spherical blended по semantic weights.

Плюсы:

- очень дешёвый runtime;
- hardware filtering/aniso/mips;
- хорошо подходит для rock/soil/snow/ice microstructure.

Минусы:

- нужны source assets;
- repeated texture artifacts нужно ломать rotation/noise blending;
- меньше procedural uniqueness.

### Option C — hybrid virtual/detail cache

Для дорогих visual functions GPU/compute или background worker может запекать reusable pages в virtual texture/cache, а не уникальные textures строго 1:1 на geometry tile.

Это имеет смысл позже, если pure shader становится дорогим или нужны authored high-detail regions.

Первый target должен быть проще: **remove per-tile albedo/roughness/normal baking from the critical flight streaming path**.

### Option D — GPU adaptive terrain / CBT-like tessellation

Concurrent Binary Tree / related GPU-driven adaptive triangulation интересен как возможная поздняя замена client-side cube-sphere render mesh streaming.

Архитектурная граница жёсткая:

```text
PlanetField / authoritative CPU surface
          |
          +--> server queries/contact: CPU only
          |
          `--> client visual consumer
                    |
                    `--> GPU CBT / adaptive tessellation
```

CBT не становится источником истины и не требуется на dedicated server. Он только отвечает на renderer-вопрос: **какой набор треугольников сейчас нужен для изображения canonical surface с заданной screen-space error**.

Potential benefits:

- continuous/adaptive visual triangulation instead of fixed tile mesh density;
- GPU split/merge based on projected error;
- меньше CPU geometry churn на быстром пролёте;
- естественный путь к очень мелкой near-view triangulation без огромного количества independently built CPU meshes.

Но CBT не решает автоматически:

- authoritative contact geometry;
- automation terrain queries;
- procedural height evaluation cost if every vertex still requires expensive field evaluation;
- material detail;
- physical objects/scatter;
- server CPU performance.

Поэтому это **не Phase 1 optimization**. Сначала нужно убрать CPU material baking и измерить оставшийся geometry bottleneck.

## 7. High-speed visual streaming must be predictive, not only reactive

Сейчас movement trigger быстро понимает, что камера ушла далеко, но selection/build pipeline в основном реагирует на текущий eye/frustum. При km/s скоростях аппарат проходит значительную дистанцию за один build latency.

Этот раздел относится к client visual representation. Automation/physics prediction имеет собственные query horizons и не должна использовать camera streaming queue как источник поверхности.

Нужен bounded look-ahead в направлении camera/vehicle motion.

Target selection inputs:

```rust
pub struct TerrainStreamingView {
    pub eye_body_m: DVec3,
    pub forward_body: DVec3,
    pub velocity_body_mps: DVec3,
    pub angular_velocity_hint: DVec3,
    pub fov_rad: f64,
}
```

Из скорости и измеренного build latency вычисляется predicted eye:

```text
lookahead_s = clamp(p95_tile_ready_latency * safety_factor,
                    min_lookahead,
                    max_lookahead)

predicted_eye = eye + velocity * lookahead_s
```

Selection должна резервировать часть tile budget под corridor между `eye` и `predicted_eye`, а не просто смещать весь frustum вперёд.

Иначе быстрый craft при резком pitch/yaw сможет иметь идеальный terrain впереди траектории, но hole/coarse ground в текущем кадре.

Пример budget split:

```text
coarse guaranteed cover       fixed
current-frame fine detail     ~50-65%
predictive velocity corridor  ~25-40%
turn / reserve                remainder
```

Точные доли должны идти из benchmark, не из этой доки.

## 8. Selection generation must not wait for nearly empty workers

Текущий guard вида `jobs.len() <= 2` полезен как backpressure, но при высокой скорости может задержать новый selection даже когда movement trigger уже знает, что старые jobs становятся менее ценными.

Нужен priority scheduler, где queued work можно reprioritize или drop до начала исполнения.

Не обязательно отменять уже работающий CPU job. Достаточно различать:

```text
Running jobs     small bounded set; usually finish
Queued requests  mutable priority queue; stale requests may be discarded
Ready cache      reusable if still spatially relevant
```

Priority должна учитывать:

- coverage necessity;
- current frame projected error;
- predicted future projected error;
- distance/time-to-enter view;
- parent availability;
- tile build cost estimate;
- whether tile is already partially cached.

Старая просьба на detail позади аппарата не должна блокировать coarse/fine tile впереди него только потому, что она попала в очередь на 200 ms раньше.

## 9. Different work classes need different priorities

Coarse coverage, geometry refinement и cosmetic detail имеют разную criticality.

Client render classes:

```text
P0  coverage repair / missing visible parent
P1  visible geometry refinement
P2  predicted visual geometry corridor
P3  visible cosmetic/detail preparation
P4  predicted cosmetic detail
P5  render cache warming / optional work
```

После GPU material split классы P3/P4 должны стать почти бесплатными для CPU, что как раз освобождает throughput для P0-P2.

Authoritative/contact jobs не должны конкурировать в этой очереди по тем же приоритетам. Server physics имеет отдельный scheduler/budget; contact preparation для imminent touchdown важнее любого cosmetic render work.

## 10. Geometry build optimization after the representation split

Только после удаления texture baking из client critical path имеет смысл серьёзно оптимизировать render mesh builder.

Кандидаты:

- batch evaluate height samples for several tiles;
- SIMD-friendly noise sampling where profitable;
- reuse parent samples in children;
- preserve edge samples exactly to avoid duplicate evaluation;
- precompute/reference static index buffers for fixed `TILE_CELLS`;
- avoid per-tile tangent generation if final material uses world/triplanar mapping and tangents are unnecessary;
- reduce mesh attributes to exactly what shader consumes;
- GPU compute mesh generation only if CPU remains bottleneck after simpler fixes;
- later evaluate CBT-like adaptive GPU triangulation if fixed-tile mesh churn remains dominant.

Особенно важно проверить tangents: если near terrain material уйдёт на world-space/triplanar normals, tangent-space may no longer justify CPU `generate_tangents()` for every tile.

Server-side optimization рассматривается отдельно:

- batch/SIMD field queries for automation;
- cached local contact patches;
- reuse samples across nearby wheels/legs;
- bounded refinement from predicted contact time;
- no render-only albedo/normal work.

## 11. Parent/child visual transition

Текущий finest-ready cover гарантирует отсутствие holes, но replacement всё ещё дискретный. После throughput work можно отдельно улучшить perceptual transition:

- geomorph parent -> child;
- short cross-fade/dither for material/detail only;
- shared-edge displacement constraints;
- skirts только как fallback, а не primary visual seam mechanism.

Но это **не должно блокировать throughput refactor**. Hole-free discrete replacement лучше красивого morph, который не успевает прогружаться.

Contact representation не обязана повторять этот visual transition: physics должна видеть устойчивую canonical поверхность, а не dither/morph state renderer'а.

## 12. Instrumentation required before and after refactor

Нельзя оценивать streaming только максимальной скоростью аппарата. Нужно писать client render metrics:

```text
terrain.requested_tiles
terrain.queued_tiles
terrain.running_jobs
terrain.ready_tiles
terrain.visible_tiles
terrain.stale_requests_dropped
terrain.cache_hit_rate

terrain.tile_geometry_ms p50/p95/p99
terrain.tile_material_ms p50/p95/p99
terrain.tile_upload_ms p50/p95/p99
terrain.request_to_visible_ms p50/p95/p99

terrain.viewer_speed_mps
terrain.lookahead_s
terrain.coverage_age_s
terrain.max_projected_error
terrain.visible_coarse_fallback_count
```

Особенно полезна метрика:

```text
time_to_needed = distance_to_future_view / viewer_speed
```

Tile misses deadline, если `request_to_visible > time_to_needed`, даже если сам build benchmark выглядит быстрым.

Для headless/server surface нужны отдельные metrics:

```text
surface.query_count
surface.query_ms p50/p95/p99
surface.contact_patch_build_ms p50/p95/p99
surface.contact_patch_cache_hits
surface.contact_patch_cache_misses
surface.automation_samples_per_sim_s
surface.contact_refinement_deadline_misses
```

Важно не смешивать `request_to_visible` с `request_to_contact_ready`: у них разные consumers и разные deadlines.

## 13. Benchmark scenarios

### 13.1 Client visual streaming

Минимальный repeatable set:

1. **Hover / survey** — почти нулевая скорость, aggressive near refinement.
2. **Aircraft** — 200-400 m/s на 1-10 km AGL.
3. **Fast atmospheric craft** — 1 km/s.
4. **Current stress point** — 3 km/s.
5. **Target supersonic/hypersonic** — 5 km/s и 8 km/s.
6. **Low-altitude pathological pass** — высокая скорость + небольшой AGL.
7. **Fast turn** — скорость высокая, camera/vehicle heading меняется резко.
8. **Vertical descent/ascent** — projection footprint меняется быстрее, чем ground-track distance suggests.

Для каждого:

- no holes;
- bounded cache/jobs;
- visible projected-error distribution;
- p95 request-to-visible latency;
- CPU frame cost;
- total worker utilization;
- GPU material cost после split.

### 13.2 Headless authoritative surface

Отдельный repeatable set без GPU и без client:

1. automated descent from high altitude;
2. terrain-following trajectory query;
3. landing-zone search over configurable radius;
4. final approach with predicted contact patch prefetch;
5. multi-leg touchdown on sloped/rough surface;
6. rolling/braking contact once wheel dynamics exist;
7. multiple unobserved vehicles approaching different surface regions;
8. high warp far from contact, then transition to full surface/contact fidelity before touchdown.

Для каждого:

- identical canonical surface with and without connected client;
- deterministic query results for fixed seed/state;
- bounded contact patch memory;
- no GPU dependency;
- contact representation ready before physical interaction;
- automation result independent of camera position/view mode.

## 14. Performance targets

Не следует задавать target как «должно быть быстрее KSP», потому что world complexity, hardware, visual quality и architecture разные.

### 14.1 Client visual contract

```text
At declared maximum active-flight speed and minimum supported AGL,
terrain rendering must keep a complete visible cover and satisfy the
projected-error target for the central view with bounded latency and memory.
```

Первый practical target после GPU material split:

- сохранить текущую визуальную/геометрическую fidelity;
- минимум удвоить sustainable ground speed относительно current ~3 km/s baseline на той же машине;
- не увеличивать tile cache bound только ради throughput;
- не поднимать worker count как единственную оптимизацию;
- удержать main-thread terrain insertion cost малым и bounded.

После этого отдельно измерять 8-12 km/s atmospheric/near-surface stress cases.

### 14.2 Headless/server contract

```text
Surface physics and automation must remain fully functional with no GPU,
no renderer and zero connected observers.
```

Дополнительно:

- far-from-surface automation использует bounded direct queries, а не materialized render terrain;
- contact representation создаётся заранее по physics/trajectory need;
- landing result не меняется из-за подключения/отключения spectator/client;
- server не платит за cosmetic material detail;
- fidelity transition определяется physical error/deadline, а не camera LOD.

## 15. Suggested implementation order

### Phase 1 — measure actual client bottleneck

1. Добавить отдельные p50/p95 timings geometry/material/mips/upload.
2. Записать request -> visible latency.
3. Сделать fixed 3 km/s и 5 km/s benchmark routes.

### Phase 2 — remove CPU material baking from client critical path

4. Сделать prototype terrain shader с shared/procedural near detail.
5. Убрать per-tile albedo/roughness/normal creation для prototype path.
6. Проверить, нужен ли tangent generation после нового mapping.
7. Сравнить tile throughput и total CPU.

### Phase 3 — predictive client scheduling

8. Добавить velocity look-ahead corridor.
9. Разделить queued vs running jobs.
10. Разрешить discard/reprioritize stale queued requests.
11. Сохранить guaranteed coarse cover независимо от prediction.

### Phase 4 — formalize headless surface consumers

12. Вынести/зафиксировать observer-independent surface query boundary для automation/flight.
13. Добавить normal/slope/clearance helpers с explicit wavelength/error request.
14. Спроектировать local contact patch representation и cache lifecycle.
15. Prefetch contact patch по predicted time-to-contact, не по camera distance.
16. Добавить headless landing/contact benchmarks без GPU.

### Phase 5 — geometry hot path

17. Профилировать `PlanetField` height-only sampling отдельно для render и server query workloads.
18. Добавить sample reuse / batching / SIMD только по measured hotspots.
19. Проверить parent->child sample reuse на client и nearby-query reuse на server.
20. Если client mesh churn остаётся bottleneck — spike GPU compute/CBT-like adaptive triangulation.

### Phase 6 — visual quality

21. Добавить GPU micro-normal/material bands ниже geometric cutoff.
22. Затем scatter/rocks.
23. Только потом geomorph/cross-fade, если LOD replacement остаётся заметным.

## 16. Non-goals

Этот refactor не должен:

- превращать canonical terrain в fixed global raster;
- делать renderer или GPU-tessellation источником authoritative height;
- требовать GPU на dedicated server;
- считать отсутствие observer отсутствием physical terrain;
- привязывать automation fidelity к camera position/LOD;
- заставлять headless server строить visual terrain tiles для посадки;
- синхронизировать cosmetic shader noise по сети;
- генерировать весь future flight corridor заранее без bounded budget;
- скрывать streaming failures гигантским cache;
- увеличивать detail frequency в physics только ради картинки;
- требовать, чтобы contact triangulation совпадала с render triangulation, если обе аппроксимируют одну canonical surface в заданной error bound.

## 17. Ключевой инвариант

Вместо старого разделения «CPU спрашивает физику, GPU спрашивает картинку» нужен чуть более точный контракт:

```text
Authoritative physics asks:
    "какая здесь физическая поверхность и как с ней контактировать?"

Automation asks:
    "какая поверхность будет по моей траектории/в зоне посадки?"

Renderer asks:
    "как эту же canonical поверхность представить на экране сейчас?"
```

Все три вопроса используют один `PlanetField`, но **не обязаны использовать одно representation**.

```text
canonical field
    + direct queries          -> automation / clearance / planning
    + local contact patch     -> landing / wheels / body collision
    + visual LOD / CBT / mesh -> renderer
    + cosmetic shader detail  -> pixels only
```

Это позволяет оптимизировать каждую задачу независимо:

- nobody watches -> render branch costs zero;
- automation still runs -> query branch remains active;
- touchdown approaches -> contact representation materializes;
- client connects -> visual representation builds without changing physics.

Следующий шаг — не отказаться от процедурности, а **перенести каждый вид procedural work туда, где его стоимость соответствует его consumer: queries/contact на CPU server side, visual frequency/detail на GPU client side**.

## 18. UMA / unified-memory asset residency follow-up

Отдельно от алгоритма LOD и процедурного поля у текущего client path есть более низкоуровневая возможность: уменьшить число **логических копий render data**. На UMA это особенно важно, потому что CPU и GPU конкурируют за один DRAM pool и один memory bandwidth, но выигрыш полезен и на dGPU как экономия system RAM и memcpy.

Нужно различать три задачи:

```text
A. lifetime / residency
   не держать CPU payload после того, как immutable asset подготовлен renderer'ом

B. transient build copies
   не клонировать большие Vec между TerrainTile / SurfaceTexture / Mesh / Image

C. true zero-copy / mapped GPU memory
   отдельная поздняя оптимизация; не предполагать, что A или B автоматически
   превращают Bevy/wgpu upload path в zero-copy
```

### 18.1 Current code-specific opportunities

Сейчас terrain textures уже создаются как `RenderAssetUsages::RENDER_WORLD`, то есть после extraction/preparation их CPU-side pixel payload можно выбросить. Terrain mesh при этом создаётся как:

```rust
RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD
```

Это оставляет vertex/index payload доступным в `Assets<Mesh>` даже после подготовки render representation. Для immutable terrain tile это выглядит лишним: `CachedTile` уже хранит `vertices` и `triangles`, а `WorldTerrain` уже считает visible counters из cache metadata.

Отдельный perf path пока снова проходит по `Assets<Mesh>` ради `count_vertices()` / `indices().len()`. Это не должно быть причиной держать полный CPU mesh resident. Эти counters можно брать из `WorldTerrain.visible + WorldTerrain.cache`, после чего terrain mesh может стать render-only asset:

```rust
Mesh::new(
    PrimitiveTopology::TriangleList,
    RenderAssetUsages::RENDER_WORLD,
)
```

Target lifecycle тогда такой:

```text
worker builds tile
    -> Mesh/Image payload exists on CPU temporarily
    -> Bevy extracts/prepares render asset
    -> CPU payload is dropped
    -> cache keeps Handle + anchor + vertex/triangle/byte metadata
```

Это **не обещание zero-copy**. Render-world buffer/texture allocation всё ещё существует, а backend может выполнять staging/upload. Выигрыш здесь — не держать вторую долгоживущую CPU representation без consumer.

### 18.2 Avoid worker-local clones before touching renderer internals

Текущий worker также делает копии ещё до Bevy asset extraction:

```text
TerrainTile.positions.clone() -> Mesh
TerrainTile.normals.clone()   -> Mesh
TerrainTile.indices.clone()   -> Mesh

SurfaceTexture.albedo.clone()    -> Image
SurfaceTexture.roughness.clone() -> Image
SurfaceTexture.normal.clone()    -> Image
```

После assembly исходные `TerrainTile` / `SurfaceTexture` больше не являются cache representation. Поэтому builder API стоит перестроить на ownership/move:

```text
build_tile() / build_surface_texture()
        |
        v
owned vectors
        |
        +--> move into Mesh
        `--> move into Image
```

Для mesh это может означать `mesh_from_tile(tile: TerrainTile, ...)` либо destructuring с сохранением `anchor`, `vertices`, `triangles` metadata до move. Для texture — передавать owned channel vectors в `surface_image()` без `.clone()`.

Это уменьшает transient allocation и memory traffic независимо от GPU architecture. На UMA выгода потенциально заметнее именно потому, что worker CPU traffic и renderer traffic делят один memory subsystem.

### 18.3 Memory accounting must describe logical residency, not pretend UMA is discrete VRAM

Текущий `world.cache_bytes` оценивает один mesh payload плюс texture bytes. Это полезный logical cache metric, но он не описывает:

- retained CPU mesh payload;
- render-world/GPU allocation;
- temporary worker copies;
- allocator overhead;
- staging/upload buffers;
- физическую residency UMA, где отдельного независимого VRAM pool может не быть.

Поэтому performance monitoring лучше разделить как минимум на:

```text
terrain.cache_logical_bytes
terrain.cpu_asset_payload_bytes
terrain.build_transient_bytes_estimate
terrain.upload_bytes_per_s
```

`gpu_mem_bytes`/physical UMA residency не нужно синтезировать, если backend не даёт достоверной цифры. Logical byte counters всё равно позволяют проверить, что refactor реально убрал лишнюю representation.

### 18.4 Capability-driven UMA fast path — only after profiling

Если после material split, ownership cleanup и render-only assets измерения покажут, что bottleneck остаётся именно в CPU->GPU upload/copy, тогда можно отдельно исследовать shared-memory fast path:

```text
capability detection
    |
    +--> UMA / host-visible device-local memory available
    |       -> mapped/ring-buffer or equivalent upload strategy
    |
    `--> discrete / unsuitable memory type
            -> normal staging/upload path
```

Это должен быть **capability-driven**, а не `if vendor == AMD`. Linux/Vulkan, Metal и другие wgpu backends остаются first-class; backend-specific shortcut не должен проникать в `PlanetField`, terrain semantics или server code.

В wgpu/Bevy такой путь может потребовать более глубокого render-asset/custom-buffer integration и явной синхронизации mapped vs GPU use. Поэтому он не должен предшествовать простым измеримым изменениям выше.

### 18.5 Suggested low-risk order

До крупных renderer rewrites:

1. Перевести immutable terrain `Mesh` на `RenderAssetUsages::RENDER_WORLD`.
2. Убрать perf dependency на CPU `Assets<Mesh>` и считать visible geometry из `CachedTile` metadata.
3. Убрать `.clone()` больших mesh/texture vectors в worker assembly через ownership transfer.
4. Добавить logical/transient/upload byte metrics.
5. Снять одинаковые hover / 3 km/s / 5 km/s captures на UMA и dGPU, если доступны.
6. Только если upload остаётся bottleneck — spike mapped/shared-memory path.

Acceptance criteria:

- identical visible terrain and canonical physics;
- те же visible vertex/triangle counters;
- меньше retained CPU payload и worker transient traffic;
- RSS/peak memory не хуже, на UMA ожидается ниже;
- p95 `request_to_visible` не ухудшается;
- никакой vendor lock-in и никакой GPU requirement для authoritative terrain.
