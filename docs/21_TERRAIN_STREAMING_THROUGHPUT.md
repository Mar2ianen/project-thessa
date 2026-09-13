# 21 — Terrain streaming throughput and representation split

Статус: **design target / performance follow-up**.

Цель этой доки — увеличить скорость перемещения, при которой realtime terrain успевает корректно прогружаться, не ухудшая continuity и не превращая `PlanetField` в набор заранее запечённых растров.

Текущий vertical slice уже показывает важный результат: полностью процедурный terrain, включая geometry, albedo, roughness, normal maps, mip generation и upload preparation, успевает за аппаратом примерно до порядка **3 km/s** на текущей машине. Для first implementation это хороший baseline. Проблема не в том, что cube-sphere или canonical field сами по себе медленные; проблема в том, что streaming frontier сейчас выполняет слишком много работы, часть которой вообще не обязана быть CPU tile-build work.

Основная идея следующего шага:

```text
canonical world field
       |
       +--> CPU terrain geometry / semantic coarse data
       |
       `--> GPU continuous surface material + micro detail

streaming frontier = geometry bandwidth problem,
not per-tile texture baking problem
```

## 1. Что уже работает

Текущая система имеет правильные инварианты, их нельзя потерять ради скорости:

- canonical terrain — чистая функция physical direction + wavelength, а tile только cache/address unit;
- cube-sphere исключает equirectangular seam и pole pinching;
- coarse sample является frequency-prefix более fine sample, а не другой поверхностью;
- global subtraction выполняется в `f64`, tile-local geometry хранится в `f32`;
- parent удерживается до готовности replacement children, поэтому LOD transition не создаёт holes;
- coarse horizon cover и fine camera-weighted selection разделены;
- async workers не блокируют frame thread;
- cache bounded;
- geometry, material и simulation используют одну физическую поверхность, а не независимые random fields.

Эти свойства важнее конкретного implementation текущего renderer path.

## 2. Где сейчас тратится streaming budget

Один новый tile сейчас означает существенно больше, чем quad mesh.

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

## 3. Representation bandwidth и wasted resolution

Canonical terrain height сейчас имеет physical micro bands до порядка десятков метров. При этом near tiles могут иметь mesh vertex spacing и texture texel spacing существенно меньше этой длины волны.

Это само по себе не ошибка: mesh subdivision нужен для projection, curvature, skirts и будущих более коротких bands. Но после того, как geometry resolution становится finer, чем canonical height bandwidth, дальнейшее увеличение subdivisions **не добавляет новую форму поверхности**.

Нужно явно разделить spatial-frequency responsibilities:

```text
large / medium scale
    authoritative height field
    -> mesh displacement / collision / terrain queries

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

## 4. Target split: geometry streaming vs surface shading

### 4.1 CPU / authoritative side

CPU streaming должен в основном производить данные, которые невозможно или нежелательно вычислять только shader'ом:

- tile anchor;
- positions / displaced surface geometry;
- normals sufficient for base geometry;
- indices and skirts or replacement seam strategy;
- optional low-rate semantic weights needed by gameplay;
- optional coarse material class IDs / climate scalars if их вычисление на GPU слишком дорого или должно точно совпадать с gameplay state.

Новый tile не должен по умолчанию создавать три уникальных CPU-generated texture assets.

### 4.2 GPU / visual side

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
    - deterministic feature placement

SurfaceVisualField
    - cosmetic micro normal
    - cosmetic albedo grain
    - sub-grid roughness
    - shader-only blending detail
```

`SurfaceVisualField` может быть производным от того же seed и физических координат, но **не должен менять collision/flight terrain height**.

Это позволяет renderer быть сильно более высокочастотным, не заставляя terrain queries, contact solver и server повторять shader workload.

Если visual feature должен стать gameplay-relevant (например крупный валун или кратер), он перестаёт быть shader-only и получает deterministic object/geometry representation на следующем уровне системы.

## 6. GPU implementation options

Нет необходимости сразу выбирать одну технику для всего диапазона.

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

## 7. High-speed streaming must be predictive, not only reactive

Сейчас movement trigger быстро понимает, что камера ушла далеко, но selection/build pipeline в основном реагирует на текущий eye/frustum. При km/s скоростях аппарат проходит значительную дистанцию за один build latency.

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

Target classes:

```text
P0  coverage repair / missing visible parent
P1  visible geometry refinement
P2  predicted geometry corridor
P3  visible cosmetic/detail preparation
P4  predicted cosmetic detail
P5  cache warming / optional work
```

После GPU material split классы P3/P4 должны стать почти бесплатными для CPU, что как раз освобождает throughput для P0-P2.

## 10. Geometry build optimization after the representation split

Только после удаления texture baking из critical path имеет смысл серьёзно оптимизировать mesh builder.

Кандидаты:

- batch evaluate height samples for several tiles;
- SIMD-friendly noise sampling where profitable;
- reuse parent samples in children;
- preserve edge samples exactly to avoid duplicate evaluation;
- precompute/reference static index buffers for fixed `TILE_CELLS`;
- avoid per-tile tangent generation if final material uses world/triplanar mapping and tangents are unnecessary;
- reduce mesh attributes to exactly what shader consumes;
- GPU compute mesh generation only if CPU remains bottleneck after simpler fixes.

Особенно важно проверить tangents: если near terrain material уйдёт на world-space/triplanar normals, tangent-space may no longer justify CPU `generate_tangents()` for every tile.

## 11. Parent/child visual transition

Текущий finest-ready cover гарантирует отсутствие holes, но replacement всё ещё дискретный. После throughput work можно отдельно улучшить perceptual transition:

- geomorph parent -> child;
- short cross-fade/dither for material/detail only;
- shared-edge displacement constraints;
- skirts только как fallback, а не primary visual seam mechanism.

Но это **не должно блокировать throughput refactor**. Hole-free discrete replacement лучше красивого morph, который не успевает прогружаться.

## 12. Instrumentation required before and after refactor

Нельзя оценивать streaming только максимальной скоростью аппарата. Нужно писать метрики:

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

## 13. Benchmark scenarios

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

## 14. Performance target

Не следует задавать target как «должно быть быстрее KSP», потому что world complexity, hardware, visual quality и architecture разные.

Полезнее contract:

```text
At declared maximum active-flight speed and minimum supported AGL,
terrain streaming must keep a complete visible cover and satisfy the
projected-error target for the central view with bounded latency and memory.
```

Первый practical target после GPU material split:

- сохранить текущую визуальную/геометрическую fidelity;
- минимум удвоить sustainable ground speed относительно current ~3 km/s baseline на той же машине;
- не увеличивать tile cache bound только ради throughput;
- не поднимать worker count как единственную оптимизацию;
- удержать main-thread terrain insertion cost малым и bounded.

После этого отдельно измерять 8-12 km/s atmospheric/near-surface stress cases.

## 15. Suggested implementation order

### Phase 1 — measure actual bottleneck

1. Добавить отдельные p50/p95 timings geometry/material/mips/upload.
2. Записать request -> visible latency.
3. Сделать fixed 3 km/s и 5 km/s benchmark routes.

### Phase 2 — remove CPU material baking from critical path

4. Сделать prototype terrain shader с shared/procedural near detail.
5. Убрать per-tile albedo/roughness/normal creation для prototype path.
6. Проверить, нужен ли tangent generation после нового mapping.
7. Сравнить tile throughput и total CPU.

### Phase 3 — predictive scheduling

8. Добавить velocity look-ahead corridor.
9. Разделить queued vs running jobs.
10. Разрешить discard/reprioritize stale queued requests.
11. Сохранить guaranteed coarse cover независимо от prediction.

### Phase 4 — geometry hot path

12. Профилировать `PlanetField` height-only sampling.
13. Добавить sample reuse / batching / SIMD только по measured hotspots.
14. Проверить parent->child sample reuse.

### Phase 5 — visual quality

15. Добавить GPU micro-normal/material bands ниже geometric cutoff.
16. Затем scatter/rocks.
17. Только потом geomorph/cross-fade, если LOD replacement остаётся заметным.

## 16. Non-goals

Этот refactor не должен:

- превращать canonical terrain в fixed global raster;
- делать renderer источником authoritative height;
- требовать GPU на dedicated server;
- синхронизировать cosmetic shader noise по сети;
- генерировать весь future flight corridor заранее без bounded budget;
- скрывать streaming failures гигантским cache;
- увеличивать detail frequency в physics только ради картинки.

## 17. Ключевой инвариант

Главное разделение должно остаться простым:

```text
CPU/server asks:
    "какая здесь физическая поверхность и что она означает?"

GPU asks:
    "как эта поверхность выглядит на данном пикселе?"
```

Сейчас tile worker отвечает на оба вопроса сразу. Для первого vertical slice это позволило быстро получить coherent procedural planet. Для high-speed flight именно это соединение становится throughput bottleneck.

Следующий шаг — не отказаться от процедурности, а **перенести procedural work туда, где его стоимость масштабируется с пикселями и GPU, а не с churn geometry tiles на CPU**.
