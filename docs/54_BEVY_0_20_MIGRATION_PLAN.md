# 54 — Bevy 0.20 migration plan (0.19.1 → 0.20.0)

**Status:** migrated on this branch, awaiting merge/defer decision
**Language:** English
**Updated:** 2026-10-14
**Base:** `c129181` on `main`, branched as `study/bevy-0.20-migration`
**Landed:** commit `3b3f8be` (deps + code + 8 regression tests); fmt fixes pending in working tree
**Bevy release:** 0.20.0 (2026-10-08). Migration guide:
`https://bevy.org/learn/migration-guides/0-19-to-0-20/`

This plan respects `docs/39_BEVY_EXIT_AND_ENGINE_MIGRATION.md`:
Bevy is a bootstrap host under a strangler-fig exit. The migration
must be **minimal, mechanical, and bridge-local**. No new 0.20
features (Feathers widgets, contextual theming, pan-orbit camera,
sprite materials, BSN, meshlets) are adopted as architecture.

## 1. Recommendation

- Do the migration as a **time-boxed spike on this branch**, not on `main`.
- Default decision: **stay on 0.19.1 on `main`** until the spike is green
  including render/screenshot/perf gates. Merge only if the spike closes
  without expanding Solari, render-graph, or UI architecture.
- Reason: 0.20 cost is concentrated in the exact subsystems frozen by
  doc 39 (render bridge, Solari, wgpu backend, UI). Benefit of 0.20 to
  Thessa gameplay/simulation is ~zero today.

## 2. Scope inventory (measured at base commit)

Workspace baseline (`Cargo.toml:46-57`):

| Dependency | Current | Bevy 0.20 line | Action |
| --- | --- | --- | --- |
| `bevy` | 0.19.1 | 0.20.0, Rust 1.97.1 | bump + `rust-version` 1.95 → ≥1.97.1 |
| `bevy_core_pipeline` (in `crates/bevy-rcbt`) | 0.19.1 | 0.20.0 | bump with workspace |
| `glam` | 0.32 | 0.33.2 (`bevy_render` 0.20 requires `^0.33.2`) | bump; audit `DVec3/DQuat` narrowing in client |
| `wgpu` | 29.0.4 | 30 (`bevy_render` 0.20 requires `^30`) | bump workspace + `crates/bevy-rcbt` optional + `crates/rcbt-wgpu` |
| `naga` (dev-dep, WGSL validation) | 29.0.4 | match wgpu 30 line | bump in `apps/client`, `crates/bevy-rcbt`, `crates/rcbt-wgpu` |
| `bevy_brp_extras` | 0.22.0 (locked 0.22.5) | 0.23.0 (released 2026-10-09, day after Bevy 0.20) | bump; verify `BrpExtrasPlugin` + `schedule.graph` access output |
| `bevy_solari` (via `bevy` feature) | 0.19.1 | 0.20 | keep RT-gated usage only, re-tune, do not expand |

Bevy touch surface:

- 22 Rust files use `bevy::` (`apps/client/src`: 15 files, ~26.4k lines;
  `crates/bevy-rcbt/src`: 7 files, ~8.5k lines). No other workspace crate
  depends on Bevy (per freeze boundary — keep it that way).
- 8 shader files, 0 `.wesl` today: 2 asset shaders with old `naga_oil`
  `#import` (`assets/shaders/cloud_volume.wgsl:30-32`,
  `assets/shaders/plume_volume.wgsl:15-17`); 6 `crates/bevy-rcbt/src/*.wgsl`
  plain WGSL concatenated via `include_str!` (`render.rs:535,599,831-835`).
- `ExtractResource` × 7 impls + `ExtractResourcePlugin` × 7 registrations,
  all in `crates/bevy-rcbt/src/render.rs:47-169,1139-1145`.
- Solari × ~39 hits, all in `apps/client` (`main.rs`, `atmosphere.rs`,
  `terrain.rs:2505`, `perf.rs:514`). `bevy-rcbt` has zero Solari coupling.
- Custom mesh-shader path × ~44 hits, all in `crates/bevy-rcbt`
  (`render.rs` gates at `:64,120-142,523,1122`; `draw_mesh_tasks_indirect`
  at `:2645`; `create_mesh_pipeline` at `:2679-2691`).
- UI `Button`/`Interaction` × ~25 hits (`pilot/hud.rs`, `terrain.rs:544-549`,
  `map_ui.rs:36`, `perf.rs:131`, `docking_demo.rs:214-219`).

## 3. Breaking-change triage (75 guide items → 16 applicable)

### Must migrate (compile or behavior break)

1. **WESL shaders** (guide: WESL). `naga_oil` preprocessor is gone.
   Plain WGSL keeps working; `#import` dialect does not.
   - Convert 2 asset files to `.wesl`: `import x;` syntax, `@if` instead of
     `#ifdef`, `constants::NAME` instead of `#{NAME}`, module renames
     (`bevy_pbr::forward_io` → `bevy_pbr::render::forward_io`,
     `bevy_pbr::mesh_view_bindings` → `bevy_pbr::render::mesh_view_bindings`,
     `bevy_pbr::prepass_utils` → `bevy_pbr::prepass::utils`).
     Update `ShaderRef` paths in `beauty.rs:1060`, `plume.rs:133` and the
     `Material bind group is 3 in Bevy 0.19` comments.
   - Re-validate 6 rcbt plain-WGSL chunks under naga/wgpu 30
     (`render.rs:1203-1205,1263-1265,1493-1507`, tests at `:2751`,
     `beauty.rs:1941-1966`, `gpu_tests.rs:835-861` strips mesh-shader ext).
2. **Extraction is now generic over worlds** (guide: extraction).
   `SyncComponent`/`ExtractComponent`/`ExtractResource` require the target
   `AppLabel`. Expected edit: annotate the 7 `ExtractResource` impls and 7
   plugin registrations in `render.rs` (e.g. `#[extract_app(RenderApp)]` /
   `SyncComponent<RenderApp>` style per guide), fix imports
   (`bevy_render::extract_plugin::extract` → `bevy_extract::...`,
   `TemporaryRenderEntity::default()`), plus
   `benches/extraction_snapshots.rs`.
3. **Dependency line bumps** (§2 table). `wgpu` 29→30 and `glam` 0.32→0.33
   touch `crates/rcbt-wgpu` (portable backend) and every `DVec3` render
   narrowing site. `create_and_validate_shader_module`,
   `RawComputePipelineDescriptor`, `RawRenderPipelineDescriptor`,
   mesh-pipeline descriptor, and `WgpuFeatures::EXPERIMENTAL_MESH_SHADER`
   negotiation (`main.rs:259-292`, `render.rs:129-133`) must be re-checked
   against wgpu 30 API even where Bevy re-exports hide it.
4. **`bevy_curve` split** (guide: bevy_curve). One hit:
   `atmosphere.rs:633-634` `bevy::math::cubic_splines::LinearSpline` →
   `bevy_curve::cubic_splines::LinearSpline` (crate is in default features;
   no new dep for full-feature build).
5. **`Tonemapping`/`DebandDither` moved to `bevy_render::view`**
   (guide). One import: `main.rs:35` `core_pipeline::tonemapping::Tonemapping`.
   Rust re-export still compiles; reflected path and BRP component keys
   change — update any scene/BRP key that names the old path.
6. **`ui::widgets::Button` + `ui::Interaction` deprecated** (guide).
   Hits in `pilot/hud.rs` (~20), `terrain.rs:544-549`, `map_ui.rs`,
   `perf.rs`. Target is `ui_widgets::Button` + `picking::hover::Hovered` /
   `ui::Pressed` (see updated `button.rs` example). Short-term the code
   still compiles with deprecation warnings; the spike must decide
   migrate-now vs `#[allow(deprecated)]`-with-ticket (prefer migrate-now
   only if mechanical; do not restyle HUD).
7. **`BorderRadius` fields now 2D `CornerRadius`** (guide).
   ~20 construction sites (`BorderRadius::all(px(..))`, `BorderRadius::MAX`).
   `CornerRadius: From<Val>` keeps most call sites compiling, but
   constructors are no longer `const` and `resolve_single_corner` is gone
   (not used — verified zero hits). Compile-verify all, screenshot-compare
   HUD/map UI.
8. **Solari behavior changes** (release notes + guide: no API rename, but
   ReSTIR now **off by default**, new `SolariLighting` knobs — world cache,
   per-pixel light samples, temporal accumulation, bounce count — DLSS-RR
   4.5 SDK update, macOS/Metal path). Thessa sites: `main.rs:203-270`
   (RT-gated `SolariPlugins` + `required_wgpu_features`),
   `atmosphere.rs:212-328,798-848,1222` (`SolariLighting::default()`,
   `RaytracingMesh3d` sync, precision config). Per doc-39 freeze: re-tune
   only, add explicit `restir` choice, update DLSS SDK, screenshot-compare
   raster vs RT. No new Solari architecture.
9. **Atmosphere multi-camera** (guide). `init_atmosphere_buffer` removed;
   `AtmosphereBuffer` is now a per-camera `Component`, not a `Resource`.
   Thessa does not touch `AtmosphereBuffer` directly (verified zero hits),
   but `atmosphere.rs:597,742-744` (`Atmosphere`, `AtmosphereMode`,
   `AtmosphereSettings`) assumes one managed camera — regression-test with
   ≥2 cameras (map + PFD preview in `pilot.rs:817-847`) and confirm no
   per-camera buffer assumption leaked into custom code.
10. **Depth/stencil + `ViewTarget` compositing** (guide: depth attachment,
    `ResolvedCompositingSpace`). `render.rs:2161-2171` queries
    `(&ExtractedCamera, &ExtractedView, &ViewTarget, &ViewDepthTexture,
    &ViewUniformOffset)`. Verify field renames compile; no
    `compositing_space` field access exists today (verified zero hits) —
    query `Option<&ResolvedCompositingSpace>` only if needed.

### Verify-and-close (evidence says no edit, still needs compile proof)

11. `Name` from `&str` now requires `&'static str`. All client call sites
    pass `String`/`format!`/literals (`main.rs:871`, `terrain.rs:459,1880`,
    `atmosphere.rs:610,689,706`, `beauty.rs`, `pilot.rs`, `plume.rs`,
    `docking_demo.rs:191,201,399`) — expected no change; compiler confirms.
12. `iter_many`/`par_iter_many` now yield `Result` (guide). Zero hits for
    `iter_many` in `apps`/`crates` — no change; use `.matched()` if any
    appear during compile.
13. Render-world window data now ECS (`Query<&ExtractedWindow>`).
    Zero `ExtractedWindows` hits — no change. Main-world
    `Single<&Window, With<PrimaryWindow>>` queries
    (`navigation.rs`, `perf.rs`, `hud.rs`, `map_ui.rs`) are unaffected.
14. UI rendering now retained; `ExtractedUiNode` shape changed.
    Zero custom `bevy_ui_render` hits — no change, but re-baseline HUD perf
    (retained path changes frame cost profile).
15. `TextFont::default()` is now `Rem(1.)`. All Thessa text sets
    `FontSize::Px` explicitly (`map_ui.rs:24-25`, `perf.rs:119-120`,
    `hud.rs`, `docking_demo.rs:208-209`, `terrain.rs:556-568`) — no change,
    screenshot-compare.
16. No-impact items verified by zero-hit grep: observers/`On<Add>` (only
    `.observe(save_to_disk)` in `perf.rs:404,419`), pointer events/picking
    (clicks via `ButtonInput<MouseButton>` + `Interaction`), exclusive
    systems/`&mut World`, `Entity::PLACEHOLDER`, `ComponentId`,
    `define_label!`, `MeshTag`, `CustomAttributes`, `Sprite`/`SpriteMaterial`,
    `OIT`/`oit_draw` (custom volume materials use `AlphaMode::Add`/
    `Premultiplied` but no direct `oit_draw` — still visual-compare
    translucency + decide on opt-in `ScreenSpaceTransmission`, currently
    unused), `DownsampleShaders`, `ShaderBuffer`, `WgpuWrapper`,
    `bevy_shape` primitives (via prelude only), BSN/`template_value`,
    states, `FilteredResources`, `AssetId::invalid`, reflect renames,
    `MainEntityHashMap`, render-window `PrimaryWindow` pattern.

### Known ambient risks (not compile errors, must be in test plan)

- Built-in schedules now weakly ordered (`chain_weak` rollout). Thessa's own
  `.chain()` calls (`plume.rs:249`, `pilot.rs:795-796`, `main.rs:249-250`,
  `terrain.rs:126`) stay strong, but render-internal weakening can expose
  hidden ordering assumptions → run `shuffle_seed` schedule-randomization
  session per guide.
- `RenderDebugOverlay` now in default plugins (we use
  `RenderDiagnosticsPlugin` only when `THESSA_GPU_PROFILE` is set —
  confirm no overlay keybinding/surface change).
- `bevy_settings` now errors on useless `SettingsGroup`s; BRP
  `schedule.graph` exposes accesses/filters (check `BrpExtrasPlugin`
  output shape after bump).
- `SortedCamera::hdr` removed (per-target camera indices). Single-camera
  app + `Hdr`/`Msaa::Off`/`Bloom`/`DepthPrepass`/`DeferredPrepass`
  (`main.rs:385-422`) — verify multi-view (map/PFD) indices.
- `CompressedImageSaver` reworked (`compressed_image_saver` = `ctt`
  BCn/ASTC; old Basis path = `compressed_image_saver_universal`). Only
  matters if asset processing is adopted later — record, do not act.

## 4. Migration phases (spike, on this branch)

- **Phase 0 — baseline lock (0.5d).** Record `Cargo.lock` (bevy 0.19.1,
  wgpu 29.0.4, naga 29.0.4, glam 0.32.1, `bevy_brp_extras` 0.22.5).
  Snapshot screenshots + `perf-*.csv` reference set on current toolchain.
- **Phase 1 — deps bump (1d).** `bevy`→0.20.0, `bevy_core_pipeline`→0.20.0,
  `glam`→0.33, `wgpu`→30, `naga`→30-line, `bevy_brp_extras`→0.23.0,
  `rust-version`→1.97.1. `cargo update -w`, `cargo check -w`. Expect wgpu-30
  fallout in `crates/rcbt-wgpu` + `bevy-rcbt` mesh path first.
- **Phase 2 — mechanical fixes (1–2d).** Extraction generics (§3.2),
  `Tonemapping` import, `bevy_curve` import, WESL conversion of 2 asset
  shaders (+ `ShaderRef` paths), `BorderRadius` verify, `Button`/`Interaction`
  decision. Keep diffs bridge-local (`apps/client`, `crates/bevy-rcbt` only).
- **Phase 3 — render validation (1–2d).** naga parse tests, wgpu validation,
  `cargo test -w` (with repo `TMPDIR` policy), GPU `#[ignore]` tests on the
  iGPU (`cargo test -p thessa-bevy-rcbt --features render --lib -- --ignored`),
  screenshot-compare (raster, RT-gated Solari, translucency, HUD), perf
  CSV re-baseline incl. retained-UI profile.
- **Phase 4 — Solari/atmosphere sign-off (0.5–1d).** Explicit ReSTIR choice,
  DLSS SDK note, multi-camera atmosphere check, `required_wgpu_features`
  re-verify. No architecture expansion per doc 39 §14/§24.
- **Phase 5 — merge decision.** Either merge spike → `main` (all gates green,
  screenshots + perf attached) or keep `main` on 0.19.1 and rebase this
  branch monthly. Update `docs/00_STATUS.md` + `09_BEVY_VISUAL_SLICE.md`
  + `14_VISUAL_ATMOSPHERE.md` + `22_RCBT_GPU_TERRAIN.md` in the same change
  (docs-sync rule); never land code with stale version claims.

## 5. Acceptance gates

- `cargo fmt --all -- --check` (only pre-existing rustfmt-1.99 drift remains;
  all migration hunks are clean); `cargo clippy --workspace --all-targets
  --all-features -- -D warnings` ✅; `cargo test --workspace --lib --bins`
  then `--test '*'` ✅ (0 failures); GPU ignored tests on iGPU ✅ (23/23,
  `thessa-bevy-rcbt --features render --lib -- --ignored`).
- Regression tests added (8): `wheel_lines_*` (2), `hud_button_fill_*` (2),
  `*_wesl_prologue_uses_render_paths` (2) + `cloud_volume_wesl_body_parses`,
  `extraction_targets_render_app`.
- `docs/00_STATUS.md` row for this doc + touched feature docs agree with
  the landed code. No new Bevy-0.20-only API outside
  `apps/client` + `crates/bevy-rcbt` + `crates/rcbt-wgpu` (freeze rule).
