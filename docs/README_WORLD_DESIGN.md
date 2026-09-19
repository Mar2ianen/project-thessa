# World-design documentation

Status: design baseline (index).

- `01_CELESTIAL_SYSTEM.md` — orbital architecture, resonance targets, stability and ephemeris constraints.
- `02_WORLD_ATLAS.md` — body identities, compositions, atmospheres, climate/biomes, resources and origin hypotheses.
- `02A_ATMOSPHERE_MODEL.md` — canonical atmosphere inputs and tracking table.
- `02B_BC_SUBSYSTEM.md` — expanded Asterion B–C planetary architecture and moon-system concepts.
- `02C_FAR_COMPANION.md` — independent brown-dwarf interloper at ~0.5–1.0 ly (unbound cloud member, deliberately unbaked); intended as the first expansion target.
- `02D_BIOSPHERE_CHIRALITY.md` — mirror-biosphere canon and food-compatibility rules.
- `03_INTERSTELLAR_SCOPE.md` — playable stellar-neighborhood scale and interstellar travel design targets.
- `13_THESSA_V02_DESIGN.md` — Thessa v0.2 bulk values and worldgen recipes (note: §2 stellar proposal not applied to runtime).
- Data layer: `data/system.toml` (working truth) → `data/system.baked.json` (bake via `thessa-system-baker`; regenerate after any `system.toml` edit), `data/resources.toml`, `data/worldgen/` recipe stubs.

Keep orbital/numerical truth in `01`, environmental/lore truth in the `02*` atlas files, and cross-system scale/travel assumptions in `03`.
