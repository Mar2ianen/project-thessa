# 71 — Native surface capture and WORLD00 acceptance

**Status:** presentation-only numeric survey-site and exact eye-AGL controls
implemented; WORLD00 coastal-site selection, GPU captures and visual acceptance
remain open. No new terrain, collision or flight authority is introduced here.

## A bookmark is not an acceptance site

`--site 0` selects the existing coarse COAST grid bookmark. At the reviewed
baseline (`3d8698f`) it is approximately 23° N, 165° E and 102 m above datum;
its label is not evidence that a shoreline crosses a 2 × 2 km neighborhood.
Do not promote it to WORLD00 or claim coastal acceptance from that bookmark.

WORLD00 still needs a numeric canonical-field survey identifying an actual
coastal 2 × 2 km region. Record its fixed latitude/longitude, sampled land/sea
boundary and neighborhood bounds, canonical source/build revision and dataset
hashes. A matching pair of 50 m / 2 m AGL frames is then necessary evidence,
not by itself acceptance of coast continuity, terrain detail, vegetation or
surface-to-orbit quality. No shoreline coordinate is invented by this document.

## Existing native CLI, explicit framing

- `--survey-lat DEG --survey-lon DEG` sets the **target** direction using the
  shared canonical spherical conversion. Both are required; finite latitude
  is in `[-90, 90]`, longitude in `[-180, 180]`, east-positive. Exact poles and
  either dateline spelling are valid. Conflicts with `--site`.
  The HUD identifies this target as `FIXED COORDINATES`, not the previously
  selected bookmark; numeric coordinates remain in capture metadata.
- `--survey-eye-agl METRES` sets radial eye clearance in `[2, 2000000]` m.
  It can accompany either numeric coordinates or an existing bookmark. The
  shared survey orbit/distance still selects the eye direction; the override
  sets its radius to `body radius + max(canonical height, 0) + requested AGL`.
  Ground is queried at the **actual eye direction**, not the target altitude
  or target tangent plane, with an explicit 1 m minimum height wavelength.
  This is not clearance above wave crests, canopy, rocks or a coarse resident
  triangle mesh. Render/field convergence still requires separate checks.
- `--survey-distance 80..2000000` retains the shared distance controller and
  is applied after site selection. It does **not** specify a ground footprint
  or imply that the camera eye is directly above the target. A surveyed
  2 × 2 km acceptance neighborhood and capture coverage must be recorded
  independently of this orbit parameter.
- All three new controls are survey-only: flight, preview, docking and map
  combinations are rejected before asset/GPU startup. Bookmark selection clears
  the numeric-site and AGL overrides and restores existing bookmark framing.
  Camera orbit/free-look input remains the existing presentation controller.
- With no new flags, the HIGHLANDS default, 5 km survey distance, 0.18-distance
  elevation, 32 m height query, 5 m minimum clearance and 1 m near plane are
  unchanged. Vertical FOV remains 0.9 radians. Explicit AGL uses near distance
  `min(1 m, 0.05 × AGL)`: 1 m at 50 m AGL, 0.1 m at 2 m AGL.

After the numeric survey supplies `WORLD00_LAT` and `WORLD00_LON`, capture
serially (do not compete with another GPU job):

```sh
mkdir -p target/surface-captures
for agl in 50 2; do
  TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  cargo run --release -p thessa-terrain-native -- \
    --survey-lat "${WORLD00_LAT:?set surveyed latitude}" \
    --survey-lon "${WORLD00_LON:?set surveyed longitude}" \
    --survey-distance 80 --survey-eye-agl "$agl" \
    --sim-time 0 --sim-step 0 --prestream-seconds 3 --frames 120 \
    --mute --no-hud --headless "target/surface-captures/world00-${agl}m.png" \
    --perf-capture "target/surface-captures/world00-${agl}m.json"
done
```

Use a dedicated target directory when another worktree is building. Keep
resolution, graphics configuration, source revision, epoch and streaming
conditions matched; preserve JSON/CSV and logs with each image. An 80 m
survey-distance close-up is a framing example, **not** the 2 km coverage test.

## Reproducibility through existing performance captures

Headless `--perf-capture` adds `native_survey_*` entries to the existing
`metadata.graphics.resolved` map. These record the **final** target/site,
body-space eye and target, actual view forward and free-look-adjusted look
target, controller distance, requested/actual radial AGL, queried ground
height and wavelength, resolved vertical FOV and near plane. The source
description explicitly names `PlanetField::height_m` and the sea-datum clamp;
the existing analytic source signature and frozen-erosion attachment flag are
included. The signature is not a hash of frozen erosion or the executable:
retain the build revision and hash
`data/worldgen/thessa-erosion-v3.surface.json.gz` separately.

These entries are absent for vehicle/map views. They do not reconstruct every
frame of `--sweep` or a moving-time capture; use frozen time/no sweep for the
matched acceptance pair. This metadata is outside measured frame samples and
does not enable a new renderer effect or change graphics-resolution policy.

## Verification boundary

CPU regressions pin unchanged default camera/projection arithmetic, numeric
coordinate bounds/pairing and mode conflicts, 50 m / 2 m canonical eye-ground
agreement (absolute radial residual below 1 µm), polar/dateline validity,
an explicitly exercised submerged-ground/sea-datum clamp, near-plane placement,
metadata, HUD labels and bookmark/flight ownership. They intentionally
do not prove resident geometry convergence or subjective appearance. Follow
`docs/55_AGENT_QUALITY_CONTROL.md` for independent blind comparisons after real
captures; neither passing numeric tests nor the implementer's own image review
closes WORLD00 acceptance.

Independent source review found no blocking placement/ownership issue. The
follow-up fixes the numeric-site HUD label and makes the sea-datum regression
explicitly assert a submerged eye sample. Focused default and feature-off
survey tests and app all-feature Clippy passed in the isolated worktree;
GPU/visual verification is a separate integration stage.
