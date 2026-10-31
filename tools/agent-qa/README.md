# Agent quality tools

These are **development-only** Python 3.11+ standard-library tools. They are not
runtime dependencies of the Rust engine and do not call a paid model API.

## Architecture check

```sh
python3 tools/agent-qa/check_architecture.py
python3 -m unittest discover -s tools/agent-qa/tests -v
```

The checker uses full `cargo metadata --format-version 1 --all-features`
resolution: transitive runtime/build paths and root dev edges for backend/license
checks, without inheriting downstream dev tooling. It guards 20 foundations,
MIT-to-GPL workspace paths, reverse app dependencies, mandatory game worldgen
in rendering and new generic-physics/celestial coupling. Two exact compiler
facade edges remain staged; collision’s former facade edge is forbidden. See [doc 88](../../docs/88_ENGINE_BOUNDARIES_AND_REFERENCE_SCENES.md).

```sh
cargo metadata --offline --format-version 1 --all-features > /tmp/thessa-cargo.json
python3 tools/agent-qa/check_architecture.py --metadata /tmp/thessa-cargo.json \
  --report /tmp/thessa-boundaries.json
```

Exit: 0 clean (possibly staged), 1 forbidden boundary, 2 metadata/read failure.
The CLI rejects `--no-deps` input. Type-level leaks and external license
obligations still need explicit review.

## Blind visual arena

Provide real PNG screenshots captured from matched views. The manifest uses
paths relative to itself, not to the repository root:

```toml
[[scene]]
id = "orbital-horizon"
first = "frames/thessa.png"
first_label = "Thessa@abc123"
second = "frames/reference.png"
second_label = "Reference game/version"

[[scene]]
id = "terrain-closeup"
first = "frames/terrain-new.png"
first_label = "Thessa new"
second = "frames/terrain-old.png"
second_label = "Thessa baseline"
```

Create a blind run, with private provenance outside the directory sent to
reviewers:

```sh
python3 tools/agent-qa/arena.py prepare /tmp/arena/input.toml \
  --blind /tmp/arena/review --private /tmp/arena/private
# Give reviewers ONLY /tmp/arena/review/ (cases.json, A.png, B.png, REVIEW.md).
python3 tools/agent-qa/arena.py vote --blind /tmp/arena/review \
  --id orbital-horizon --judge reviewer-1 --choice B \
  --notes 'Atmosphere smoother; inspect exposure match'
python3 tools/agent-qa/arena.py score --blind /tmp/arena/review \
  --key /tmp/arena/private/answer-key.json
```

Valid choices: `A`, `B`, `tie`, `invalid`. A tie earns each origin 0.5
preference points; an invalid comparison gets no points. Each reviewer casts
one vote per case. Scoring deliberately occurs **after** vote collection.
Private keys must not be shared with evaluators or committed to Git.

The tool strips PNG text/EXIF and unknown ancillary metadata while retaining
render-relevant color and transparency chunks. Do not include visible
watermarks, game HUDs or source-identifying decorations.

Requirements: paired PNGs must have identical dimensions; manually control
camera/FOV, scene, lighting, exposure, HUD, quality settings and resolution.
If those differ materially, vote `invalid`. Random order reduces side bias,
not composition bias. The rubric records aesthetic preference and physical
plausibility separately. Do not use this output as a physics oracle or claim
statistical certainty from a handful of pairwise samples. Repeat comparisons,
re-randomize positions and include human calibration before trusting a model
judge. The arena is a **harness**, not a running model or screenshot capture
integration. Native build/run/capture provenance now uses `capture.py`; the
general native control bridge and image judging remain separate work.

For the overall workflow and the two-machine setup, see
[`docs/55_AGENT_QUALITY_CONTROL.md`](../../docs/55_AGENT_QUALITY_CONTROL.md).

## Native capture provenance

`capture.py` builds and captures the existing native app, verifies declared
input and binary hashes, and records camera/light/settings/backend identity,
PNG, pre-tonemap HDR when available and existing perf JSON/CSV. It uses only
Python standard library plus the existing native executable/Cargo toolchain.
See [doc 89](../../docs/89_REFERENCE_CAPTURE.md) for manifest schema, commands,
measurement boundaries and dirty-source limitations. No asset fetch is implicit.

Native B01 uses `data/reference-scenes/b01-capture.toml`. The tool binds actual
scene IDs and hashes exported GLB/scene metadata separately from the requested
run manifest. [Doc 90](../../docs/90_REFERENCE_SCENES.md) records the frozen
profiles, independent native path and verified static physical-GPU evidence.

## Pinned B02 asset cache

```sh
python3 tools/agent-qa/reference_assets.py --cache /absolute/external/cache
```

`reference_assets.py` verifies existing pinned GLBs or fetches only the ten
selected Poly Haven source files and converts them with the trusted repository
Blender script. A fresh conversion requires Blender 5.2.2 LTS build
`d13f752e3b9c`; automatic source scripts are disabled. Source/converter/license/
result checksums and file budgets are enforced, and large assets stay outside
Git. Existing mismatched files fail without replacement. No asset fetch occurs
inside a benchmark capture. [Doc 91](../../docs/91_B02_ASSET_PROVENANCE.md)
records the CC0 provenance, full evaluated geometry and explicitly derived
material variant. Native B02 remains separate work; the first external B01
diagnostic is documented below.

## First external B01 diagnostic

`cycles_b01.py` runs inside an existing Blender/Cycles installation and imports
the GLB from a verified native B01 or B05 capture. It matches the actual captured camera
and uses explicit numeric irradiance mapping and common HDR/exposure/Reinhard/
sRGB output. It retains raw float32 radiance, images, hashes and diagnostic
metrics. No renderer code or external package is added to Rust. See
[doc 92](../../docs/92_EXTERNAL_RENDERER_COMPARISON.md) for the command, tested
tool versions and BSDF/AA/light-unit limits. This is not a performance comparison
or physical/visual parity acceptance; Godot rendering remains pending.

`unreal_b05.py` runs inside the stock Unreal PythonScript commandlet over the
same verified B05 capture. Copy `data/reference-scenes/unreal-b05` to an external
writable project first. [Doc 97](../../docs/97_UNREAL_B05_COMPARISON.md) gives
the actual headless command, SDL/DDC settings, parameter-mapping control,
linear-HDR results and scope. It is not a frame-performance comparison.

## Transparent/emissive diagnostic

`data/reference-scenes/b05-capture.toml` exercises standard BLEND coverage and
HDR emission through the existing native consumer. Run `capture.py` with that
manifest and pass the resulting directory to `cycles_b01.py --samples 1024`
inside Blender. B01 uses zero transparent bounces; B05 uses eight, with zero
incident light to isolate emission/composition. Native and external source,
GLB, camera, exposure and artifact hashes stay bound by the same checks. See
[doc 95](../../docs/95_TRANSPARENT_EMISSIVE_SURFACES.md) for actual evidence and
unsupported refraction/GI/temporal/shadow/RT coverage.

The original Khronos emission asset is available separately:

```sh
python3 tools/agent-qa/reference_assets.py \
  --manifest data/reference-scenes/khronos-emission-assets.json --cache /absolute/cache
THESSA_REFERENCE_ASSET_CACHE=/absolute/cache cargo test -p thessa-render-wgpu \
  --test transparency original_khronos_emissive_strength -- --ignored --nocapture
```

This opt-in oracle requires an actual adapter. It retains original geometry and
materials and checks all five HDR strength levels on Low/High. Identity ingestion
runs no converter; its cache includes the pinned CC-BY-4.0 license, Ed Mackey/AGI
attribution and provenance. Do not redistribute its GLB without these notices.
It is not a native B06 capture or an upstream BabylonJS screenshot match.

The opt-in `fixed_transparent_instance_submission_cost` GPU test writes
per-frame CSV and raw HDR to a new `THESSA_BENCH_OUTPUT` directory. It uses
8,192 identical-source transparent instances and existing timestamp intervals;
see [doc 96](../../docs/96_TRANSPARENT_INSTANCE_RUNS.md) for the exact workload,
release ABBA evidence and limits. It measures submission cost, not B02 foliage.

The opt-in `published_profile_lookup_cost` test writes ten frozen plume HDR
fields/publications and per-frame CSV to a new `THESSA_BENCH_OUTPUT` directory.
[Doc 99](../../docs/99_PLUME_EXPOSURE_AND_LOOKUP_AUDIT.md) records the release
ABBA protocol and all rejected candidates. It is not an external volume render
or a whole-scene speedup. Camera gain=1 images remain exact after the separate
pre-exposure correction.

## Analytic plume-volume references

`plume_reference.py capture --output /absolute/new-native-volume --profile dev`
builds/runs the actual native volume fixture with source and binary identities.
For a recorded development tree, add `--allow-dirty`. Pass that output to the
same script's `cycles` command inside Blender, for example:

```sh
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --threads 2 --python tools/agent-qa/plume_reference.py -- cycles \
  /absolute/native-volume --output /absolute/new-cycles-volume --samples 2048
```

[Doc 100](../../docs/100_PLUME_SEGMENT_TRANSPORT.md) records the frozen field,
actual Cycles Volume Coefficients/camera, closed-form Gaussian oracle,
finite-cell correction, fixed-budget A/B and measured scope. Raw HDR, source
fingerprints and saved node graphs are retained. These four analytic controls
are separate from a calibrated-engine gallery, blind preference or full-frame
performance acceptance.

Add `--suite compiled` to `capture` for the nine generated methalox operating-
point controls, including 14/16-step native HDR. The same `cycles` command
loads either publication type. It validates uniform frozen samples and uses
packed Non-Color float tables with stock Volume Coefficients; no fitted gain.
Cycles uses its minimum 1e−6 m near clip to match native volume integration
from the optical camera. [Doc 102](../../docs/102_COMPILED_PLUME_GALLERY.md)
records the actual comparisons, near-clip control, sampling differences and
remaining calibration/motion/live-source limits.
