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
checks, without inheriting downstream dev tooling. It guards 19 foundations,
MIT-to-GPL workspace paths, reverse app dependencies, mandatory game worldgen
in rendering and new generic-physics/celestial coupling. Three exact existing
facade edges are staged. See [doc 88](../../docs/88_ENGINE_BOUNDARIES_AND_REFERENCE_SCENES.md).

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
integration. Frame capture/headless Thessa control remains a separate project.

For the overall workflow and the two-machine setup, see
[`docs/55_AGENT_QUALITY_CONTROL.md`](../../docs/55_AGENT_QUALITY_CONTROL.md).
