# References and provenance

Status: reference list (values for the fictional system live in `data/` + `01`/`02*`).

These sources support external technical facts and validation workflows. Values
for the fictional system are design values calculated and stored separately.

## Orbital mechanics and ephemerides

- NASA/JPL Solar System Dynamics: https://ssd.jpl.nasa.gov/
- Nyx Space documentation: https://nyxspace.com/
- ANISE documentation: https://nyxspace.com/anise/
- Vallado, *Fundamentals of Astrodynamics and Applications*.
- Battin, *An Introduction to the Mathematics and Methods of Astrodynamics*.

Nyx/ANISE are isolated validation references. Their code is not copied into
the MIT runtime and their packages are not root runtime dependencies.

## Gravity fields and bounded approximation

- Binney and Tremaine, *Galactic Dynamics*, for multipole/field reasoning.
- Greengard and Rokhlin, fast multipole method foundations.
- Classical tidal-tensor and state-transition formulations for a frozen affine
  field.

The repository implementation is independent and carries its own tests,
absolute error envelopes, and fallback rules.

## Aerodynamics and flight

- JSBSim: https://github.com/JSBSim-Team/jsbsim
- RocketPy: https://github.com/RocketPy-Team/RocketPy
- AVL: https://web.mit.edu/drela/Public/web/avl/
- OpenVSP/VSPAERO: https://openvsp.org/
- SU2: https://su2code.github.io/
- OpenRocket: https://openrocket.info/
- NASA X-15 technical reports and aerodynamic data where cited by a specific
  validation case.

External solvers and data remain reference-only. Imported coefficient tables
must preserve geometry, reference area, units, sign conventions, source, and
license provenance.

## Rendering and atmosphere

- Bevy: https://bevyengine.org/
- wgpu: https://wgpu.rs/
- GPU Gems and standard Rayleigh/Mie/absorption scattering literature for
  visual atmosphere reasoning.

Visual atmosphere code is renderer-independent at the shared optics boundary;
Bevy/wgpu adapters are client implementation details.

## Autopilot UX references

- MechJeb documentation and user-facing vocabulary for ascent, maneuver,
  landing, rendezvous, and docking workflows.
- Scratch/dataflow graph concepts for visual composition.

These are UX references, not copied code or runtime dependencies.

## Cross-platform rendering

- WebGPU: https://www.w3.org/TR/webgpu/
- Vulkan: https://www.khronos.org/vulkan/
- Metal: https://developer.apple.com/metal/

DirectX-specific APIs are not part of the gameplay or simulation contract.

## Licensing rule

Before adding a reference implementation, model, texture, coefficient table,
or dataset, record its license and provenance. Do not copy code into an MIT
engine crate without a compatible license review.
