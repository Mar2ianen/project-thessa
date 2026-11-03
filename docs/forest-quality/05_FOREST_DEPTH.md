# 5 — Forest depth and aerial perspective

Status: reference/applicability reviewed; finite-path transport integration open.

## Research

[Bruneton's maintained atmospheric scattering implementation](https://ebruneton.github.io/precomputed_atmospheric_scattering/) includes planetary density profiles, solar spectra and CPU/GPU reference tests. Its finite camera-to-surface transport is the target mechanism; an arbitrary distance fog color is insufficient.

## Applicability review

Thessa already shares atmosphere-derived sunlight and sky irradiance. `environment.wgsl::env_local_haze` applies extinction within a spherical near-surface layer and an equilibrium sky-source approximation to terrain, water and meshes. The shader explicitly documents this shortcut. It is not full finite-path single/multiple scattering. Increasing extinction cannot establish correct transport or fix a hard geometric horizon.

## Implementation decision

Keep the existing quality/fallback path while adding a reference finite-path numerical oracle before a shared LUT integration. Avoid independent mesh-only haze or multiplying sunset color twice. No atmosphere coefficients changed in this vegetation iteration.

## Verification needed

The 100 m forest view must distinguish geometry/LOD contrast from transport. Compare linear radiance and transmittance with atmosphere disabled and with a finite-path reference; preserve camera/time/exposure. Production performance must be measured separately from serial debug captures.
