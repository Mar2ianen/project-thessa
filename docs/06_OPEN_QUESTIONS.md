# 06 — Open questions

Пункты здесь **специально не зафиксированы** в v0.1.

## Мир / lore

- финальное название проекта;
- названия объектов (Asterion/Nereid/Thessa/etc. пока working);
- есть ли native complex biosphere на Thessa/Pelagos/Janus;
- breathable ли Thessa без оборудования;
- почему игрок/колония находится именно там — минимальный lore hook;
- насколько явно объясняется formation history системы (вероятно почти никак).

## Celestial numbers

- canonical epoch и orbital angles;
- exact resonant offsets/libration amplitudes Nereid chain;
- Nix tidal `Q/k2` и lifetime;
- final Cinder orbit;
- Nereid obliquity/ring tilt;
- atmospheric scale profiles;
- J2/Jn coefficients каждого meaningful body;
- weather/climate fields.

## Gameplay

- inventory model игрока;
- насколько физичны отдельные belt items;
- exact research unlock rules;
- maintenance intensity;
- consequences player death/vehicle loss;
- whether economy/contracts exist at all;
- how much manual flight is expected before route certification;
- какие MechJeb-like high-level blocks входят в минимальную standard library;
- семантика parallel graph при staging/docking/vehicle ownership;
- whether route automation requires one successful manual/reference flight.

## Vehicle editor

- cross-section parameterization;
- material thickness UI;
- how much engine design is exposed;
- whether engine cycle is discrete class or continuous sub-parameters;
- procedural wheel/gear editor depth;
- design validation UX;
- how to visualize structural/thermal graphs without turning UI into CAD pain.

## Physics

- exact aero panel method;
- transonic model;
- wake model;
- structural solver order;
- reduced-order aeroelasticity method;
- slosh fidelity;
- atmospheric heating correlation;
- ablative heat-shield model;
- CPU ray sampling budget;
- deterministic tolerance across AVX builds.

## Runtime

- Avian vs Parry-only local contact implementation;
- Lightyear adoption after spike;
- in-process vs separate local server default;
- save DB/file format;
- web client scope;
- whether client prediction shares full sim-flight or reduced model;
- protocol transport (UDP/QUIC/WebTransport etc.);
- Windows packaging policy: разрешить wgpu D3D12 backend или форсить Vulkan; architecture от этого не зависит.

## Performance targets

Need benchmark-backed numbers for:

- 780M 1080p Low/Medium target FPS;
- max active atmospheric craft at x1;
- max warp with 1k/5k/10k vehicles;
- factory object count;
- thermal/structural node budget per design;
- acceptable memory on 24 GiB systems.

## Licensing / distribution

Baseline уже принят: engine MIT, game code GPL-3.0-or-later. Открыты:

- лицензия ассетов/музыки;
- граница generic `protocol` vs game-specific protocol/rules;
- допустимость LGPL dependency внутри MIT engine;
- AGPL tooling только как external validation или возможен optional isolated tool.
