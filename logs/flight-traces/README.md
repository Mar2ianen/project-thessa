# Flight traces

`pilot-flight-2026-09-09.csv` is a captured live X-15 pilot-mode trace from the
Bevy client. It records the state, aerodynamic forces, moments, acceleration,
and control channels needed to reproduce the reported in-flight shaking.

The CSV is diagnostic evidence, not authoritative gameplay data. New local
captures are written to `target/flight-traces/` and remain ignored by Git;
selected captures can be copied here when they are intentionally preserved.
