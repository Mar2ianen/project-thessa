# ADR-0009 — Composable MechJeb-like autopilot graphs

Status: accepted.

## Decision
Use MechJeb-like high-level autopilot operations as the UX reference, but represent operations as typed composable graph blocks executed by an event-driven runtime. Support sequence, conditions, waits, retry/fallback, reusable subgraphs, and first-class parallel/fork/join.

Staging can produce multiple VehicleIds and automation branches can independently control detached vehicles. All high-level guidance ultimately commands physical actuators through normal guidance/FBW layers.
