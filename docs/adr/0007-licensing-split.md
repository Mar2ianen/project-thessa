# ADR-0007 — MIT engine / GPL game split

Status: accepted.

## Decision
Reusable engine/simulation/tooling crates use MIT. Project Thessa game applications and game-specific code use GPL-3.0-or-later. Assets are licensed separately.

## Consequences
Game may depend on engine; engine may not depend on GPL game code. Each Cargo package declares its license explicitly. Copyleft engine dependencies require separate review.
