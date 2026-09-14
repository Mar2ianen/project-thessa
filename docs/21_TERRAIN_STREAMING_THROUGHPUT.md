# 21 — Terrain streaming and throughput

Status: current implementation contract, 2026-09-14.

## Current runtime split

The canonical `PlanetField` is owned by `thessa-worldgen-rocky` and remains
the source for authoritative height and material queries. The client currently
streams cube-sphere `TerrainTile` meshes and surface textures through bounded
Bevy async tasks. The server and headless tools do not depend on Bevy, wgpu,
or the CBT renderer.

The render path now has a second, backend-neutral topology observer:

```text
PlanetField
    +--> authoritative/query consumers
    `--> cube-sphere TileKey adapter
              +--> legacy CPU mesh/texture fallback
              `--> CBT split/merge candidates
                         `--> universal Bevy CBT plugin
```

The CBT state is render topology only. It does not replace `PlanetField`,
does not define contact geometry, and does not become server state.

## Budget boundaries

- Tile selection remains deterministic and bounded by its explicit tile budget.
- CBT mutations are bounded by `FrameBudget::max_operations` per frame.
- Async terrain work is bounded by the existing in-flight job limit.
- A CBT candidate is a declared split or merge of a logical domain node; it is
  not a hidden gain, teleport, or terrain-specific correction coefficient.

## Cube-sphere bridge

The terrain adapter maps six cube faces into depth-three binary prefixes. Each
quadtree level appends two Morton bits, `(x_bit, y_bit)`, so a tile at level
`L` has CBT depth `3 + 2L`. The mapping is reversible and covered by a
round-trip test. Odd CBT depths are intermediate half-steps and are not
treated as complete cube tiles.

## What is and is not shipped

The current game still renders the proven CPU mesh/texture path. The CBT
plugin is already scheduled in the client and receives the live camera view
and terrain selection, so topology planning is exercised in the game without
making the fallback renderer unsafe. The next render milestone is to consume
`CbtFrameOutput::updates` in a render-world draw-list adapter and then retire
duplicate CPU topology work after an error-bound and benchmark comparison.
