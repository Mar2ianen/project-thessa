# Height (OPTIONAL macro hint, grayscale)

Preferred: skip this layer — bake derives base height from tectonics,
landmarks and erosion. If painted, it is a loose macro constrain only.

Piecewise datum-centered mapping (documented, NOT naive linear):

- `#000000` (0)   = height_min_m (e.g. -8000 m, deepest basin floor)
- `#808080` (128) = 0 m datum, EXACTLY
- `#FFFFFF` (255) = height_max_m (e.g. +12000 m, highest peak)

Lower half maps [0,128] => [min, 0], upper half [128,255] => [0, max].
Decode to f64 metres at import; all later stages work in metres.

Rules: smooth gradients, no terracing, no lighting/shading. Coastline
(datum crossing) must match the hydrology and biome maps exactly.
