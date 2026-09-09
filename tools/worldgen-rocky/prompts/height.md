# Height (grayscale displacement)

Single channel replicated to RGB. Linear mapping, no gamma tricks:

- `#000000` (0) = height_min_m (-8000 m, deepest basin floor)
- `#808080` (128) = 0 m datum
- `#FFFFFF` (255) = height_max_m (+12000 m, highest peak)

Rules: smooth gradients, no terracing, no lighting/shading, ocean floor darker
than datum, mountains brighter. Coastline (datum crossing) must match the
hydrology and biome maps exactly.
