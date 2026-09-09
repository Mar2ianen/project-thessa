# Normal (tangent-space RGB)

Standard tangent-space normal map:

- R = X (east) slope, `#80` = flat
- G = Y (north) slope, `#80` = flat
- B = up, `#FF` = flat ground, darker = steeper cliffs

Flat plains near `#8080FF`, slopes tint toward red/green by aspect, cliffs
drop blue toward `#806080`. No object-space tricks. Must derive from the same
height shapes, not invented ridges.
