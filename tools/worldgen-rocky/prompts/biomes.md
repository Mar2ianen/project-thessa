# Biomes (broad REGION hints, flat fills)

Paint LARGE coherent regions the bake turns into taxonomy sites
(primary biome + geology + feature tags). Do NOT shuffle tiny patches:
the planet must stay readable from orbit (KSP-like, not texture soup).

Suggested region colors (bake re-derives exact sites from terrain):

- deep ocean `#0018A8`, shallow shelf `#1E6FFF`
- ice cap / glacier `#E8F4FF`
- barren rock `#8A7B6B`, sand desert `#D9B380`, highlands `#6B6B6B`
- volcanic field `#3A1E14`, active lava `#FF4A00`
- canyon `#5A3A2E`, crater ejecta `#A8A094`

Landmark geology (giant craters, arcs, canyons) comes from the recipe
`[[features]]` / `[[tectonics]]`, not from paint. Coastlines match the
height datum crossing.
