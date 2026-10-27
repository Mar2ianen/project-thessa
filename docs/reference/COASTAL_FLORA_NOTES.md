# Coastal plant representation notes (2026-10-27)

The retained coastal artwork is the composition target. Modern fern descriptions
below supply morphology vocabulary and geometric ideas; they do not identify
Thessa's fictional flora as these species or establish fossil reconstruction,
measured pigments, placement density or a climate model. The botanical research photographs below are not redistributed as game assets.
The existing CC0 Poly Haven plant atlas and separately documented CC0 Forest
Leaves 02 substrate remain licensed texture sources.

- [NParks, Is it a Fern? Double Con-fern!](https://www.nparks.gov.sg/publications-resources/articles/is-it-a-fern--double-con-fern%21)
  describes tightly coiled emerging fronds and the further division of leaflets
  in a bipinnate tree fern. The useful representation distinction is a young,
  partially unfurled rosette versus a scaled copy of a dense mature one.
- [Royal Botanic Gardens Victoria, VicFlora: Cyathea](https://vicflora.rbg.vic.gov.au/flora/taxon/68445b6f-8b49-4158-84f4-ed55d3bef4dd)
  describes repeatedly divided fronds and trunk bases covered with adventitious
  roots or retained leaf bases. This motivates fine secondary silhouettes and
  structural root/stipe geometry, without assigning those botanical measurements
  to the procedural prototypes.
- [University of Auckland, Cyathea smithii](https://www.nzplants.auckland.ac.nz/en/about/ferns/native-ferns/cyatheaceae-black-silver-tree-ferns/cyathea-smithii.html)
  describes a three-times-divided frond and retained dead stalks forming a skirt.
  Mixed living and senescent organs should therefore have different actual
  silhouettes/material groups; uniform darkening of an entire crown is not an
  equivalent representation.
- [PBRT, Image Texture](https://www.pbr-book.org/4ed/Textures_and_Materials/Image_Texture)
  explains footprint filtering and mip storage. Existing world-addressed geometry
  and footprint-aware material sampling remain the renderer path; adding fine
  leaf geometry must preserve bounded full/reduced error, rather than relying
  on screen-space noise to disguise a silhouette mismatch.

- [PBRT, Roughness Using Microfacet Theory](https://www.pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory)
  separates resolved geometry from unresolved microsurface orientation. The
  existing shared GGX material consumes roughness; per-organ optical priors
  belong in that material response, not an extra lighting multiplier. The
  coastal cohort values are authored, not measurements of living fern cuticles.

Authored centre spacings, relative plant sizes, clone proportions and leaf
reflectance/roughness factors remain explicit scene-development priors. They
must not be described as measured ecology or photometric convergence. Review
population/size changes as scene composition; strict same-content renderer blind
comparisons are invalid when those populations intentionally differ.

WORLD115 keeps intrinsic substrate reflectance separate from canopy illumination,
following the distinction between material reflectance and incoming lighting in
[PBRT, Diffuse Reflection](https://www.pbr-book.org/4ed/Reflection_Models/Diffuse_Reflection).
The numerical soil/leaf priors used here remain authored; these sources do not
measure Thessa's fictional pigments or certify reference-image parity.

[British Bryological Society, Polytrichum commune](https://www.britishbryologicalsociety.org.uk/wp-content/uploads/2020/12/Polytrichum-commune-var.-commune.pdf)
describes upright shoots with small leaves on damp ground. WORLD118 uses that
as morphology inspiration for connected, differently scaled shoot and leaf
geometry; it does not identify the fictional moss as that species or model growth.

- WORLD137 contour review uses [Flora of New Zealand: Cyathea dealbata](https://nzflora.info/factsheet/taxon/Cyathea-dealbata.html) as a modern morphological analog for multiple leaf divisions and tapered segments. The procedural `LycopsidTree` remains fictional and retains the authored fork architecture; no taxonomic reconstruction or source-image redistribution is claimed. Rounded subdivisions are explicit opaque geometry shared by raster, shadows and ray queries.

## Compound-frond reconstruction

- [University of Auckland: fern frond structure](https://www.nzplants.auckland.ac.nz/en/about/ferns/structure/frond.html)
  provides the rachis / pinna / pinnule hierarchy. The full procedural mesh now
  expresses both branching levels rather than using an incised single blade
  for a whole primary feather.
- [University of Auckland: Cyathea medullaris](https://www.nzplants.auckland.ac.nz/en/about/ferns/native-ferns/cyatheaceae-black-silver-tree-ferns/cyathea-medullaris.html)
  and [Flora of New Zealand: Cyathea dealbata](https://www.nzflora.info/factsheet/taxon/Cyathea-dealbata.html)
  supplied close photographic checks of broad attached pinnule bases, tapered
  primary feathers and the progressively simpler apex. These are morphology
  analogs, not a species identification or a photometric calibration.

The retained artwork still defines the composition target. Research photos are
private inspection material, not new texture assets; all new blade topology is
independently generated geometry. Numerical geometry checks cannot establish
subjective parity with the artwork.
