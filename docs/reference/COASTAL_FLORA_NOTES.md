# Coastal plant representation notes (2026-10-26)

The retained coastal artwork is the composition target. Modern fern descriptions
below supply morphology vocabulary and geometric ideas; they do not identify
Thessa's fictional flora as these species or establish fossil reconstruction,
measured pigments, placement density or a climate model. No third-party plant
photograph, mesh, implementation or texture is copied in this continuation.

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
