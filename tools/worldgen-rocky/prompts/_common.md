# Common gore rules (all 7 layers)

- Rocky planet only. No clouds, no haze, no atmosphere beauty pass.
- Projection: orange-slice gore, one longitude segment per image + polar caps
  separately. Same coastline/shape across all 7 layers, pixel-aligned.
- Framing: flat orthographic strip, no perspective, no vignette, no text.
- Overlap: paint ~6% extra beyond each edge so neighbours stitch seamlessly.
- Resolution-agnostic: keep large shapes clean at any size; fine grain is added
  procedurally later, so do NOT bake film grain or JPEG artifacts.
- Model: GPT Image 2.5 (Flare for drafts, Sunburst for final), quality
  high/xhigh, size free within 16px / ratio / 3840px limits.
