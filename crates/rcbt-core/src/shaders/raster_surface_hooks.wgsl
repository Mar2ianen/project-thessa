// Default legacy surface hooks. Native replaces this complete named block at
// shader construction; the shared BRDF/raster source itself is never rewritten.
fn cbt_shadow_visibility(position:vec3<f32>, normal:vec3<f32>, sun:u32)->f32 { return 1.0; }
fn cbt_specular_enabled()->bool { return true; }
