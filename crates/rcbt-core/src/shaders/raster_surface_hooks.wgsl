// Default legacy surface hooks. Native replaces this complete named block at
// shader construction; the shared BRDF/raster source itself is never rewritten.
fn cbt_shadow_visibility(position:vec3<f32>, normal:vec3<f32>, sun:u32)->f32 { return 1.0; }
fn cbt_specular_enabled()->bool { return true; }
// RGB reflected radiance; alpha directional-hemispherical dielectric reflectance.
// Legacy consumers keep their existing macro-normal Schlick split unchanged.
fn cbt_land_environment(position:vec3<f32>,normal:vec3<f32>,eye:vec3<f32>,roughness:f32,ambient:vec3<f32>)->vec4<f32>{return vec4(vec3(0.0),ocean_schlick(0.04,max(dot(normal,eye),0.0)));}
fn cbt_land_transmission(normal:vec3<f32>,light:vec3<f32>,roughness:f32)->f32{return 1.0;}
fn cbt_land_ambient_transmission(roughness:f32)->f32{return 1.0;}
fn cbt_spectral_ocean(position:vec3<f32>,normal:vec3<f32>,direction:vec3<f32>,height:f32,roughness:f32,albedo:vec3<f32>,footprint:f32)->vec4<f32>{return vec4(0.0);}
