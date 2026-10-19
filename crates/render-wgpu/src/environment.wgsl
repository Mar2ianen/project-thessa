// Radiance environment field shared by native sky background, indirect diffuse
// and specular reflection terms. Included after `ocean.wgsl` (for OCEAN_PI and
// ocean_safe_normalize); `ray_scene.wgsl` may appear before it, because WGSL
// resolves module-scope declarations across the whole module.
//
// Layout matches `thessa-render-core::environment`:
//   azimuth  = atan2(d.z, d.x), elevation = asin(clamp(d.y, -1, 1))
//   u = (azimuth / TAU + 0.5) * width - 0.5   (wrap)
//   v = (elevation / PI + 0.5) * height - 0.5 (clamp)
// Bilinear in (u, v). `header.x == 0` is the disabled state: the environment
// term is then exactly the legacy isotropic `ambient_lux / PI`, so a renderer
// that never received a field computes byte-identical reflections.
//
// Radiance is stored in the surface-lighting scale (lux per steradian). The
// field is data, never an invented light: a caller evaluates a physical sky
// model onto the grid and the ambient irradiance it reports.

const ENV_TAU: f32 = 6.283185307179586;

struct EnvMeta {
    // x = enabled (0 or 1), y = width, z = height, w = reserved.
    header: vec4<u32>,
    // rgb = cos-weighted local-up hemisphere irradiance in lux, w = reserved.
    ambient: vec4<f32>,
}

@group(0) @binding(3) var<uniform> env_meta: EnvMeta;
@group(0) @binding(4) var<storage, read> env_texels: array<vec4<f32>>;

fn env_enabled() -> bool {
    return env_meta.header.x != 0u;
}

// Ambient irradiance seen by indirect terms: the field's own integrated value
// when a field is bound, otherwise the caller's explicit `light.ambient`.
fn env_ambient_lux(fallback_lux: vec3<f32>) -> vec3<f32> {
    if (!env_enabled()) {
        return fallback_lux;
    }
    return env_meta.ambient.rgb;
}

// Reflected radiance arriving from `direction`, in the same scale as the
// direct light colours. Disabled reproduces the legacy term exactly.
fn env_base_radiance(direction: vec3<f32>, fallback_lux: vec3<f32>) -> vec3<f32> {
    let ambient = env_ambient_lux(fallback_lux);
    if (!env_enabled()) {
        return ambient / OCEAN_PI;
    }
    let width = i32(env_meta.header.y);
    let height = i32(env_meta.header.z);
    if (width < 2 || height < 2) {
        return ambient / OCEAN_PI;
    }
    let d = ocean_safe_normalize(direction, vec3(0.0, 1.0, 0.0));
    let elevation = asin(clamp(d.y, -1.0, 1.0));
    let azimuth = atan2(d.z, d.x);
    let v = clamp((elevation / OCEAN_PI + 0.5) * f32(height) - 0.5, 0.0, f32(height - 1));
    let u = (azimuth / ENV_TAU + 0.5) * f32(width) - 0.5;
    let x0 = floor(u);
    let y0 = floor(v);
    let tx = u - x0;
    let ty = v - y0;
    let x0i = i32(x0);
    let y0i = i32(y0);
    let x0w = ((x0i % width) + width) % width;
    let x1w = (((x0i + 1) % width) + width) % width;
    let y0c = clamp(y0i, 0, height - 1);
    let y1c = clamp(y0i + 1, 0, height - 1);
    let top = mix(env_texels[y0c * width + x0w].rgb, env_texels[y0c * width + x1w].rgb, tx);
    let bottom = mix(env_texels[y1c * width + x0w].rgb, env_texels[y1c * width + x1w].rgb, tx);
    return mix(top, bottom, ty);
}

fn env_radiance_from(position:vec3<f32>,direction:vec3<f32>,fallback_lux:vec3<f32>)->vec3<f32> {
    let column=cloud_transport(position,direction,1e30);
    return env_base_radiance(direction,fallback_lux)*column.a+column.rgb;
}
fn env_radiance(direction:vec3<f32>,fallback_lux:vec3<f32>)->vec3<f32> {
    return env_radiance_from(vec3(0.0),direction,fallback_lux);
}
