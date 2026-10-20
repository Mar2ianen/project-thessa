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

// Deterministic GGX visible-normal importance integration of the existing field
// (Heitz 2018, Sampling the GGX Distribution of Visible Normals, JCGT 7(4)).
// Same Schlick-Smith visibility, alpha and Fresnel as surface_sun_reflection.
// Sample count changes quadrature only, never source flux or the material.
// Keeping rejected below-surface samples in the denominator is essential:
// renormalising only the surviving directions would create energy at grazing.
fn env_specular_from(position:vec3<f32>,normal:vec3<f32>,eye:vec3<f32>,roughness:f32,f0:vec3<f32>,ambient:vec3<f32>,samples:u32,cloud_budget:vec3<u32>)->vec3<f32> {
    let nv=max(dot(normal,eye),0.0);
    if(!env_enabled() || samples==0u){return vec3(0.0);}
    let helper=select(vec3(0.0,1.0,0.0),vec3(1.0,0.0,0.0),abs(normal.y)>0.9);
    let tangent=normalize(cross(helper,normal));let bitangent=cross(normal,tangent);
    let r=clamp(roughness,0.02,1.0);let alpha=max(r*r,0.0025);let a2=alpha*alpha;
    let k=(r+1.0)*(r+1.0)*0.125;
    let gv=nv/max(nv*(1.0-k)+k,OCEAN_EPSILON);
    // Stretch the local view, then sample the projected visible disk. Unlike
    // ordinary NDF sampling this remains bounded and useful at grazing angles.
    let vh_local=normalize(vec3(alpha*dot(eye,tangent),alpha*dot(eye,bitangent),nv));
    let lensq=dot(vh_local.xy,vh_local.xy);
    var t1=vec3(1.0,0.0,0.0);
    if(lensq>0.0){t1=vec3(-vh_local.y,vh_local.x,0.0)*inverseSqrt(lensq);}
    let t2=cross(vh_local,t1);
    let gv_exact=2.0*nv/max(nv+sqrt(a2+(1.0-a2)*nv*nv),OCEAN_EPSILON);
    // Bound nested volume work: rough sky quadrature is cheap, a nested full
    // primary cloud march per sample is not. Keep every cloud phenomenon and
    // change integration budgets only. Surface Low/Medium/High (4/8/16 clear
    // samples) become 1/2/4 directions with 1/2/4 cloud steps and <=1 sun step.
    let cloudy=cloud_frame.wind.y!=0.0 && cloud_budget.x>0u;
    let count=select(samples,cloud_budget.x,cloudy);
    var result=vec3(0.0);
    for(var i=0u;i<count;i++) {
        let xi=(f32(i)+0.5)/f32(count);
        let phi=ENV_TAU*f32(reverseBits(i))*2.3283064365386963e-10;
        let radius=sqrt(xi);let x=radius*cos(phi);
        let s=0.5*(1.0+vh_local.z);
        let y=(1.0-s)*sqrt(max(0.0,1.0-x*x))+s*radius*sin(phi);
        let disk=t1*x+t2*y+vh_local*sqrt(max(0.0,1.0-x*x-y*y));
        let local_h=normalize(vec3(alpha*disk.x,alpha*disk.y,max(0.0,disk.z)));
        let h=tangent*local_h.x+bitangent*local_h.y+normal*local_h.z;
        let vh=dot(eye,h);if(vh<=0.0){continue;}
        let l=reflect(-eye,h);let nl=dot(normal,l);if(nl<=0.0){continue;}
        let gl=nl/max(nl*(1.0-k)+k,OCEAN_EPSILON);
        let fresnel=f0+(vec3(1.0)-f0)*pow(1.0-vh,5.0);
        // The sampling PDF uses exact GGX G1. Correct to the Schlick-Smith
        // constitutive BRDF shared with the direct-light kernel, not a second
        // material law. Do not add an empirical roughness brightness factor.
        let weight=gv*gl/max(gv_exact,OCEAN_EPSILON);
        var incoming=env_base_radiance(l,ambient);
        if(cloudy){
            let column=cloud_transport_column(position,l,cloud_budget.y,cloud_budget.z);
            incoming=incoming*column.a+column.rgb;
        }
        result+=incoming*fresnel*weight;
    }
    let integrated=result/f32(count);
    // At a silhouette the visible-normal quadrature can reject every sample
    // because its reflected ray points below the local horizon. Fresnel then
    // removes the diffuse term too, so the fragment would tonemap to black
    // along every ridge. Keep a continuous, bounded sky floor in the grazing
    // band; it is a numerical horizon guard, not a second BRDF or an empirical
    // brightness factor. Rejected samples still stay in the denominator above
    // the band, and the nv >= 0.2 oracle remains unchanged.
    let horizon=1.0-smoothstep(0.0,0.2,nv);
    let grazing=f0+(vec3(1.0)-f0)*pow(1.0-nv,5.0);
    let floor=env_base_radiance(normal,ambient)*grazing*horizon;
    return max(integrated,floor);
}
