// Merge integration boundaries, then sum emission/extinction before transport.
// Independent alpha-sorted plume draws are incorrect for overlapping media.
struct Multi { rect: vec4<f32>, config: vec4<f32>, }
@group(1) @binding(0) var<uniform> multi: Multi;
@group(1) @binding(3) var<storage,read> fields: array<Draw>;

fn select_field(index: u32) {
    draw = fields[index]; station_base = index*64u; center_base = index*33u;
}

@vertex fn multi_vertex(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let corners = array<vec2<f32>,6>(vec2(0.,0.),vec2(1.,0.),vec2(1.,1.),vec2(0.,0.),vec2(1.,1.),vec2(0.,1.));
    let pad = 2.0/vec2<f32>(textureDimensions(scene_depth));
    return vec4(mix(multi.rect.xy-pad,multi.rect.zw+pad,corners[index]),0.,1.);
}

fn field_medium(p: vec3<f32>) -> Station {
    var local = CurveLocation(vec3(0.),0.,draw.axis_radius.xyz,false,1.);
    if draw.flow.w >= 2. {
        local = curve_location(p);
    } else {
        let offset = p-draw.origin_len.xyz;
        let z = dot(offset,draw.axis_radius.xyz);
        local = CurveLocation(offset-draw.axis_radius.xyz*z,z,draw.axis_radius.xyz,z>=0. && z<=draw.origin_len.w,1.);
    }
    if !local.valid || length(local.radial)>draw.axis_radius.w {
        return Station(vec4(0.),vec4(0.));
    }
    let station = sample_station(local.z);
    let radius = max(station.geometry.y,1e-6);
    let core_r = length(local.radial);
    var r = core_r;
    if draw.controls.w>0. {
        let u1 = transported_radial_reference(local.axis);
        let u2 = cross(local.axis,u1);
        let az = atan2(dot(local.radial,u2),dot(local.radial,u1)+1e-6);
        let phase = (local.z-draw.controls.z*draw.flow.x)/(4.*radius);
        let n = (sin(phase+3.*az)+sin(phase*1.7-5.*az))*.25;
        r += n*draw.controls.w*radius*smoothstep(.1,.9,r/radius);
    }
    let u = abs(r)/radius;
    let core_u = core_r/radius;
    var weight = (exp(-2.2*core_u*core_u)+.18*exp(-.7*u*u))*local.metric;
    if draw.flow.z>0. { weight *= 1.-smoothstep(.70,1.,local.z/draw.origin_len.w); }
    var shock = 1.; var cells = vec2(1.);
    if draw.flow.y>0. {
        shock = shock_multiplier(station.geometry.w,core_u);
        cells = spatial_shock(local.z,core_r,draw.shock_cells);
    }
    return Station(vec4(max(station.geometry.z*weight*shock*cells.x,0.),0.,0.,0.),
        vec4(max(station.emission.rgb*weight*shock*cells.y,vec3(0.)),0.));
}

// Clip a conservative capsule to an actual flat endpoint, before choosing bins.
fn clip_endpoint(hit:vec2<f32>,ro:vec3<f32>,rd:vec3<f32>,point:vec3<f32>,inward:vec3<f32>)->vec2<f32> {
    let origin=dot(ro-point,inward);let direction=dot(rd,inward);
    if abs(direction)<1e-7 {
        if origin<0. {return vec2(1e30,-1e30);}
        return hit;
    }
    let crossing=-origin/direction;
    if direction>0. {return vec2(max(hit.x,crossing),hit.y);}
    return vec2(hit.x,min(hit.y,crossing));
}

// near/far, nearest ray distance, radial warp scale. Conservative capsule
// bounds preserve curved crossings; field_medium retains flat inlet/outlet.
fn field_chord(ro:vec3<f32>, rd:vec3<f32>, opaque:f32) -> vec4<f32> {
    var near = 1e30; var far = -1e30;
    var nearest2 = 1e30; var closest = 0.; var closest_z = 0.;
    let count = max(2u,u32(draw.flow.w));
    for (var j=0u;j+1u<count;j+=1u) {
        var a = vec4(draw.origin_len.xyz,0.);
        var b = vec4(draw.origin_len.xyz+draw.axis_radius.xyz*draw.origin_len.w,draw.origin_len.w);
        if draw.flow.w>=2. { a=centers[center_base+j]; b=centers[center_base+j+1u]; }
        var hit = curve_capsule_interval(ro,rd,a.xyz,b.xyz,draw.axis_radius.w);
        if draw.flow.w>=2. {
            let axis=normalize(b.xyz-a.xyz);
            if j==0u {hit=clip_endpoint(hit,ro,rd,a.xyz,axis);}
            if j+2u==count {hit=clip_endpoint(hit,ro,rd,b.xyz,-axis);}
        }
        if hit.y>hit.x { near=min(near,hit.x); far=max(far,min(hit.y,opaque)); }
        let ab=b.xyz-a.xyz; let offset=a.xyz-ro; let lateral=ab-rd*dot(ab,rd);
        let fraction=clamp(-dot(offset-rd*dot(offset,rd),lateral)/max(dot(lateral,lateral),1e-12),0.,1.);
        let p=mix(a.xyz,b.xyz,fraction); let t=max(dot(p-ro,rd),0.);
        let delta=p-ro-rd*t; let d2=dot(delta,delta);
        if d2<nearest2 { nearest2=d2; closest=t; closest_z=mix(a.w,b.w,fraction); }
    }
    if draw.flow.w<2. {
        let da=dot(rd,draw.axis_radius.xyz);
        let oa=dot(ro-draw.origin_len.xyz,draw.axis_radius.xyz);
        if abs(da)<1e-7 {
            if oa<0. || oa>draw.origin_len.w { return vec4(1e30,-1e30,0.,1.); }
        } else {
            let a=-oa/da;let b=(draw.origin_len.w-oa)/da;
            near=max(near,min(a,b));far=min(far,max(a,b));
        }
    }
    if far<=near { return vec4(1e30,-1e30,0.,1.); }
    closest=clamp(closest,near,far);
    var radius=max(2.*sample_station(closest_z).geometry.y,1e-4);
    if draw.flow.y>0. && draw.shock_cells.x>0. && draw.shock_cells.z>1.
        && nearest2<1.44*draw.shock_cells.y*draw.shock_cells.y {
        radius=min(radius,draw.shock_cells.y);
        if abs(dot(rd,draw.axis_radius.xyz))>.975 {
            closest=near; radius=max(draw.shock_cells.x,1e-4);
        }
    }
    return vec4(near,far,closest,radius);
}

fn field_steps(chord:vec4<f32>,rd:vec3<f32>) -> u32 {
    var steps=u32(clamp(draw.controls.y,1.,16.));
    if draw.flow.y>0. && draw.shock_cells.x>0. && draw.shock_cells.z>1. {
        let axial=select(abs(dot(rd,draw.axis_radius.xyz)),1.,draw.flow.w>=2.);
        steps=max(steps,u32(clamp(ceil(draw.controls.y*draw.shock_cells.z),1.,64.)));
        steps=max(steps,u32(clamp(ceil(axial*(chord.y-chord.x)/draw.shock_cells.x*8.),1.,64.)));
    }
    return steps;
}

fn bin_edge(chord:vec4<f32>,cursor:u32,steps:u32)->f32 {
    if cursor>steps { return 1e30; }
    if cursor==0u { return chord.x; }
    if cursor==steps { return chord.y; }
    let a=atan((chord.x-chord.z)/chord.w); let b=atan((chord.y-chord.z)/chord.w);
    return chord.z+chord.w*tan(mix(a,b,f32(cursor)/f32(steps)));
}

fn trace_multi(pixel:vec2<f32>)->vec4<f32> {
    let ro=frame.camera.xyz; let rd=normalize(unproject(pixel,.5)-ro);
    let depth=textureLoad(scene_depth,vec2<i32>(pixel),0);
    var opaque=1e30;
    if depth<1. { opaque=dot(unproject(pixel,depth)-ro,rd); }
    let count=u32(multi.config.x);
    var chords:array<vec4<f32>,64>;
    var cursors:array<u32,64>; var steps:array<u32,64>; var edges:array<f32,64>;
    let extent=vec2<f32>(textureDimensions(scene_depth));
    let ndc=vec2(pixel.x/extent.x*2.-1.,1.-pixel.y/extent.y*2.);
    var position=1e30;
    for(var i=0u;i<count;i+=1u) {
        select_field(i);
        chords[i]=vec4(1e30,-1e30,0.,1.); cursors[i]=0u; steps[i]=0u; edges[i]=1e30;
        let pad=2./extent;
        if any(ndc<draw.rect.xy-pad) || any(ndc>draw.rect.zw+pad) { continue; }
        chords[i]=field_chord(ro,rd,opaque);
        if chords[i].y<=chords[i].x { continue; }
        steps[i]=field_steps(chords[i],rd); edges[i]=chords[i].x;
        position=min(position,edges[i]);
    }
    var rgb=vec3(0.); var transmittance=1.;
    // At most 65 edges per supported source. Each step consumes an edge.
    for(var event=0u;event<4160u;event+=1u) {
        var next=1e30;
        for(var i=0u;i<count;i+=1u) { next=min(next,edges[i]); }
        if next>=1e30 { break; }
        if next>position {
            let point=ro+rd*((position+next)*.5);
            var sigma=0.; var emission=vec3(0.);
            for(var i=0u;i<count;i+=1u) {
                if position>=chords[i].y || next<=chords[i].x { continue; }
                select_field(i); let medium=field_medium(point);
                sigma+=medium.geometry.x; emission+=medium.emission.rgb;
            }
            let dt=next-position; let tau=sigma*dt; let absorb=exp(-tau);
            var length=dt*(1.-tau*(.5-tau/6.));
            if tau>=.01 { length=(1.-absorb)/sigma; }
            rgb+=transmittance*emission*length; transmittance*=absorb;
        }
        for(var i=0u;i<count;i+=1u) {
            if edges[i]<=next { cursors[i]+=1u; edges[i]=bin_edge(chords[i],cursors[i],steps[i]); }
        }
        position=next;
    }
    return vec4(rgb*frame.exposure.x,clamp(1.-transmittance,0.,1.));
}

@fragment fn multi_fragment(@builtin(position) pixel:vec4<f32>)->@location(0) vec4<f32> {
    var grid=1u;
    let ro=frame.camera.xyz; let rd=normalize(unproject(pixel.xy,.5)-ro);
    for(var i=0u;i<u32(multi.config.x);i+=1u) {
        select_field(i);
        if draw.radial_reference.w<=1. { continue; }
        var nearest=1e30; var point=draw.origin_len.xyz; var z=0.;
        let count=max(2u,u32(draw.flow.w));
        for(var j=0u;j+1u<count;j+=1u) {
            var a=vec4(draw.origin_len.xyz,0.);
            var b=vec4(draw.origin_len.xyz+draw.axis_radius.xyz*draw.origin_len.w,draw.origin_len.w);
            if draw.flow.w>=2. { a=centers[center_base+j];b=centers[center_base+j+1u]; }
            let ab=b.xyz-a.xyz;let offset=a.xyz-ro;let lateral=ab-rd*dot(ab,rd);
            let f=clamp(-dot(offset-rd*dot(offset,rd),lateral)/max(dot(lateral,lateral),1e-12),0.,1.);
            let p=mix(a.xyz,b.xyz,f);let delta=p-ro-rd*max(dot(p-ro,rd),0.);
            if dot(delta,delta)<nearest {nearest=dot(delta,delta);point=p;z=mix(a.w,b.w,f);}
        }
        grid=max(grid,pixel_grid(pixel.xy,point,z));
    }
    var sum=vec4(0.);
    for(var y=0u;y<grid;y+=1u) { for(var x=0u;x<grid;x+=1u) {
        let offset=(vec2<f32>(f32(x),f32(y))+.5)/f32(grid)-.5;
        sum+=trace_multi(pixel.xy+offset);
    }}
    return sum/f32(grid*grid);
}
