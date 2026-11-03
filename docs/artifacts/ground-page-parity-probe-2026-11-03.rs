use thessa_flight_authority::launch_site::canonical_world_field;
use thessa_worldgen_rocky::lod::{self, TileKey};

fn main() {
    let field = canonical_world_field().unwrap();
    let p = [-2835051.776623983_f64, 1279289.634545036, -752292.6983244818];
    let dir = {
        let n = p.iter().map(|x| x*x).sum::<f64>().sqrt();
        p.map(|x| x/n)
    };
    let (face,u,v) = if dir[0].abs() >= dir[1].abs() && dir[0].abs() >= dir[2].abs() {
        if dir[0] > 0.0 { (0, -dir[2]/dir[0], dir[1]/dir[0]) }
        else { (1, dir[2]/-dir[0], dir[1]/-dir[0]) }
    } else if dir[1].abs() >= dir[2].abs() {
        if dir[1] > 0.0 { (2, dir[0]/dir[1], -dir[2]/dir[1]) }
        else { (3, dir[0]/-dir[1], dir[2]/-dir[1]) }
    } else if dir[2] > 0.0 { (4, dir[0]/dir[2], dir[1]/dir[2]) }
      else { (5, -dir[0]/-dir[2], dir[1]/-dir[2]) };
    let cpu = field.height_m(dir, 32.0).max(0.0) + lod::ground_relief_height_m(&field, dir, 0.012);
    println!("dir={dir:?} u={u:.9} v={v:.9} cpu_ground_height={cpu:.9}");
    for level in [7_u8, 17, 18, 22] {
        let n = (1_u32 << level) as f64;
        let tx = ((u+1.0)*0.5*n).clamp(0.0,n-f64::EPSILON);
        let ty = ((v+1.0)*0.5*n).clamp(0.0,n-f64::EPSILON);
        let key=TileKey{face,level,x:tx.floor() as u32,y:ty.floor() as u32};
        let local=[tx.fract(),ty.fract()];
        let max_error = if key.span_m(field.params.radius_m) / 32.0 < 4.0 { 0.001 } else { 0.5 };
        let page=lod::bake_height_page_with_ground_detail(&field,key,33,max_error,true).unwrap();
        let h=page.sample(local[0] as f32,local[1] as f32) as f64;
        let base=field.height_m(dir,(key.span_m(field.params.radius_m)/32.0).max(32.0)).max(0.0);
        let relief=if base>0.0 {lod::ground_relief_height_m(&field,dir,key.span_m(field.params.radius_m)/32.0)} else {0.0};
        println!("L{level} key=({},{},{}) uv={local:?} page_height={h:.9} delta_page_minus_cpu={:.9} base={base:.9} relief={relief:.9} max_error={max_error:.3} qerr={:.9}",key.face,key.x,key.y,h-cpu,page.max_residual_error_m());
    }
}
