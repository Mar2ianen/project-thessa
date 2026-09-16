//! Validation for the standalone ocean helper consumed by raster shaders.

const OCEAN_WGSL: &str = include_str!("../src/ocean.wgsl");

#[test]
fn ocean_shader_is_portable_wgsl() {
    let module =
        naga::front::wgsl::parse_str(OCEAN_WGSL).expect("ocean helper must parse as portable WGSL");
    naga::valid::Validator::new(Default::default(), Default::default())
        .validate(&module)
        .expect("ocean helper must validate as portable WGSL");
}
