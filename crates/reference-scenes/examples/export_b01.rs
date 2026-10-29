//! Export the exact generated scene without a GPU or game runtime.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::path::PathBuf::from(
        std::env::args()
            .nth(1)
            .ok_or("usage: export_b01 NEW_DIRECTORY")?,
    );
    std::fs::create_dir(&path)?;
    let scene = thessa_reference_scenes::MaterialLab::new();
    std::fs::write(path.join("scene.glb"), scene.glb())?;
    std::fs::write(
        path.join("scene.json"),
        serde_json::to_vec_pretty(&scene.description())?,
    )?;
    std::fs::write(
        path.join("scene.LICENSE.txt"),
        thessa_reference_scenes::GEOMETRY_LICENSE_NOTICE,
    )?;
    Ok(())
}
