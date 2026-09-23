use std::{
    error::Error,
    fs::{self, File},
    io::{self, Write},
    path::Path,
};

use glam::DVec3;
use thessa_fuselage::{FuselageMesh, ProceduralBody, tessellate_fuselage};

const AXIAL_SAMPLES_PER_STATION: usize = 4;
const RADIAL_SAMPLES: usize = 32;

pub(super) fn export_body_meshes(
    bodies: &[ProceduralBody],
    output_dir: &Path,
) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(output_dir)?;

    for (index, body) in bodies.iter().enumerate() {
        let mesh = tessellate_fuselage(body, AXIAL_SAMPLES_PER_STATION, RADIAL_SAMPLES)
            .map_err(|error| format!("body '{}': {error}", body.name))?;
        let path = output_dir.join(format!("{index}_{}.obj", sanitize_name(&body.name)));
        let mut file = File::create(path)?;
        write_obj(&mesh, &mut file)?;
    }

    Ok(())
}

fn sanitize_name(name: &str) -> String {
    let mut sanitized = String::with_capacity(name.len());
    let mut separator = false;

    for character in name.chars() {
        if character.is_ascii_alphanumeric() || matches!(character, '-' | '_') {
            if separator && !sanitized.is_empty() {
                sanitized.push('_');
            }
            sanitized.push(character);
            separator = false;
        } else {
            separator = true;
        }
    }

    if sanitized.is_empty() {
        "unnamed".to_owned()
    } else {
        sanitized
    }
}

fn write_obj(mesh: &FuselageMesh, writer: &mut impl Write) -> io::Result<()> {
    if mesh.positions.len() != mesh.normals.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "OBJ mesh positions and normals must be aligned",
        ));
    }
    if mesh.indices.len() % 3 != 0
        || mesh
            .indices
            .iter()
            .any(|index| *index as usize >= mesh.positions.len())
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "OBJ mesh indices must contain in-range triangles",
        ));
    }

    for position in &mesh.positions {
        write_vector(writer, "v", *position)?;
    }
    for normal in &mesh.normals {
        write_vector(writer, "vn", *normal)?;
    }
    for triangle in mesh.indices.chunks_exact(3) {
        writeln!(
            writer,
            "f {}//{} {}//{} {}//{}",
            triangle[0] + 1,
            triangle[0] + 1,
            triangle[1] + 1,
            triangle[1] + 1,
            triangle[2] + 1,
            triangle[2] + 1,
        )?;
    }

    Ok(())
}

fn write_vector(writer: &mut impl Write, kind: &str, vector: DVec3) -> io::Result<()> {
    // Seventeen digits after the leading digit are sufficient to round-trip
    // every f64 component; scientific notation also preserves tiny values.
    writeln!(
        writer,
        "{kind} {:.17e} {:.17e} {:.17e}",
        vector.x, vector.y, vector.z
    )
}

#[cfg(test)]
mod tests {
    use std::{
        fs,
        path::PathBuf,
        process,
        sync::atomic::{AtomicUsize, Ordering},
    };

    use super::*;
    use thessa_fuselage::BodyStation;

    #[test]
    fn obj_counts_vertices_normals_faces_and_aligns_indices() {
        let precise = 0.123_456_789_012_345_66_f64;
        let mesh = FuselageMesh {
            positions: vec![
                DVec3::new(precise, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![DVec3::Z; 4],
            indices: vec![0, 1, 2, 0, 2, 3],
        };
        let mut obj = Vec::new();
        write_obj(&mesh, &mut obj).unwrap();
        let obj = String::from_utf8(obj).unwrap();
        let lines: Vec<_> = obj.lines().collect();

        assert_eq!(
            lines.iter().filter(|line| line.starts_with("v ")).count(),
            4
        );
        assert_eq!(
            lines.iter().filter(|line| line.starts_with("vn ")).count(),
            4
        );
        let faces: Vec<_> = lines
            .iter()
            .filter(|line| line.starts_with("f "))
            .copied()
            .collect();
        assert_eq!(faces.len(), 2);
        assert_eq!(faces[0], "f 1//1 2//2 3//3");
        assert_eq!(faces[1], "f 1//1 3//3 4//4");

        let serialized_precise = lines[0]
            .split_whitespace()
            .nth(1)
            .expect("first vertex has an x coordinate")
            .parse::<f64>()
            .unwrap();
        assert_eq!(serialized_precise.to_bits(), precise.to_bits());
    }

    #[test]
    fn export_creates_missing_directories_and_sanitizes_path_separators() {
        let root = unique_test_directory();
        let output_dir = root.join("new").join("obj");
        let body = ProceduralBody::new(
            "../nested\\body",
            vec![
                BodyStation::new(0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
                BodyStation::new(2.0, 1.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
            ],
            DVec3::ZERO,
        )
        .unwrap();

        export_body_meshes(&[body], &output_dir).unwrap();

        let exported = output_dir.join("0_nested_body.obj");
        assert!(exported.is_file());
        assert!(!root.join("nested").exists());
        fs::remove_dir_all(root).unwrap();
    }

    fn unique_test_directory() -> PathBuf {
        static NEXT_ID: AtomicUsize = AtomicUsize::new(0);

        loop {
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "thessa-vehicle-baker-debug-mesh-{}-{id}",
                process::id()
            ));
            match fs::create_dir(&path) {
                Ok(()) => return path,
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("could not create test directory: {error}"),
            }
        }
    }
}
