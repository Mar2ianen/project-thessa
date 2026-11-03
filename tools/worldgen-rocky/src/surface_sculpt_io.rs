//! World-owned file envelope for the backend-neutral sparse sculpt snapshot.
//! The caller supplies a stable body/source key; radius alone is not identity.
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};
use thessa_rcbt_core::surface_sculpt::SurfaceSculpt;

const MAGIC: &[u8; 4] = b"TSA1";
const MAX_WORLD_KEY_BYTES: usize = 4096;
static NEXT_TEMP: AtomicU64 = AtomicU64::new(0);

fn validate_key(key: &str) -> Result<(), String> {
    if key.is_empty() || key.len() > MAX_WORLD_KEY_BYTES {
        return Err("sculpt world key must contain 1..4096 UTF-8 bytes".into());
    }
    Ok(())
}

/// Publish a complete checkpoint by same-directory atomic rename. Prior files
/// survive failures before publication. File content is synced before rename;
/// power-loss durability of directory metadata is filesystem/platform dependent.
pub fn save(path: &Path, world_key: &str, layer: &SurfaceSculpt) -> Result<(), String> {
    validate_key(world_key)?;
    // create_new below also refuses a stale name after PID/counter reuse.
    let sequence = NEXT_TEMP.fetch_add(1, Ordering::Relaxed);
    let mut name = path.as_os_str().to_os_string();
    name.push(format!(".partial-{}-{sequence}", std::process::id()));
    let partial = std::path::PathBuf::from(name);
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&partial)
        .map_err(|e| e.to_string())?;
    let result = (|| {
        file.write_all(MAGIC).map_err(|e| e.to_string())?;
        file.write_all(&(world_key.len() as u32).to_le_bytes())
            .map_err(|e| e.to_string())?;
        file.write_all(world_key.as_bytes())
            .map_err(|e| e.to_string())?;
        file.write_all(&layer.to_bytes())
            .map_err(|e| e.to_string())?;
        file.sync_all().map_err(|e| e.to_string())?;
        drop(file);
        fs::rename(&partial, path).map_err(|e| e.to_string())
    })();
    let _ = fs::remove_file(&partial);
    result
}

/// Read only the expected world and caller-owned page budget. Length is bounded
/// during the read as well as during decoding; no capacity is trusted from disk.
pub fn load(
    path: &Path,
    expected_world_key: &str,
    max_pages: usize,
) -> Result<SurfaceSculpt, String> {
    validate_key(expected_world_key)?;
    let limit = SurfaceSculpt::snapshot_size_limit(max_pages)
        .and_then(|n| n.checked_add(1))
        .ok_or("sculpt snapshot byte limit overflow")?;
    let mut file = File::open(path).map_err(|e| e.to_string())?;
    let mut header = [0; 8];
    file.read_exact(&mut header).map_err(|e| e.to_string())?;
    if &header[..4] != MAGIC {
        return Err("unsupported sculpt archive format".into());
    }
    let key_size = u32::from_le_bytes(header[4..8].try_into().unwrap()) as usize;
    if key_size == 0 || key_size > MAX_WORLD_KEY_BYTES {
        return Err("invalid sculpt world key length".into());
    }
    let mut key = vec![0; key_size];
    file.read_exact(&mut key).map_err(|e| e.to_string())?;
    if key != expected_world_key.as_bytes() {
        return Err("sculpt snapshot belongs to a different world".into());
    }
    let mut bytes = Vec::new();
    file.take(limit as u64)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() >= limit {
        return Err("sculpt file exceeds caller byte budget".into());
    }
    SurfaceSculpt::from_bytes(&bytes, max_pages).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Temp(std::path::PathBuf);
    impl Temp {
        fn new() -> Self {
            let n = NEXT_TEMP.fetch_add(1, Ordering::Relaxed);
            let dir =
                std::env::temp_dir().join(format!("thessa-sculpt-io-{}-{n}", std::process::id()));
            fs::create_dir(&dir).unwrap();
            Self(dir)
        }
        fn path(&self) -> std::path::PathBuf {
            self.0.join("planet.sculpt")
        }
    }
    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }
    fn layer() -> SurfaceSculpt {
        let mut layer = SurfaceSculpt::new(3_200_000.0, 0.025, 64).unwrap();
        layer.apply_dab([1.0, 0.0, 1.0], 0.20, -0.04).unwrap();
        layer
    }
    #[test]
    fn checkpoint_round_trip_and_replacement_preserve_exact_state() {
        let temp = Temp::new();
        let path = temp.path();
        let mut layer = layer();
        save(&path, "body/1/source-a", &layer).unwrap();
        assert_eq!(
            load(&path, "body/1/source-a", 64).unwrap().to_bytes(),
            layer.to_bytes()
        );
        layer.apply_dab([1.0, 1.0, 1.0], 0.20, -0.03).unwrap();
        save(&path, "body/1/source-a", &layer).unwrap();
        assert_eq!(
            load(&path, "body/1/source-a", 64).unwrap().to_bytes(),
            layer.to_bytes()
        );
        assert_eq!(
            fs::read_dir(&temp.0).unwrap().count(),
            1,
            "no partial files remain"
        );
    }
    #[test]
    fn wrong_world_budget_and_corruption_do_not_replace_current_state() {
        let temp = Temp::new();
        let path = temp.path();
        let layer = layer();
        let original = layer.to_bytes();
        save(&path, "body/1", &layer).unwrap();
        assert!(load(&path, "body/2", 64).is_err());
        assert!(load(&path, "body/1", 1).is_err());
        let mut bytes = fs::read(&path).unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 1;
        fs::write(&path, bytes).unwrap();
        assert!(load(&path, "body/1", 64).is_err());
        assert_eq!(
            layer.to_bytes(),
            original,
            "failed load cannot mutate live source"
        );
    }
    #[test]
    fn saved_layer_reenters_canonical_contact_and_prefix_queries() {
        let recipe: crate::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        let field = crate::field::field_from_manifest(
            &crate::spec_recipe::manifest_from_spec(&recipe).unwrap(),
        )
        .unwrap();
        let inv = 1.0 / 2.0_f64.sqrt();
        let dir = [inv, 0.0, inv];
        let before = field.height_m(dir, 32.0);
        let mut sculpt = SurfaceSculpt::new(field.params.radius_m, 0.025, 64).unwrap();
        sculpt.apply_dab(dir, 0.2, -0.04).unwrap();
        let offset = sculpt.sample(dir).unwrap();
        let temp = Temp::new();
        save(&temp.path(), "canonical-test-body/source-v1", &sculpt).unwrap();
        let loaded = load(&temp.path(), "canonical-test-body/source-v1", 64).unwrap();
        let field = field.with_surface_sculpt(loaded).unwrap();
        assert!((field.height_m(dir, 32.0) - before - offset).abs() < 1e-8);
        let (prefix, macro_h) = field.height_prefix_m(dir);
        assert!(
            (field
                .sample_surface_from_prefix(dir, prefix, macro_h, 32.0)
                .height_m
                - field.height_m(dir, 32.0))
            .abs()
                < 1e-8
        );
    }

    #[test]
    fn publication_failure_keeps_previous_data_and_cleans_partial() {
        let temp = Temp::new();
        let path = temp.path();
        let layer = layer();
        save(&path, "body/1", &layer).unwrap();
        let old = fs::read(&path).unwrap();
        assert!(save(&path, "", &layer).is_err());
        assert_eq!(fs::read(&path).unwrap(), old);
        let directory = temp.0.join("occupied");
        fs::create_dir(&directory).unwrap();
        fs::write(directory.join("keep"), b"prior data").unwrap();
        assert!(save(&directory, "body/1", &layer).is_err());
        assert_eq!(fs::read(directory.join("keep")).unwrap(), b"prior data");
        assert_eq!(
            fs::read_dir(&temp.0).unwrap().count(),
            2,
            "failed publication removes its owned partial"
        );
    }
}

#[cfg(test)]
mod envelope_acceptance {
    use super::*;
    #[test]
    fn file_envelope_lengths_and_byte_budget_are_bounded_before_decode() {
        let n = NEXT_TEMP.fetch_add(1, Ordering::Relaxed);
        let path =
            std::env::temp_dir().join(format!("thessa-sculpt-envelope-{}-{n}", std::process::id()));
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .unwrap();
        file.write_all(MAGIC).unwrap();
        file.write_all(&u32::MAX.to_le_bytes()).unwrap();
        drop(file);
        assert!(
            load(&path, "body/1", 1)
                .err()
                .unwrap()
                .contains("key length")
        );
        let mut bytes = MAGIC.to_vec();
        bytes.extend(6_u32.to_le_bytes());
        bytes.extend(b"body/1");
        bytes.resize(
            bytes.len() + SurfaceSculpt::snapshot_size_limit(1).unwrap() + 1,
            0,
        );
        fs::write(&path, bytes).unwrap();
        assert!(
            load(&path, "body/1", 1)
                .err()
                .unwrap()
                .contains("byte budget")
        );
        fs::write(&path, b"TSA").unwrap();
        assert!(load(&path, "body/1", 1).is_err());
        fs::remove_file(path).unwrap();
    }
}
