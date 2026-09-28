//! Decoded comparison image dumps (doc §18 Phase A: "dump decoded
//! comparison images").
//!
//! Binary PGM (`P5`) output needs no dependency and opens in every image
//! viewer. [`comparison`] lays out original, decoded, and amplified
//! absolute difference side by side with a 1-texel separator.

use std::{fs, io, path::Path};

use crate::codec::ScalarField;

/// Write one 8-bit grayscale PGM file.
pub fn write_pgm(path: &Path, field: &ScalarField) -> io::Result<()> {
    let mut out = Vec::with_capacity(field.data.len() + 32);
    out.extend_from_slice(format!("P5\n{} {}\n255\n", field.width, field.height).as_bytes());
    out.extend_from_slice(&field.data);
    fs::write(path, out)
}

/// Read back a PGM written by [`write_pgm`] (exact `P5\nW H\n255\n` header).
pub fn read_pgm(path: &Path) -> io::Result<ScalarField> {
    let bytes = fs::read(path)?;
    let mut parts = bytes.splitn(4, |b| *b == b'\n');
    let magic = parts.next().unwrap_or(&[]);
    let dims = parts.next().unwrap_or(&[]);
    let maxval = parts.next().unwrap_or(&[]);
    let pixels = parts.next().unwrap_or(&[]);
    if magic != b"P5" || maxval != b"255" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "not a P5/maxval-255 pgm",
        ));
    }
    let dims = std::str::from_utf8(dims)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "bad pgm dims"))?;
    let mut it = dims.split_whitespace();
    let width: u32 = it
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "bad pgm width"))?;
    let height: u32 = it
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "bad pgm height"))?;
    if pixels.len() != width as usize * height as usize {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "pgm pixel count mismatch",
        ));
    }
    Ok(ScalarField {
        width,
        height,
        data: pixels.to_vec(),
    })
}

/// Side-by-side triptych: original | decoded | amplified `|diff| x 16`
/// (saturating), with a 1-texel black separator between panels.
pub fn comparison(original: &ScalarField, decoded: &ScalarField) -> ScalarField {
    assert_eq!(
        (original.width, original.height),
        (decoded.width, decoded.height),
        "comparison needs identical extents"
    );
    let w = original.width as usize;
    let h = original.height as usize;
    let out_w = (w * 3 + 2) as u32;
    let mut data = vec![0u8; out_w as usize * h];
    for y in 0..h {
        for x in 0..w {
            let a = original.data[y * w + x];
            let b = decoded.data[y * w + x];
            let d = a.abs_diff(b).saturating_mul(16);
            data[y * out_w as usize + x] = a;
            data[y * out_w as usize + w + 1 + x] = b;
            data[y * out_w as usize + 2 * w + 2 + x] = d;
        }
    }
    ScalarField {
        width: out_w,
        height: h as u32,
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static COUNTER: AtomicU64 = AtomicU64::new(0);

    fn scratch(name: &str) -> std::path::PathBuf {
        let id = COUNTER.fetch_add(1, Ordering::SeqCst);
        std::env::temp_dir().join(format!(
            "microstore_{}_{}_{}.pgm",
            std::process::id(),
            id,
            name
        ))
    }

    #[test]
    fn write_read_round_trip() {
        let field = crate::fixtures::gradient(24, 16);
        let path = scratch("roundtrip");
        write_pgm(&path, &field).expect("write");
        let back = read_pgm(&path).expect("read");
        assert_eq!(back, field);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn read_rejects_garbage_and_short_pixels() {
        let path = scratch("garbage");
        std::fs::write(&path, b"not a pgm at all").expect("write");
        assert!(read_pgm(&path).is_err());
        std::fs::write(&path, b"P5\n4 4\n255\n\x01\x02").expect("write");
        assert!(read_pgm(&path).is_err());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn comparison_layout_panels_and_diff() {
        let original = ScalarField {
            width: 2,
            height: 1,
            data: vec![10, 20],
        };
        let decoded = ScalarField {
            width: 2,
            height: 1,
            data: vec![12, 18],
        };
        let tri = comparison(&original, &decoded);
        assert_eq!((tri.width, tri.height), (8, 1));
        // Panels: orig | sep | decoded | sep | diff*16.
        assert_eq!(&tri.data[0..2], &[10, 20]);
        assert_eq!(tri.data[2], 0);
        assert_eq!(&tri.data[3..5], &[12, 18]);
        assert_eq!(tri.data[5], 0);
        assert_eq!(&tri.data[6..8], &[32, 32]);
        // Triptych itself round-trips through PGM.
        let path = scratch("triptych");
        write_pgm(&path, &tri).expect("write");
        assert_eq!(read_pgm(&path).expect("read"), tri);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    #[should_panic(expected = "identical extents")]
    fn comparison_rejects_mismatched_extents() {
        let a = ScalarField {
            width: 2,
            height: 2,
            data: vec![0; 4],
        };
        let b = ScalarField {
            width: 3,
            height: 2,
            data: vec![0; 6],
        };
        let _ = comparison(&a, &b);
    }
}
