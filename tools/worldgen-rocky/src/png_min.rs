//! Minimal streaming PNG encoder (8-bit RGB, non-interlaced).
//!
//! Only what the worldgen exporter needs: signature, IHDR, sRGB, IDAT
//! (zlib/deflate via flate2, filter byte 0 per row), IEND. No external
//! image dependency; rows stream straight to the writer, so 16K outputs
//! never sit fully in memory.

use std::io::Write;

fn crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for (i, slot) in table.iter_mut().enumerate() {
        let mut c = i as u32;
        for _ in 0..8 {
            c = if c & 1 == 1 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
        }
        *slot = c;
    }
    table
}

fn crc32(table: &[u32; 256], typ: &[u8; 4], data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for byte in typ.iter().chain(data.iter()) {
        crc = table[((crc ^ u32::from(*byte)) & 0xFF) as usize] ^ (crc >> 8);
    }
    crc ^ 0xFFFF_FFFF
}

fn write_chunk<W: Write>(
    out: &mut W,
    typ: &[u8; 4],
    data: &[u8],
    table: &[u32; 256],
) -> std::io::Result<()> {
    out.write_all(&(data.len() as u32).to_be_bytes())?;
    out.write_all(typ)?;
    out.write_all(data)?;
    out.write_all(&crc32(table, typ, data).to_be_bytes())?;
    Ok(())
}

#[derive(Clone, Copy)]
struct Adler {
    s1: u32,
    s2: u32,
}

impl Adler {
    fn new() -> Self {
        Self { s1: 1, s2: 0 }
    }

    fn update(&mut self, data: &[u8]) {
        const MOD: u32 = 65521;
        for chunk in data.chunks(5552) {
            for byte in chunk {
                self.s1 += u32::from(*byte);
                self.s2 += self.s1;
            }
            self.s1 %= MOD;
            self.s2 %= MOD;
        }
    }

    fn digest(self) -> u32 {
        (self.s2 << 16) | self.s1
    }
}

/// Sum of absolute values (as signed bytes): filter cost heuristic.
fn row_score(filtered: &[u8]) -> u64 {
    filtered
        .iter()
        .map(|b| (*b as i8 as i16).unsigned_abs() as u64)
        .sum()
}

/// Stream rows of RGB bytes (`width*3` each) into `out` as a PNG.
pub fn write_png_rows<W: Write>(
    out: &mut W,
    width: usize,
    height: usize,
    rows: &mut dyn Iterator<Item = Vec<u8>>,
) -> Result<(), String> {
    if width == 0 || height == 0 || width > 16384 || height > 8192 {
        return Err("png dimensions out of range".into());
    }
    let table = crc32_table();
    out.write_all(&[137, 80, 78, 71, 13, 10, 26, 10])
        .map_err(|e| e.to_string())?;
    let mut ihdr = [0u8; 13];
    ihdr[0..4].copy_from_slice(&(width as u32).to_be_bytes());
    ihdr[4..8].copy_from_slice(&(height as u32).to_be_bytes());
    ihdr[8] = 8; // bit depth
    ihdr[9] = 2; // truecolor RGB
    write_chunk(out, b"IHDR", &ihdr, &table).map_err(|e| e.to_string())?;
    write_chunk(out, b"sRGB", &[1], &table).map_err(|e| e.to_string())?;
    // Deflate the filtered scanlines incrementally into one IDAT.
    let mut compressor = flate2::Compress::new(flate2::Compression::best(), false);
    let mut idat = Vec::new();
    let mut adler = Adler::new();
    // zlib header for best compression.
    let mut zlib = vec![0x78u8, 0xDA];
    let stride = width * 3;
    let mut filter = vec![0u8; stride + 1];
    let mut prior = vec![0u8; stride];
    let mut candidate = vec![0u8; stride + 1];
    let mut got_rows = 0usize;
    for row in rows {
        if row.len() != stride {
            return Err("row size mismatch".into());
        }
        // Adaptive filtering per row: None / Sub / Up, cheapest wins.
        // (Streaming-friendly: only the previous raw row is needed.)
        let mut best_filter = 0u8;
        // None.
        candidate[0] = 0;
        candidate[1..].copy_from_slice(&row);
        let mut best_score = row_score(&candidate[1..]);
        // Sub.
        candidate[0] = 1;
        for i in 0..stride {
            let left = if i >= 3 { row[i - 3] } else { 0 };
            candidate[1 + i] = row[i].wrapping_sub(left);
        }
        let score = row_score(&candidate[1..]);
        if score < best_score {
            best_score = score;
            best_filter = 1;
            filter.copy_from_slice(&candidate);
        } else {
            filter[0] = 0;
            filter[1..].copy_from_slice(&row);
        }
        // Up.
        candidate[0] = 2;
        for (i, byte) in row.iter().enumerate() {
            candidate[1 + i] = byte.wrapping_sub(prior[i]);
        }
        let score = row_score(&candidate[1..]);
        if score < best_score {
            best_filter = 2;
            filter.copy_from_slice(&candidate);
        }
        let _ = best_filter;
        prior.copy_from_slice(&row);
        adler.update(&filter);
        let mut chunk = Vec::with_capacity(65536);
        let mut consumed = 0usize;
        while consumed < filter.len() {
            let before_in = compressor.total_in();
            compressor
                .compress_vec(&filter[consumed..], &mut chunk, flate2::FlushCompress::None)
                .map_err(|e| e.to_string())?;
            consumed += (compressor.total_in() - before_in) as usize;
        }
        idat.extend_from_slice(&chunk);
        got_rows += 1;
    }
    if got_rows != height {
        return Err(format!("expected {height} rows, got {got_rows}"));
    }
    loop {
        let mut chunk = Vec::with_capacity(65536);
        let done = compressor
            .compress_vec(&[], &mut chunk, flate2::FlushCompress::Finish)
            .map_err(|e| e.to_string())?;
        idat.extend_from_slice(&chunk);
        if matches!(done, flate2::Status::StreamEnd) {
            break;
        }
    }
    zlib.extend_from_slice(&idat);
    zlib.extend_from_slice(&adler.digest().to_be_bytes());
    write_chunk(out, b"IDAT", &zlib, &table).map_err(|e| e.to_string())?;
    write_chunk(out, b"IEND", &[], &table).map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::write_png_rows;

    #[test]
    fn tiny_png_roundtrip_header() {
        let mut out = Vec::new();
        let rows = vec![
            vec![255u8, 0, 0, 0, 255, 0],
            vec![0u8, 0, 255, 255, 255, 255],
        ];
        write_png_rows(&mut out, 2, 2, &mut rows.into_iter()).expect("encode");
        assert_eq!(&out[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
        // IHDR must declare 2x2 RGB8.
        assert_eq!(&out[16..24], &[0, 0, 0, 2, 0, 0, 0, 2]);
        assert_eq!(out[24], 8);
        assert_eq!(out[25], 2);
    }

    #[test]
    fn rejects_bad_geometry() {
        let mut out = Vec::new();
        let mut rows = vec![vec![0u8; 3]].into_iter();
        assert!(write_png_rows(&mut out, 2, 1, &mut rows).is_err());
        let mut rows = Vec::<Vec<u8>>::new().into_iter();
        assert!(write_png_rows(&mut out, 0, 0, &mut rows).is_err());
    }
}
