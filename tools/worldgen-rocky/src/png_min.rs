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

/// Bounds compressed output independently of image size. All chunks belong
/// to one zlib stream; the compressor is not restarted at chunk boundaries.
const IDAT_CAPACITY: usize = 1024 * 1024;
struct IdatWriter<'a, W: Write> {
    out: &'a mut W,
    buffer: Vec<u8>,
    table: [u32; 256],
}
impl<W: Write> IdatWriter<'_, W> {
    fn emit(&mut self) -> std::io::Result<()> {
        if !self.buffer.is_empty() {
            write_chunk(self.out, b"IDAT", &self.buffer, &self.table)?;
            self.buffer.clear();
        }
        Ok(())
    }
}
impl<W: Write> Write for IdatWriter<'_, W> {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        let count = bytes.len().min(IDAT_CAPACITY - self.buffer.len());
        self.buffer.extend_from_slice(&bytes[..count]);
        if self.buffer.len() == IDAT_CAPACITY {
            self.emit()?;
        }
        Ok(count)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.emit()?;
        self.out.flush()
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
    let chunks = IdatWriter {
        out,
        buffer: Vec::with_capacity(IDAT_CAPACITY),
        table,
    };
    let mut compressor = flate2::write::ZlibEncoder::new(chunks, flate2::Compression::best());
    let stride = width * 3;
    let mut filter = vec![0u8; stride + 1];
    let mut prior = vec![0u8; stride];
    let mut candidate = vec![0u8; stride + 1];
    let mut got_rows = 0usize;
    for row in rows {
        if got_rows >= height {
            return Err("too many rows".into());
        }
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
        compressor.write_all(&filter).map_err(|e| e.to_string())?;
        got_rows += 1;
    }
    if got_rows != height {
        return Err(format!("expected {height} rows, got {got_rows}"));
    }
    let mut chunks = compressor.finish().map_err(|e| e.to_string())?;
    chunks.emit().map_err(|e| e.to_string())?;
    write_chunk(chunks.out, b"IEND", &[], &table).map_err(|e| e.to_string())?;
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
    #[test]
    fn multi_idat_roundtrip_crc_and_incremental_delivery() {
        use std::{
            cell::Cell,
            io::{Read, Write},
            rc::Rc,
        };
        struct Sink {
            bytes: Vec<u8>,
            rows_seen: Rc<Cell<usize>>,
            first_idat_row: Option<usize>,
        }
        impl Write for Sink {
            fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
                if b == b"IDAT" && self.first_idat_row.is_none() {
                    self.first_idat_row = Some(self.rows_seen.get());
                }
                self.bytes.extend_from_slice(b);
                Ok(b.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let count = Rc::new(Cell::new(0));
        let mut sink = Sink {
            bytes: vec![],
            rows_seen: count.clone(),
            first_idat_row: None,
        };
        let mut seed = 42_u64;
        let original: Vec<Vec<u8>> = (0..600)
            .map(|_| {
                (0..3072)
                    .map(|_| {
                        seed ^= seed << 13;
                        seed ^= seed >> 7;
                        seed ^= seed << 17;
                        seed as u8
                    })
                    .collect()
            })
            .collect();
        let mut rows = original
            .clone()
            .into_iter()
            .inspect(|_| count.set(count.get() + 1));
        write_png_rows(&mut sink, 1024, 600, &mut rows).unwrap();
        assert!(
            sink.first_idat_row.unwrap() < 600,
            "output must arrive before final row"
        );
        let mut compressed = vec![];
        let mut offset = 8;
        let mut idats = 0;
        while offset < sink.bytes.len() {
            let n = u32::from_be_bytes(sink.bytes[offset..offset + 4].try_into().unwrap()) as usize;
            let typ: &[u8; 4] = sink.bytes[offset + 4..offset + 8].try_into().unwrap();
            let data = &sink.bytes[offset + 8..offset + 8 + n];
            let crc = u32::from_be_bytes(
                sink.bytes[offset + 8 + n..offset + 12 + n]
                    .try_into()
                    .unwrap(),
            );
            assert_eq!(crc, super::crc32(&super::crc32_table(), typ, data));
            if typ == b"IDAT" {
                assert!(n <= super::IDAT_CAPACITY);
                compressed.extend_from_slice(data);
                idats += 1;
            }
            offset += n + 12;
        }
        assert!(idats >= 2);
        let mut decoded = vec![];
        flate2::read::ZlibDecoder::new(compressed.as_slice())
            .read_to_end(&mut decoded)
            .unwrap();
        let mut prior = vec![0_u8; 3072];
        for (raw, expected) in decoded.as_chunks::<3073>().0.iter().zip(&original) {
            let mut row = raw[1..].to_vec();
            for i in 0..row.len() {
                let predictor = match raw[0] {
                    0 => 0,
                    1 => {
                        if i >= 3 {
                            row[i - 3]
                        } else {
                            0
                        }
                    }
                    2 => prior[i],
                    _ => panic!("filter"),
                };
                row[i] = row[i].wrapping_add(predictor);
            }
            assert_eq!(&row, expected);
            prior = row;
        }
        assert_eq!(decoded.len(), 600 * 3073);
    }

    #[test]
    fn propagates_output_failure_and_rejects_extra_rows() {
        struct Broken;
        impl std::io::Write for Broken {
            fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("disk full"))
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        assert!(
            write_png_rows(&mut Broken, 1, 1, &mut vec![vec![0; 3]].into_iter())
                .unwrap_err()
                .contains("disk full")
        );
        assert!(write_png_rows(&mut vec![], 1, 1, &mut vec![vec![0; 3]; 2].into_iter()).is_err());
    }
}
