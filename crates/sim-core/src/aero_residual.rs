//! Bounded residual storage for aerodynamic coefficient tables.
//!
//! This is the CPU reference prototype from
//! `docs/43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md`. It leaves the analytic
//! aerodynamic path untouched and compresses only table-backed coefficient
//! fields. The representation is deterministic and keeps the existing
//! Mach/alpha bilinear sampling semantics: grid points are decoded first, then
//! interpolated exactly like `AeroCoefficientTable::sample`.
//!
//! Each 4x4 tile stores four f64 corner predictors plus per-coefficient scales.
//! Residual4/6/8/16 encode signed residuals around the bilinear predictor;
//! tiles that cannot satisfy the requested coefficient-space error budget fall
//! back to verbatim Raw64 samples.
//!
//! The current implementation is deliberately scalar. It exists as a
//! correctness/error oracle before AVX2/AVX-512 fused decode paths are added.

use std::mem::size_of;

use crate::{AeroCoefficientTable, AeroCoefficients, AeroError};

/// Logical tile edge in Mach/alpha grid points.
pub const AERO_RESIDUAL_TILE_EDGE: usize = 4;
const COEFFICIENTS: usize = 4;
const R4_MAX: f64 = 7.0;
const R6_MAX: f64 = 31.0;
const R8_MAX: f64 = i8::MAX as f64;
const R16_MAX: f64 = i16::MAX as f64;

/// Storage rung selected independently for every tile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AeroResidualCodec {
    /// Four signed 4-bit residuals packed into two bytes per sample.
    Residual4,
    /// Four signed 6-bit residuals packed into three bytes per sample.
    Residual6,
    /// Signed 8-bit residual per coefficient.
    Residual8,
    /// Signed 16-bit residual per coefficient.
    Residual16,
    /// Verbatim f64 coefficient samples.
    Raw64,
}

impl AeroResidualCodec {
    /// Payload bytes per grid sample.
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            Self::Residual4 => 2,
            Self::Residual6 => 3,
            Self::Residual8 => COEFFICIENTS,
            Self::Residual16 => COEFFICIENTS * 2,
            Self::Raw64 => COEFFICIENTS * 8,
        }
    }

    const fn qmax(self) -> f64 {
        match self {
            Self::Residual4 => R4_MAX,
            Self::Residual6 => R6_MAX,
            Self::Residual8 => R8_MAX,
            Self::Residual16 => R16_MAX,
            Self::Raw64 => 0.0,
        }
    }

    const fn is_quantized(self) -> bool {
        !matches!(self, Self::Raw64)
    }
}

/// Maximum absolute error per dimensionless aerodynamic coefficient.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AeroCoefficientError {
    pub lift: f64,
    pub drag: f64,
    pub side_force: f64,
    pub pitching_moment: f64,
}

impl AeroCoefficientError {
    fn from_array(values: [f64; COEFFICIENTS]) -> Self {
        Self {
            lift: values[0],
            drag: values[1],
            side_force: values[2],
            pitching_moment: values[3],
        }
    }

    fn component_max(self, other: Self) -> Self {
        Self {
            lift: self.lift.max(other.lift),
            drag: self.drag.max(other.drag),
            side_force: self.side_force.max(other.side_force),
            pitching_moment: self.pitching_moment.max(other.pitching_moment),
        }
    }

    /// Convert coefficient error into conservative panel force/moment bounds.
    ///
    /// The force envelope uses the triangle inequality:
    /// `q*S*(eps_CL + eps_CD + eps_CY)`. The moment envelope adds the direct
    /// coefficient term `q*S*c*eps_Cm` and the worst force-error lever arm.
    pub fn physical_bound(
        self,
        dynamic_pressure_pa: f64,
        area_m2: f64,
        reference_chord_m: f64,
        moment_arm_m: f64,
    ) -> Result<AeroPhysicalErrorBound, AeroError> {
        let inputs = [
            dynamic_pressure_pa,
            area_m2,
            reference_chord_m,
            moment_arm_m,
        ];
        if inputs
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(AeroError::InvalidModel(
                "aero residual physical-bound inputs must be finite and non-negative".into(),
            ));
        }
        let force_n = dynamic_pressure_pa * area_m2 * (self.lift + self.drag + self.side_force);
        let moment_nm = dynamic_pressure_pa * area_m2 * reference_chord_m * self.pitching_moment
            + moment_arm_m * force_n;
        Ok(AeroPhysicalErrorBound { force_n, moment_nm })
    }
}

/// Conservative physical error envelope derived from coefficient-space error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroPhysicalErrorBound {
    pub force_n: f64,
    pub moment_nm: f64,
}

/// Worst-case panel operating envelope used to derive a conservative uniform
/// coefficient-space encode budget.
///
/// This is intentionally a simple baseline policy: all four coefficient
/// channels receive the same epsilon. It is conservative under both the force
/// and moment limits and leaves channel-specific budget allocation for a later
/// optimizer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroPhysicalBudget {
    pub dynamic_pressure_pa: f64,
    pub area_m2: f64,
    pub reference_chord_m: f64,
    pub moment_arm_m: f64,
    pub max_force_error_n: f64,
    pub max_moment_error_nm: f64,
}

impl AeroPhysicalBudget {
    /// Convert the physical envelope into one uniform per-coefficient budget.
    ///
    /// With `eps` on CL/CD/CY/Cm:
    ///
    /// `dF <= q*S*3*eps`
    ///
    /// `dM <= q*S*(c + 3*r)*eps`
    pub fn uniform_coefficient_budget(self) -> Result<AeroResidualBudget, AeroError> {
        let values = [
            self.dynamic_pressure_pa,
            self.area_m2,
            self.reference_chord_m,
            self.moment_arm_m,
            self.max_force_error_n,
            self.max_moment_error_nm,
        ];
        if values
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(AeroError::InvalidModel(
                "aero residual physical budget must be finite and non-negative".into(),
            ));
        }

        let q_area = self.dynamic_pressure_pa * self.area_m2;
        if q_area == 0.0 {
            return Ok(AeroResidualBudget::uniform(f64::MAX));
        }
        if !q_area.is_finite() {
            return Err(AeroError::InvalidModel(
                "aero residual physical budget q*S overflowed".into(),
            ));
        }

        let force_eps = self.max_force_error_n / (3.0 * q_area);
        let moment_scale = q_area * (self.reference_chord_m + 3.0 * self.moment_arm_m);
        let moment_eps = if moment_scale == 0.0 {
            f64::MAX
        } else {
            self.max_moment_error_nm / moment_scale
        };
        let epsilon = force_eps.min(moment_eps);
        if !epsilon.is_finite() || epsilon < 0.0 {
            return Err(AeroError::InvalidModel(
                "aero residual physical budget produced an invalid coefficient budget".into(),
            ));
        }
        Ok(AeroResidualBudget::uniform(epsilon))
    }
}

/// Per-coefficient encode budget. A tile takes the cheapest rung whose
/// measured decoded grid-point error is within every component budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroResidualBudget {
    pub lift: f64,
    pub drag: f64,
    pub side_force: f64,
    pub pitching_moment: f64,
}

impl AeroResidualBudget {
    /// Use one maximum absolute coefficient error for all four channels.
    pub const fn uniform(max_abs_error: f64) -> Self {
        Self {
            lift: max_abs_error,
            drag: max_abs_error,
            side_force: max_abs_error,
            pitching_moment: max_abs_error,
        }
    }

    fn validate(self) -> Result<(), AeroError> {
        let values = [self.lift, self.drag, self.side_force, self.pitching_moment];
        if values
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(AeroError::InvalidModel(
                "aero residual budget must be finite and non-negative".into(),
            ));
        }
        Ok(())
    }

    fn contains(self, error: AeroCoefficientError) -> bool {
        error.lift <= self.lift
            && error.drag <= self.drag
            && error.side_force <= self.side_force
            && error.pitching_moment <= self.pitching_moment
    }
}

/// One logical 4x4 (or edge-partial) tile.
///
/// Layout fields are private: decode indexes the payload with
/// `mach_start/alpha_start/len` and reads it with `codec`, so safe code
/// must not be able to desynchronize them (a witness: setting
/// `mach_start = 999` used to turn the next decode into a `usize`
/// underflow/index panic). Read through the accessors; build through
/// [`AeroResidualTable::encode`].
#[derive(Debug, Clone, PartialEq)]
pub struct AeroResidualTile {
    codec: AeroResidualCodec,
    mach_start: usize,
    alpha_start: usize,
    mach_len: usize,
    alpha_len: usize,
    /// Worst decoded grid-point error in this tile.
    pub error: AeroCoefficientError,
    corners: [AeroCoefficients; 4],
    scales: AeroCoefficients,
    payload_offset: usize,
    payload_len: usize,
}

impl AeroResidualTile {
    pub fn payload_len(&self) -> usize {
        self.payload_len
    }

    pub fn codec(&self) -> AeroResidualCodec {
        self.codec
    }

    pub fn mach_start(&self) -> usize {
        self.mach_start
    }

    pub fn alpha_start(&self) -> usize {
        self.alpha_start
    }

    pub fn mach_len(&self) -> usize {
        self.mach_len
    }

    pub fn alpha_len(&self) -> usize {
        self.alpha_len
    }
}

/// Storage/codec telemetry for one encoded table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroResidualStats {
    pub tiles: usize,
    pub residual4_tiles: usize,
    pub residual6_tiles: usize,
    pub residual8_tiles: usize,
    pub residual16_tiles: usize,
    pub raw64_tiles: usize,
    pub raw_coefficient_bytes: usize,
    pub payload_bytes: usize,
    pub tile_metadata_bytes: usize,
    pub grid_bytes: usize,
    pub logical_resident_bytes: usize,
    pub max_error: AeroCoefficientError,
}

/// Residual-compressed twin of `AeroCoefficientTable`.
///
/// The grid arrays are retained verbatim so clamping/bracketing semantics stay
/// identical. Payload is a single contiguous byte buffer; tiles carry offsets
/// into it, avoiding one heap allocation per tile.
///
/// Grids and tiles are private for the same reason as the tile layout:
/// decode trusts them jointly (grid bracketing, tile lookup, payload
/// slicing), so mutating one side through safe code could panic the other.
/// Read through the accessors or `sample`/`grid_sample`.
#[derive(Debug, Clone, PartialEq)]
pub struct AeroResidualTable {
    mach_grid: Vec<f64>,
    alpha_grid_rad: Vec<f64>,
    tiles: Vec<AeroResidualTile>,
    payload: Vec<u8>,
    tile_alpha_count: usize,
    max_error: AeroCoefficientError,
}

impl AeroResidualTable {
    /// Encode a validated canonical table with cheapest-first R4 -> R6 ->
    /// R8 -> R16 -> Raw64 selection under the requested coefficient-space
    /// budget.
    pub fn encode(
        table: &AeroCoefficientTable,
        budget: AeroResidualBudget,
    ) -> Result<Self, AeroError> {
        budget.validate()?;
        if table.mach_grid.is_empty() || table.alpha_grid_rad.is_empty() {
            return Err(AeroError::InvalidModel(
                "aero residual table needs non-empty grids".into(),
            ));
        }
        if table.samples.len() != table.mach_grid.len() * table.alpha_grid_rad.len() {
            return Err(AeroError::InvalidModel(
                "aero residual source table has inconsistent extents".into(),
            ));
        }

        let tile_mach_count = table.mach_grid.len().div_ceil(AERO_RESIDUAL_TILE_EDGE);
        let tile_alpha_count = table.alpha_grid_rad.len().div_ceil(AERO_RESIDUAL_TILE_EDGE);
        let mut tiles = Vec::with_capacity(tile_mach_count * tile_alpha_count);
        let mut payload = Vec::new();
        let mut global_error = AeroCoefficientError::default();

        for tile_mach in 0..tile_mach_count {
            let mach_start = tile_mach * AERO_RESIDUAL_TILE_EDGE;
            let mach_len = (table.mach_grid.len() - mach_start).min(AERO_RESIDUAL_TILE_EDGE);
            for tile_alpha in 0..tile_alpha_count {
                let alpha_start = tile_alpha * AERO_RESIDUAL_TILE_EDGE;
                let alpha_len =
                    (table.alpha_grid_rad.len() - alpha_start).min(AERO_RESIDUAL_TILE_EDGE);
                let corners = tile_corners(table, mach_start, alpha_start, mach_len, alpha_len);

                let mut selected = None;
                for codec in [
                    AeroResidualCodec::Residual4,
                    AeroResidualCodec::Residual6,
                    AeroResidualCodec::Residual8,
                    AeroResidualCodec::Residual16,
                ] {
                    let encoded = encode_quantized_tile(
                        table,
                        mach_start,
                        alpha_start,
                        mach_len,
                        alpha_len,
                        corners,
                        codec,
                    );
                    if budget.contains(encoded.1) {
                        selected = Some((codec, encoded.0, encoded.1, encoded.2));
                        break;
                    }
                }
                let (codec, scales, error, bytes) = selected.unwrap_or_else(|| {
                    (
                        AeroResidualCodec::Raw64,
                        zero_coefficients(),
                        AeroCoefficientError::default(),
                        encode_raw_tile(table, mach_start, alpha_start, mach_len, alpha_len),
                    )
                });

                let payload_offset = payload.len();
                let payload_len = bytes.len();
                payload.extend_from_slice(&bytes);
                global_error = global_error.component_max(error);
                tiles.push(AeroResidualTile {
                    codec,
                    mach_start,
                    alpha_start,
                    mach_len,
                    alpha_len,
                    error,
                    corners,
                    scales,
                    payload_offset,
                    payload_len,
                });
            }
        }

        Ok(Self {
            mach_grid: table.mach_grid.clone(),
            alpha_grid_rad: table.alpha_grid_rad.clone(),
            tiles,
            payload,
            tile_alpha_count,
            max_error: global_error,
        })
    }

    /// Encode against a declared worst-case physical panel envelope.
    ///
    /// The initial policy maps force/moment limits to one conservative uniform
    /// coefficient epsilon, then reuses the normal adaptive codec ladder.
    pub fn encode_for_physical_budget(
        table: &AeroCoefficientTable,
        budget: AeroPhysicalBudget,
    ) -> Result<Self, AeroError> {
        Self::encode(table, budget.uniform_coefficient_budget()?)
    }

    /// Decode one canonical grid point. Returns None for out-of-range indices.
    pub fn grid_sample(&self, mach_index: usize, alpha_index: usize) -> Option<AeroCoefficients> {
        if mach_index >= self.mach_grid.len() || alpha_index >= self.alpha_grid_rad.len() {
            return None;
        }
        Some(self.decode_grid_point(mach_index, alpha_index))
    }

    /// Encoded tiles in row-major order. Read-only: layout and payload
    /// stay consistent by construction.
    pub fn tiles(&self) -> &[AeroResidualTile] {
        &self.tiles
    }

    /// Retained Mach grid (verbatim copy of the source table).
    pub fn mach_grid(&self) -> &[f64] {
        &self.mach_grid
    }

    /// Retained alpha grid in radians (verbatim copy of the source table).
    pub fn alpha_grid_rad(&self) -> &[f64] {
        &self.alpha_grid_rad
    }

    /// Sample with the same bracketing/clamping and bilinear order as the
    /// canonical `AeroCoefficientTable`.
    pub fn sample(&self, mach: f64, alpha_rad: f64) -> AeroCoefficients {
        self.sample_with_error(mach, alpha_rad).0
    }

    /// Sample plus a conservative local coefficient-error envelope.
    ///
    /// Bilinear interpolation is a convex combination of its four decoded
    /// grid points, so the interpolated error in each coefficient cannot
    /// exceed the componentwise maximum bound of those four source tiles.
    /// This is tighter than `max_error()` when only a small sharp region
    /// forced high-error/Raw64 choices elsewhere in the table.
    pub fn sample_with_error(
        &self,
        mach: f64,
        alpha_rad: f64,
    ) -> (AeroCoefficients, AeroCoefficientError) {
        let (mach_lo, mach_hi, mach_t) = bracket(&self.mach_grid, mach);
        let (alpha_lo, alpha_hi, alpha_t) = bracket(&self.alpha_grid_rad, alpha_rad);
        let at = |mi: usize, ai: usize| self.decode_grid_point(mi, ai);
        let low = interpolate_coefficients(at(mach_lo, alpha_lo), at(mach_lo, alpha_hi), alpha_t);
        let high = interpolate_coefficients(at(mach_hi, alpha_lo), at(mach_hi, alpha_hi), alpha_t);
        let sample = interpolate_coefficients(low, high, mach_t);

        let mut error = AeroCoefficientError::default();
        for (mi, ai) in [
            (mach_lo, alpha_lo),
            (mach_lo, alpha_hi),
            (mach_hi, alpha_lo),
            (mach_hi, alpha_hi),
        ] {
            error = error.component_max(self.tile_for_grid_point(mi, ai).error);
        }
        (sample, error)
    }

    /// Sample and convert the local coefficient envelope directly into a
    /// conservative physical force/moment error bound for one panel.
    pub fn sample_with_physical_bound(
        &self,
        mach: f64,
        alpha_rad: f64,
        dynamic_pressure_pa: f64,
        area_m2: f64,
        reference_chord_m: f64,
        moment_arm_m: f64,
    ) -> Result<(AeroCoefficients, AeroPhysicalErrorBound), AeroError> {
        let (sample, error) = self.sample_with_error(mach, alpha_rad);
        let bound = error.physical_bound(
            dynamic_pressure_pa,
            area_m2,
            reference_chord_m,
            moment_arm_m,
        )?;
        Ok((sample, bound))
    }

    /// Worst grid-point coefficient error across all tiles.
    pub const fn max_error(&self) -> AeroCoefficientError {
        self.max_error
    }

    /// Codec/storage telemetry. `logical_resident_bytes` counts grid arrays,
    /// current Rust tile structs, and the contiguous payload. It intentionally
    /// excludes Vec allocator bookkeeping/capacity slack.
    pub fn stats(&self) -> AeroResidualStats {
        let mut r4 = 0;
        let mut r6 = 0;
        let mut r8 = 0;
        let mut r16 = 0;
        let mut raw = 0;
        for tile in &self.tiles {
            match tile.codec {
                AeroResidualCodec::Residual4 => r4 += 1,
                AeroResidualCodec::Residual6 => r6 += 1,
                AeroResidualCodec::Residual8 => r8 += 1,
                AeroResidualCodec::Residual16 => r16 += 1,
                AeroResidualCodec::Raw64 => raw += 1,
            }
        }
        let raw_coefficient_bytes =
            self.mach_grid.len() * self.alpha_grid_rad.len() * COEFFICIENTS * size_of::<f64>();
        let tile_metadata_bytes = self.tiles.len() * size_of::<AeroResidualTile>();
        let grid_bytes = (self.mach_grid.len() + self.alpha_grid_rad.len()) * size_of::<f64>();
        let logical_resident_bytes = self.payload.len() + tile_metadata_bytes + grid_bytes;
        AeroResidualStats {
            tiles: self.tiles.len(),
            residual4_tiles: r4,
            residual6_tiles: r6,
            residual8_tiles: r8,
            residual16_tiles: r16,
            raw64_tiles: raw,
            raw_coefficient_bytes,
            payload_bytes: self.payload.len(),
            tile_metadata_bytes,
            grid_bytes,
            logical_resident_bytes,
            max_error: self.max_error,
        }
    }

    fn tile_for_grid_point(&self, mach_index: usize, alpha_index: usize) -> &AeroResidualTile {
        let tile_mach = mach_index / AERO_RESIDUAL_TILE_EDGE;
        let tile_alpha = alpha_index / AERO_RESIDUAL_TILE_EDGE;
        &self.tiles[tile_mach * self.tile_alpha_count + tile_alpha]
    }

    fn decode_grid_point(&self, mach_index: usize, alpha_index: usize) -> AeroCoefficients {
        let tile = self.tile_for_grid_point(mach_index, alpha_index);
        let local_mach = mach_index - tile.mach_start;
        let local_alpha = alpha_index - tile.alpha_start;
        let local_index = local_mach * tile.alpha_len + local_alpha;
        let bytes = &self.payload[tile.payload_offset..tile.payload_offset + tile.payload_len];

        match tile.codec {
            AeroResidualCodec::Raw64 => {
                let base = local_index * COEFFICIENTS * size_of::<f64>();
                let mut values = [0.0; COEFFICIENTS];
                for (component, value) in values.iter_mut().enumerate() {
                    let start = base + component * size_of::<f64>();
                    let raw: [u8; 8] = bytes[start..start + 8]
                        .try_into()
                        .expect("raw64 tile payload validated at encode time");
                    *value = f64::from_le_bytes(raw);
                }
                coefficients_from_array(values)
            }
            codec => {
                debug_assert!(codec.is_quantized());
                let predictor = predictor_at(tile, local_mach, local_alpha);
                let mut values = coefficients_array(predictor);
                let scales = coefficients_array(tile.scales);
                let quantized = decode_quantized_sample(bytes, codec, local_index);
                for component in 0..COEFFICIENTS {
                    values[component] += quantized[component] as f64 * scales[component];
                }
                coefficients_from_array(values)
            }
        }
    }
}

fn tile_corners(
    table: &AeroCoefficientTable,
    mach_start: usize,
    alpha_start: usize,
    mach_len: usize,
    alpha_len: usize,
) -> [AeroCoefficients; 4] {
    let mach_end = mach_start + mach_len - 1;
    let alpha_end = alpha_start + alpha_len - 1;
    [
        source_at(table, mach_start, alpha_start),
        source_at(table, mach_start, alpha_end),
        source_at(table, mach_end, alpha_start),
        source_at(table, mach_end, alpha_end),
    ]
}

fn source_at(
    table: &AeroCoefficientTable,
    mach_index: usize,
    alpha_index: usize,
) -> AeroCoefficients {
    table.samples[mach_index * table.alpha_grid_rad.len() + alpha_index]
}

fn predictor_at(
    tile: &AeroResidualTile,
    local_mach: usize,
    local_alpha: usize,
) -> AeroCoefficients {
    let mach_t = normalized_local(local_mach, tile.mach_len);
    let alpha_t = normalized_local(local_alpha, tile.alpha_len);
    bilinear_predict(tile.corners, mach_t, alpha_t)
}

fn normalized_local(index: usize, len: usize) -> f64 {
    if len <= 1 {
        0.0
    } else {
        index as f64 / (len - 1) as f64
    }
}

fn bilinear_predict(corners: [AeroCoefficients; 4], mach_t: f64, alpha_t: f64) -> AeroCoefficients {
    let low_mach = interpolate_coefficients(corners[0], corners[1], alpha_t);
    let high_mach = interpolate_coefficients(corners[2], corners[3], alpha_t);
    interpolate_coefficients(low_mach, high_mach, mach_t)
}

/// Returns (scales, measured error, payload).
fn encode_quantized_tile(
    table: &AeroCoefficientTable,
    mach_start: usize,
    alpha_start: usize,
    mach_len: usize,
    alpha_len: usize,
    corners: [AeroCoefficients; 4],
    codec: AeroResidualCodec,
) -> (AeroCoefficients, AeroCoefficientError, Vec<u8>) {
    debug_assert!(codec.is_quantized());
    let mut maxima = [0.0f64; COEFFICIENTS];
    for local_mach in 0..mach_len {
        for local_alpha in 0..alpha_len {
            let actual = coefficients_array(source_at(
                table,
                mach_start + local_mach,
                alpha_start + local_alpha,
            ));
            let predicted = coefficients_array(bilinear_predict(
                corners,
                normalized_local(local_mach, mach_len),
                normalized_local(local_alpha, alpha_len),
            ));
            for component in 0..COEFFICIENTS {
                maxima[component] =
                    maxima[component].max((actual[component] - predicted[component]).abs());
            }
        }
    }

    let qmax = codec.qmax();
    let scales = maxima.map(|max| if max == 0.0 { 0.0 } else { max / qmax });
    let mut payload = Vec::with_capacity(mach_len * alpha_len * codec.bytes_per_sample());
    let mut error = [0.0f64; COEFFICIENTS];

    for local_mach in 0..mach_len {
        for local_alpha in 0..alpha_len {
            let actual = coefficients_array(source_at(
                table,
                mach_start + local_mach,
                alpha_start + local_alpha,
            ));
            let predicted = coefficients_array(bilinear_predict(
                corners,
                normalized_local(local_mach, mach_len),
                normalized_local(local_alpha, alpha_len),
            ));
            let mut quantized = [0i16; COEFFICIENTS];
            for component in 0..COEFFICIENTS {
                let residual = actual[component] - predicted[component];
                let scale = scales[component];
                let q = if scale == 0.0 {
                    0.0
                } else {
                    (residual / scale).round().clamp(-qmax, qmax)
                };
                quantized[component] = q as i16;
                let decoded = predicted[component] + q * scale;
                error[component] = error[component].max((decoded - actual[component]).abs());
            }
            encode_quantized_sample(&mut payload, codec, quantized);
        }
    }

    (
        coefficients_from_array(scales),
        AeroCoefficientError::from_array(error),
        payload,
    )
}

fn encode_quantized_sample(
    payload: &mut Vec<u8>,
    codec: AeroResidualCodec,
    quantized: [i16; COEFFICIENTS],
) {
    match codec {
        AeroResidualCodec::Residual4 => {
            let q = quantized.map(|value| (value as u16 & 0x0f) as u8);
            payload.push(q[0] | (q[1] << 4));
            payload.push(q[2] | (q[3] << 4));
        }
        AeroResidualCodec::Residual6 => {
            let q = quantized.map(|value| value as u32 & 0x3f);
            let packed = q[0] | (q[1] << 6) | (q[2] << 12) | (q[3] << 18);
            payload.push(packed as u8);
            payload.push((packed >> 8) as u8);
            payload.push((packed >> 16) as u8);
        }
        AeroResidualCodec::Residual8 => {
            for value in quantized {
                payload.push((value as i8) as u8);
            }
        }
        AeroResidualCodec::Residual16 => {
            for value in quantized {
                payload.extend_from_slice(&value.to_le_bytes());
            }
        }
        AeroResidualCodec::Raw64 => unreachable!(),
    }
}

fn decode_quantized_sample(
    payload: &[u8],
    codec: AeroResidualCodec,
    sample_index: usize,
) -> [i16; COEFFICIENTS] {
    match codec {
        AeroResidualCodec::Residual4 => {
            let base = sample_index * 2;
            let a = payload[base];
            let b = payload[base + 1];
            [
                sign_extend_4(a & 0x0f) as i16,
                sign_extend_4(a >> 4) as i16,
                sign_extend_4(b & 0x0f) as i16,
                sign_extend_4(b >> 4) as i16,
            ]
        }
        AeroResidualCodec::Residual6 => {
            let base = sample_index * 3;
            let packed = payload[base] as u32
                | ((payload[base + 1] as u32) << 8)
                | ((payload[base + 2] as u32) << 16);
            [
                sign_extend_6((packed & 0x3f) as u8) as i16,
                sign_extend_6(((packed >> 6) & 0x3f) as u8) as i16,
                sign_extend_6(((packed >> 12) & 0x3f) as u8) as i16,
                sign_extend_6(((packed >> 18) & 0x3f) as u8) as i16,
            ]
        }
        AeroResidualCodec::Residual8 => {
            let base = sample_index * COEFFICIENTS;
            [
                payload[base] as i8 as i16,
                payload[base + 1] as i8 as i16,
                payload[base + 2] as i8 as i16,
                payload[base + 3] as i8 as i16,
            ]
        }
        AeroResidualCodec::Residual16 => {
            let base = sample_index * COEFFICIENTS * 2;
            let read = |component: usize| {
                let start = base + component * 2;
                i16::from_le_bytes(
                    payload[start..start + 2]
                        .try_into()
                        .expect("residual16 payload validated at encode time"),
                )
            };
            [read(0), read(1), read(2), read(3)]
        }
        AeroResidualCodec::Raw64 => unreachable!(),
    }
}

fn sign_extend_4(value: u8) -> i8 {
    ((value << 4) as i8) >> 4
}

fn sign_extend_6(value: u8) -> i8 {
    ((value << 2) as i8) >> 2
}

fn encode_raw_tile(
    table: &AeroCoefficientTable,
    mach_start: usize,
    alpha_start: usize,
    mach_len: usize,
    alpha_len: usize,
) -> Vec<u8> {
    let mut payload = Vec::with_capacity(mach_len * alpha_len * COEFFICIENTS * size_of::<f64>());
    for local_mach in 0..mach_len {
        for local_alpha in 0..alpha_len {
            for value in coefficients_array(source_at(
                table,
                mach_start + local_mach,
                alpha_start + local_alpha,
            )) {
                payload.extend_from_slice(&value.to_le_bytes());
            }
        }
    }
    payload
}

fn zero_coefficients() -> AeroCoefficients {
    coefficients_from_array([0.0; COEFFICIENTS])
}

fn coefficients_array(value: AeroCoefficients) -> [f64; COEFFICIENTS] {
    [
        value.lift,
        value.drag,
        value.side_force,
        value.pitching_moment,
    ]
}

fn coefficients_from_array(value: [f64; COEFFICIENTS]) -> AeroCoefficients {
    AeroCoefficients {
        lift: value[0],
        drag: value[1],
        side_force: value[2],
        pitching_moment: value[3],
    }
}

fn bracket(grid: &[f64], value: f64) -> (usize, usize, f64) {
    if grid.len() == 1 || value <= grid[0] {
        return (0, 0, 0.0);
    }
    if value >= grid[grid.len() - 1] {
        let last = grid.len() - 1;
        return (last, last, 0.0);
    }
    let upper = grid.partition_point(|entry| *entry < value);
    let lower = upper - 1;
    let t = (value - grid[lower]) / (grid[upper] - grid[lower]);
    (lower, upper, t)
}

fn interpolate_coefficients(
    low: AeroCoefficients,
    high: AeroCoefficients,
    t: f64,
) -> AeroCoefficients {
    AeroCoefficients {
        lift: lerp(low.lift, high.lift, t),
        drag: lerp(low.drag, high.drag, t),
        side_force: lerp(low.side_force, high.side_force, t),
        pitching_moment: lerp(low.pitching_moment, high.pitching_moment, t),
    }
}

fn lerp(low: f64, high: f64, t: f64) -> f64 {
    low + (high - low) * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AeroCase, AeroConfig, AeroEnvironment, AeroGeometry, AeroModel, AeroPanel, AeroState,
        PanelAeroModel,
    };
    use glam::DVec3;

    fn table_from(
        mach_count: usize,
        alpha_count: usize,
        f: impl Fn(f64, f64) -> AeroCoefficients,
    ) -> AeroCoefficientTable {
        let mach_grid = (0..mach_count).map(|i| i as f64 * 0.17).collect::<Vec<_>>();
        let alpha_grid_rad = (0..alpha_count)
            .map(|i| -0.45 + i as f64 * 0.09)
            .collect::<Vec<_>>();
        let mut samples = Vec::with_capacity(mach_count * alpha_count);
        for &mach in &mach_grid {
            for &alpha in &alpha_grid_rad {
                samples.push(f(mach, alpha));
            }
        }
        AeroCoefficientTable::new(mach_grid, alpha_grid_rad, samples).unwrap()
    }

    fn bilinear_coefficients(mach: f64, alpha: f64) -> AeroCoefficients {
        AeroCoefficients {
            lift: 0.2 + 0.7 * mach + 2.1 * alpha + 0.3 * mach * alpha,
            drag: 0.04 + 0.08 * mach - 0.02 * alpha + 0.01 * mach * alpha,
            side_force: -0.03 + 0.12 * mach + 0.4 * alpha - 0.05 * mach * alpha,
            pitching_moment: 0.01 - 0.09 * mach - 0.3 * alpha + 0.02 * mach * alpha,
        }
    }

    fn nonlinear_coefficients(mach: f64, alpha: f64) -> AeroCoefficients {
        AeroCoefficients {
            lift: (2.7 * alpha).sin() * (1.0 + 0.15 * mach) + 0.03 * mach * mach,
            drag: 0.025 + 0.11 * alpha * alpha + 0.015 * (1.7 * mach).sin().abs(),
            side_force: 0.07 * (alpha + 0.2 * mach).sin(),
            pitching_moment: -0.08 * alpha + 0.015 * (mach * alpha * 7.0).cos(),
        }
    }

    fn assert_coeff_error_le(
        got: AeroCoefficients,
        expected: AeroCoefficients,
        error: AeroCoefficientError,
    ) {
        assert!((got.lift - expected.lift).abs() <= error.lift + 2.0e-14);
        assert!((got.drag - expected.drag).abs() <= error.drag + 2.0e-14);
        assert!((got.side_force - expected.side_force).abs() <= error.side_force + 2.0e-14);
        assert!(
            (got.pitching_moment - expected.pitching_moment).abs()
                <= error.pitching_moment + 2.0e-14
        );
    }

    #[test]
    fn public_layout_views_tile_the_grid_exactly() {
        // Encapsulation pin: layout is readable (tiles, grids, codec,
        // starts, lengths) but only constructible via encode, so safe
        // code cannot desynchronize decode from its payload. Every grid
        // point decodes through the public API: no out-of-bounds access,
// no arithmetic underflow, finite coefficients everywhere.
        let source = table_from(13, 11, nonlinear_coefficients);
        let packed =
            AeroResidualTable::encode(&source, AeroResidualBudget::uniform(1.0e-4)).unwrap();
        assert_eq!(packed.mach_grid(), source.mach_grid.as_slice());
        assert_eq!(packed.alpha_grid_rad(), source.alpha_grid_rad.as_slice());
        let (nm, na) = (source.mach_grid.len(), source.alpha_grid_rad.len());
        let mut covered = vec![false; nm * na];
        for tile in packed.tiles() {
            assert!(tile.mach_len() > 0 && tile.alpha_len() > 0);
            for mi in tile.mach_start()..tile.mach_start() + tile.mach_len() {
                for ai in tile.alpha_start()..tile.alpha_start() + tile.alpha_len() {
                    assert!(mi < nm && ai < na, "tile exceeds grid");
                    assert!(!covered[mi * na + ai], "tiles overlap");
                    covered[mi * na + ai] = true;
                    let got = packed.grid_sample(mi, ai).unwrap();
                    assert!(got.lift.is_finite() && got.drag.is_finite());
                }
            }
        }
        assert!(covered.iter().all(|c| *c), "tiles must cover the grid");
    }

    #[test]
    fn smooth_bilinear_tiles_choose_r8_and_preserve_samples() {
        let source = table_from(9, 7, bilinear_coefficients);
        let packed =
            AeroResidualTable::encode(&source, AeroResidualBudget::uniform(1.0e-12)).unwrap();
        let stats = packed.stats();
        assert_eq!(stats.residual4_tiles, stats.tiles);
        assert_eq!(stats.residual6_tiles, 0);
        assert_eq!(stats.residual8_tiles, 0);
        assert_eq!(stats.residual16_tiles, 0);
        assert_eq!(stats.raw64_tiles, 0);
        for mach in 0..source.mach_grid.len() {
            for alpha in 0..source.alpha_grid_rad.len() {
                let got = packed.grid_sample(mach, alpha).unwrap();
                let expected = source.samples[mach * source.alpha_grid_rad.len() + alpha];
                assert_coeff_error_le(got, expected, packed.max_error());
            }
        }
    }

    #[test]
    fn adaptive_codec_respects_declared_grid_error() {
        let source = table_from(13, 11, nonlinear_coefficients);
        let budget = AeroResidualBudget {
            lift: 2.0e-4,
            drag: 5.0e-5,
            side_force: 5.0e-5,
            pitching_moment: 5.0e-5,
        };
        let packed = AeroResidualTable::encode(&source, budget).unwrap();
        let error = packed.max_error();
        assert!(budget.contains(error));
        for mach in 0..source.mach_grid.len() {
            for alpha in 0..source.alpha_grid_rad.len() {
                let got = packed.grid_sample(mach, alpha).unwrap();
                let expected = source.samples[mach * source.alpha_grid_rad.len() + alpha];
                assert_coeff_error_le(got, expected, error);
            }
        }
    }

    #[test]
    fn bilinear_sampling_does_not_amplify_grid_point_error() {
        let source = table_from(13, 11, nonlinear_coefficients);
        let packed =
            AeroResidualTable::encode(&source, AeroResidualBudget::uniform(1.0e-4)).unwrap();

        for i in 0..97 {
            let mach = -0.1 + i as f64 * 0.027;
            let alpha = -0.6 + (i * 37 % 101) as f64 * 0.012;
            let (sample, local_error) = packed.sample_with_error(mach, alpha);
            assert_coeff_error_le(sample, source.sample(mach, alpha), local_error);
            assert!(local_error.lift <= packed.max_error().lift);
            assert!(local_error.drag <= packed.max_error().drag);
            assert!(local_error.side_force <= packed.max_error().side_force);
            assert!(local_error.pitching_moment <= packed.max_error().pitching_moment);
        }
    }

    #[test]
    fn zero_budget_falls_back_where_quantization_is_not_exact() {
        let source = table_from(8, 8, nonlinear_coefficients);
        let packed = AeroResidualTable::encode(&source, AeroResidualBudget::uniform(0.0)).unwrap();
        assert!(packed.stats().raw64_tiles > 0);
        for mach in 0..source.mach_grid.len() {
            for alpha in 0..source.alpha_grid_rad.len() {
                let got = packed.grid_sample(mach, alpha).unwrap();
                let expected = source.samples[mach * source.alpha_grid_rad.len() + alpha];
                assert_eq!(got, expected);
            }
        }
    }

    #[test]
    fn odd_extents_and_clamping_match_canonical_table() {
        let source = table_from(5, 7, nonlinear_coefficients);
        let packed =
            AeroResidualTable::encode(&source, AeroResidualBudget::uniform(1.0e-5)).unwrap();
        let error = packed.max_error();
        for (mach, alpha) in [(-10.0, -10.0), (0.0, -0.45), (0.38, -0.17), (2.0, 4.0)] {
            assert_coeff_error_le(
                packed.sample(mach, alpha),
                source.sample(mach, alpha),
                error,
            );
        }
    }

    #[test]
    fn panel_model_can_sample_residual_table_without_expansion() {
        let source = table_from(25, 25, nonlinear_coefficients);
        let packed =
            AeroResidualTable::encode(&source, AeroResidualBudget::uniform(1.0e-4)).unwrap();

        let config = AeroConfig {
            control_effectiveness: 0.0,
            side_force_slope_per_rad: 0.0,
            roll_damping_coefficient: 0.0,
            pitch_damping_coefficient: 0.0,
            yaw_damping_coefficient: 0.0,
            ..AeroConfig::default()
        };
        let exact_model = PanelAeroModel::from_table(config, source.clone()).unwrap();
        let packed_model = PanelAeroModel::from_residual_table(config, packed.clone()).unwrap();

        let panel = AeroPanel::flat_plate(DVec3::new(2.0, 0.0, 0.0), 2.4, 1.3).unwrap();
        let area = panel.area_m2;
        let chord = panel.chord_m;
        let moment_arm = panel.center_of_pressure_body_m.length();
        let geometry = AeroGeometry::new(vec![panel]).unwrap();
        let environment = AeroEnvironment::standard_sea_level();
        let case = AeroCase::new(
            AeroState::new(DVec3::new(220.0, 0.0, -24.0), DVec3::ZERO),
            environment,
            geometry,
        )
        .unwrap();

        let exact = exact_model.evaluate_detailed(&case).unwrap();
        let got = packed_model.evaluate_detailed(&case).unwrap();
        let exact_load = &exact.panel_loads.as_ref().unwrap()[0];
        let (_, bound) = packed
            .sample_with_physical_bound(
                exact_load.mach,
                exact_load.angle_of_attack_rad,
                exact_load.dynamic_pressure_pa,
                area,
                chord,
                moment_arm,
            )
            .unwrap();

        assert!((got.force_body_n - exact.force_body_n).length() <= bound.force_n + 1.0e-9);
        assert!((got.moment_body_nm - exact.moment_body_nm).length() <= bound.moment_nm + 1.0e-9);
        assert!(packed_model.residual_coefficient_table().is_some());
        assert!(packed_model.coefficient_table.is_none());
    }

    #[test]
    fn sample_physical_bound_contains_actual_force_error() {
        let source = table_from(13, 11, nonlinear_coefficients);
        let packed =
            AeroResidualTable::encode(&source, AeroResidualBudget::uniform(1.0e-4)).unwrap();
        let mach = 0.73;
        let alpha = 0.11;
        let q = 18_000.0;
        let area = 2.4;
        let chord = 1.3;
        let arm = 2.7;
        let (got, bound) = packed
            .sample_with_physical_bound(mach, alpha, q, area, chord, arm)
            .unwrap();
        let exact = source.sample(mach, alpha);
        let actual_force_error = q
            * area
            * ((got.lift - exact.lift).abs()
                + (got.drag - exact.drag).abs()
                + (got.side_force - exact.side_force).abs());
        let actual_moment_error =
            q * area * chord * (got.pitching_moment - exact.pitching_moment).abs()
                + arm * actual_force_error;
        assert!(actual_force_error <= bound.force_n + 1.0e-12);
        assert!(actual_moment_error <= bound.moment_nm + 1.0e-12);
    }

    #[test]
    fn physical_budget_maps_to_safe_uniform_codec_budget() {
        let source = table_from(13, 11, nonlinear_coefficients);
        let physical = AeroPhysicalBudget {
            dynamic_pressure_pa: 25_000.0,
            area_m2: 3.0,
            reference_chord_m: 1.4,
            moment_arm_m: 2.2,
            max_force_error_n: 20.0,
            max_moment_error_nm: 50.0,
        };
        let coefficient_budget = physical.uniform_coefficient_budget().unwrap();
        let packed = AeroResidualTable::encode_for_physical_budget(&source, physical).unwrap();
        let error = packed.max_error();
        assert!(coefficient_budget.contains(error));
        let bound = error
            .physical_bound(
                physical.dynamic_pressure_pa,
                physical.area_m2,
                physical.reference_chord_m,
                physical.moment_arm_m,
            )
            .unwrap();
        assert!(bound.force_n <= physical.max_force_error_n + 1.0e-12);
        assert!(bound.moment_nm <= physical.max_moment_error_nm + 1.0e-12);
    }

    #[test]
    fn coefficient_error_maps_to_force_and_moment_bound() {
        let error = AeroCoefficientError {
            lift: 1.0e-3,
            drag: 2.0e-3,
            side_force: 3.0e-3,
            pitching_moment: 4.0e-3,
        };
        let bound = error.physical_bound(20_000.0, 2.0, 1.5, 3.0).unwrap();
        let expected_force = 20_000.0 * 2.0 * 6.0e-3;
        let expected_moment = 20_000.0 * 2.0 * 1.5 * 4.0e-3 + 3.0 * expected_force;
        assert!((bound.force_n - expected_force).abs() < 1.0e-12);
        assert!((bound.moment_nm - expected_moment).abs() < 1.0e-12);
    }

    #[test]
    fn packed_signed_rungs_round_trip_extremes() {
        for (codec, values) in [
            (AeroResidualCodec::Residual4, [-7, -1, 0, 7]),
            (AeroResidualCodec::Residual6, [-31, -1, 0, 31]),
            (AeroResidualCodec::Residual8, [-127, -1, 0, 127]),
            (
                AeroResidualCodec::Residual16,
                [i16::MIN + 1, -1, 0, i16::MAX],
            ),
        ] {
            let mut payload = Vec::new();
            encode_quantized_sample(&mut payload, codec, values);
            assert_eq!(payload.len(), codec.bytes_per_sample());
            assert_eq!(decode_quantized_sample(&payload, codec, 0), values);
        }
    }

    #[test]
    fn invalid_budget_is_rejected() {
        let source = table_from(4, 4, bilinear_coefficients);
        let error = AeroResidualTable::encode(
            &source,
            AeroResidualBudget {
                lift: f64::NAN,
                drag: 0.0,
                side_force: 0.0,
                pitching_moment: 0.0,
            },
        )
        .unwrap_err();
        assert!(matches!(error, AeroError::InvalidModel(_)));
    }
}
