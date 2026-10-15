//! Backend-neutral direct surface lighting; prefix of the shared Lighting ABI.
/// Directions point toward sources in render-world coordinates. RGB values are
/// linear illuminance in lux. Three slots match the current terrain shader.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SurfaceLighting {
    pub ambient_lux: [f32; 4],
    pub directions: [[f32; 4]; 3],
    pub colors_lux: [[f32; 4]; 3],
}

impl SurfaceLighting {
    /// Seven vec4 records. Reject invalid illuminance and non-unit active
    /// directions; unused zero-intensity slots may have zero directions.
    pub fn records(&self) -> Option<[[f32; 4]; 7]> {
        let records = [
            self.ambient_lux,
            self.directions[0],
            self.directions[1],
            self.directions[2],
            self.colors_lux[0],
            self.colors_lux[1],
            self.colors_lux[2],
        ];
        if records.iter().flatten().any(|v| !v.is_finite())
            || self.ambient_lux[..3]
                .iter()
                .chain(self.colors_lux.iter().flat_map(|c| &c[..3]))
                .any(|v| *v < 0.0)
            || self.directions.iter().zip(self.colors_lux).any(|(d, c)| {
                c[..3].iter().any(|v| *v > 0.0)
                    && (d[..3].iter().map(|v| v * v).sum::<f32>() - 1.0).abs() > 1e-4
            })
        {
            return None;
        }
        Some(records)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lighting_abi_rejects_invalid_values_before_publication() {
        let mut light = SurfaceLighting::default();
        assert!(light.records().is_some());
        light.directions[0] = [0.0, 0.0, 1.0, 0.0];
        light.colors_lux[0] = [50_000.0, 40_000.0, 30_000.0, 0.0];
        assert_eq!(light.records().unwrap()[4], light.colors_lux[0]);
        light.colors_lux[0][0] = -1.0;
        assert!(light.records().is_none());
        light.colors_lux[0][0] = 1.0;
        light.directions[0][2] = 0.5;
        assert!(light.records().is_none());
        light.directions[0][2] = 1.0;
        light.ambient_lux[0] = f32::NAN;
        assert!(light.records().is_none());
    }
}
