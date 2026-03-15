//! Metadata export

use serde::{Deserialize, Serialize};
use std::path::Path;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum MetadataError {
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),
}

/// Metadata structure for mold generation
#[derive(Debug, Serialize, Deserialize)]
pub struct Metadata {
    /// Input file path
    pub input_file: String,

    /// Input unit (as string)
    pub input_unit: String,

    /// Normalized unit (always mm)
    pub normalized_unit: String,

    /// Bounding box in mm [x, y, z]
    pub bounding_box_mm: [f32; 3],

    /// Number of triangles in input
    pub triangles_in: usize,

    /// Number of triangles in output
    pub triangles_out: usize,

    /// Split axis used
    pub split_axis: String,

    /// Wall thickness in mm
    pub wall_thickness_mm: f32,

    /// Tolerance in mm
    pub tolerance_mm: f32,

    /// Whether pins were generated
    pub generate_pins: bool,
}

impl Metadata {
    /// Create new metadata
    pub fn new(
        input_file: &str,
        input_unit: &str,
        bounding_box_mm: [f32; 3],
        triangles_in: usize,
        triangles_out: usize,
        split_axis: &str,
        wall_thickness_mm: f32,
        tolerance_mm: f32,
        generate_pins: bool,
    ) -> Self {
        Self {
            input_file: input_file.to_string(),
            input_unit: input_unit.to_string(),
            normalized_unit: "mm".to_string(),
            bounding_box_mm,
            triangles_in,
            triangles_out,
            split_axis: split_axis.to_string(),
            wall_thickness_mm,
            tolerance_mm,
            generate_pins,
        }
    }
}

/// Write metadata to JSON file
pub fn write_metadata(metadata: &Metadata, path: &Path) -> Result<(), MetadataError> {
    let json = serde_json::to_string_pretty(metadata)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// Read metadata from JSON file
pub fn read_metadata(path: &Path) -> Result<Metadata, MetadataError> {
    let json = std::fs::read_to_string(path)?;
    let metadata: Metadata = serde_json::from_str(&json)?;
    Ok(metadata)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metadata_serialization() {
        let metadata = Metadata::new(
            "model.stl",
            "mm",
            [10.0, 20.0, 30.0],
            1000,
            2000,
            "Z",
            12.0,
            0.2,
            true,
        );

        let json = serde_json::to_string_pretty(&metadata).unwrap();
        let loaded: Metadata = serde_json::from_str(&json).unwrap();

        assert_eq!(loaded.input_file, "model.stl");
        assert_eq!(loaded.triangles_in, 1000);
    }
}
