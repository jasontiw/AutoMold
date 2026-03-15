//! Unit conversion utilities

use crate::core::config::Unit;

/// Convert a value from one unit to millimeters
pub fn to_millimeters(value: f32, unit: Unit) -> f32 {
    value * unit.to_mm()
}

/// Convert a value from millimeters to another unit
pub fn from_millimeters(value: f32, unit: Unit) -> f32 {
    value / unit.to_mm()
}

/// Parse unit from string
pub fn parse_unit(s: &str) -> Option<Unit> {
    Unit::from_str(s)
}

/// Unit display name
pub fn unit_name(unit: Unit) -> &'static str {
    match unit {
        Unit::Millimeters => "millimeters",
        Unit::Centimeters => "centimeters",
        Unit::Inches => "inches",
    }
}

/// Unit abbreviation
pub fn unit_abbrev(unit: Unit) -> &'static str {
    unit.as_str()
}

/// Convert all vertices in a mesh to millimeters
pub fn convert_mesh_to_mm(vertices: &mut [[f32; 3]], unit: Unit) {
    let factor = unit.to_mm();
    for v in vertices.iter_mut() {
        v[0] *= factor;
        v[1] *= factor;
        v[2] *= factor;
    }
}

/// Detect likely unit from bounding box size
/// Returns the most likely unit based on typical object sizes
pub fn detect_unit_from_size(bbox_size: f32) -> Unit {
    // If bounding box is very small (< 1mm), likely inches
    if bbox_size < 1.0 {
        return Unit::Inches;
    }
    // If very large (> 2000mm), could be cm
    if bbox_size > 2000.0 {
        return Unit::Centimeters;
    }
    // Default to mm
    Unit::Millimeters
}

/// Suggested unit based on bounding box
pub fn suggest_unit(bbox_min: [f32; 3], bbox_max: [f32; 3]) -> (Unit, f32) {
    let size_x = (bbox_max[0] - bbox_min[0]).abs();
    let size_y = (bbox_max[1] - bbox_min[1]).abs();
    let size_z = (bbox_max[2] - bbox_min[2]).abs();
    let max_size = size_x.max(size_y).max(size_z);

    // Check if likely inches (typical objects 1-20 inches)
    if max_size > 0.5 && max_size < 50.0 {
        // Could be inches - convert and check if reasonable in mm
        let in_mm = max_size * 25.4;
        if in_mm > 10.0 && in_mm < 1000.0 {
            return (Unit::Inches, 25.4);
        }
    }

    // Check if likely cm (typical objects 0.1-100 cm)
    if max_size > 0.1 && max_size < 100.0 {
        let in_mm = max_size * 10.0;
        if in_mm > 1.0 && in_mm < 1000.0 {
            return (Unit::Centimeters, 10.0);
        }
    }

    // Default to mm
    (Unit::Millimeters, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conversion() {
        assert_eq!(to_millimeters(1.0, Unit::Millimeters), 1.0);
        assert_eq!(to_millimeters(1.0, Unit::Centimeters), 10.0);
        assert_eq!(to_millimeters(1.0, Unit::Inches), 25.4);

        assert_eq!(from_millimeters(10.0, Unit::Millimeters), 10.0);
        assert_eq!(from_millimeters(10.0, Unit::Centimeters), 1.0);
        assert_eq!(from_millimeters(25.4, Unit::Inches), 1.0);
    }
}
