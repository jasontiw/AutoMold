//! Mesh loader - loads STL and OBJ files

use crate::core::config::Unit;
use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::Point3;
use std::io::Read;
use std::path::Path;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum LoadError {
    #[error("File not found: {0}")]
    FileNotFound(String),

    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("Failed to parse: {0}")]
    ParseError(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

/// Detect file format from extension
fn detect_format(path: &Path) -> Option<&'static str> {
    match path.extension()?.to_str()?.to_lowercase().as_str() {
        "stl" => Some("stl"),
        "obj" => Some("obj"),
        "3mf" => Some("3mf"),
        _ => None,
    }
}

/// Load mesh from file (auto-detects format)
pub fn load_mesh(path: &Path, unit: Unit) -> Result<Mesh, LoadError> {
    let format = detect_format(path).ok_or_else(|| {
        LoadError::UnsupportedFormat(
            path.extension()
                .map(|e| e.to_string_lossy().to_string())
                .unwrap_or_else(|| "unknown".to_string()),
        )
    })?;

    match format {
        "stl" => load_stl(path, unit),
        "obj" => load_obj(path, unit),
        "3mf" => load_3mf(path, unit),
        _ => Err(LoadError::UnsupportedFormat(format.to_string())),
    }
}

/// Load STL file (binary or ASCII)
pub fn load_stl(path: &Path, unit: Unit) -> Result<Mesh, LoadError> {
    if !path.exists() {
        return Err(LoadError::FileNotFound(path.to_string_lossy().to_string()));
    }

    let mut file = std::fs::File::open(path)?;
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer)?;

    // Try to detect if binary or ASCII
    let mesh = if is_stl_ascii(&buffer) {
        parse_stl_ascii(&buffer, unit)?
    } else {
        parse_stl_binary(&buffer, unit)?
    };

    Ok(mesh)
}

/// Check if STL content is ASCII
fn is_stl_ascii(data: &[u8]) -> bool {
    // Look for "solid" at the start, common in ASCII STL
    let start = std::str::from_utf8(&data[..data.len().min(80)]).unwrap_or("");
    start.starts_with("solid")
}

/// Parse binary STL
fn parse_stl_binary(data: &[u8], unit: Unit) -> Result<Mesh, LoadError> {
    // Binary STL: 80 bytes header, 4 bytes triangle count, then triangles
    if data.len() < 84 {
        return Err(LoadError::ParseError("File too small".to_string()));
    }

    let triangle_count = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;

    // Each triangle is 50 bytes (12 bytes normal + 9 bytes vertices + 2 bytes attribute)
    let expected_size = 84 + triangle_count * 50;
    if data.len() < expected_size {
        return Err(LoadError::ParseError(format!(
            "Expected {} bytes, got {}",
            expected_size,
            data.len()
        )));
    }

    let factor = unit.to_mm();
    let mut vertices: Vec<Point3<f32>> = Vec::with_capacity(triangle_count * 3);
    let mut triangles: Vec<Triangle> = Vec::with_capacity(triangle_count);

    for i in 0..triangle_count {
        let offset = 84 + i * 50;

        // Skip normal (12 bytes)
        // Read 3 vertices (36 bytes total)
        let v0_x = f32::from_le_bytes([
            data[offset + 12],
            data[offset + 13],
            data[offset + 14],
            data[offset + 15],
        ]) * factor;
        let v0_y = f32::from_le_bytes([
            data[offset + 16],
            data[offset + 17],
            data[offset + 18],
            data[offset + 19],
        ]) * factor;
        let v0_z = f32::from_le_bytes([
            data[offset + 20],
            data[offset + 21],
            data[offset + 22],
            data[offset + 23],
        ]) * factor;

        let v1_x = f32::from_le_bytes([
            data[offset + 24],
            data[offset + 25],
            data[offset + 26],
            data[offset + 27],
        ]) * factor;
        let v1_y = f32::from_le_bytes([
            data[offset + 28],
            data[offset + 29],
            data[offset + 30],
            data[offset + 31],
        ]) * factor;
        let v1_z = f32::from_le_bytes([
            data[offset + 32],
            data[offset + 33],
            data[offset + 34],
            data[offset + 35],
        ]) * factor;

        let v2_x = f32::from_le_bytes([
            data[offset + 36],
            data[offset + 37],
            data[offset + 38],
            data[offset + 39],
        ]) * factor;
        let v2_y = f32::from_le_bytes([
            data[offset + 40],
            data[offset + 41],
            data[offset + 42],
            data[offset + 43],
        ]) * factor;
        let v2_z = f32::from_le_bytes([
            data[offset + 44],
            data[offset + 45],
            data[offset + 46],
            data[offset + 47],
        ]) * factor;

        let idx = vertices.len();
        vertices.push(Point3::new(v0_x, v0_y, v0_z));
        vertices.push(Point3::new(v1_x, v1_y, v1_z));
        vertices.push(Point3::new(v2_x, v2_y, v2_z));

        triangles.push(Triangle::new(idx, idx + 1, idx + 2));
    }

    let mesh = Mesh::from_parts(vertices, triangles.iter().map(|t| t.indices).collect());
    Ok(mesh)
}

/// Parse ASCII STL
fn parse_stl_ascii(data: &[u8], unit: Unit) -> Result<Mesh, LoadError> {
    let content = std::str::from_utf8(data).map_err(|e| LoadError::ParseError(e.to_string()))?;

    let factor = unit.to_mm();
    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut triangles: Vec<Triangle> = Vec::new();

    let _current_vertex: Option<[f32; 3]> = None;
    let mut vertex_count = 0;

    for line in content.lines() {
        let line = line.trim();

        if line.starts_with("vertex") {
            let coords: Vec<f32> = line
                .split_whitespace()
                .skip(1)
                .filter_map(|s| s.parse().ok())
                .collect();

            if coords.len() >= 3 {
                let x = coords[0] * factor;
                let y = coords[1] * factor;
                let z = coords[2] * factor;

                let idx = vertices.len();
                vertices.push(Point3::new(x, y, z));

                vertex_count += 1;
                if vertex_count % 3 == 0 {
                    triangles.push(Triangle::new(idx - 2, idx - 1, idx));
                }
            }
        }
    }

    if triangles.is_empty() {
        return Err(LoadError::ParseError("No triangles found".to_string()));
    }

    let mesh = Mesh::from_parts(vertices, triangles.iter().map(|t| t.indices).collect());
    Ok(mesh)
}

/// Load OBJ file
pub fn load_obj(path: &Path, unit: Unit) -> Result<Mesh, LoadError> {
    if !path.exists() {
        return Err(LoadError::FileNotFound(path.to_string_lossy().to_string()));
    }

    let content = std::fs::read_to_string(path)?;

    let factor = unit.to_mm();
    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut indices: Vec<[usize; 3]> = Vec::new();

    for line in content.lines() {
        let line = line.trim();

        if line.starts_with("v ") {
            let coords: Vec<f32> = line
                .split_whitespace()
                .skip(1)
                .filter_map(|s| s.parse().ok())
                .collect();

            if coords.len() >= 3 {
                vertices.push(Point3::new(
                    coords[0] * factor,
                    coords[1] * factor,
                    coords[2] * factor,
                ));
            }
        } else if line.starts_with("f ") {
            let face_vertices: Vec<usize> = line
                .split_whitespace()
                .skip(1)
                .filter_map(|s| {
                    let idx = s.split('/').next()?.parse::<isize>().ok()?;
                    if idx > 0 {
                        Some(idx as usize - 1)
                    } else {
                        None
                    }
                })
                .collect();

            // Triangulate face (assumes convex polygon)
            if face_vertices.len() >= 3 {
                for i in 1..face_vertices.len() - 1 {
                    indices.push([face_vertices[0], face_vertices[i], face_vertices[i + 1]]);
                }
            }
        }
    }

    if vertices.is_empty() || indices.is_empty() {
        return Err(LoadError::ParseError(
            "No vertices or faces found".to_string(),
        ));
    }

    let mesh = Mesh::from_parts(vertices, indices);
    Ok(mesh)
}

/// Load 3MF file (simplified - just extracts STL-like data)
pub fn load_3mf(_path: &Path, _unit: Unit) -> Result<Mesh, LoadError> {
    // 3MF is a ZIP file - for now, just return error indicating not implemented
    // In full implementation, would extract and parse the internal model
    Err(LoadError::UnsupportedFormat(
        "3MF loading not yet implemented".to_string(),
    ))
}
