//! STL export

use crate::geometry::mesh::{safe_normalize, Mesh};
use std::io::Write;
use std::path::Path;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum StlError {
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Invalid mesh")]
    InvalidMesh,
}

/// Write mesh to binary STL file
pub fn write_stl(mesh: &Mesh, path: &Path) -> Result<(), StlError> {
    if mesh.triangles.is_empty() {
        return Err(StlError::InvalidMesh);
    }

    let mut file = std::fs::File::create(path)?;

    // Write 80-byte header (zeros)
    let header = [0u8; 80];
    file.write_all(&header)?;

    // Write triangle count (4 bytes)
    let count = mesh.triangles.len() as u32;
    file.write_all(&count.to_le_bytes())?;

    // Write each triangle
    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);

        // Calculate normal
        let e1 = v[1] - v[0];
        let e2 = v[2] - v[0];
        let normal = safe_normalize(e1.cross(&e2));

        // Write normal (12 bytes)
        file.write_all(&normal.x.to_le_bytes())?;
        file.write_all(&normal.y.to_le_bytes())?;
        file.write_all(&normal.z.to_le_bytes())?;

        // Write vertices (36 bytes)
        for vertex in v {
            file.write_all(&vertex.x.to_le_bytes())?;
            file.write_all(&vertex.y.to_le_bytes())?;
            file.write_all(&vertex.z.to_le_bytes())?;
        }

        // Write attribute byte count (2 bytes) - always 0
        let attr = 0u16;
        file.write_all(&attr.to_le_bytes())?;
    }

    Ok(())
}

/// Write ASCII STL (for debugging)
pub fn write_stl_ascii(mesh: &Mesh, path: &Path) -> Result<(), StlError> {
    let mut file = std::fs::File::create(path)?;

    writeln!(file, "solid model")?;

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);

        // Calculate normal
        let e1 = v[1] - v[0];
        let e2 = v[2] - v[0];
        let normal = safe_normalize(e1.cross(&e2));

        writeln!(
            file,
            "  facet normal {} {} {}",
            normal.x, normal.y, normal.z
        )?;
        writeln!(file, "    outer loop")?;

        for vertex in v {
            writeln!(file, "      vertex {} {} {}", vertex.x, vertex.y, vertex.z)?;
        }

        writeln!(file, "    endloop")?;
        writeln!(file, "  endfacet")?;
    }

    writeln!(file, "endsolid model")?;

    Ok(())
}

/// Streaming write for large meshes (writes in chunks)
pub fn write_stl_streaming(mesh: &Mesh, path: &Path, chunk_size: usize) -> Result<(), StlError> {
    let mut file = std::fs::File::create(path)?;

    // Write header
    let header = [0u8; 80];
    file.write_all(&header)?;

    // Write triangle count
    let count = mesh.triangles.len() as u32;
    file.write_all(&count.to_le_bytes())?;

    // Write in chunks
    for chunk in mesh.triangles.chunks(chunk_size) {
        for tri in chunk {
            let v = tri.get_vertices(&mesh.vertices);

            let e1 = v[1] - v[0];
            let e2 = v[2] - v[0];
            let normal = safe_normalize(e1.cross(&e2));

            file.write_all(&normal.x.to_le_bytes())?;
            file.write_all(&normal.y.to_le_bytes())?;
            file.write_all(&normal.z.to_le_bytes())?;

            for vertex in v {
                file.write_all(&vertex.x.to_le_bytes())?;
                file.write_all(&vertex.y.to_le_bytes())?;
                file.write_all(&vertex.z.to_le_bytes())?;
            }

            let attr = 0u16;
            file.write_all(&attr.to_le_bytes())?;
        }

        // Flush to avoid keeping too much in buffer
        file.flush()?;
    }

    Ok(())
}

/// Read STL file
pub fn read_stl(path: &Path) -> Result<Mesh, StlError> {
    use crate::pipeline::loader;
    loader::load_stl(path, crate::core::config::Unit::Millimeters)
        .map_err(|_e| StlError::InvalidMesh) // Simplify error conversion
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::mesh::Mesh;

    /// Fixture mesh: one valid triangle plus one collinear zero-area sliver
    fn sliver_mesh() -> Mesh {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(0.0, 1.0, 0.0),
            nalgebra::Point3::new(2.0, 0.0, 0.0),
        ];
        let indices = vec![[0, 1, 2], [0, 1, 3]];
        Mesh::from_parts(vertices, indices)
    }

    /// Read 3 little-endian f32 values from a 12-byte STL normal block
    fn read_f32x3(bytes: &[u8]) -> [f32; 3] {
        [
            f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
        ]
    }

    /// Parse the normal of every binary STL record (skip 84-byte header,
    /// 50 bytes per record: 12 normal + 36 vertices + 2 attribute)
    fn binary_normal_records(bytes: &[u8]) -> Vec<[f32; 3]> {
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        bytes[84..]
            .chunks(50)
            .take(count)
            .map(|record| read_f32x3(&record[0..12]))
            .collect()
    }

    /// Binary writer emits only finite normals; the sliver facet exports (0,0,0)
    #[test]
    fn test_write_stl_binary_sliver_normals_finite() {
        let file = tempfile::NamedTempFile::new().expect("create temp file");
        let path = file.path().to_path_buf();
        write_stl(&sliver_mesh(), &path).expect("write_stl should succeed");

        let bytes = std::fs::read(&path).expect("read binary file");
        let normals = binary_normal_records(&bytes);
        assert_eq!(normals.len(), 2);

        for (i, n) in normals.iter().enumerate() {
            assert!(
                n.iter().all(|v| v.is_finite()),
                "Record {} normal must be finite, got {:?}",
                i,
                n
            );
        }
        assert_eq!(normals[1], [0.0, 0.0, 0.0]);
    }

    /// ASCII writer never prints NaN; the sliver facet normal line is "0 0 0"
    #[test]
    fn test_write_stl_ascii_sliver_normal_zero() {
        let file = tempfile::NamedTempFile::new().expect("create temp file");
        let path = file.path().to_path_buf();
        write_stl_ascii(&sliver_mesh(), &path).expect("write_stl_ascii should succeed");

        let content = std::fs::read_to_string(&path).expect("read ascii file");
        let normal_lines: Vec<&str> = content
            .lines()
            .filter(|line| line.trim_start().starts_with("facet normal"))
            .collect();
        assert_eq!(normal_lines.len(), 2);

        for line in &normal_lines {
            assert!(
                !line.to_lowercase().contains("nan"),
                "Facet normal line must not contain NaN: {}",
                line
            );
        }
        assert!(
            normal_lines[1].contains("0 0 0"),
            "Sliver facet normal must be zero: {}",
            normal_lines[1]
        );
    }

    /// Streaming writer (chunk_size = 1) emits only finite normals too
    #[test]
    fn test_write_stl_streaming_sliver_normals_finite() {
        let file = tempfile::NamedTempFile::new().expect("create temp file");
        let path = file.path().to_path_buf();
        write_stl_streaming(&sliver_mesh(), &path, 1).expect("write_stl_streaming should succeed");

        let bytes = std::fs::read(&path).expect("read streaming file");
        let normals = binary_normal_records(&bytes);
        assert_eq!(normals.len(), 2);

        for (i, n) in normals.iter().enumerate() {
            assert!(
                n.iter().all(|v| v.is_finite()),
                "Record {} normal must be finite, got {:?}",
                i,
                n
            );
        }
        assert_eq!(normals[1], [0.0, 0.0, 0.0]);
    }
}
