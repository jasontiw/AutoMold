//! STL export

use crate::geometry::mesh::Mesh;
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
        let normal = e1.cross(&e2).normalize();

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
        let normal = e1.cross(&e2).normalize();

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
            let normal = e1.cross(&e2).normalize();

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
