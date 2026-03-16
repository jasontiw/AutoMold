//! Geometry module - core data structures for 3D geometry

use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};

/// A 3D triangle mesh
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mesh {
    pub vertices: Vec<Point3<f32>>,
    pub triangles: Vec<Triangle>,
    pub normals: Vec<Vector3<f32>>,
}

impl Default for Mesh {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            normals: Vec::new(),
        }
    }
}

impl Mesh {
    /// Create a new empty mesh
    pub fn new() -> Self {
        Self::default()
    }

    /// Create mesh from vertices and indices
    pub fn from_parts(vertices: Vec<Point3<f32>>, indices: Vec<[usize; 3]>) -> Self {
        let triangles: Vec<Triangle> = indices
            .iter()
            .map(|&[i0, i1, i2]| Triangle {
                indices: [i0, i1, i2],
            })
            .collect();

        let normals = Self::calculate_normals(&vertices, &triangles);

        Self {
            vertices,
            triangles,
            normals,
        }
    }

    /// Calculate face normals for all triangles
    pub fn calculate_normals(
        vertices: &[Point3<f32>],
        triangles: &[Triangle],
    ) -> Vec<Vector3<f32>> {
        triangles
            .iter()
            .map(|t| {
                let v0 = &vertices[t.indices[0]];
                let v1 = &vertices[t.indices[1]];
                let v2 = &vertices[t.indices[2]];

                let e1 = v1 - v0;
                let e2 = v2 - v0;
                let n = e1.cross(&e2);

                if n.magnitude_squared() > 1e-10 {
                    n.normalize()
                } else {
                    Vector3::zeros()
                }
            })
            .collect()
    }

    /// Recalculate all vertex normals by averaging face normals
    pub fn recalculate_vertex_normals(&mut self) {
        let mut accum: Vec<Vector3<f32>> = vec![Vector3::zeros(); self.vertices.len()];

        for (i, tri) in self.triangles.iter().enumerate() {
            let n = self.normals[i];
            for &idx in &tri.indices {
                accum[idx] += n;
            }
        }

        for n in accum.iter_mut() {
            if n.magnitude_squared() > 1e-10 {
                *n = n.normalize();
            }
        }
    }

    /// Calculate bounding box of the mesh
    pub fn calculate_bounding_box(&self) -> super::bbox::BoundingBox {
        super::bbox::BoundingBox::from_points(&self.vertices)
    }

    /// Get triangle count
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Get vertex count
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Apply transformation to all vertices
    pub fn transform(&mut self, matrix: &nalgebra::Matrix4<f32>) {
        for v in &mut self.vertices {
            let p = matrix.transform_point(v);
            *v = p;
        }
        // Recalculate normals
        self.normals = Self::calculate_normals(&self.vertices, &self.triangles);
    }

    /// Apply offset to all vertices (for tolerance)
    pub fn apply_offset(&mut self, offset: f32) {
        // Clone vertices first to avoid borrow conflict
        let new_vertices: Vec<Point3<f32>> = self
            .vertices
            .iter()
            .enumerate()
            .map(|(i, v)| {
                let n = if i < self.normals.len() {
                    self.normals[i]
                } else {
                    Vector3::zeros()
                };
                let displacement = n * offset;
                // This is simplified - real implementation needs per-vertex normals
                *v + displacement
            })
            .collect();

        self.vertices = new_vertices;
    }
}

/// A single triangle (face)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct Triangle {
    pub indices: [usize; 3], // indices into vertex array
}

impl Triangle {
    pub fn new(i0: usize, i1: usize, i2: usize) -> Self {
        Self {
            indices: [i0, i1, i2],
        }
    }

    /// Get vertices from this triangle
    pub fn get_vertices<'a>(&self, vertices: &'a [Point3<f32>]) -> [&'a Point3<f32>; 3] {
        [
            &vertices[self.indices[0]],
            &vertices[self.indices[1]],
            &vertices[self.indices[2]],
        ]
    }
}

/// A vertex with position and optional normal
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Vertex {
    pub position: Point3<f32>,
    pub normal: Option<Vector3<f32>>,
}

impl Vertex {
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            position,
            normal: None,
        }
    }

    pub fn with_normal(position: Point3<f32>, normal: Vector3<f32>) -> Self {
        Self {
            position,
            normal: Some(normal),
        }
    }
}

/// Edge information for mesh analysis
#[derive(Debug, Clone)]
pub struct Edge {
    pub vertices: [usize; 2],
    pub triangles: [Option<usize>; 2], // adjacent triangles
}

impl Edge {
    pub fn is_manifold(&self) -> bool {
        self.triangles.iter().filter(|t| t.is_some()).count() <= 1
    }

    pub fn is_boundary(&self) -> bool {
        self.triangles.iter().filter(|t| t.is_some()).count() == 1
    }
}

/// Mesh statistics
#[derive(Debug, Default)]
pub struct MeshStats {
    pub vertices: usize,
    pub triangles: usize,
    pub non_manifold_edges: usize,
    pub degenerate_triangles: usize,
    pub holes: usize,
}

impl Mesh {
    /// Analyze mesh quality
    pub fn analyze(&self) -> MeshStats {
        let mut stats = MeshStats {
            vertices: self.vertices.len(),
            triangles: self.triangles.len(),
            ..Default::default()
        };

        // Count degenerate triangles
        for tri in &self.triangles {
            let v = tri.get_vertices(&self.vertices);
            let e1 = v[1] - v[0];
            let e2 = v[2] - v[0];
            let cross = e1.cross(&e2);
            if cross.magnitude_squared() < 1e-10 {
                stats.degenerate_triangles += 1;
            }
        }

        stats
    }
}

/// Iterator over mesh triangles
pub struct TriangleIter<'a> {
    mesh: &'a Mesh,
    index: usize,
}

impl<'a> Iterator for TriangleIter<'a> {
    type Item = (&'a Point3<f32>, &'a Point3<f32>, &'a Point3<f32>);

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.mesh.triangles.len() {
            return None;
        }

        let tri = &self.mesh.triangles[self.index];
        let v = tri.get_vertices(&self.mesh.vertices);
        self.index += 1;

        Some((v[0], v[1], v[2]))
    }
}

impl<'a> IntoIterator for &'a Mesh {
    type Item = (&'a Point3<f32>, &'a Point3<f32>, &'a Point3<f32>);
    type IntoIter = TriangleIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        TriangleIter {
            mesh: self,
            index: 0,
        }
    }
}

/// Trait for converting Mesh to CSG representation
pub trait MeshToCSG {
    type CSG;

    /// Convert mesh to CSG representation
    fn to_csg(&self) -> Result<Self::CSG, String>;

    /// Get triangle count threshold for auto strategy selection
    fn csg_triangle_threshold() -> usize {
        50000
    }
}

/// Trait for converting CSG representation back to Mesh
pub trait CSGToMesh {
    type CSG;

    /// Convert CSG representation back to mesh
    fn to_mesh(&self) -> Result<Mesh, String>;

    /// Perform boolean subtraction using CSG
    fn csg_subtract(&self, other: &Self::CSG) -> Result<Self::CSG, String>;
}

/// Placeholder for CSG conversion - csgrs integration pending API simplification
pub fn mesh_to_csgrs(_mesh: &Mesh) -> Result<csgrs::mesh::Mesh<()>, String> {
    Err("CSG integration pending - use voxel fallback".to_string())
}

/// Placeholder for CSG to Mesh conversion
pub fn csgrs_to_mesh(_csg: &csgrs::mesh::Mesh<()>) -> Result<Mesh, String> {
    Err("CSG integration pending - use voxel fallback".to_string())
}

// ============================================================================
// csgrs Integration - Conversion Functions via STL
// ============================================================================

use crate::pipeline::boolean::BooleanError;
use std::io::Write;

/// Write mesh to binary STL in memory
fn write_stl_to_vec(mesh: &Mesh) -> Result<Vec<u8>, BooleanError> {
    if mesh.triangles.is_empty() {
        return Err(BooleanError::InvalidMesh("Empty mesh".to_string()));
    }

    let mut bytes = Vec::new();

    // Write 80-byte header (zeros)
    let header = [0u8; 80];
    bytes
        .write_all(&header)
        .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;

    // Write triangle count (4 bytes)
    let count = mesh.triangles.len() as u32;
    bytes
        .write_all(&count.to_le_bytes())
        .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;

    // Write each triangle
    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);

        // Calculate normal
        let e1 = v[1] - v[0];
        let e2 = v[2] - v[0];
        let normal = e1.cross(&e2).normalize();

        // Write normal (12 bytes)
        bytes
            .write_all(&normal.x.to_le_bytes())
            .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;
        bytes
            .write_all(&normal.y.to_le_bytes())
            .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;
        bytes
            .write_all(&normal.z.to_le_bytes())
            .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;

        // Write vertices (36 bytes)
        for vertex in v {
            bytes
                .write_all(&vertex.x.to_le_bytes())
                .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;
            bytes
                .write_all(&vertex.y.to_le_bytes())
                .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;
            bytes
                .write_all(&vertex.z.to_le_bytes())
                .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;
        }

        // Write attribute byte count (2 bytes) - always 0
        let attr = 0u16;
        bytes
            .write_all(&attr.to_le_bytes())
            .map_err(|e| BooleanError::InvalidMesh(e.to_string()))?;
    }

    Ok(bytes)
}

/// Read mesh from binary STL in memory
fn read_stl_from_slice(data: &[u8]) -> Result<Mesh, BooleanError> {
    use crate::geometry::mesh::Triangle;
    use nalgebra::Point3;

    if data.len() < 84 {
        return Err(BooleanError::InvalidMesh("STL data too short".to_string()));
    }

    // Skip 80-byte header
    let triangle_count = u32::from_le_bytes([data[80], data[81], data[82], data[83]]);

    let expected_size = 84 + (triangle_count as usize * 50);
    if data.len() < expected_size {
        return Err(BooleanError::InvalidMesh("STL data truncated".to_string()));
    }

    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut triangles: Vec<Triangle> = Vec::new();

    let mut offset = 84;
    for _ in 0..triangle_count {
        // Skip normal (12 bytes)
        offset += 12;

        // Read 3 vertices (36 bytes each)
        for _ in 0..3 {
            let x = f32::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
            ]);
            let y = f32::from_le_bytes([
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]);
            let z = f32::from_le_bytes([
                data[offset + 8],
                data[offset + 9],
                data[offset + 10],
                data[offset + 11],
            ]);
            offset += 12;

            vertices.push(Point3::new(x, y, z));
        }

        // Skip attribute byte count (2 bytes)
        offset += 2;

        // Create triangle with indices
        let idx = triangles.len() * 3;
        triangles.push(Triangle::new(idx, idx + 1, idx + 2));
    }

    // Calculate normals
    let normals = Mesh::calculate_normals(&vertices, &triangles);

    Ok(Mesh {
        vertices,
        triangles,
        normals,
    })
}

/// Convert AutoMold Mesh to csgrs Mesh via STL format
/// This approach avoids nalgebra version conflicts by using STL as an intermediate format
pub fn mesh_to_csgrs_mesh(mesh: &Mesh) -> Result<csgrs::mesh::Mesh<()>, BooleanError> {
    if mesh.vertices.is_empty() {
        return Err(BooleanError::InvalidMesh("Empty vertices".to_string()));
    }
    if mesh.triangles.is_empty() {
        return Err(BooleanError::InvalidMesh("Empty triangles".to_string()));
    }

    // Export to STL bytes
    let stl_bytes = write_stl_to_vec(mesh)?;

    // Import from STL bytes using csgrs
    let csg_mesh = csgrs::mesh::Mesh::from_stl(&stl_bytes, None)
        .map_err(|e| BooleanError::InvalidMesh(format!("CSG import failed: {}", e)))?;

    Ok(csg_mesh)
}

/// Convert csgrs Mesh back to AutoMold Mesh via STL format
pub fn csgrs_mesh_to_mesh(csg: csgrs::mesh::Mesh<()>) -> Result<Mesh, BooleanError> {
    // Export csgrs mesh to STL bytes
    let stl_bytes = csg
        .to_stl_binary("result")
        .map_err(|e| BooleanError::InvalidMesh(format!("CSG STL export failed: {}", e)))?;

    // Read from STL bytes
    let mesh = read_stl_from_slice(&stl_bytes)?;

    Ok(mesh)
}
