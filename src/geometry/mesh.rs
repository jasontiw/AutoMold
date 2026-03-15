//! Geometry module - core data structures for 3D geometry

use nalgebra::{Point3, Vector3};
use serde::{Serialize, Deserialize};

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
    pub fn calculate_normals(vertices: &[Point3<f32>>, triangles: &[Triangle]) -> Vec<Vector3<f32>> {
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
        for (i, v) in self.vertices.iter().enumerate() {
            let n = if i < self.normals.len() {
                self.normals[i]
            } else {
                Vector3::zeros()
            };
            let displacement = n * offset;
            // This is simplified - real implementation needs per-vertex normals
            self.vertices[i] = *v + displacement;
        }
    }
}

/// A single triangle (face)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct Triangle {
    pub indices: [usize; 3], // indices into vertex array
}

impl Triangle {
    pub fn new(i0: usize, i1: usize, i2: usize) -> Self {
        Self { indices: [i0, i1, i2] }
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
        Self { position, normal: None }
    }
    
    pub fn with_normal(position: Point3<f32>, normal: Vector3<f32>) -> Self {
        Self { position, normal: Some(normal) }
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
        TriangleIter { mesh: self, index: 0 }
    }
}