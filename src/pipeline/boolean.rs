//! Boolean operations - mesh subtraction (block - model)

use crate::geometry::mesh::{Mesh, Triangle};
use bvh::aabb::{Aabb, Bounded};
use bvh::bounding_hierarchy::{BHShape, BoundingHierarchy};
use bvh::bvh::Bvh;
use bvh::ray::Ray;
use nalgebra::{Point3, Vector3};
use thiserror::Error;
use tracing::info;

/// Memory estimation constants (from PRD)
/// Estimated memory per triangle in bytes
pub const MEMORY_PER_TRIANGLE: usize = 470;

#[derive(Error, Debug)]
pub enum BooleanError {
    #[error("BVH construction failed")]
    BVHError,

    #[error("No intersection found")]
    NoIntersection,

    #[error("Too many intersections")]
    TooManyIntersections,
}

/// Perform boolean subtraction: block - model
/// This function tries CSG (if available) or uses ray-casting fallback
pub fn boolean_subtract(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    let block_triangles = block.triangles.len();
    let model_triangles = model.triangles.len();
    let estimated_memory = (block_triangles + model_triangles) * MEMORY_PER_TRIANGLE;
    info!(
        "Boolean operation: block={} triangles, model={} triangles, estimated memory: {} bytes ({:.1} KB)",
        block_triangles,
        model_triangles,
        estimated_memory,
        estimated_memory as f32 / 1024.0
    );

    // Use ray-casting approach
    let bvh = build_bvh(model)?;

    let mut result_vertices: Vec<Point3<f32>> = Vec::new();
    let mut result_indices: Vec<[usize; 3]> = Vec::new();

    // For each triangle in block, check if it intersects model
    for tri in &block.triangles {
        let v = tri.get_vertices(&block.vertices);

        // Simple check: if triangle center is inside model, skip it
        let center = Point3::new(
            (v[0].x + v[1].x + v[2].x) / 3.0,
            (v[0].y + v[1].y + v[2].y) / 3.0,
            (v[0].z + v[1].z + v[2].z) / 3.0,
        );

        if !is_inside_mesh(center, &bvh, model) {
            // This face doesn't intersect the model, keep it
            let idx = result_vertices.len();
            result_vertices.push(*v[0]);
            result_vertices.push(*v[1]);
            result_vertices.push(*v[2]);
            result_indices.push([idx, idx + 1, idx + 2]);
        }
    }

    // Rebuild normals
    let triangles: Vec<Triangle> = result_indices
        .iter()
        .map(|i| Triangle::new(i[0], i[1], i[2]))
        .collect();
    let normals = Mesh::calculate_normals(&result_vertices, &triangles);

    let result_triangles = triangles.len();
    let result_memory = result_triangles * MEMORY_PER_TRIANGLE;
    info!(
        "Boolean result: {} triangles (estimated memory: {} bytes / {:.1} KB)",
        result_triangles,
        result_memory,
        result_memory as f32 / 1024.0
    );

    Ok(Mesh {
        vertices: result_vertices,
        triangles,
        normals,
    })
}

/// Wrapper for triangle to implement BVH traits
struct BvhTriangle {
    vertices: [Point3<f32>; 3],
    node_index: usize,
}

impl BvhTriangle {
    fn new(v0: Point3<f32>, v1: Point3<f32>, v2: Point3<f32>) -> Self {
        Self {
            vertices: [v0, v1, v2],
            node_index: 0,
        }
    }
}

impl Bounded<f32, 3> for BvhTriangle {
    fn aabb(&self) -> Aabb<f32, 3> {
        let min = Point3::new(
            self.vertices[0]
                .x
                .min(self.vertices[1].x)
                .min(self.vertices[2].x),
            self.vertices[0]
                .y
                .min(self.vertices[1].y)
                .min(self.vertices[2].y),
            self.vertices[0]
                .z
                .min(self.vertices[1].z)
                .min(self.vertices[2].z),
        );
        let max = Point3::new(
            self.vertices[0]
                .x
                .max(self.vertices[1].x)
                .max(self.vertices[2].x),
            self.vertices[0]
                .y
                .max(self.vertices[1].y)
                .max(self.vertices[2].y),
            self.vertices[0]
                .z
                .max(self.vertices[1].z)
                .max(self.vertices[2].z),
        );
        Aabb::with_bounds(min, max)
    }
}

impl BHShape<f32, 3> for BvhTriangle {
    fn set_bh_node_index(&mut self, index: usize) {
        self.node_index = index;
    }

    fn bh_node_index(&self) -> usize {
        self.node_index
    }
}

/// Build BVH for mesh
fn build_bvh(mesh: &Mesh) -> Result<Bvh<f32, 3>, BooleanError> {
    let mut bvh_triangles: Vec<BvhTriangle> = Vec::new();

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);
        bvh_triangles.push(BvhTriangle::new(*v[0], *v[1], *v[2]));
    }

    if bvh_triangles.is_empty() {
        return Err(BooleanError::NoIntersection);
    }

    Ok(Bvh::build(&mut bvh_triangles))
}

/// Check if a point is inside a mesh using ray casting
fn is_inside_mesh(point: Point3<f32>, bvh: &Bvh<f32, 3>, mesh: &Mesh) -> bool {
    let ray = Ray::new(point, Vector3::new(0.0, 0.0, 1.0));

    let triangles: Vec<BvhTriangle> = mesh
        .triangles
        .iter()
        .map(|tri| {
            let v = tri.get_vertices(&mesh.vertices);
            BvhTriangle::new(*v[0], *v[1], *v[2])
        })
        .collect();

    let mut intersections = 0;
    let hit_aabbs = bvh.traverse(&ray, &triangles);

    for _aabb_hit in hit_aabbs {
        for triangle in &triangles {
            let v0 = triangle.vertices[0];
            let v1 = triangle.vertices[1];
            let v2 = triangle.vertices[2];

            if ray_triangle_intersect(point, v0, v1, v2) {
                intersections += 1;
            }
        }
    }

    intersections % 2 == 1
}

/// Ray-triangle intersection (Möller–Trumbore)
fn ray_triangle_intersect(
    p: Point3<f32>,
    v0: Point3<f32>,
    v1: Point3<f32>,
    v2: Point3<f32>,
) -> bool {
    let eps = 0.0000001;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = Vector3::new(0.0, 0.0, 1.0).cross(&edge2);
    let a = edge1.dot(&h);

    if a.abs() < eps {
        return false;
    }

    let f = 1.0 / a;
    let s = p - v0;
    let u = f * s.dot(&h);

    if u < 0.0 || u > 1.0 {
        return false;
    }

    let q = s.cross(&edge1);
    let v = f * Vector3::new(0.0, 0.0, 1.0).dot(&q);

    if v < 0.0 || u + v > 1.0 {
        return false;
    }

    true
}
