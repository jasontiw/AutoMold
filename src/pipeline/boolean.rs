//! Boolean operations - mesh subtraction (block - model)

use crate::geometry::mesh::{Mesh, Triangle};
use bvh::aabb::{Aabb, Bounded};
use bvh::bounding_hierarchy::{BHShape, BoundingHierarchy};
use bvh::bvh::Bvh;
use bvh::ray::Ray;
use nalgebra::{Point3, Vector3};
use std::collections::HashSet;
use thiserror::Error;

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
/// Returns the cavity mesh (block with model subtracted)
pub fn boolean_subtract(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    // For this simplified implementation, we'll use a different approach:
    // Instead of full CSG boolean, we'll create the cavity by:
    // 1. Taking the block mesh
    // 2. Keeping only faces that don't intersect the model

    // Build BVH for the model
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
    // Convert mesh to BVH triangles
    let mut bvh_triangles: Vec<BvhTriangle> = Vec::new();

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);
        bvh_triangles.push(BvhTriangle::new(*v[0], *v[1], *v[2]));
    }

    if bvh_triangles.is_empty() {
        return Err(BooleanError::NoIntersection);
    }

    // Build BVH
    Ok(Bvh::build(&mut bvh_triangles))
}

/// Check if a point is inside a mesh using ray casting
fn is_inside_mesh(point: Point3<f32>, bvh: &Bvh<f32, 3>, mesh: &Mesh) -> bool {
    // Cast a ray in positive Z direction
    let ray = Ray::new(point, Vector3::new(0.0, 0.0, 1.0));

    // We need to rebuild triangles for traversal
    let triangles: Vec<BvhTriangle> = mesh
        .triangles
        .iter()
        .map(|tri| {
            let v = tri.get_vertices(&mesh.vertices);
            BvhTriangle::new(*v[0], *v[1], *v[2])
        })
        .collect();

    // Find all intersections
    let mut intersections = 0;
    let hit_aabbs = bvh.traverse(&ray, &triangles);

    for _aabb_hit in hit_aabbs {
        // For each AABB hit, check the actual triangle intersection
        for triangle in &triangles {
            let v0 = triangle.vertices[0];
            let v1 = triangle.vertices[1];
            let v2 = triangle.vertices[2];

            // Ray-triangle intersection
            if ray_triangle_intersect(point, v0, v1, v2) {
                intersections += 1;
            }
        }
    }

    // Odd number of intersections = inside
    intersections % 2 == 1
}

/// Ray-triangle intersection (simplified)
fn ray_triangle_intersect(
    p: Point3<f32>,
    v0: Point3<f32>,
    v1: Point3<f32>,
    v2: Point3<f32>,
) -> bool {
    // Möller–Trumbore algorithm (simplified)
    let eps = 0.0000001;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = Vector3::new(0.0, 0.0, 1.0).cross(&edge2);
    let a = edge1.dot(&h);

    if a.abs() < eps {
        return false; // Parallel
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

    // t = f * edge2.dot(&q)
    true
}

/// Alternative: simple boolean using spatial approach
pub fn boolean_subtract_simple(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    // Get model bounding box
    let model_bbox = model.calculate_bounding_box();

    let mut result_vertices: Vec<Point3<f32>> = Vec::new();
    let mut result_indices: Vec<[usize; 3]> = Vec::new();

    for tri in &block.triangles {
        let v = tri.get_vertices(&block.vertices);

        // Check if triangle is outside model bounding box + margin
        let margin = 0.1;

        let all_outside = v.iter().all(|p| {
            p.x < model_bbox.min.x - margin
                || p.x > model_bbox.max.x + margin
                || p.y < model_bbox.min.y - margin
                || p.y > model_bbox.max.y + margin
                || p.z < model_bbox.min.z - margin
                || p.z > model_bbox.max.z + margin
        });

        if all_outside {
            // Keep this triangle
            let idx = result_vertices.len();
            result_vertices.push(*v[0]);
            result_vertices.push(*v[1]);
            result_vertices.push(*v[2]);
            result_indices.push([idx, idx + 1, idx + 2]);
        }
    }

    let triangles: Vec<Triangle> = result_indices
        .iter()
        .map(|i| Triangle::new(i[0], i[1], i[2]))
        .collect();
    let normals = Mesh::calculate_normals(&result_vertices, &triangles);

    Ok(Mesh {
        vertices: result_vertices,
        triangles,
        normals,
    })
}
