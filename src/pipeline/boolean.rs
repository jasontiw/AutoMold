//! Boolean operations - mesh subtraction (block - model)

use crate::geometry::mesh::{Mesh, Triangle};
use bvh::aabb::AABB;
use bvh::bvh::BVH;
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
            result_vertices.push(v[0]);
            result_vertices.push(v[1]);
            result_vertices.push(v[2]);
            result_indices.push([idx, idx + 1, idx + 2]);
        }
    }

    // Rebuild normals
    let normals = Mesh::calculate_normals(&result_vertices, &result_indices);

    Ok(Mesh {
        vertices: result_vertices,
        triangles: result_indices
            .iter()
            .map(|i| Triangle::new(i[0], i[1], i[2]))
            .collect(),
        normals,
    })
}

/// Build BVH for mesh
fn build_bvh(mesh: &Mesh) -> Result<BVH, BooleanError> {
    // Convert mesh to BVH triangles
    let mut bvh_triangles: Vec<bvh::primitive::Triangle> = Vec::new();

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);
        bvh_triangles.push(bvh::primitive::Triangle::new(
            [v[0].x, v[0].y, v[0].z],
            [v[1].x, v[1].y, v[1].z],
            [v[2].x, v[2].y, v[2].z],
        ));
    }

    if bvh_triangles.is_empty() {
        return Err(BooleanError::NoIntersection);
    }

    // Build BVH
    bvh::bvh::BVH::build(&mut bvh_triangles).map_err(|_| BooleanError::BVHError)
}

/// Check if a point is inside a mesh using ray casting
fn is_inside_mesh(point: Point3<f32>, bvh: &BVH, mesh: &Mesh) -> bool {
    // Cast a ray in positive Z direction
    let ray = Ray::new(
        bvh::point::Point3::new(point.x, point.y, point.z),
        bvh::vec3::Vec3::new(0.0, 0.0, 1.0),
    );

    // Find all intersections
    let mut intersections = 0;
    bvh.traverse(&ray, |triangle| {
        let v = triangle.vertex0;
        let v0 = Point3::new(v.0[0], v.0[1], v.0[2]);
        let v1 = Point3::new(
            triangle.vertex1[0],
            triangle.vertex1[1],
            triangle.vertex1[2],
        );
        let v2 = Point3::new(
            triangle.vertex2[0],
            triangle.vertex2[1],
            triangle.vertex2[2],
        );

        // Ray-triangle intersection
        if ray_triangle_intersect(point, v0, v1, v2) {
            intersections += 1;
        }

        true // Continue traversal
    });

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

    let normals = Mesh::calculate_normals(&result_vertices, &result_indices);

    Ok(Mesh {
        vertices: result_vertices,
        triangles: result_indices
            .iter()
            .map(|i| Triangle::new(i[0], i[1], i[2]))
            .collect(),
        normals,
    })
}
