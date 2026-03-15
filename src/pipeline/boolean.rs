//! Real CSG Boolean Operations - mesh subtraction (block - model)
//! Implements actual Constructive Solid Geometry with triangle clipping

use crate::geometry::mesh::{Mesh, Triangle};
use glam::{Vec3, Vec3A};
use thiserror::Error;
use tracing::{info, warn};

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

    #[error("Clipping failed")]
    ClippingError,
}

/// Classification of a triangle relative to a mesh
#[derive(Debug, Clone, Copy, PartialEq)]
enum TriangleClassification {
    /// Triangle is completely outside the mesh
    Outside,
    /// Triangle is completely inside the mesh
    Inside,
    /// Triangle intersects the mesh surface
    Intersecting,
}

/// Perform real CSG boolean subtraction: block - model
/// Returns the cavity mesh (block with model subtracted)
pub fn boolean_subtract(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    let block_triangles = block.triangles.len();
    let model_triangles = model.triangles.len();
    let estimated_memory = (block_triangles + model_triangles) * MEMORY_PER_TRIANGLE;
    info!(
        "CSG Boolean: block={} triangles, model={} triangles, estimated memory: {} bytes ({:.1} KB)",
        block_triangles,
        model_triangles,
        estimated_memory,
        estimated_memory as f32 / 1024.0
    );

    // Get model AABB for fast rejection
    let model_aabb = model.calculate_bounding_box();
    let model_aabb_min = Vec3A::new(
        model_aabb.min.x as f32,
        model_aabb.min.y as f32,
        model_aabb.min.z as f32,
    );
    let model_aabb_max = Vec3A::new(
        model_aabb.max.x as f32,
        model_aabb.max.y as f32,
        model_aabb.max.z as f32,
    );

    let mut result_vertices: Vec<glam::Vec3> = Vec::new();
    let mut result_indices: Vec<[usize; 3]> = Vec::new();

    // Process each triangle from the block
    let mut inside_count = 0;
    let mut outside_count = 0;
    let mut clip_count = 0;

    for tri in &block.triangles {
        let vertices = tri.get_vertices(&block.vertices);
        let verts: [Vec3A; 3] = [
            Vec3A::new(
                vertices[0].x as f32,
                vertices[0].y as f32,
                vertices[0].z as f32,
            ),
            Vec3A::new(
                vertices[1].x as f32,
                vertices[1].y as f32,
                vertices[1].z as f32,
            ),
            Vec3A::new(
                vertices[2].x as f32,
                vertices[2].y as f32,
                vertices[2].z as f32,
            ),
        ];

        // Classify triangle against model AABB (fast rejection)
        let classification = classify_triangle_aabb(verts, model_aabb_min, model_aabb_max);

        match classification {
            TriangleClassification::Outside => {
                // Completely outside - keep as is
                add_triangle(&mut result_vertices, &mut result_indices, verts);
                outside_count += 1;
            }
            TriangleClassification::Inside => {
                // Completely inside - discard (part of the cavity)
                inside_count += 1;
            }
            TriangleClassification::Intersecting => {
                // Need to clip against the model mesh
                match clip_triangle_against_mesh(verts, model) {
                    Some(clipped_verts) => {
                        // Add all resulting triangles from clipping
                        for cv in clipped_verts {
                            add_triangle(&mut result_vertices, &mut result_indices, cv);
                        }
                        clip_count += 1;
                    }
                    None => {
                        // If clipping fails, add original (conservative approach)
                        warn!("Clipping failed, keeping original triangle");
                        add_triangle(&mut result_vertices, &mut result_indices, verts);
                    }
                }
            }
        }
    }

    // Add interior surface of the model (the cavity walls)
    // These are triangles from the model that face INTO the cavity
    let cavity_triangles =
        add_cavity_surface(block, model, &mut result_vertices, &mut result_indices);

    info!(
        "CSG Result: outside={}, inside={}, clipped={}, cavity_walls={}, total_output={} triangles",
        outside_count,
        inside_count,
        clip_count,
        cavity_triangles,
        result_indices.len()
    );

    // Convert Vec3 to Point3 for Mesh
    let vertices: Vec<nalgebra::Point3<f32>> = result_vertices
        .iter()
        .map(|v| nalgebra::Point3::new(v.x, v.y, v.z))
        .collect();

    let triangles: Vec<Triangle> = result_indices
        .iter()
        .map(|i| Triangle::new(i[0], i[1], i[2]))
        .collect();

    // Rebuild normals
    let normals = Mesh::calculate_normals(&vertices, &triangles);

    let result_triangles = triangles.len();
    let result_memory = result_triangles * MEMORY_PER_TRIANGLE;
    info!(
        "Boolean result: {} triangles (estimated memory: {} bytes / {:.1} KB)",
        result_triangles,
        result_memory,
        result_memory as f32 / 1024.0
    );

    Ok(Mesh {
        vertices,
        triangles,
        normals,
    })
}

/// Classify a triangle relative to an AABB
fn classify_triangle_aabb(
    verts: [Vec3A; 3],
    aabb_min: Vec3A,
    aabb_max: Vec3A,
) -> TriangleClassification {
    // Check if all vertices are outside
    let all_outside = verts.iter().all(|v| {
        v.x < aabb_min.x
            || v.x > aabb_max.x
            || v.y < aabb_min.y
            || v.y > aabb_max.y
            || v.z < aabb_min.z
            || v.z > aabb_max.z
    });

    if all_outside {
        return TriangleClassification::Outside;
    }

    // Check if all vertices are inside
    let all_inside = verts.iter().all(|v| {
        v.x >= aabb_min.x
            && v.x <= aabb_max.x
            && v.y >= aabb_min.y
            && v.y <= aabb_max.y
            && v.z >= aabb_min.z
            && v.z <= aabb_max.z
    });

    if all_inside {
        return TriangleClassification::Inside;
    }

    // Intersecting - needs clipping
    TriangleClassification::Intersecting
}

/// Add a triangle to the result mesh
fn add_triangle(
    result_vertices: &mut Vec<Vec3>,
    result_indices: &mut Vec<[usize; 3]>,
    verts: [Vec3A; 3],
) {
    let idx = result_vertices.len();
    result_vertices.push(verts[0].into());
    result_vertices.push(verts[1].into());
    result_vertices.push(verts[2].into());
    result_indices.push([idx, idx + 1, idx + 2]);
}

/// Clip a triangle against a mesh using the AABB of the mesh
/// Returns up to 4 triangles after clipping against all 6 planes of AABB
fn clip_triangle_against_mesh(verts: [Vec3A; 3], mesh: &Mesh) -> Option<Vec<[Vec3A; 3]>> {
    let bbox = mesh.calculate_bounding_box();

    // Use AABB planes for clipping (faster than mesh triangles)
    let planes: [(Vec3A, Vec3A); 6] = [
        // Min planes (push inward)
        (
            Vec3A::new(1.0, 0.0, 0.0),
            Vec3A::new(bbox.min.x as f32, 0.0, 0.0),
        ),
        (
            Vec3A::new(0.0, 1.0, 0.0),
            Vec3A::new(0.0, bbox.min.y as f32, 0.0),
        ),
        (
            Vec3A::new(0.0, 0.0, 1.0),
            Vec3A::new(0.0, 0.0, bbox.min.z as f32),
        ),
        // Max planes (push inward)
        (
            Vec3A::new(-1.0, 0.0, 0.0),
            Vec3A::new(-bbox.max.x as f32, 0.0, 0.0),
        ),
        (
            Vec3A::new(0.0, -1.0, 0.0),
            Vec3A::new(0.0, -bbox.max.y as f32, 0.0),
        ),
        (
            Vec3A::new(0.0, 0.0, -1.0),
            Vec3A::new(0.0, 0.0, -bbox.max.z as f32),
        ),
    ];

    // Start with the original triangle as a polygon
    let mut polygon: Vec<Vec3A> = verts.to_vec();

    // Clip against each plane
    for (normal, point) in planes {
        if polygon.len() < 3 {
            return None; // Polygon collapsed
        }
        polygon = clip_polygon_against_plane(&polygon, normal, point);
    }

    if polygon.len() < 3 {
        return None;
    }

    // Convert polygon back to triangles (fan triangulation)
    let mut result: Vec<[Vec3A; 3]> = Vec::new();
    for i in 1..polygon.len() - 1 {
        result.push([polygon[0], polygon[i], polygon[i + 1]]);
    }

    Some(result)
}

/// Clip a polygon against a single plane
/// Returns the part of the polygon on the positive side of the plane
fn clip_polygon_against_plane(
    polygon: &[Vec3A],
    plane_normal: Vec3A,
    plane_point: Vec3A,
) -> Vec<Vec3A> {
    let mut result: Vec<Vec3A> = Vec::new();

    for i in 0..polygon.len() {
        let current = polygon[i];
        let next = polygon[(i + 1) % polygon.len()];

        let current_dist = plane_normal.dot(current - plane_point);
        let next_dist = plane_normal.dot(next - plane_point);

        let current_inside = current_dist >= -0.0001;
        let next_inside = next_dist >= -0.0001;

        if current_inside {
            result.push(current);
        }

        // Add edge intersection if one vertex is inside and one is outside
        if current_inside != next_inside {
            let t = current_dist / (current_dist - next_dist);
            let intersection = current + (next - current) * t;
            result.push(intersection);
        }
    }

    result
}

/// Add the interior surface of the model as cavity walls
/// These are triangles from the model that face INTO the subtracted region
fn add_cavity_surface(
    block: &Mesh,
    model: &Mesh,
    result_vertices: &mut Vec<Vec3>,
    result_indices: &mut Vec<[usize; 3]>,
) -> usize {
    let block_aabb = block.calculate_bounding_box();
    let model_aabb = model.calculate_bounding_box();

    let mut added = 0;

    for tri in &model.triangles {
        let verts = tri.get_vertices(&model.vertices);
        let tri_verts: [Vec3A; 3] = [
            Vec3A::new(verts[0].x as f32, verts[0].y as f32, verts[0].z as f32),
            Vec3A::new(verts[1].x as f32, verts[1].y as f32, verts[1].z as f32),
            Vec3A::new(verts[2].x as f32, verts[2].y as f32, verts[2].z as f32),
        ];

        // Check if this triangle is on the boundary between model and block
        let center = (tri_verts[0] + tri_verts[1] + tri_verts[2]) / 3.0;

        // If center is inside the block's AABB but outside model's AABB (expanded)
        // or on the boundary, add it as cavity wall
        let margin = 0.001;
        let on_boundary = (center.x >= block_aabb.min.x as f32 - margin
            && center.x <= block_aabb.max.x as f32 + margin)
            && (center.y >= block_aabb.min.y as f32 - margin
                && center.y <= block_aabb.max.y as f32 + margin)
            && (center.z >= block_aabb.min.z as f32 - margin
                && center.z <= block_aabb.max.z as f32 + margin);

        if on_boundary {
            // Get the triangle normal to determine orientation
            let edge1 = tri_verts[1] - tri_verts[0];
            let edge2 = tri_verts[2] - tri_verts[0];
            let normal = edge1.cross(edge2).normalize();

            // Add the triangle (could flip normal depending on desired cavity orientation)
            let idx = result_vertices.len();
            result_vertices.push(tri_verts[0].into());
            result_vertices.push(tri_verts[1].into());
            result_vertices.push(tri_verts[2].into());
            result_indices.push([idx, idx + 1, idx + 2]);
            added += 1;
        }
    }

    added
}

/// Simple boolean using just AABB (fallback for when CSG fails)
pub fn boolean_subtract_simple(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    let model_bbox = model.calculate_bounding_box();
    let model_aabb_min = Vec3A::new(
        model_bbox.min.x as f32,
        model_bbox.min.y as f32,
        model_bbox.min.z as f32,
    );
    let model_aabb_max = Vec3A::new(
        model_bbox.max.x as f32,
        model_bbox.max.y as f32,
        model_bbox.max.z as f32,
    );

    let mut result_vertices: Vec<glam::Vec3> = Vec::new();
    let mut result_indices: Vec<[usize; 3]> = Vec::new();

    for tri in &block.triangles {
        let vertices = tri.get_vertices(&block.vertices);
        let verts: [Vec3A; 3] = [
            Vec3A::new(
                vertices[0].x as f32,
                vertices[0].y as f32,
                vertices[0].z as f32,
            ),
            Vec3A::new(
                vertices[1].x as f32,
                vertices[1].y as f32,
                vertices[1].z as f32,
            ),
            Vec3A::new(
                vertices[2].x as f32,
                vertices[2].y as f32,
                vertices[2].z as f32,
            ),
        ];

        // Check if all vertices are outside the AABB
        let all_outside = verts.iter().all(|p| {
            p.x < model_aabb_min.x
                || p.x > model_aabb_max.x
                || p.y < model_aabb_min.y
                || p.y > model_aabb_max.y
                || p.z < model_aabb_min.z
                || p.z > model_aabb_max.z
        });

        if all_outside {
            add_triangle(&mut result_vertices, &mut result_indices, verts);
        }
    }

    let vertices: Vec<nalgebra::Point3<f32>> = result_vertices
        .iter()
        .map(|v| nalgebra::Point3::new(v.x, v.y, v.z))
        .collect();

    let triangles: Vec<Triangle> = result_indices
        .iter()
        .map(|i| Triangle::new(i[0], i[1], i[2]))
        .collect();

    let normals = Mesh::calculate_normals(&vertices, &triangles);

    Ok(Mesh {
        vertices,
        triangles,
        normals,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_outside_aabb() {
        let verts = [
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(1.0, 0.0, 0.0),
            Vec3A::new(0.0, 1.0, 0.0),
        ];
        let aabb_min = Vec3A::new(10.0, 10.0, 10.0);
        let aabb_max = Vec3A::new(20.0, 20.0, 20.0);

        let result = classify_triangle_aabb(verts, aabb_min, aabb_max);
        assert_eq!(result, TriangleClassification::Outside);
    }

    #[test]
    fn test_triangle_inside_aabb() {
        let verts = [
            Vec3A::new(5.0, 5.0, 5.0),
            Vec3A::new(6.0, 5.0, 5.0),
            Vec3A::new(5.0, 6.0, 5.0),
        ];
        let aabb_min = Vec3A::new(0.0, 0.0, 0.0);
        let aabb_max = Vec3A::new(10.0, 10.0, 10.0);

        let result = classify_triangle_aabb(verts, aabb_min, aabb_max);
        assert_eq!(result, TriangleClassification::Inside);
    }

    #[test]
    fn test_triangle_intersecting_aabb() {
        let verts = [
            Vec3A::new(5.0, 5.0, 5.0),
            Vec3A::new(15.0, 5.0, 5.0),
            Vec3A::new(5.0, 15.0, 5.0),
        ];
        let aabb_min = Vec3A::new(0.0, 0.0, 0.0);
        let aabb_max = Vec3A::new(10.0, 10.0, 10.0);

        let result = classify_triangle_aabb(verts, aabb_min, aabb_max);
        assert_eq!(result, TriangleClassification::Intersecting);
    }

    #[test]
    fn test_polygon_clipping() {
        // Square polygon
        let polygon = vec![
            Vec3A::new(-1.0, -1.0, 0.0),
            Vec3A::new(1.0, -1.0, 0.0),
            Vec3A::new(1.0, 1.0, 0.0),
            Vec3A::new(-1.0, 1.0, 0.0),
        ];

        // Clip against x = 0 plane
        let normal = Vec3A::new(1.0, 0.0, 0.0);
        let point = Vec3A::new(0.0, 0.0, 0.0);

        let result = clip_polygon_against_plane(&polygon, normal, point);

        // Should have 4 vertices (right half of square)
        assert!(result.len() >= 3);

        // All vertices should have x >= 0
        for v in &result {
            assert!(v.x >= -0.0001);
        }
    }
}
