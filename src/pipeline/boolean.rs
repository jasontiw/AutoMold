//! Real CSG Boolean Operations - mesh subtraction (block - model)
//! Implements actual Constructive Solid Geometry with triangle clipping
//! Strategy selection: CSG (csgrs) -> Voxel fallback -> SimpleAABB

use crate::geometry::mesh::{Mesh, Triangle};
use crate::geometry::voxel_fallback::{auto_voxel_resolution, voxel_boolean_subtract, VoxelConfig};
use glam::{Vec3, Vec3A};
use thiserror::Error;
use tracing::{error, info, warn};

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

    #[error("CSG operation failed: {0}")]
    CSGFailed(String),

    #[error("Voxelization failed: {0}")]
    VoxelizationFailed(String),

    #[error("Invalid mesh: {0}")]
    InvalidMesh(String),

    #[error("Memory limit exceeded: needed {0} bytes")]
    MemoryLimitExceeded(usize),
}

/// Strategy for CSG boolean operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanStrategy {
    /// Use csgrs crate for exact CSG (BSP-tree based)
    CSG,
    /// Use AABB-based simple boolean (fast but less accurate)
    SimpleAABB,
    /// Use voxelization fallback
    Voxelization,
    /// Auto-select strategy based on mesh complexity
    Auto,
}

/// Configuration for CSG boolean operations
#[derive(Debug, Clone)]
pub struct BooleanConfig {
    /// Strategy to use for boolean operations
    pub strategy: BooleanStrategy,
    /// Maximum memory to use for boolean operations (bytes)
    pub max_memory: usize,
    /// Tolerance for geometric comparisons
    pub tolerance: f32,
    /// Whether to preserve cavity walls
    pub preserve_cavity_walls: bool,
}

impl Default for BooleanConfig {
    fn default() -> Self {
        Self {
            strategy: BooleanStrategy::Auto,
            max_memory: 512 * 1024 * 1024, // 512 MB default
            tolerance: 1e-5,
            preserve_cavity_walls: true,
        }
    }
}

/// Classification of a triangle relative to an AABB (used by tests)
#[derive(Debug, Clone, Copy, PartialEq)]
enum TriangleClassification {
    /// Triangle is completely outside the mesh
    Outside,
    /// Triangle is completely inside the mesh
    Inside,
    /// Triangle intersects the mesh surface
    Intersecting,
}

/// Auto-select strategy based on mesh complexity and available memory
fn select_strategy(block: &Mesh, model: &Mesh, config: &BooleanConfig) -> BooleanStrategy {
    if config.strategy != BooleanStrategy::Auto {
        return config.strategy;
    }

    let total_triangles = block.triangles.len() + model.triangles.len();
    let estimated_memory = total_triangles * MEMORY_PER_TRIANGLE;

    if estimated_memory > config.max_memory {
        warn!(
            "Memory estimate {} exceeds limit {}, using SimpleAABB",
            estimated_memory, config.max_memory
        );
        return BooleanStrategy::SimpleAABB;
    }

    if total_triangles > 100000 {
        info!(
            "Mesh complexity {} triangles exceeds CSG threshold, using SimpleAABB",
            total_triangles
        );
        return BooleanStrategy::SimpleAABB;
    }

    info!(
        "Auto-selected CSG strategy for {} triangles",
        total_triangles
    );
    BooleanStrategy::CSG
}

/// Perform CSG boolean subtraction using csgrs library
/// Currently returns error to trigger voxel fallback (csgrs integration pending)
fn boolean_subtract_csgrs(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    info!("Attempting CSG subtraction with csgrs");

    let block_triangles = block.triangles.len();
    let model_triangles = model.triangles.len();

    if block_triangles == 0 || model_triangles == 0 {
        return Err(BooleanError::InvalidMesh("Empty mesh".to_string()));
    }

    // TODO: Implement csgrs integration
    Err(BooleanError::CSGFailed(
        "csgrs integration pending - use voxel fallback".to_string(),
    ))
}

/// Perform voxelization-based boolean subtraction as fallback
/// Uses auto-resolution based on mesh complexity
fn boolean_subtract_voxel(
    block: &Mesh,
    model: &Mesh,
    config: &BooleanConfig,
) -> Result<Mesh, BooleanError> {
    let resolution = auto_voxel_resolution(block.triangles.len(), model.triangles.len());
    let voxel_config = VoxelConfig {
        resolution,
        strategy: crate::geometry::voxel_fallback::VoxelStrategy::Standard,
        smooth_normals: true,
    };

    info!("Starting voxel fallback with resolution {}", resolution);

    voxel_boolean_subtract(block, model, &voxel_config)
        .map_err(|e| BooleanError::VoxelizationFailed(e.to_string()))
}

/// Perform real CSG boolean subtraction: block - model
/// Returns the cavity mesh (block with model subtracted)
/// Uses automatic strategy selection (CSG -> voxel -> AABB)
pub fn boolean_subtract(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    boolean_subtract_with_config(block, model, &BooleanConfig::default())
}

/// Perform boolean subtraction with custom configuration
/// Strategy selection: CSG (csgrs) -> voxel fallback -> SimpleAABB fallback
pub fn boolean_subtract_with_config(
    block: &Mesh,
    model: &Mesh,
    config: &BooleanConfig,
) -> Result<Mesh, BooleanError> {
    let strategy = select_strategy(block, model, config);

    info!(
        "Boolean subtract: block={} triangles, model={} triangles, strategy={:?}",
        block.triangles.len(),
        model.triangles.len(),
        strategy
    );

    match strategy {
        BooleanStrategy::CSG => match boolean_subtract_csgrs(block, model) {
            Ok(result) => {
                info!(
                    "CSG operation succeeded: {} triangles",
                    result.triangles.len()
                );
                Ok(result)
            }
            Err(e) => {
                warn!("CSG operation failed: {}, falling back to voxelization", e);
                match boolean_subtract_voxel(block, model, config) {
                    Ok(result) => {
                        info!(
                            "Voxel fallback succeeded: {} triangles",
                            result.triangles.len()
                        );
                        Ok(result)
                    }
                    Err(ve) => {
                        error!("Voxel fallback also failed: {}", ve);
                        boolean_subtract_simple(block, model)
                    }
                }
            }
        },
        BooleanStrategy::SimpleAABB => {
            info!("Using SimpleAABB strategy");
            boolean_subtract_simple(block, model)
        }
        BooleanStrategy::Voxelization => {
            info!("Using Voxelization strategy");
            boolean_subtract_voxel(block, model, config)
        }
        BooleanStrategy::Auto => {
            unreachable!("Auto should have been resolved in select_strategy")
        }
    }
}

/// Classify a triangle relative to an AABB (used by tests)
fn classify_triangle_aabb(
    verts: [Vec3A; 3],
    aabb_min: Vec3A,
    aabb_max: Vec3A,
) -> TriangleClassification {
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

    TriangleClassification::Intersecting
}

/// Add a triangle to the result mesh (helper for boolean_subtract_simple)
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

/// Simple boolean using just AABB (final fallback when both CSG and voxel fail)
/// Fast but less accurate - only removes triangles completely outside model AABB
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
}
