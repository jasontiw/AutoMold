//! Real CSG Boolean Operations - mesh subtraction (block - model)
//! Implements actual Constructive Solid Geometry with triangle clipping
//! Strategy selection: CSG (csgrs) -> Voxel fallback -> SimpleAABB

use crate::geometry::mesh::{Mesh, Triangle};
use crate::geometry::voxel_fallback::{auto_voxel_resolution, voxel_boolean_subtract, VoxelConfig};
use crate::pipeline::boolean::BooleanError::CSGFailed;
use glam::{Vec3, Vec3A};
use thiserror::Error;
use tracing::{error, info, warn};

/// Memory estimation constants (from PRD)
/// Estimated memory per triangle in bytes
pub const MEMORY_PER_TRIANGLE: usize = 470;

#[derive(Error, Debug)]
pub enum BooleanError {
    #[error("BVH construction failed: {0}")]
    BVHError(String),

    #[error("No intersection found between block ({block_triangles} triangles) and model ({model_triangles} triangles)")]
    NoIntersection {
        block_triangles: usize,
        model_triangles: usize,
    },

    #[error("Too many intersections detected ({count}), mesh may be too complex")]
    TooManyIntersections { count: usize },

    #[error("Clipping failed during CSG operation")]
    ClippingError,

    #[error("CSG operation failed: {0}")]
    CSGFailed(String),

    #[error("Voxelization failed: {0}")]
    VoxelizationFailed(String),

    #[error("Invalid mesh: {0}")]
    InvalidMesh(String),

    #[error("Memory limit exceeded: needed {needed} bytes, limit {limit} bytes")]
    MemoryLimitExceeded { needed: usize, limit: usize },

    #[error("All fallback strategies failed. CSG: {csgrs_error}, Voxel: {voxel_error}, AABB: {aabb_error}")]
    AllStrategiesFailed {
        csgrs_error: String,
        voxel_error: String,
        aabb_error: String,
    },
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
fn boolean_subtract_csgrs(block: &Mesh, model: &Mesh) -> Result<Mesh, BooleanError> {
    use crate::geometry::mesh::{csgrs_mesh_to_mesh, mesh_to_csgrs_mesh};
    use csgrs::traits::CSG;

    info!("Attempting CSG subtraction with csgrs");

    let block_triangles = block.triangles.len();
    let model_triangles = model.triangles.len();

    if block_triangles == 0 || model_triangles == 0 {
        return Err(BooleanError::InvalidMesh("Empty mesh".to_string()));
    }

    info!("Converting block ({} triangles) to csgrs", block_triangles);
    let block_csg = mesh_to_csgrs_mesh(block)
        .map_err(|e| CSGFailed(format!("Failed to convert block: {}", e)))?;

    info!("Converting model ({} triangles) to csgrs", model_triangles);
    let model_csg = mesh_to_csgrs_mesh(model)
        .map_err(|e| CSGFailed(format!("Failed to convert model: {}", e)))?;

    info!("Performing CSG difference operation");
    let result_csg = block_csg.difference(&model_csg);

    info!("Converting result back to Mesh");
    let result_mesh = csgrs_mesh_to_mesh(result_csg)
        .map_err(|e| CSGFailed(format!("Failed to convert result: {}", e)))?;

    let result_triangles = result_mesh.triangles.len();
    info!(
        "CSG subtraction complete: {} output triangles",
        result_triangles
    );

    if result_triangles == 0 {
        return Err(BooleanError::InvalidMesh("CSG result is empty".to_string()));
    }

    Ok(result_mesh)
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
        BooleanStrategy::CSG => {
            let csgrs_result = boolean_subtract_csgrs(block, model);

            match csgrs_result {
                Ok(result) => {
                    info!(
                        "CSG operation succeeded: {} triangles (primary strategy)",
                        result.triangles.len()
                    );
                    Ok(result)
                }
                Err(csgrs_err) => {
                    warn!(
                        "CSG operation failed: {}, attempting voxel fallback",
                        csgrs_err
                    );

                    let voxel_result = boolean_subtract_voxel(block, model, config);

                    match voxel_result {
                        Ok(result) => {
                            info!(
                                "Voxel fallback succeeded: {} triangles",
                                result.triangles.len()
                            );
                            Ok(result)
                        }
                        Err(voxel_err) => {
                            error!(
                                "Voxel fallback also failed: {}, attempting SimpleAABB",
                                voxel_err
                            );

                            let aabb_result = boolean_subtract_simple(block, model);

                            match aabb_result {
                                Ok(result) => {
                                    warn!(
                                        "SimpleAABB fallback succeeded: {} triangles (WARNING: may have accuracy issues)",
                                        result.triangles.len()
                                    );
                                    Ok(result)
                                }
                                Err(aabb_err) => {
                                    error!("All fallback strategies exhausted");
                                    Err(BooleanError::AllStrategiesFailed {
                                        csgrs_error: csgrs_err.to_string(),
                                        voxel_error: voxel_err.to_string(),
                                        aabb_error: aabb_err.to_string(),
                                    })
                                }
                            }
                        }
                    }
                }
            }
        }
        BooleanStrategy::SimpleAABB => {
            info!("Using SimpleAABB strategy (memory/complexity limits exceeded)");
            boolean_subtract_simple(block, model)
        }
        BooleanStrategy::Voxelization => {
            info!("Using Voxelization strategy (user requested)");
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
