//! Real CSG Boolean Operations - mesh subtraction (block - model)
//! Implements actual Constructive Solid Geometry with triangle clipping
//! Strategy selection: CSG (csgrs) -> Voxel fallback -> SimpleAABB

use crate::geometry::mesh::{Mesh, Triangle};
use crate::geometry::voxel_fallback::{auto_voxel_resolution, voxel_boolean_subtract, VoxelConfig};
use crate::pipeline::boolean::BooleanError::CSGFailed;
use crate::pipeline::decimate::{calculate_decimation_ratio, decimate_mesh};
use glam::{Vec3, Vec3A};
use std::time::Instant;
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

/// Result of a boolean operation with metadata about what happened
#[derive(Debug, Clone)]
pub struct BooleanResult {
    /// The strategy that was actually used after fallback chain
    pub strategy_used: BooleanStrategy,
    /// Execution time in milliseconds
    pub execution_time_ms: u64,
    /// Number of triangles in the result
    pub triangle_count: usize,
    /// Whether the result was repaired (post-boolean repair was applied)
    pub was_repaired: bool,
    /// Warning messages about the operation
    pub warnings: Vec<String>,
}

impl BooleanResult {
    /// Create a new BooleanResult
    pub fn new(
        strategy_used: BooleanStrategy,
        execution_time_ms: u64,
        triangle_count: usize,
    ) -> Self {
        Self {
            strategy_used,
            execution_time_ms,
            triangle_count,
            was_repaired: false,
            warnings: Vec::new(),
        }
    }

    /// Add a warning message
    pub fn add_warning(&mut self, warning: impl Into<String>) {
        self.warnings.push(warning.into());
    }

    /// Mark that repair was applied
    pub fn set_repaired(&mut self) {
        self.was_repaired = true;
    }
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
    /// Total-triangle complexity threshold above which CSG is skipped and the
    /// voxel fallback is attempted first (SimpleAABB last).
    pub complexity_threshold: usize,
    /// Voxel resolution override for the voxel fallback; `None` auto-selects
    /// via `auto_voxel_resolution`.
    pub voxel_resolution: Option<u32>,
}

impl Default for BooleanConfig {
    fn default() -> Self {
        Self {
            strategy: BooleanStrategy::Auto,
            max_memory: 512 * 1024 * 1024, // 512 MB default
            tolerance: 1e-5,
            preserve_cavity_walls: true,
            complexity_threshold: 100_000,
            voxel_resolution: None,
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
            "Memory estimate {} exceeds limit {}, using SimpleAABB (voxel-first)",
            estimated_memory, config.max_memory
        );
        return BooleanStrategy::SimpleAABB;
    }

    if total_triangles > config.complexity_threshold {
        info!(
            "Mesh complexity {} triangles exceeds complexity threshold {} (from BooleanConfig), using SimpleAABB (voxel-first)",
            total_triangles, config.complexity_threshold
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

/// Cap on model triangle count fed to the voxel SDF. The SDF is O(tris * res^3)
/// with no BVH acceleration, so models above this cap are pre-decimated to keep
/// voxel booleans tractable (see `boolean_subtract_voxel`).
const VOXEL_MODEL_TRI_LIMIT: usize = 100_000;

/// Perform voxelization-based boolean subtraction as fallback
/// Pre-decimates the model to `VOXEL_MODEL_TRI_LIMIT` triangles when needed and
/// honors `config.voxel_resolution` (None -> auto resolution).
fn boolean_subtract_voxel(
    block: &Mesh,
    model: &Mesh,
    config: &BooleanConfig,
) -> Result<Mesh, BooleanError> {
    // SDF cost is O(tris * resolution^3) with no BVH, so pre-decimate a CLONE
    // of the model down to VOXEL_MODEL_TRI_LIMIT triangles before sampling.
    let mut model_for_voxel = model.clone();
    if model_for_voxel.triangles.len() > VOXEL_MODEL_TRI_LIMIT {
        let ratio =
            calculate_decimation_ratio(model_for_voxel.triangles.len(), VOXEL_MODEL_TRI_LIMIT);
        info!(
            "Pre-decimating model for voxel boolean: {} -> {} triangles (ratio {:.3})",
            model_for_voxel.triangles.len(),
            VOXEL_MODEL_TRI_LIMIT,
            ratio
        );
        decimate_mesh(&mut model_for_voxel, ratio);
        // calculate_decimation_ratio clamps at 0.1, so models >10x the cap may
        // still exceed it after one pass; keep decimating until the cap holds.
        let mut guard = 0;
        while model_for_voxel.triangles.len() > VOXEL_MODEL_TRI_LIMIT && guard < 8 {
            let before = model_for_voxel.triangles.len();
            decimate_mesh(
                &mut model_for_voxel,
                VOXEL_MODEL_TRI_LIMIT as f32 / before as f32,
            );
            guard += 1;
            if model_for_voxel.triangles.len() >= before {
                break; // no progress - meshopt cannot reduce further
            }
        }
    }

    let resolution = config.voxel_resolution.unwrap_or_else(|| {
        auto_voxel_resolution(block.triangles.len(), model_for_voxel.triangles.len())
    });
    let voxel_config = VoxelConfig {
        resolution,
        strategy: crate::geometry::voxel_fallback::VoxelStrategy::Standard,
        smooth_normals: true,
    };

    info!("Starting voxel fallback with resolution {}", resolution);

    voxel_boolean_subtract(block, &model_for_voxel, &voxel_config)
        .map_err(|e| BooleanError::VoxelizationFailed(e.to_string()))
}

/// Perform real CSG boolean subtraction: block - model
/// Returns the cavity mesh (block with model subtracted)
/// Uses automatic strategy selection (CSG -> voxel -> AABB)
pub fn boolean_subtract(block: &Mesh, model: &Mesh) -> Result<(Mesh, BooleanResult), BooleanError> {
    boolean_subtract_with_config(block, model, &BooleanConfig::default())
}

/// Validate a carve result against the input block.
///
/// A boolean that returns the unmodified block as the "cavity" (SimpleAABB
/// keeps every block face when the model AABB sits fully inside the block) or
/// an empty mesh is garbage that must fail loudly instead of exiting 0.
/// Topology-equality is exact: the block is 12 tris / 8 verts, so any real
/// carve differs.
///
/// Public so the pipeline can run the same sanity check on the boolean result
/// regardless of which strategy produced it (Design D8 / Spec R4).
pub fn validate_carve_result(mesh: Mesh, block: &Mesh) -> Result<Mesh, BooleanError> {
    if mesh.triangles.is_empty() {
        return Err(BooleanError::InvalidMesh("empty result".to_string()));
    }
    if mesh.triangles.len() == block.triangles.len()
        && mesh.vertices.len() == block.vertices.len()
    {
        return Err(BooleanError::InvalidMesh(
            "unmodified block - no cavity carved".to_string(),
        ));
    }
    Ok(mesh)
}

/// Perform boolean subtraction with custom configuration
/// Strategy selection: CSG (csgrs) -> voxel fallback -> SimpleAABB fallback
/// Returns both the resulting mesh and metadata about the operation
pub fn boolean_subtract_with_config(
    block: &Mesh,
    model: &Mesh,
    config: &BooleanConfig,
) -> Result<(Mesh, BooleanResult), BooleanError> {
    let start_time = Instant::now();
    let strategy = select_strategy(block, model, config);

    info!(
        "Boolean subtract: block={} triangles, model={} triangles, strategy={:?}",
        block.triangles.len(),
        model.triangles.len(),
        strategy
    );

    let result = match strategy {
        BooleanStrategy::CSG => {
            let csgrs_result = boolean_subtract_csgrs(block, model);

            match csgrs_result {
                Ok(result) => {
                    info!(
                        "CSG operation succeeded: {} triangles (primary strategy)",
                        result.triangles.len()
                    );
                    validate_carve_result(result, block)
                        .map(|m| (m, BooleanStrategy::CSG, vec![]))
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
                            validate_carve_result(result, block)
                                .map(|m| (m, BooleanStrategy::Voxelization, vec![]))
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
                                    validate_carve_result(result, block).map(|m| {
                                        (
                                            m,
                                            BooleanStrategy::SimpleAABB,
                                            vec!["Used SimpleAABB fallback - may have accuracy issues"
                                                .to_string()],
                                        )
                                    })
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
            info!(
                "Using SimpleAABB strategy (memory/complexity limits exceeded) - attempting voxel boolean first"
            );
            // Voxel-first: the threshold path (too complex / memory exceeded)
            // used to return SimpleAABB directly, which silently returned the
            // full block as "cavity" (exit 0 with garbage). Try the voxel
            // boolean first and keep SimpleAABB as the last resort.
            let voxel_result = boolean_subtract_voxel(block, model, config);
            match voxel_result {
                Ok(result) => {
                    info!(
                        "Voxel boolean (threshold path) succeeded: {} triangles",
                        result.triangles.len()
                    );
                    validate_carve_result(result, block)
                        .map(|m| (m, BooleanStrategy::Voxelization, vec![]))
                }
                Err(voxel_err) => {
                    warn!(
                        "Voxel boolean (threshold path) failed: {}, falling back to SimpleAABB",
                        voxel_err
                    );
                    let aabb_result = boolean_subtract_simple(block, model);
                    match aabb_result {
                        Ok(result) => validate_carve_result(result, block).map(|m| {
                            (
                                m,
                                BooleanStrategy::SimpleAABB,
                                vec!["Using SimpleAABB - reduced accuracy due to mesh complexity"
                                    .to_string()],
                            )
                        }),
                        Err(aabb_err) => Err(BooleanError::AllStrategiesFailed {
                            csgrs_error: "skipped (threshold path)".to_string(),
                            voxel_error: voxel_err.to_string(),
                            aabb_error: aabb_err.to_string(),
                        }),
                    }
                }
            }
        }
        BooleanStrategy::Voxelization => {
            info!("Using Voxelization strategy (user requested)");
            boolean_subtract_voxel(block, model, config)
                .and_then(|m| validate_carve_result(m, block))
                .map(|m| (m, BooleanStrategy::Voxelization, vec![]))
        }
        BooleanStrategy::Auto => {
            unreachable!("Auto should have been resolved in select_strategy")
        }
    }?;

    let execution_time_ms = start_time.elapsed().as_millis() as u64;
    let mut bool_result = BooleanResult::new(result.1, execution_time_ms, result.0.triangles.len());
    bool_result.warnings = result.2;

    Ok((result.0, bool_result))
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
    use std::time::Instant;

    // ============================================================================
    // Shared synthetic mesh builders (kept local so test line count stays low)
    // ============================================================================

    /// A closed box centered at the origin: 8 vertices / 12 triangles. This is
    /// exactly the topology `boolean_subtract_simple` returns when the model
    /// AABB sits fully inside the block (the Bug 3 silent-garbage output).
    fn block_mesh(half_size: f32) -> Mesh {
        let v = |x: f32, y: f32, z: f32| nalgebra::Point3::new(x, y, z);
        let vertices = vec![
            v(-half_size, -half_size, -half_size),
            v(half_size, -half_size, -half_size),
            v(half_size, half_size, -half_size),
            v(-half_size, half_size, -half_size),
            v(-half_size, -half_size, half_size),
            v(half_size, -half_size, half_size),
            v(half_size, half_size, half_size),
            v(-half_size, half_size, half_size),
        ];
        let triangles = vec![
            Triangle::new(0, 1, 2),
            Triangle::new(0, 2, 3),
            Triangle::new(4, 6, 5),
            Triangle::new(4, 7, 6),
            Triangle::new(3, 2, 6),
            Triangle::new(3, 6, 7),
            Triangle::new(0, 5, 1),
            Triangle::new(0, 4, 5),
            Triangle::new(1, 5, 6),
            Triangle::new(1, 6, 2),
            Triangle::new(4, 0, 3),
            Triangle::new(4, 3, 7),
        ];
        let normals = Mesh::calculate_normals(&vertices, &triangles);
        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// UV sphere centered at the origin with `lat_segments` latitude rings and
    /// `lon_segments` longitude segments -> lat*lon*2 triangles.
    fn uv_sphere(radius: f32, lat_segments: usize, lon_segments: usize) -> Mesh {
        let mut vertices: Vec<nalgebra::Point3<f32>> = Vec::new();
        for i in 0..=lat_segments {
            let phi = std::f32::consts::PI * i as f32 / lat_segments as f32;
            for j in 0..lon_segments {
                let theta = 2.0 * std::f32::consts::PI * j as f32 / lon_segments as f32;
                vertices.push(nalgebra::Point3::new(
                    radius * phi.sin() * theta.cos(),
                    radius * phi.cos(),
                    radius * phi.sin() * theta.sin(),
                ));
            }
        }
        let mut triangles: Vec<Triangle> = Vec::new();
        for i in 0..lat_segments {
            for j in 0..lon_segments {
                let a = i * lon_segments + j;
                let b = i * lon_segments + (j + 1) % lon_segments;
                let c = (i + 1) * lon_segments + j;
                let d = (i + 1) * lon_segments + (j + 1) % lon_segments;
                triangles.push(Triangle::new(a, b, c));
                triangles.push(Triangle::new(c, b, d));
            }
        }
        let normals = Mesh::calculate_normals(&vertices, &triangles);
        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    // ============================================================================
    // (a) validate_carve_result
    // ============================================================================

    #[test]
    fn test_validate_carve_result() {
        let block = block_mesh(50.0);

        // The unmodified block passed back as the "cavity" is garbage.
        let block_as_result = validate_carve_result(block.clone(), &block);
        assert!(
            matches!(block_as_result, Err(BooleanError::InvalidMesh(_))),
            "block-as-result must be rejected, got {:?}",
            block_as_result.err()
        );

        // An empty result is garbage too.
        let empty = validate_carve_result(Mesh::new(), &block);
        assert!(
            matches!(empty, Err(BooleanError::InvalidMesh(_))),
            "empty result must be rejected"
        );

        // A real cavity (different topology) is accepted.
        let sphere = uv_sphere(20.0, 8, 8);
        let accepted = validate_carve_result(sphere, &block);
        assert!(accepted.is_ok(), "real cavity must be accepted");
    }

    // ============================================================================
    // (b) select_strategy honors complexity_threshold
    // ============================================================================

    #[test]
    fn test_select_strategy_honors_complexity_threshold() {
        let block = block_mesh(50.0);
        let model = uv_sphere(20.0, 28, 28); // 1568 triangles

        // Total = 12 + 1568 = 1580 > 1000 -> threshold path -> SimpleAABB.
        let low_threshold = BooleanConfig {
            complexity_threshold: 1000,
            ..Default::default()
        };
        assert_eq!(
            select_strategy(&block, &model, &low_threshold),
            BooleanStrategy::SimpleAABB,
            "threshold 1000 with 1580 total triangles must select SimpleAABB"
        );

        // Default threshold (100_000) keeps small meshes on the CSG path.
        let default_config = BooleanConfig::default();
        assert_eq!(
            select_strategy(&block, &model, &default_config),
            BooleanStrategy::CSG,
            "small mesh below default threshold must stay on CSG"
        );
    }

    // ============================================================================
    // (c) threshold path is voxel-first
    // ============================================================================

    #[test]
    fn test_threshold_path_voxel_first() {
        let block = block_mesh(50.0);
        let model = uv_sphere(20.0, 28, 28); // 1568 triangles, fully inside block

        let config = BooleanConfig {
            complexity_threshold: 1000, // force the threshold path
            voxel_resolution: Some(16),
            ..Default::default()
        };

        let result = boolean_subtract_with_config(&block, &model, &config);
        assert!(result.is_ok(), "voxel-first boolean should succeed: {:?}", result.err());

        let (cavity, meta) = result.unwrap();
        assert_eq!(
            meta.strategy_used,
            BooleanStrategy::Voxelization,
            "threshold path must attempt voxel before SimpleAABB"
        );
        assert!(!cavity.triangles.is_empty(), "cavity must not be empty");
        assert!(
            cavity.triangles.len() != block.triangles.len()
                || cavity.vertices.len() != block.vertices.len(),
            "cavity must differ from the unmodified block"
        );

        let block_volume = crate::pipeline::repair::calculate_volume(&block);
        let cavity_volume = crate::pipeline::repair::calculate_volume(&cavity);
        assert!(
            cavity_volume < block_volume,
            "cavity volume {} must be less than block volume {}",
            cavity_volume,
            block_volume
        );
    }

    // ============================================================================
    // (d) memory-exceeded path is voxel-first
    // ============================================================================

    #[test]
    fn test_memory_path_voxel_first() {
        let block = block_mesh(50.0);
        let model = uv_sphere(20.0, 10, 10); // 200 triangles, fully inside block

        // estimated = (12 + 200) * 470 = 99,640 > 1000 -> memory path.
        let config = BooleanConfig {
            max_memory: 1000,
            voxel_resolution: Some(16),
            ..Default::default()
        };

        let result = boolean_subtract_with_config(&block, &model, &config);
        assert!(result.is_ok(), "memory-path boolean should succeed: {:?}", result.err());

        let (cavity, meta) = result.unwrap();
        assert_eq!(
            meta.strategy_used,
            BooleanStrategy::Voxelization,
            "memory-exceeded path must attempt voxel before SimpleAABB"
        );
        assert!(!cavity.triangles.is_empty(), "cavity must not be empty");
        assert!(
            cavity.triangles.len() != block.triangles.len()
                || cavity.vertices.len() != block.vertices.len(),
            "cavity must differ from the unmodified block"
        );
        assert!(
            crate::pipeline::repair::calculate_volume(&cavity)
                < crate::pipeline::repair::calculate_volume(&block),
            "cavity volume must be less than block volume"
        );
    }

    // ============================================================================
    // (e) voxel pre-decimation caps over-limit models
    // ============================================================================

    #[test]
    fn test_voxel_pre_decimation() {
        let block = block_mesh(50.0);
        // 224*224*2 = 100,352 triangles: just over VOXEL_MODEL_TRI_LIMIT so the
        // pre-decimation step runs.
        let model = uv_sphere(20.0, 224, 224);

        // Resolution 6 keeps the SDF sample count (6^3) low enough for CI:
        // the debug-mode SDF at res 16 over ~100K triangles measured ~6 min
        // (parallelized sample_grid brings it to ~30s), so res 6 is the
        // smallest resolution that still yields a valid carve in ~2s.
        let config = BooleanConfig {
            voxel_resolution: Some(6),
            ..Default::default()
        };

        let result = boolean_subtract_with_config(&block, &model, &config);
        assert!(result.is_ok(), "over-limit model must still carve: {:?}", result.err());

        let (cavity, _meta) = result.unwrap();
        assert!(!cavity.triangles.is_empty(), "cavity must not be empty");
        assert!(
            cavity.triangles.len() != block.triangles.len()
                || cavity.vertices.len() != block.vertices.len(),
            "cavity must differ from the unmodified block"
        );
        assert!(
            crate::pipeline::repair::calculate_volume(&cavity)
                < crate::pipeline::repair::calculate_volume(&block),
            "cavity volume must be less than block volume"
        );
    }

    // ============================================================================
    // (f) CSG arm validates the carve and reports the CSG strategy
    // ============================================================================

    #[test]
    fn test_csg_arm_returns_csg_on_valid_carve() {
        let block = block_mesh(50.0);
        let model = uv_sphere(20.0, 8, 8); // 128 triangles, fully inside block

        // Force the CSG (csgrs) strategy; select_strategy returns it
        // unconditionally when the config does not say Auto.
        let config = BooleanConfig {
            strategy: BooleanStrategy::CSG,
            ..Default::default()
        };

        let result = boolean_subtract_with_config(&block, &model, &config);
        assert!(result.is_ok(), "CSG boolean should succeed: {:?}", result.err());

        let (cavity, meta) = result.unwrap();
        assert_eq!(
            meta.strategy_used,
            BooleanStrategy::CSG,
            "forced CSG strategy must report CSG"
        );
        assert!(!cavity.triangles.is_empty(), "cavity must not be empty");
        assert!(
            cavity.triangles.len() != block.triangles.len()
                || cavity.vertices.len() != block.vertices.len(),
            "cavity must differ from the unmodified block"
        );

        let block_volume = crate::pipeline::repair::calculate_volume(&block);
        let cavity_volume = crate::pipeline::repair::calculate_volume(&cavity);
        assert!(
            cavity_volume < block_volume,
            "cavity volume {} must be less than block volume {}",
            cavity_volume,
            block_volume
        );
    }

    // ============================================================================
    // (T3.8) Voxel perf probe - MANUAL ONLY, never in CI.
    // Run: cargo test --lib boolean -- --ignored
    // A ~2.9M-triangle model must complete a voxel boolean. Pre-decimation +
    // resolution clamp bound the SDF cost; if this takes too long, lower
    // VOXEL_MODEL_TRI_LIMIT.
    // ============================================================================

    #[test]
    #[ignore]
    fn test_voxel_perf_probe_large_model() {
        let block = block_mesh(50.0);
        let model = uv_sphere(20.0, 850, 1700); // 850*1700*2 = 2,890,000 triangles
        let config = BooleanConfig {
            voxel_resolution: Some(16),
            ..Default::default()
        };

        let start = Instant::now();
        let result = boolean_subtract_voxel(&block, &model, &config);
        let elapsed = start.elapsed();

        assert!(result.is_ok(), "large-model voxel boolean must complete");
        info!(
            "voxel perf probe: {:?} for {} input triangles",
            elapsed,
            model.triangles.len()
        );
    }

    // ============================================================================
    // Existing AABB classification tests
    // ============================================================================

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
