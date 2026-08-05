//! Pipeline orchestration - coordinates all processing stages

use crate::core::{config::*, context::*};
use crate::geometry::mesh::Mesh;
use std::path::Path;
use tracing::{debug, error, info, warn};

use super::{boolean, decimate, loader, mold_block, orientation, pins, repair, split};

/// Maximum non-manifold edges tolerated before the boolean strategy is forced
/// to voxel. CSG backends are unsafe on non-manifold meshes (stack overflow or
/// silent garbage), so only fully manifold meshes may use the CSG strategy.
const MAX_CSG_NON_MANIFOLD_EDGES: usize = 0;

/// Whether a post-repair mesh must be routed to the voxel boolean strategy.
///
/// CSG backends are unsafe on meshes that are not fully manifold: non-manifold
/// edges (edges used by more than 2 triangles) cause stack overflow or silent
/// garbage, and boundary edges (holes) that pre-repair can create — when
/// removing a triangle that was the 2nd user of another edge leaves a 1-user
/// edge — would otherwise reach CSG undetected. Both counts come from the
/// `QualityMetrics` already computed for the pre-repair debug log, so routing
/// adds no extra edge scan.
pub(crate) fn should_route_to_voxel(non_manifold_edges: usize, boundary_edges: usize) -> bool {
    non_manifold_edges > MAX_CSG_NON_MANIFOLD_EDGES || boundary_edges > 0
}

/// Pipeline cavity sanity check (Design D8 / Spec R4 clause 2).
///
/// The boolean layer only validates its fallback paths; a "successful" CSG
/// result can still be garbage — empty, or identical to the unmodified mold
/// block, meaning nothing was carved (the historical silent full-block
/// "cavity" bug). The pipeline must reject such results instead of exiting 0
/// and splitting a useless full block.
fn check_cavity(cavity: Mesh, block: &Mesh) -> Result<Mesh, (ExitCode, String)> {
    boolean::validate_carve_result(cavity, block).map_err(|e| {
        (
            ExitCode::BooleanFailed,
            format!("Cavity sanity check failed: {}", e),
        )
    })
}

/// Main pipeline execution
pub fn run_pipeline(ctx: &mut Context) -> Result<(), (ExitCode, String)> {
    ctx.start();

    info!("AutoMold v{}", env!("CARGO_PKG_VERSION"));
    info!("Input: {:?}", ctx.config.input);

    // Stage 1: Load mesh
    info!("Loading mesh...");
    let mesh = loader::load_mesh(&ctx.config.input, ctx.config.input_unit)
        .map_err(|e| (ExitCode::FileNotFound, e.to_string()))?;

    ctx.stats.triangles_in = mesh.triangles.len();
    info!("Loaded {} triangles", ctx.stats.triangles_in);

    // Calculate bounding box
    let bbox = mesh.calculate_bounding_box();
    ctx.bounding_box = Some(bbox.clone());
    info!(
        "Bounding box: {:.1} x {:.1} x {:.1} mm",
        bbox.size().x,
        bbox.size().y,
        bbox.size().z
    );

    // Check scale warning
    let size = bbox.size();
    if size.x < 1.0 || size.y < 1.0 || size.z < 1.0 {
        warn!("Model bounding box is very small: {:.1} x {:.1} x {:.1} mm. Did you mean to use --unit in?", 
            size.x, size.y, size.z);
    }
    if size.x > 2000.0 || size.y > 2000.0 || size.z > 2000.0 {
        warn!(
            "Model bounding box is very large: {:.1} x {:.1} x {:.1} mm",
            size.x, size.y, size.z
        );
    }

    // Store original mesh for analysis
    ctx.original_mesh = Some(mesh.clone());
    ctx.mesh = Some(mesh);

    // Stage 2: Mesh repair
    info!("Repairing mesh...");
    let repair_result = repair::repair_mesh(&mut ctx.mesh.as_mut().unwrap());
    ctx.stats.holes_filled = repair_result.holes_filled;
    ctx.stats.normals_fixed = repair_result.normals_fixed;
    ctx.stats.non_manifold_edges = repair_result.non_manifold_edges;

    if ctx.stats.non_manifold_edges > ctx.stats.triangles_in / 10 {
        error!("Too many non-manifold edges (>10%)");
        return Err((
            ExitCode::MeshUnrecoverable,
            format!("Mesh repair failed — too many non-manifold edges (>10%)"),
        ));
    }

    info!(
        "Mesh repaired ({} holes filled, {} normals fixed, {} non-manifold edges)",
        ctx.stats.holes_filled, ctx.stats.normals_fixed, ctx.stats.non_manifold_edges
    );

    // Stage 3: Memory budget estimation and auto-decimation
    let memory_estimate = ctx.estimate_memory();
    let memory_limit = ctx
        .config
        .memory_limit
        .unwrap_or(ctx.available_memory * 75 / 100);
    let memory_limit_mb = memory_limit / (1024 * 1024);

    info!(
        "Memory estimate: {} MB, available: {} MB",
        memory_estimate / (1024 * 1024),
        memory_limit_mb
    );

    // Apply auto-decimation if needed
    let decimate_ratio = if let Some(ratio) = ctx.config.decimate {
        Some(ratio)
    } else if memory_estimate > memory_limit {
        let ratio = ctx.auto_decimate_ratio();
        let estimated_mb = memory_estimate / (1024 * 1024);
        let reason = format!(
            "Estimated {} MB exceeds limit of {} MB",
            estimated_mb, memory_limit_mb
        );
        warn!(
            "Auto-decimating to {:.0}% to fit memory budget - {}",
            ratio * 100.0,
            reason
        );
        ctx.decisions.auto_decimate = Some(ratio);
        ctx.decisions.auto_decimate_reason = Some(reason);
        Some(ratio)
    } else {
        None
    };

    if let Some(ratio) = decimate_ratio {
        if ratio < 1.0 {
            info!(
                "Applying decimation ({} -> {})",
                ctx.stats.triangles_in,
                (ctx.stats.triangles_in as f32 * ratio) as usize
            );
            decimate::decimate_mesh(ctx.mesh.as_mut().unwrap(), ratio);
            ctx.stats.triangles_out = Some(ctx.mesh.as_ref().unwrap().triangles.len());
            ctx.stats.decimation_ratio = Some(1.0 - ratio);
        }
    }

    // Stage 4: Orientation analysis (using original mesh for accuracy)
    info!("Analyzing orientation...");
    ctx.suggested_split_axis = orientation::analyze_orientation(
        ctx.original_mesh
            .as_ref()
            .unwrap_or(ctx.mesh.as_ref().unwrap()),
    );

    let split_axis = match ctx.config.split_axis {
        SplitAxis::Auto => ctx.suggested_split_axis.unwrap_or(SplitAxis::Z),
        a => a,
    };

    let axis_str = match split_axis {
        SplitAxis::X => "X",
        SplitAxis::Y => "Y",
        SplitAxis::Z => "Z",
        SplitAxis::Auto => unreachable!(),
    };

    ctx.decisions.split_axis = axis_str.to_string();
    info!("Split axis: {}", axis_str);

    // Set wall thickness
    ctx.decisions.wall_thickness = ctx
        .config
        .wall_thickness
        .unwrap_or_else(|| ctx.calculate_wall_thickness());

    // Set tolerance
    ctx.decisions.tolerance = ctx.config.tolerance;

    // Set pins
    ctx.decisions.pins_enabled = ctx.config.generate_pins;

    // Set threads
    ctx.decisions.threads = ctx
        .config
        .threads
        .unwrap_or_else(|| rayon::current_num_threads().min(4).max(1));

    // Stage 5: Generate mold block
    info!("Generating mold block...");
    let mold_block = mold_block::generate_block(
        &ctx.bounding_box.as_ref().unwrap(),
        ctx.decisions.wall_thickness,
    );

    // Stage 6: Boolean operation (block - model)
    info!("Performing boolean operation...");

    // Phase 2: Pre-boolean repair - clean up mesh before CSG
    let (mesh_for_boolean, gate_stats) = {
        let input_mesh = ctx.mesh.as_ref().unwrap();
        match repair::pre_repair_mesh(input_mesh) {
            Ok(repaired) => {
                let stats = repair::calculate_quality_metrics(&repaired);
                tracing::debug!(
                    "Pre-boolean repair: {} triangles, {} vertices, {} non-manifold edges, {} boundary edges",
                    stats.triangle_count,
                    stats.vertex_count,
                    stats.non_manifold_edges,
                    stats.boundary_edges
                );
                (repaired, stats)
            }
            Err(e) => {
                warn!("Pre-boolean repair failed: {}, using original mesh", e);
                let stats = repair::calculate_quality_metrics(input_mesh);
                (input_mesh.clone(), stats)
            }
        }
    };

    // CSG is unsafe on non-manifold meshes (stack overflow / silent garbage)
    // and on meshes with boundary edges (holes), which pre-repair can create
    // when it removes a triangle that was the 2nd user of another edge — so
    // route to the voxel strategy when either survives pre-repair.
    let strategy =
        if should_route_to_voxel(gate_stats.non_manifold_edges, gate_stats.boundary_edges) {
            warn!(
                "Mesh still has {} non-manifold / {} boundary edge(s) after pre-repair; using voxel strategy",
                gate_stats.non_manifold_edges, gate_stats.boundary_edges
            );
            boolean::BooleanStrategy::Voxelization
        } else {
            boolean::BooleanStrategy::Auto
        };

    let bool_config = boolean::BooleanConfig {
        strategy,
        max_memory: ctx.available_memory,
        tolerance: ctx.decisions.tolerance,
        preserve_cavity_walls: true,
        complexity_threshold: 100_000,
        voxel_resolution: None,
    };

    let boolean_result =
        boolean::boolean_subtract_with_config(&mold_block, &mesh_for_boolean, &bool_config);

    let (cavity_mesh, bool_metadata) = match boolean_result {
        Ok((m, metadata)) => (m, metadata),
        Err(e) => {
            error!("Boolean failed: {}", e);
            return Err((
                ExitCode::BooleanFailed,
                format!("Boolean operation failed: {}", e),
            ));
        }
    };

    // Expose the strategy actually used (CSG, Voxelization, ...) so callers
    // and tests can assert the routing decision on `ctx.decisions`.
    ctx.decisions.boolean_strategy = Some(format!("{:?}", bool_metadata.strategy_used));

    // Log boolean strategy used
    info!(
        "Boolean completed using {:?} strategy in {}ms ({} triangles)",
        bool_metadata.strategy_used, bool_metadata.execution_time_ms, bool_metadata.triangle_count
    );

    // Log any warnings from boolean operation
    for warning in &bool_metadata.warnings {
        warn!("Boolean warning: {}", warning);
    }

    // Phase 3: Post-boolean repair - fix issues from CSG operation
    let repaired_cavity_mesh = {
        match repair::post_repair_mesh(&cavity_mesh) {
            Ok(repaired) => {
                let stats = repair::calculate_quality_metrics(&repaired);
                tracing::debug!(
                    "Post-boolean repair: {} triangles, {} vertices, {} non-manifold edges",
                    stats.triangle_count,
                    stats.vertex_count,
                    stats.non_manifold_edges
                );
                repaired
            }
            Err(e) => {
                warn!("Post-boolean repair failed: {}, using original mesh", e);
                cavity_mesh
            }
        }
    };

    // Use the repaired mesh for further processing (re-bound mutably by the
    // cavity sanity check below, where it is validated and later split).
    let cavity_mesh = repaired_cavity_mesh;
    let mut bool_metadata = bool_metadata;

    // Track whether post-boolean repair was applied
    let original_triangle_count = bool_metadata.triangle_count;
    if cavity_mesh.triangles.len() != original_triangle_count {
        bool_metadata.set_repaired();
        bool_metadata.add_warning(format!(
            "Post-boolean repair changed triangle count from {} to {}",
            original_triangle_count,
            cavity_mesh.triangles.len()
        ));
    }

    // Update triangle count after repair
    bool_metadata.triangle_count = cavity_mesh.triangles.len();

    // Post-boolean quality validation
    let quality = repair::calculate_quality_metrics(&cavity_mesh);
    ctx.decisions.watertight = Some(quality.is_watertight);

    if quality.is_watertight {
        info!(
            "Boolean result: watertight ({} triangles, {} vertices)",
            quality.triangle_count, quality.vertex_count
        );
    } else {
        warn!(
            "Boolean result: NOT watertight ({} boundary edges, {} non-manifold edges, {} degenerate)",
            quality.boundary_edges, quality.non_manifold_edges, quality.degenerate_triangles
        );
    }

    if quality.non_manifold_edges > 0 {
        warn!(
            "Boolean mesh has {} non-manifold edges",
            quality.non_manifold_edges
        );
    }

    if quality.duplicate_vertices > 0 {
        warn!(
            "Boolean mesh has {} duplicate vertices",
            quality.duplicate_vertices
        );
    }

    // Log volume if watertight
    if quality.is_watertight {
        let volume = repair::calculate_volume(&cavity_mesh);
        info!("Cavity volume: {:.2} cubic units", volume);
    }

    // Stage 6.5: Pipeline cavity sanity check (Design D8 / Spec R4 clause 2).
    // Reject an empty result or one identical to the unmodified block (no
    // carving happened) before splitting, on every strategy path.
    let mut cavity_mesh = check_cavity(cavity_mesh, &mold_block)?;

    // Stage 7: Split the mold
    let split_axis_vec = match split_axis {
        SplitAxis::X => nalgebra::Vector3::x(),
        SplitAxis::Y => nalgebra::Vector3::y(),
        SplitAxis::Z => nalgebra::Vector3::z(),
        SplitAxis::Auto => nalgebra::Vector3::z(),
    };

    // FIX: usar el bbox de cavity_mesh (el bloque con cavidad), NO el del modelo original.
    // ctx.bounding_box es el bbox del cilindro/modelo de entrada — para modelos no centrados
    // en el origen el centro del bloque y el del modelo difieren, produciendo un split incorrecto.
    let cavity_bbox = cavity_mesh.calculate_bounding_box();
    let split_point = cavity_bbox.center();

    let axis = match split_axis {
        SplitAxis::X => split::Axis::X,
        SplitAxis::Y => split::Axis::Y,
        SplitAxis::Z | SplitAxis::Auto => split::Axis::Z,
    };
    let split_coord = match split_axis {
        SplitAxis::X => split_point.x,
        SplitAxis::Y => split_point.y,
        SplitAxis::Z | SplitAxis::Auto => split_point.z,
    };

    // FIX: eliminar triángulos degenerados (área cero) producidos por csgrs durante el boolean.
    // Estos generan normales NaN en calculate_normals, lo que corrompe la geometría del molde
    // y puede hacer que el split produzca caras incorrectas o invisibles en el STL resultante.
    {
        let verts = &cavity_mesh.vertices;
        cavity_mesh.triangles.retain(|tri| {
            let v = tri.get_vertices(verts);
            let e1 = v[1] - v[0];
            let e2 = v[2] - v[0];
            e1.cross(&e2).magnitude_squared() > 1e-10
        });
        cavity_mesh.normals = crate::geometry::mesh::Mesh::calculate_normals(
            &cavity_mesh.vertices,
            &cavity_mesh.triangles,
        );
        debug!(
            "Pre-split cleanup: {} triangles after removing degenerates",
            cavity_mesh.triangles.len()
        );
    }

    let (mold_a, mold_b) = split::split_mesh(&cavity_mesh, axis, split_coord).map_err(|e| {
        (
            ExitCode::BooleanFailed,
            format!("Failed to split mesh: {}", e),
        )
    })?;

    debug!(
        "Split complete: mold_a has {} tris, mold_b has {} tris",
        mold_a.triangles.len(),
        mold_b.triangles.len()
    );
    debug!(
        "Input to split: cavity_mesh has {} tris, {} verts",
        cavity_mesh.triangles.len(),
        cavity_mesh.vertices.len()
    );

    // Stage 8: Generate pins if requested
    let _pins = if ctx.decisions.pins_enabled {
        Some(pins::generate_pins(&mold_a, &mold_b, split_axis_vec))
    } else {
        None
    };

    // Stage 9: Export
    let output_format = ctx.config.output_format;
    let base_stem = ctx
        .config
        .input
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "model".to_string());

    let output_dir = ctx.config.output_dir.clone().unwrap_or_else(|| {
        ctx.config
            .input
            .parent()
            .map(|p| p.to_path_buf())
            .unwrap_or_default()
    });

    // Export mold parts
    let parts: Vec<(&Mesh, &str)> = vec![(&mold_a, "mold_A"), (&mold_b, "mold_B")];
    for (_i, (part, suffix)) in parts.iter().enumerate() {
        let filename = format!("{}_{}", base_stem, suffix);

        match output_format {
            OutputFormat::Stl => {
                let path = output_dir.join(format!("{}.stl", filename));
                crate::export::stl::write_stl(part, &path)
                    .map_err(|e| (ExitCode::BooleanFailed, e.to_string()))?;
            }
            OutputFormat::ThreeMF => {
                let path = output_dir.join(format!("{}.3mf", filename));
                crate::export::threemf::write_3mf(part, &path, ctx.config.input_unit)
                    .map_err(|e| (ExitCode::BooleanFailed, e.to_string()))?;
            }
        }
    }

    // Export metadata
    let metadata = crate::export::metadata::Metadata {
        input_file: ctx.config.input.to_string_lossy().to_string(),
        input_unit: ctx.config.input_unit.as_str().to_string(),
        normalized_unit: "mm".to_string(),
        bounding_box_mm: [size.x, size.y, size.z],
        triangles_in: ctx.stats.triangles_in,
        triangles_out: ctx.stats.triangles_out.unwrap_or(ctx.stats.triangles_in),
        split_axis: ctx.decisions.split_axis.clone(),
        wall_thickness_mm: ctx.decisions.wall_thickness,
        tolerance_mm: ctx.decisions.tolerance,
        generate_pins: ctx.decisions.pins_enabled,
    };

    let metadata_path = output_dir.join("metadata.json");
    crate::export::metadata::write_metadata(&metadata, &metadata_path)
        .map_err(|e| (ExitCode::BooleanFailed, e.to_string()))?;

    // Print summary
    ctx.stats.processing_time_ms = ctx.elapsed_ms();
    info!(
        "Done in {:.1}s",
        ctx.stats.processing_time_ms.unwrap_or(0) as f32 / 1000.0
    );

    Ok(())
}

/// Quick validation that a mesh can be processed
pub fn validate_mesh(path: &Path) -> Result<usize, String> {
    let mesh = loader::load_mesh(path, Unit::Millimeters).map_err(|e| e.to_string())?;
    Ok(mesh.triangles.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 50-unit half-size cube block (12 triangles, 8 vertices) — same shape
    /// the boolean layer uses in its unit tests.
    fn block_mesh(half: f32) -> Mesh {
        let vertices = vec![
            nalgebra::Point3::new(-half, -half, -half),
            nalgebra::Point3::new(half, -half, -half),
            nalgebra::Point3::new(half, half, -half),
            nalgebra::Point3::new(-half, half, -half),
            nalgebra::Point3::new(-half, -half, half),
            nalgebra::Point3::new(half, -half, half),
            nalgebra::Point3::new(half, half, half),
            nalgebra::Point3::new(-half, half, half),
        ];
        let triangles = vec![
            crate::geometry::mesh::Triangle::new(0, 1, 2),
            crate::geometry::mesh::Triangle::new(0, 2, 3),
            crate::geometry::mesh::Triangle::new(4, 6, 5),
            crate::geometry::mesh::Triangle::new(4, 7, 6),
            crate::geometry::mesh::Triangle::new(0, 4, 5),
            crate::geometry::mesh::Triangle::new(0, 5, 1),
            crate::geometry::mesh::Triangle::new(3, 2, 6),
            crate::geometry::mesh::Triangle::new(3, 6, 7),
            crate::geometry::mesh::Triangle::new(0, 3, 7),
            crate::geometry::mesh::Triangle::new(0, 7, 4),
            crate::geometry::mesh::Triangle::new(1, 5, 6),
            crate::geometry::mesh::Triangle::new(1, 6, 2),
        ];
        let normals = crate::geometry::mesh::Mesh::calculate_normals(&vertices, &triangles);
        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// A valid cavity: the block with the top face carved open (differs in
    /// topology from the block).
    fn carved_mesh() -> Mesh {
        let mut mesh = block_mesh(50.0);
        // Remove one face's worth of triangles to represent a carve: 12 -> 10.
        mesh.triangles.truncate(10);
        mesh.normals =
            crate::geometry::mesh::Mesh::calculate_normals(&mesh.vertices, &mesh.triangles);
        mesh
    }

    #[test]
    fn test_check_cavity_accepts_valid_carve() {
        let block = block_mesh(50.0);
        let cavity = carved_mesh();
        let result = check_cavity(cavity, &block);
        assert!(result.is_ok(), "a real carve must pass the sanity check");
        assert_eq!(result.unwrap().triangles.len(), 10);
    }

    #[test]
    fn test_check_cavity_rejects_empty_result() {
        let block = block_mesh(50.0);
        let result = check_cavity(Mesh::new(), &block);
        let err = result.expect_err("empty cavity must be rejected");
        assert_eq!(err.0, ExitCode::BooleanFailed);
        assert!(
            err.1.contains("Cavity sanity check failed"),
            "unexpected message: {}",
            err.1
        );
    }

    #[test]
    fn test_check_cavity_rejects_unmodified_block() {
        let block = block_mesh(50.0);
        // The boolean returned the block unchanged: nothing was carved.
        let result = check_cavity(block_mesh(50.0), &block);
        let err = result.expect_err("block-identical cavity must be rejected");
        assert_eq!(err.0, ExitCode::BooleanFailed);
        assert!(
            err.1.contains("no cavity carved") || err.1.contains("Cavity sanity check"),
            "unexpected message: {}",
            err.1
        );
    }

    #[test]
    fn test_should_route_to_voxel_clean_mesh() {
        assert!(
            !should_route_to_voxel(0, 0),
            "a clean post-repair mesh must stay on the CSG path"
        );
    }

    #[test]
    fn test_should_route_to_voxel_non_manifold() {
        assert!(
            should_route_to_voxel(1, 0),
            "surviving non-manifold edges must route to the voxel strategy"
        );
    }

    #[test]
    fn test_should_route_to_voxel_boundary_hole() {
        // R3-02: the old gate (non_manifold_edges > 0 only) returned false here,
        // letting hole-ridden meshes reach CSG; the hardened gate must route to
        // voxel whenever a boundary edge (hole) survives pre-repair.
        assert!(
            should_route_to_voxel(0, 1),
            "boundary edges (holes) must route to the voxel strategy"
        );
    }
}