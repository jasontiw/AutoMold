//! Pipeline orchestration - coordinates all processing stages

use crate::core::{config::*, context::*};
use crate::geometry::mesh::Mesh;
use std::path::Path;
use tracing::{debug, error, info, warn};

use super::{boolean, decimate, loader, mold_block, orientation, pins, repair, split};

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
    let mesh_for_boolean = {
        let input_mesh = ctx.mesh.as_ref().unwrap();
        match repair::pre_repair_mesh(input_mesh) {
            Ok(repaired) => {
                let stats = repair::calculate_quality_metrics(&repaired);
                tracing::debug!(
                    "Pre-boolean repair: {} triangles, {} vertices, {} non-manifold edges",
                    stats.triangle_count,
                    stats.vertex_count,
                    stats.non_manifold_edges
                );
                repaired
            }
            Err(e) => {
                warn!("Pre-boolean repair failed: {}, using original mesh", e);
                input_mesh.clone()
            }
        }
    };

    let bool_config = boolean::BooleanConfig {
        strategy: boolean::BooleanStrategy::Auto,
        max_memory: ctx.available_memory,
        tolerance: ctx.decisions.tolerance,
        preserve_cavity_walls: true,
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

    // Use the repaired mesh for further processing
    let mut cavity_mesh = repaired_cavity_mesh;
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

    // Stage 7: Split the mold
    let split_axis_vec = match split_axis {
        SplitAxis::X => nalgebra::Vector3::x(),
        SplitAxis::Y => nalgebra::Vector3::y(),
        SplitAxis::Z => nalgebra::Vector3::z(),
        SplitAxis::Auto => nalgebra::Vector3::z(),
    };

    let split_point = ctx.bounding_box.as_ref().unwrap().center();
    let (mold_a_opt, mold_b_opt) = split::split_mesh(&cavity_mesh, split_axis_vec, split_point);
    let mold_a = mold_a_opt.expect("Failed to split mesh - part A is None");
    let mold_b = mold_b_opt.expect("Failed to split mesh - part B is None");

    // Stage 8: Generate pins if requested
    let pins = if ctx.decisions.pins_enabled {
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
    for (i, (part, suffix)) in parts.iter().enumerate() {
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
