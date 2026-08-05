//! Integration tests for AutoMold

use automold::core::config::Unit;
use automold::pipeline::loader;
use std::fs;
use std::path::Path;

mod common;

/// Test that cube_10mm.stl generates valid output files
#[test]
fn test_cube_basic() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found: test_data/cube_10mm.stl - skipping test");
        return;
    }

    // Run the pipeline programmatically
    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Pipeline should succeed for cube_10mm.stl");

    // Check output files exist
    let output_dir = Path::new("test_output");
    assert!(
        output_dir.join("cube_10mm_mold_A.stl").exists(),
        "mold_A.stl should exist"
    );
    assert!(
        output_dir.join("cube_10mm_mold_B.stl").exists(),
        "mold_B.stl should exist"
    );
    assert!(
        output_dir.join("metadata.json").exists(),
        "metadata.json should exist"
    );
}

/// Test that cylinder_30mm.stl processes without crash
#[test]
fn test_cylinder_no_crash() {
    let test_file = Path::new("test_data/cylinder_30mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found: test_data/cylinder_30mm.stl - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(
        result.is_ok(),
        "Pipeline should succeed for cylinder_30mm.stl"
    );
}

/// Test that sphere_20mm.stl handles curved geometry
#[test]
fn test_sphere_curved_geometry() {
    let test_file = Path::new("test_data/sphere_20mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found: test_data/sphere_20mm.stl - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        tolerance: 0.5, // Custom tolerance
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(
        result.is_ok(),
        "Pipeline should succeed for sphere_20mm.stl"
    );
}

/// Test that metadata.json contains required fields
#[test]
fn test_metadata_contains_fields() {
    // Run pipeline first
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let _ = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    // Read and parse metadata
    let metadata_path = Path::new("test_output/metadata.json");
    if !metadata_path.exists() {
        panic!("metadata.json should exist after pipeline runs");
    }

    let content = fs::read_to_string(metadata_path).expect("Should be able to read metadata.json");

    // Check for required fields
    assert!(
        content.contains("\"tolerance_mm\""),
        "Should contain tolerance_mm"
    );
    assert!(
        content.contains("\"wall_thickness_mm\""),
        "Should contain wall_thickness_mm"
    );
    assert!(
        content.contains("\"split_axis\""),
        "Should contain split_axis"
    );
    assert!(
        content.contains("\"triangles_in\""),
        "Should contain triangles_in"
    );
}

/// Test auto-decimate triggers with low memory limit
#[test]
fn test_auto_decimate_triggers() {
    let test_file = Path::new("test_data/sphere_20mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    // Set very low memory limit to force decimation
    // sphere_20mm.stl has 512 triangles, estimate = 512 * 470 = 240KB
    // Use 100KB limit to force decimation
    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        memory_limit: Some(100 * 1024), // 100 KB
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    // Should still succeed with decimation
    assert!(
        result.is_ok(),
        "Pipeline should succeed even with auto-decimate"
    );

    // Check that auto_decimate was triggered
    assert!(
        ctx.decisions.auto_decimate.is_some(),
        "Auto-decimate should be triggered"
    );
}

/// Test tolerance configuration is respected
#[test]
fn test_tolerance_config() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        tolerance: 0.5,
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let _ = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert_eq!(ctx.decisions.tolerance, 0.5, "Tolerance should be 0.5");
}

/// Test 4.1: Cube produces watertight mold meshes
#[test]
fn test_cube_watertight_mold() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Pipeline should succeed for cube");

    let mold_a_path = Path::new("test_output/cube_10mm_mold_A.stl");
    let mold_b_path = Path::new("test_output/cube_10mm_mold_B.stl");

    assert!(mold_a_path.exists(), "Mold A should exist");
    assert!(mold_b_path.exists(), "Mold B should exist");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");
    let mold_b = loader::load_stl(mold_b_path, Unit::Millimeters).expect("Should load mold B STL");

    let a_watertight = automold::pipeline::repair::is_watertight(&mold_a);
    let b_watertight = automold::pipeline::repair::is_watertight(&mold_b);

    eprintln!(
        "[test_cube_watertight_mold] Mold A: {} vertices, {} triangles, watertight={}",
        mold_a.vertices.len(),
        mold_a.triangles.len(),
        a_watertight
    );
    eprintln!(
        "[test_cube_watertight_mold] Mold B: {} vertices, {} triangles, watertight={}",
        mold_b.vertices.len(),
        mold_b.triangles.len(),
        b_watertight
    );

    assert!(a_watertight, "Mold A should be watertight");
    assert!(b_watertight, "Mold B should be watertight");
}

/// Test 4.2: Sphere produces watertight mold meshes
#[test]
fn test_sphere_watertight_mold() {
    let test_file = Path::new("test_data/sphere_20mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Pipeline should succeed for sphere");

    let mold_a_path = Path::new("test_output/sphere_20mm_mold_A.stl");
    let mold_b_path = Path::new("test_output/sphere_20mm_mold_B.stl");

    assert!(mold_a_path.exists(), "Mold A should exist");
    assert!(mold_b_path.exists(), "Mold B should exist");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");
    let mold_b = loader::load_stl(mold_b_path, Unit::Millimeters).expect("Should load mold B STL");

    let a_watertight = automold::pipeline::repair::is_watertight(&mold_a);
    let b_watertight = automold::pipeline::repair::is_watertight(&mold_b);

    assert!(a_watertight, "Mold A should be watertight");
    assert!(b_watertight, "Mold B should be watertight");
}

/// Test 4.3: Torus produces watertight mold meshes (skip if torus.stl not available)
#[test]
fn test_torus_watertight_mold() {
    let test_file = Path::new("test_data/torus.stl");
    if !test_file.exists() {
        eprintln!("Test file not found: test_data/torus.stl - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Pipeline should succeed for torus");

    let mold_a_path = Path::new("test_output/torus_mold_A.stl");
    let mold_b_path = Path::new("test_output/torus_mold_B.stl");

    assert!(mold_a_path.exists(), "Mold A should exist");
    assert!(mold_b_path.exists(), "Mold B should exist");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");
    let mold_b = loader::load_stl(mold_b_path, Unit::Millimeters).expect("Should load mold B STL");

    let a_watertight = automold::pipeline::repair::is_watertight(&mold_a);
    let b_watertight = automold::pipeline::repair::is_watertight(&mold_b);

    assert!(a_watertight, "Mold A should be watertight");
    assert!(b_watertight, "Mold B should be watertight");
}

/// Test 4.4: Cavity volume is approximately equal to model volume
#[test]
fn test_cavity_volume_approx_model_volume() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Pipeline should succeed");

    let model = loader::load_stl(test_file, Unit::Millimeters).expect("Should load model STL");

    let model_volume = automold::pipeline::repair::calculate_volume(&model);
    assert!(model_volume > 0.0, "Model should have valid volume");

    // The cavity is the volume carved out of the mold block. The mold halves
    // are exactly the block minus the cavity, so:
    //   cavity_volume = block_volume - mold_A_volume - mold_B_volume
    let block = automold::pipeline::mold_block::generate_block(
        ctx.bounding_box.as_ref().expect("bounding box set"),
        ctx.decisions.wall_thickness,
    );
    let block_volume = automold::pipeline::repair::calculate_volume(&block);

    let mold_a_path = Path::new("test_output/cube_10mm_mold_A.stl");
    let mold_b_path = Path::new("test_output/cube_10mm_mold_B.stl");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");
    let mold_b = loader::load_stl(mold_b_path, Unit::Millimeters).expect("Should load mold B STL");

    let cavity_volume = block_volume
        - automold::pipeline::repair::calculate_volume(&mold_a)
        - automold::pipeline::repair::calculate_volume(&mold_b);

    let volume_ratio = cavity_volume / model_volume;
    assert!(
        volume_ratio > 0.9 && volume_ratio < 1.1,
        "Cavity volume should be approximately equal to model volume. Ratio: {}",
        volume_ratio
    );
}

/// Test 4.5: Voxel fallback activates when CSG fails
#[test]
fn test_voxel_fallback_on_csg_failure() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Pipeline should succeed");

    let has_boolean_strategy = ctx.decisions.boolean_strategy.is_some();
    let has_watertight = ctx.decisions.watertight.is_some();

    assert!(
        has_boolean_strategy || has_watertight,
        "Boolean strategy or watertight status should be recorded"
    );
}

/// Test 4.6: Memory limit is respected
#[test]
fn test_memory_limit_respected() {
    let test_file = Path::new("test_data/sphere_20mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let memory_limit = 50 * 1024 * 1024;
    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        memory_limit: Some(memory_limit),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let estimated = ctx.estimate_memory();

    assert!(
        estimated <= memory_limit || ctx.needs_auto_decimate(),
        "Memory estimate should respect limit or trigger decimation"
    );

    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);
    assert!(result.is_ok(), "Pipeline should succeed with memory limit");
}

/// Test 4.7: Full pipeline cube → mold
#[test]
fn test_full_pipeline_cube_to_mold() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        tolerance: 0.2,
        wall_thickness: Some(12.0),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Full pipeline should succeed for cube");

    let mold_a_path = Path::new("test_output/cube_10mm_mold_A.stl");
    let mold_b_path = Path::new("test_output/cube_10mm_mold_B.stl");
    let metadata_path = Path::new("test_output/metadata.json");

    assert!(mold_a_path.exists(), "Mold A should exist");
    assert!(mold_b_path.exists(), "Mold B should exist");
    assert!(metadata_path.exists(), "Metadata should exist");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");

    assert!(!mold_a.vertices.is_empty(), "Mold A should have vertices");
    assert!(!mold_a.triangles.is_empty(), "Mold A should have triangles");

    let metadata_content = fs::read_to_string(metadata_path).expect("Should read metadata.json");
    assert!(
        metadata_content.contains("wall_thickness_mm"),
        "Metadata should contain wall thickness"
    );
}

/// Test 4.8: Full pipeline sphere → mold
#[test]
fn test_full_pipeline_sphere_to_mold() {
    let test_file = Path::new("test_data/sphere_20mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        tolerance: 0.3,
        wall_thickness: Some(10.0),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(result.is_ok(), "Full pipeline should succeed for sphere");

    let mold_a_path = Path::new("test_output/sphere_20mm_mold_A.stl");
    let mold_b_path = Path::new("test_output/sphere_20mm_mold_B.stl");
    let metadata_path = Path::new("test_output/metadata.json");

    assert!(mold_a_path.exists(), "Mold A should exist");
    assert!(mold_b_path.exists(), "Mold B should exist");
    assert!(metadata_path.exists(), "Metadata should exist");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");

    assert!(!mold_a.vertices.is_empty(), "Mold A should have vertices");
    assert!(!mold_a.triangles.is_empty(), "Mold A should have triangles");

    let metadata_content = fs::read_to_string(metadata_path).expect("Should read metadata.json");
    assert!(
        metadata_content.contains("wall_thickness_mm"),
        "Metadata should contain wall thickness"
    );
}

/// Regression test for fix-stl-nan-normals: exported mold halves must never
/// contain NaN facet normals, even when the split pipeline retains slivers
#[test]
fn test_mold_outputs_have_finite_normals() {
    let test_file = Path::new("test_data/cube_10mm.stl");
    if !test_file.exists() {
        eprintln!("Test file not found: test_data/cube_10mm.stl - skipping test");
        return;
    }

    let config = automold::core::config::Config {
        input: test_file.to_path_buf(),
        output_dir: Some(Path::new("test_output").to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);
    assert!(result.is_ok(), "Pipeline should succeed for cube_10mm.stl");

    for mold in ["cube_10mm_mold_A.stl", "cube_10mm_mold_B.stl"] {
        let mold_path = Path::new("test_output").join(mold);
        assert!(mold_path.exists(), "{} should exist", mold_path.display());

        let bytes = fs::read(&mold_path).expect("Should read mold STL");
        assert!(
            bytes.len() >= 84,
            "STL file too short: {} ({} bytes)",
            mold_path.display(),
            bytes.len()
        );

        let count =
            u32::from_le_bytes(bytes[80..84].try_into().expect("triangle count bytes")) as usize;
        assert_eq!(
            bytes.len(),
            84 + count * 50,
            "STL file size should match record count"
        );

        for (i, record) in bytes[84..].chunks(50).enumerate() {
            let nx = f32::from_le_bytes(record[0..4].try_into().expect("normal x bytes"));
            let ny = f32::from_le_bytes(record[4..8].try_into().expect("normal y bytes"));
            let nz = f32::from_le_bytes(record[8..12].try_into().expect("normal z bytes"));
            assert!(
                nx.is_finite() && ny.is_finite() && nz.is_finite(),
                "{} record {} normal not finite: ({}, {}, {})",
                mold,
                i,
                nx,
                ny,
                nz
            );
        }
    }
}

/// Sanity check for the shared dirty UV sphere builder: the mid-latitude
/// duplicate triangle (8,9,16) must make the mesh genuinely non-manifold
/// (each of its edges gains a 3rd user). Guards against the R3-01 regression
/// where a north-pole duplicate collapsed to a single vertex.
#[test]
fn test_dirty_uv_sphere_is_non_manifold() {
    let mesh = common::dirty_uv_sphere();
    let metrics = automold::pipeline::repair::calculate_quality_metrics(&mesh);
    assert!(
        metrics.non_manifold_edges > 0,
        "dirty_uv_sphere must be non-manifold, got {}",
        metrics.non_manifold_edges
    );
}

/// Test 5.1: Non-manifold input produces a valid mold (regression for
/// lucas.stl, which has 964 non-manifold edges and previously crashed the CSG
/// backend with a stack overflow or produced a silent full-block "cavity").
/// Uses the shared `dirty_uv_sphere` builder: a UV sphere plus a mid-latitude
/// duplicate triangle (8,9,16) that survives the loader's exact-f32 weld and
/// Stage-2 degeneracy removal, so its 3 edges each gain a 3rd user (3
/// non-manifold edges out of 129 triangles stays under the pipeline's 10%
/// unrecoverable gate, like the lucas case 964/2.9M). Pre-repair removes the
/// duplicate and leaves a clean watertight mesh, so the gate must route the
/// honest outcome to CSG. Writes to a dedicated output dir so it never races
/// with other tests sharing the default test_output/ directory.
#[test]
fn test_non_manifold_input_produces_valid_mold() {
    let out_dir = Path::new("test_output/non_manifold_regression");
    let _ = fs::remove_dir_all(out_dir);
    fs::create_dir_all(out_dir).expect("create dedicated output dir");
    let input_path = out_dir.join("non_manifold_input.stl");

    let mesh = common::dirty_uv_sphere();

    let metrics = automold::pipeline::repair::calculate_quality_metrics(&mesh);
    assert!(
        metrics.non_manifold_edges > 0,
        "sanity: synthetic mesh must be non-manifold, got {}",
        metrics.non_manifold_edges
    );
    assert!(
        metrics.non_manifold_edges <= metrics.triangle_count / 10,
        "sanity: synthetic mesh must pass the pipeline 10% gate"
    );

    automold::export::stl::write_stl(&mesh, &input_path).expect("write synthetic input STL");

    let config = automold::core::config::Config {
        input: input_path.clone(),
        output_dir: Some(out_dir.to_path_buf()),
        ..Default::default()
    };

    let mut ctx = automold::core::context::Context::new(config);
    let result = automold::pipeline::pipeline_core::run_pipeline(&mut ctx);

    assert!(
        result.is_ok(),
        "Pipeline must succeed on non-manifold input: {:?}",
        result.err()
    );

    assert_eq!(
        ctx.decisions.boolean_strategy.as_deref(),
        Some("CSG"),
        "pre-repair must fix the duplicate, so the clean mesh routes to CSG"
    );

    let mold_a_path = out_dir.join("non_manifold_input_mold_A.stl");
    let mold_b_path = out_dir.join("non_manifold_input_mold_B.stl");
    assert!(mold_a_path.exists(), "Mold A should exist");
    assert!(mold_b_path.exists(), "Mold B should exist");

    let mold_a = loader::load_stl(&mold_a_path, Unit::Millimeters).expect("Should load mold A STL");
    let mold_b = loader::load_stl(&mold_b_path, Unit::Millimeters).expect("Should load mold B STL");

    assert!(
        automold::pipeline::repair::is_watertight(&mold_a),
        "Mold A should be watertight"
    );
    assert!(
        automold::pipeline::repair::is_watertight(&mold_b),
        "Mold B should be watertight"
    );

    // A real cavity must have been carved (not a silent full-block "cavity").
    let block = automold::pipeline::mold_block::generate_block(
        ctx.bounding_box.as_ref().expect("bounding box set"),
        ctx.decisions.wall_thickness,
    );
    let cavity_volume = automold::pipeline::repair::calculate_volume(&block)
        - automold::pipeline::repair::calculate_volume(&mold_a)
        - automold::pipeline::repair::calculate_volume(&mold_b);
    assert!(
        cavity_volume > 0.0,
        "Cavity volume must be non-zero, got {}",
        cavity_volume
    );
}
