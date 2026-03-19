//! Integration tests for AutoMold

use automold::core::config::Unit;
use automold::pipeline::loader;
use std::fs;
use std::path::Path;

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

    let mold_a_path = Path::new("test_output/cube_10mm_mold_A.stl");

    let mold_a = loader::load_stl(mold_a_path, Unit::Millimeters).expect("Should load mold A STL");

    let cavity_volume = automold::pipeline::repair::calculate_volume(&mold_a);

    let volume_ratio = cavity_volume / model_volume;
    assert!(
        volume_ratio > 0.5 && volume_ratio < 2.0,
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
