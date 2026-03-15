//! Integration tests for AutoMold

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
