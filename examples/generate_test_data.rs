//! Generate test STL files for Phase 0 validation
//! Run with: cargo run --bin gen-test-data

use automold::export::stl;
use automold::geometry::mesh::Mesh;
use automold::pipeline::mold_block;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Generating test STL files...");

    let test_data_dir = Path::new("test_data");
    std::fs::create_dir_all(test_data_dir)?;

    // 1. Generate cube (10mm)
    println!("Generating cube_10mm.stl...");
    let cube = mold_block::generate_box(10.0, 10.0, 10.0);
    println!(
        "  Vertices: {}, Triangles: {}",
        cube.vertices.len(),
        cube.triangles.len()
    );
    stl::write_stl(&cube, &test_data_dir.join("cube_10mm.stl"))?;

    // 2. Generate cylinder (15mm radius, 30mm height, 24 segments)
    println!("Generating cylinder_30mm.stl...");
    let cylinder = mold_block::generate_cylinder(15.0, 30.0, 24);
    println!(
        "  Vertices: {}, Triangles: {}",
        cylinder.vertices.len(),
        cylinder.triangles.len()
    );
    stl::write_stl(&cylinder, &test_data_dir.join("cylinder_30mm.stl"))?;

    // 3. Generate sphere (20mm radius, 16 segments)
    println!("Generating sphere_20mm.stl...");
    let sphere = mold_block::generate_sphere(20.0, 16);
    println!(
        "  Vertices: {}, Triangles: {}",
        sphere.vertices.len(),
        sphere.triangles.len()
    );
    stl::write_stl(&sphere, &test_data_dir.join("sphere_20mm.stl"))?;

    println!("\nGenerated 3 test files in test_data/");
    Ok(())
}
