//! Mold block generation

use crate::geometry::bbox::BoundingBox;
use crate::geometry::mesh::Mesh;
use nalgebra::Point3;
use std::f32::consts::PI;

/// Generate a rectangular block for the mold
pub fn generate_block(bbox: &BoundingBox, wall_thickness: f32) -> Mesh {
    let size = bbox.size();
    let center = bbox.center();

    // Block dimensions = model size + wall thickness on all sides
    let block_width = size.x + wall_thickness * 2.0;
    let block_height = size.y + wall_thickness * 2.0;
    let block_depth = size.z + wall_thickness * 2.0;

    // Create box vertices
    let half_w = block_width / 2.0;
    let half_h = block_height / 2.0;
    let half_d = block_depth / 2.0;

    let cx = center.x;
    let cy = center.y;
    let cz = center.z;

    let vertices: Vec<Point3<f32>> = vec![
        // Front face
        Point3::new(cx - half_w, cy - half_h, cz + half_d),
        Point3::new(cx + half_w, cy - half_h, cz + half_d),
        Point3::new(cx + half_w, cy + half_h, cz + half_d),
        Point3::new(cx - half_w, cy + half_h, cz + half_d),
        // Back face
        Point3::new(cx - half_w, cy - half_h, cz - half_d),
        Point3::new(cx + half_w, cy - half_h, cz - half_d),
        Point3::new(cx + half_w, cy + half_h, cz - half_d),
        Point3::new(cx - half_w, cy + half_h, cz - half_d),
    ];

    // Define triangles (6 faces, 2 triangles each)
    let indices: Vec<[usize; 3]> = vec![
        // Front
        [0, 1, 2],
        [0, 2, 3],
        // Back
        [5, 4, 7],
        [5, 7, 6],
        // Top
        [3, 2, 6],
        [3, 6, 7],
        // Bottom
        [4, 5, 1],
        [4, 1, 0],
        // Right
        [1, 5, 6],
        [1, 6, 2],
        // Left
        [4, 0, 3],
        [4, 3, 7],
    ];

    let mesh = Mesh::from_parts(vertices, indices);
    mesh
}

/// Generate a box mesh with custom dimensions
pub fn generate_box(width: f32, height: f32, depth: f32) -> Mesh {
    let half_w = width / 2.0;
    let half_h = height / 2.0;
    let half_d = depth / 2.0;

    let vertices: Vec<Point3<f32>> = vec![
        // Front
        Point3::new(-half_w, -half_h, half_d),
        Point3::new(half_w, -half_h, half_d),
        Point3::new(half_w, half_h, half_d),
        Point3::new(-half_w, half_h, half_d),
        // Back
        Point3::new(-half_w, -half_h, -half_d),
        Point3::new(half_w, -half_h, -half_d),
        Point3::new(half_w, half_h, -half_d),
        Point3::new(-half_w, half_h, -half_d),
    ];

    let indices: Vec<[usize; 3]> = vec![
        [0, 1, 2],
        [0, 2, 3],
        [5, 4, 7],
        [5, 7, 6],
        [3, 2, 6],
        [3, 6, 7],
        [4, 5, 1],
        [4, 1, 0],
        [1, 5, 6],
        [1, 6, 2],
        [4, 0, 3],
        [4, 3, 7],
    ];

    Mesh::from_parts(vertices, indices)
}

/// Generate a cylinder (for testing)
pub fn generate_cylinder(radius: f32, height: f32, segments: usize) -> Mesh {
    let mut vertices: Vec<Point3<f32>> = Vec::with_capacity(segments * 2 + 2);
    let mut indices: Vec<[usize; 3]> = Vec::new();

    // Top and bottom center
    vertices.push(Point3::new(0.0, 0.0, height / 2.0));
    vertices.push(Point3::new(0.0, 0.0, -height / 2.0));

    let top_center = 0;
    let bottom_center = 1;

    // Generate ring vertices
    for i in 0..segments {
        let angle = 2.0 * PI * i as f32 / segments as f32;
        let x = radius * angle.cos();
        let y = radius * angle.sin();

        // Top ring
        vertices.push(Point3::new(x, y, height / 2.0));
        // Bottom ring
        vertices.push(Point3::new(x, y, -height / 2.0));
    }

    // Generate triangles
    for i in 0..segments {
        let next = (i + 1) % segments;

        // Top cap
        indices.push([top_center, 2 + i * 2, 2 + next * 2]);

        // Bottom cap
        indices.push([bottom_center, 2 + next * 2 + 1, 2 + i * 2 + 1]);

        // Side triangles
        let tl = 2 + i * 2;
        let tr = 2 + next * 2;
        let bl = 2 + i * 2 + 1;
        let br = 2 + next * 2 + 1;

        indices.push([tl, bl, tr]);
        indices.push([tr, bl, br]);
    }

    Mesh::from_parts(vertices, indices)
}

/// Generate sphere (for testing)
pub fn generate_sphere(radius: f32, segments: usize) -> Mesh {
    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut indices: Vec<[usize; 3]> = Vec::new();

    // Generate vertices using spherical coordinates
    for lat in 0..=segments {
        let theta = PI * lat as f32 / segments as f32;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        for lon in 0..=segments {
            let phi = 2.0 * PI * lon as f32 / segments as f32;

            let x = radius * sin_theta * phi.cos();
            let y = radius * sin_theta * phi.sin();
            let z = radius * cos_theta;

            vertices.push(Point3::new(x, y, z));
        }
    }

    // Generate triangles
    for lat in 0..segments {
        for lon in 0..segments {
            let first = lat * (segments + 1) + lon;
            let second = first + segments + 1;

            indices.push([first, second, first + 1]);
            indices.push([second, second + 1, first + 1]);
        }
    }

    Mesh::from_parts(vertices, indices)
}
