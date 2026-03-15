//! Pour channel generation

use crate::geometry::bbox::BoundingBox;
use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::Point3;

/// Generate pour channel (sprue) for mold
pub fn generate_pour_channel(mold: &Mesh, _model_bbox: &BoundingBox) -> Option<Mesh> {
    // Find top of mold to place pour channel
    let bbox = mold.calculate_bounding_box();

    // Place pour channel at center of top face
    let center_x = (bbox.min.x + bbox.max.x) / 2.0;
    let center_y = (bbox.min.y + bbox.max.y) / 2.0;
    let top_z = bbox.max.z;

    // Create funnel shape
    let funnel = create_funnel(
        Point3::new(center_x, center_y, top_z),
        8.0,  // top radius
        5.0,  // bottom radius
        15.0, // height
    );

    Some(funnel)
}

/// Create a funnel/sprue shape
fn create_funnel(base: Point3<f32>, top_radius: f32, bottom_radius: f32, height: f32) -> Mesh {
    let segments = 16;
    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut indices: Vec<[usize; 3]> = Vec::new();

    // Generate vertices
    for i in 0..segments {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / segments as f32;
        let cos = angle.cos();
        let sin = angle.sin();

        // Top ring
        vertices.push(Point3::new(
            base.x + top_radius * cos,
            base.y + top_radius * sin,
            base.z + height,
        ));

        // Bottom ring
        vertices.push(Point3::new(
            base.x + bottom_radius * cos,
            base.y + bottom_radius * sin,
            base.z,
        ));
    }

    // Center top
    let top_center = vertices.len();
    vertices.push(Point3::new(base.x, base.y, base.z + height));

    // Center bottom (closed)
    let bottom_center = vertices.len();
    vertices.push(base);

    // Generate triangles
    for i in 0..segments {
        let next = (i + 1) % segments;

        // Funnel sides
        indices.push([i * 2, next * 2, i * 2 + 1]);
        indices.push([next * 2, next * 2 + 1, i * 2 + 1]);

        // Top cap
        indices.push([top_center, next * 2, i * 2]);

        // Bottom (closed)
        indices.push([bottom_center, i * 2 + 1, next * 2 + 1]);
    }

    let triangles: Vec<Triangle> = indices
        .iter()
        .map(|i| Triangle::new(i[0], i[1], i[2]))
        .collect();

    let normals = Mesh::calculate_normals(&vertices, &triangles);

    Mesh {
        vertices,
        triangles,
        normals,
    }
}

/// Calculate optimal pour channel position based on model
pub fn find_pour_position(model_bbox: &BoundingBox) -> Point3<f32> {
    // Place at highest point of model
    let center = model_bbox.center();
    Point3::new(center.x, center.y, model_bbox.max.z)
}

/// Estimate pour channel size based on model
pub fn estimate_channel_size(model_volume: f32) -> (f32, f32) {
    // Rough estimation based on model volume
    // Larger models need larger channels

    let top_radius = (model_volume.sqrt() / 10.0).clamp(5.0, 15.0);
    let bottom_radius = top_radius * 0.6;

    (top_radius, bottom_radius)
}

/// Generate runner system (optional, for advanced molds)
pub fn generate_runner_system(mold: &Mesh, pour_position: Point3<f32>) -> Option<Mesh> {
    // Simple runner from pour to cavity center
    let bbox = mold.calculate_bounding_box();
    let cavity_center = bbox.center();

    // Create rectangular runner
    let runner_width = 6.0;
    let runner_height = 4.0;

    let start = pour_position;
    let end = Point3::new(cavity_center.x, cavity_center.y, pour_position.z);

    // Create box runner
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Simple representation - just return pour channel for now
    // Full runner system would be more complex

    generate_pour_channel(mold, &bbox)
}
