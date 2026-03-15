//! Pin generation - alignment pins for mold halves

use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::{Point3, Vector3};

/// Generate alignment pins for both mold halves
pub fn generate_pins(mold_a: &Mesh, mold_b: &Mesh, split_axis: Vector3<f32>) -> Vec<Pin> {
    // Find suitable positions for pins on the split plane
    // Look for flat areas near the edges

    let bbox_a = mold_a.calculate_bounding_box();
    let bbox_b = mold_b.calculate_bounding_box();

    // Find split plane position
    let split_z = (bbox_a.max.z + bbox_a.min.z) / 2.0;

    // Generate 4 pins at corners
    let corners = [
        (bbox_a.min.x, bbox_a.min.y),
        (bbox_a.max.x, bbox_a.min.y),
        (bbox_a.min.x, bbox_a.max.y),
        (bbox_a.max.x, bbox_b.max.y),
    ];

    let mut pins = Vec::new();

    for (x, y) in corners {
        // Pin on side A
        let pin_a = Pin {
            position: Point3::new(x, y, split_z + 2.0),
            direction: Vector3::z(),
            diameter: 5.0,
            height: 10.0,
            side: PinSide::A,
        };

        // Corresponding hole on side B
        let hole_b = Pin {
            position: Point3::new(x, y, split_z - 2.0),
            direction: -Vector3::z(),
            diameter: 5.2, // Slightly larger for tolerance
            height: 10.0,
            side: PinSide::B,
        };

        pins.push(pin_a);
        pins.push(hole_b);
    }

    pins
}

/// A single alignment pin
#[derive(Debug, Clone)]
pub struct Pin {
    pub position: Point3<f32>,
    pub direction: Vector3<f32>,
    pub diameter: f32,
    pub height: f32,
    pub side: PinSide,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinSide {
    A,
    B,
}

/// Generate a pin mesh (cylinder)
pub fn generate_pin_mesh(pin: &Pin) -> Mesh {
    let radius = pin.diameter / 2.0;
    let segments = 16;

    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut indices: Vec<[usize; 3]> = Vec::new();

    // Direction determines pin orientation
    let (forward, right) = if pin.direction.z.abs() > 0.9 {
        (Vector3::z(), Vector3::x())
    } else {
        (
            pin.direction,
            Vector3::y().cross(&pin.direction).normalize(),
        )
    };

    let up = forward;
    let right = right.normalize();
    let up_cross_right = up.cross(&right);

    // Generate cylinder vertices
    for i in 0..segments {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / segments as f32;
        let cos = angle.cos();
        let sin = angle.sin();

        let offset = right * (radius * cos) + up_cross_right * (radius * sin);

        // Bottom vertex
        let bottom = pin.position - up * pin.height;
        vertices.push(bottom + offset);

        // Top vertex
        let top = pin.position;
        vertices.push(top + offset);
    }

    // Center bottom
    let bottom_center_idx = vertices.len();
    vertices.push(pin.position - up * pin.height);

    // Center top
    let top_center_idx = vertices.len();
    vertices.push(pin.position);

    // Generate triangles
    for i in 0..segments {
        let next = (i + 1) % segments;

        // Side
        indices.push([i * 2, next * 2, i * 2 + 1]);
        indices.push([next * 2, next * 2 + 1, i * 2 + 1]);

        // Bottom cap
        indices.push([bottom_center_idx, next * 2, i * 2]);

        // Top cap (reversed for outward normals)
        indices.push([top_center_idx, i * 2 + 1, next * 2 + 1]);
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

/// Generate hole (recess) in mesh
pub fn generate_hole_mesh(pin: &Pin) -> Mesh {
    // For hole, we just return a cylinder that will be subtracted
    // Similar to pin but with different orientation
    let mut pin_hole = pin.clone();
    pin_hole.diameter *= 1.2; // Make hole slightly larger

    generate_pin_mesh(&pin_hole)
}
