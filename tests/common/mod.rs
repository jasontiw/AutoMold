//! Shared helpers for integration tests.
//!
//! Files under `tests/common/` are not compiled as their own test crate (only
//! top-level `.rs` files in `tests/` are); they are included from integration
//! tests with `mod common;`.

use automold::geometry::mesh::{Mesh, Triangle};

/// UV sphere centered at the origin with `lat` latitude rings and `lon`
/// longitude segments -> `lat * lon * 2` triangles. Mirrors the construction
/// used in the boolean-layer unit tests (src/pipeline/boolean.rs).
pub fn uv_sphere(radius: f32, lat: usize, lon: usize) -> Mesh {
    let mut vertices: Vec<nalgebra::Point3<f32>> = Vec::new();
    for i in 0..=lat {
        let phi = std::f32::consts::PI * i as f32 / lat as f32;
        for j in 0..lon {
            let theta = 2.0 * std::f32::consts::PI * j as f32 / lon as f32;
            vertices.push(nalgebra::Point3::new(
                radius * phi.sin() * theta.cos(),
                radius * phi.cos(),
                radius * phi.sin() * theta.sin(),
            ));
        }
    }
    let mut triangles: Vec<Triangle> = Vec::new();
    for i in 0..lat {
        for j in 0..lon {
            let a = i * lon + j;
            let b = i * lon + (j + 1) % lon;
            let c = (i + 1) * lon + j;
            let d = (i + 1) * lon + (j + 1) % lon;
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

/// A genuinely non-manifold UV sphere: an 8x8 sphere (radius 10) with one
/// duplicate mid-latitude triangle pushed at the end.
///
/// The duplicate is `Triangle::new(8, 9, 16)` — vertices 8/9 (ring i=1) and 16
/// (ring i=2) differ by O(1) coordinates, so they survive the loader's
/// exact-f32 weld and Stage-2 degeneracy removal, and each edge of the
/// duplicate gains a 3rd user (3 non-manifold edges). A north-pole duplicate
/// (i=0) must NOT be used: its vertices are bit-identical and collapse to a
/// single point, so the triangle is removed as degenerate before the routing
/// gate ever sees it.
pub fn dirty_uv_sphere() -> Mesh {
    let mut mesh = uv_sphere(10.0, 8, 8);
    mesh.triangles.push(Triangle::new(8, 9, 16));
    mesh.normals = Mesh::calculate_normals(&mesh.vertices, &mesh.triangles);
    mesh
}
