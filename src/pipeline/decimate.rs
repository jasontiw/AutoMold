//! Decimation - reduce polygon count using Quadric Error Metrics

use crate::geometry::mesh::{Mesh, Triangle};
use meshopt::simplify::{self, SimplifyOptions};
use meshopt::VertexDataAdapter;
use nalgebra::Point3;
use std::collections::HashMap;

/// Decimate mesh using meshopt library
pub fn decimate_mesh(mesh: &mut Mesh, ratio: f32) {
    let target_count = (mesh.triangles.len() as f32 * ratio) as usize;

    if target_count >= mesh.triangles.len() {
        return; // No decimation needed
    }

    // meshopt works on its own data structures
    // We need to convert and convert back

    let positions: Vec<f32> = mesh
        .vertices
        .iter()
        .flat_map(|p| vec![p.x, p.y, p.z])
        .collect();

    // Convert to bytes for VertexDataAdapter
    let positions_bytes: Vec<u8> = unsafe {
        std::slice::from_raw_parts(
            positions.as_ptr() as *const u8,
            positions.len() * std::mem::size_of::<f32>(),
        )
        .to_vec()
    };

    let indices: Vec<u32> = mesh
        .triangles
        .iter()
        .flat_map(|t| {
            vec![
                t.indices[0] as u32,
                t.indices[1] as u32,
                t.indices[2] as u32,
            ]
        })
        .collect();

    // Use meshopt for decimation
    // Create vertex data adapter for meshopt
    // data: byte slice, stride: bytes per vertex (12 = 3 floats * 4 bytes), count: number of vertices
    let vertex_count = mesh.vertices.len();
    let vertex_adapter = match VertexDataAdapter::new(&positions_bytes, 12, vertex_count) {
        Ok(adapter) => adapter,
        Err(_) => return, // If we can't create adapter, skip decimation
    };

    // Target index count (triangles * 3)
    let target_index_count = target_count * 3;

    // Use meshopt simplify
    let simplified_indices = simplify::simplify(
        &indices,
        &vertex_adapter,
        target_index_count,
        1e-2f32, // target error threshold
        SimplifyOptions::empty(),
        None, // no result error needed
    );

    let index_buffer = simplified_indices;

    // Rebuild mesh with decimated indices
    // First, find which original vertices are still used
    let mut used_vertices: HashMap<usize, usize> = HashMap::new();
    let mut new_index = 0;

    for idx in &index_buffer {
        let old_idx = *idx as usize;
        if !used_vertices.contains_key(&old_idx) {
            used_vertices.insert(old_idx, new_index);
            new_index += 1;
        }
    }

    // Build new vertex array
    let mut new_vertices: Vec<Point3<f32>> = Vec::with_capacity(used_vertices.len());
    new_vertices.resize(used_vertices.len(), Point3::origin());

    for (old_idx, new_idx) in &used_vertices {
        new_vertices[*new_idx] = mesh.vertices[*old_idx];
    }

    // Build new triangle array
    let new_triangles: Vec<Triangle> = index_buffer
        .chunks(3)
        .filter_map(|chunk| {
            if chunk.len() == 3 {
                let i0 = used_vertices.get(&(chunk[0] as usize)).copied()?;
                let i1 = used_vertices.get(&(chunk[1] as usize)).copied()?;
                let i2 = used_vertices.get(&(chunk[2] as usize)).copied()?;
                Some(Triangle::new(i0, i1, i2))
            } else {
                None
            }
        })
        .collect();

    // Recalculate normals
    let new_normals = Mesh::calculate_normals(&new_vertices, &new_triangles);

    mesh.vertices = new_vertices;
    mesh.triangles = new_triangles;
    mesh.normals = new_normals;
}

/// Simple decimation fallback (percentage-based vertex removal)
/// Used when meshopt is not available or fails
pub fn decimate_simple(mesh: &mut Mesh, ratio: f32) {
    let target_count = (mesh.triangles.len() as f32 * ratio) as usize;

    if target_count >= mesh.triangles.len() {
        return;
    }

    // Simple approach: remove every Nth triangle
    let step = mesh.triangles.len() / target_count;
    if step < 1 {
        return;
    }

    let mut new_triangles: Vec<Triangle> = Vec::with_capacity(target_count);

    for (i, tri) in mesh.triangles.iter().enumerate() {
        if i % step == 0 && new_triangles.len() < target_count {
            new_triangles.push(*tri);
        }
    }

    // Update vertex references (some vertices may now be orphaned)
    // This is a simplification - proper decimation would also remove orphaned vertices

    mesh.triangles = new_triangles;
    mesh.normals = Mesh::calculate_normals(&mesh.vertices, &mesh.triangles);
}

/// Decimate for analysis (lightweight, preserves shape details)
pub fn decimate_for_analysis(mesh: &Mesh, ratio: f32) -> Mesh {
    // Create a copy and decimate
    let mut copy = mesh.clone();
    decimate_mesh(&mut copy, ratio);
    copy
}

/// Estimate decimation ratio to achieve target triangle count
pub fn calculate_decimation_ratio(current: usize, target: usize) -> f32 {
    if current == 0 {
        return 1.0;
    }

    let ratio = target as f32 / current as f32;
    ratio.clamp(0.1, 1.0)
}

/// Get optimal decimation for memory constraint
pub fn decimate_for_memory(mesh: &Mesh, memory_budget: usize) -> f32 {
    // Each triangle roughly needs 150 bytes in processing
    let triangle_budget = memory_budget / 150;
    let current = mesh.triangles.len();

    calculate_decimation_ratio(current, triangle_budget.min(current))
}
