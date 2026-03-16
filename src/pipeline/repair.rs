//! Mesh repair - fixes common mesh errors

use crate::geometry::mesh::Mesh;
use std::collections::HashSet;

/// Result of mesh repair
#[derive(Debug, Default)]
pub struct RepairResult {
    pub holes_filled: usize,
    pub normals_fixed: usize,
    pub non_manifold_edges: usize,
    pub degenerate_fixed: usize,
}

/// Repair mesh issues
pub fn repair_mesh(mesh: &mut Mesh) -> RepairResult {
    let mut result = RepairResult::default();

    // Step 1: Fix degenerate triangles (zero-area)
    result.degenerate_fixed = fix_degenerate_triangles(mesh);

    // Step 2: Fix inverted normals
    result.normals_fixed = fix_inverted_normals(mesh);

    // Step 3: Fix non-manifold edges (simple case)
    result.non_manifold_edges = count_non_manifold_edges(mesh);

    // Step 4: Fill small holes (if any detected)
    result.holes_filled = fill_holes(mesh);

    result
}

/// Fix triangles with zero area
fn fix_degenerate_triangles(mesh: &mut Mesh) -> usize {
    let mut to_remove: Vec<usize> = Vec::new();

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let v = tri.get_vertices(&mesh.vertices);
        let e1 = v[1] - v[0];
        let e2 = v[2] - v[0];
        let cross = e1.cross(&e2);

        if cross.magnitude_squared() < 1e-10 {
            to_remove.push(i);
        }
    }

    // Remove degenerate triangles
    for idx in to_remove.iter().rev() {
        mesh.triangles.remove(*idx);
    }

    to_remove.len()
}

/// Fix inverted normals by detecting and flipping
fn fix_inverted_normals(mesh: &mut Mesh) -> usize {
    // Recalculate all face normals
    let new_normals = Mesh::calculate_normals(&mesh.vertices, &mesh.triangles);

    let mut fixed = 0;

    // Compare old normals with new ones to detect flips
    // This is a simplification - real implementation would check adjacency
    if mesh.normals.len() == new_normals.len() {
        for (old, new) in mesh.normals.iter().zip(new_normals.iter()) {
            if old.dot(new) < 0.0 {
                fixed += 1;
            }
        }
    } else {
        // Just recalculate
        fixed = mesh.normals.len();
    }

    mesh.normals = new_normals;
    fixed
}

/// Count non-manifold edges
fn count_non_manifold_edges(mesh: &Mesh) -> usize {
    use std::collections::HashMap;

    let mut edge_map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let edges = [
            (tri.indices[0], tri.indices[1]),
            (tri.indices[1], tri.indices[2]),
            (tri.indices[2], tri.indices[0]),
        ];

        for (a, b) in edges {
            // Store edge with min/max to avoid direction issues
            let key = (a.min(b), a.max(b));
            edge_map.entry(key).or_insert_with(Vec::new).push(tri_idx);
        }
    }

    // Count edges that belong to more than 2 triangles
    edge_map.values().filter(|v| v.len() > 2).count()
}

/// Fill simple holes (single missing faces)
fn fill_holes(mesh: &mut Mesh) -> usize {
    // This is a simplified implementation
    // Real implementation would detect boundary loops and triangulate

    // For now, just detect holes by looking for disconnected regions
    // A full implementation would be complex, so we return 0 for now
    // The CLI will warn if there are issues but won't fill holes automatically
    // unless they're very simple

    0
}

/// Remove duplicate triangles
pub fn remove_duplicates(mesh: &mut Mesh) -> usize {
    let mut seen: HashSet<(usize, usize, usize)> = HashSet::new();
    let mut to_remove: Vec<usize> = Vec::new();

    for (i, tri) in mesh.triangles.iter().enumerate() {
        // Normalize vertex order for comparison
        let mut indices = tri.indices;
        indices.sort();
        let key = (indices[0], indices[1], indices[2]);

        if seen.contains(&key) {
            to_remove.push(i);
        } else {
            seen.insert(key);
        }
    }

    for idx in to_remove.iter().rev() {
        mesh.triangles.remove(*idx);
    }

    to_remove.len()
}

/// Weld close vertices (for mesh repair)
pub fn weld_vertices(mesh: &mut Mesh, threshold: f32) -> usize {
    let threshold_sq = threshold * threshold;
    let mut merged = 0;

    let mut i = 0;
    while i < mesh.vertices.len() {
        let mut j = i + 1;
        while j < mesh.vertices.len() {
            let dist_sq = (mesh.vertices[i] - mesh.vertices[j]).magnitude_squared();

            if dist_sq < threshold_sq {
                // Merge vertex j into i - update all triangle references
                let old_idx = j;

                for tri in &mut mesh.triangles {
                    if tri.indices[0] == old_idx {
                        tri.indices[0] = i;
                    } else if tri.indices[1] == old_idx {
                        tri.indices[1] = i;
                    } else if tri.indices[2] == old_idx {
                        tri.indices[2] = i;
                    }

                    // Adjust indices higher than removed vertex
                    if tri.indices[0] > old_idx {
                        tri.indices[0] -= 1;
                    }
                    if tri.indices[1] > old_idx {
                        tri.indices[1] -= 1;
                    }
                    if tri.indices[2] > old_idx {
                        tri.indices[2] -= 1;
                    }
                }

                mesh.vertices.remove(j);
                merged += 1;
            } else {
                j += 1;
            }
        }
        i += 1;
    }

    merged
}

/// Ensure mesh has valid normals
pub fn ensure_normals(mesh: &mut Mesh) {
    if mesh.normals.len() != mesh.triangles.len() {
        mesh.normals = Mesh::calculate_normals(&mesh.vertices, &mesh.triangles);
    }
}

/// Check if mesh is watertight (closed manifold)
/// A watertight mesh has every edge shared by exactly 2 triangles
pub fn is_watertight(mesh: &Mesh) -> bool {
    if mesh.triangles.is_empty() {
        return false;
    }

    let mut edge_counts: std::collections::HashMap<(usize, usize), usize> =
        std::collections::HashMap::new();

    for tri in &mesh.triangles {
        let edges = [
            (tri.indices[0], tri.indices[1]),
            (tri.indices[1], tri.indices[2]),
            (tri.indices[2], tri.indices[0]),
        ];

        for (a, b) in edges {
            let key = (a.min(b), a.max(b));
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }

    edge_counts.values().all(|&count| count == 2)
}

/// Validate mesh quality after boolean operation
#[derive(Debug, Default)]
pub struct QualityMetrics {
    pub is_watertight: bool,
    pub triangle_count: usize,
    pub vertex_count: usize,
    pub non_manifold_edges: usize,
    pub degenerate_triangles: usize,
}

/// Calculate quality metrics for a mesh
pub fn calculate_quality_metrics(mesh: &Mesh) -> QualityMetrics {
    let mut metrics = QualityMetrics {
        triangle_count: mesh.triangles.len(),
        vertex_count: mesh.vertices.len(),
        ..Default::default()
    };

    metrics.is_watertight = is_watertight(mesh);
    metrics.non_manifold_edges = count_non_manifold_edges(mesh);
    metrics.degenerate_triangles = count_degenerate_triangles(mesh);

    metrics
}

fn count_degenerate_triangles(mesh: &Mesh) -> usize {
    mesh.triangles
        .iter()
        .filter(|tri| {
            let v = tri.get_vertices(&mesh.vertices);
            let e1 = v[1] - v[0];
            let e2 = v[2] - v[0];
            e1.cross(&e2).magnitude_squared() < 1e-10
        })
        .count()
}

/// Calculate the volume of a closed watertight mesh using the divergence theorem
/// Returns volume in cubic units (same units as mesh coordinates)
pub fn calculate_volume(mesh: &Mesh) -> f32 {
    if !is_watertight(mesh) {
        return 0.0;
    }

    let mut total_volume = 0.0;

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);
        let v0 = v[0].coords;
        let v1 = v[1].coords;
        let v2 = v[2].coords;

        let signed_volume = v0.dot(&v1.cross(&v2)) / 6.0;
        total_volume += signed_volume;
    }

    total_volume.abs()
}
