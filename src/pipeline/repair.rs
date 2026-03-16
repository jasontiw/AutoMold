//! Mesh repair - fixes common mesh errors

use crate::geometry::mesh::Mesh;
use std::collections::{HashMap, HashSet};

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

// ============================================================================
// Phase 2: Pre-Boolean Repair
// These functions prepare meshes for CSG boolean operations
// ============================================================================

use thiserror::Error;

#[derive(Error, Debug)]
pub enum PreRepairError {
    #[error("Empty mesh: {0}")]
    EmptyMesh(String),

    #[error("Too few vertices: {0}")]
    TooFewVertices(String),

    #[error("Repair failed: {0}")]
    RepairFailed(String),
}

/// Epsilon tolerance for vertex merging
const VERTEX_MERGE_EPSILON: f32 = 1e-5;

/// Minimum triangle area threshold (relative to average)
const MIN_TRIANGLE_AREA_RATIO: f32 = 1e-6;

/// Pre-repair mesh for boolean operations
/// Chains: duplicate vertex removal -> degenerate removal -> normal consistency
pub fn pre_repair_mesh(mesh: &Mesh) -> Result<Mesh, PreRepairError> {
    if mesh.triangles.is_empty() {
        return Err(PreRepairError::EmptyMesh("No triangles".to_string()));
    }
    if mesh.vertices.len() < 3 {
        return Err(PreRepairError::TooFewVertices(
            "Less than 3 vertices".to_string(),
        ));
    }

    let mut result = mesh.clone();

    // Step 1: Remove duplicate vertices
    let vertex_merges = remove_duplicate_vertices(&mut result);
    if vertex_merges > 0 {
        tracing::debug!("Merged {} duplicate vertices", vertex_merges);
    }

    // If all vertices were merged away, return error
    if result.vertices.is_empty() {
        return Err(PreRepairError::RepairFailed(
            "All vertices merged away".to_string(),
        ));
    }

    // Step 2: Remove degenerate triangles
    let degenerate_removed = remove_degenerate_triangles(&mut result);
    if degenerate_removed > 0 {
        tracing::debug!("Removed {} degenerate triangles", degenerate_removed);
    }

    // If all triangles were removed, return error
    if result.triangles.is_empty() {
        return Err(PreRepairError::RepairFailed(
            "All triangles removed as degenerate".to_string(),
        ));
    }

    // Step 3: Fix normal consistency
    let normals_fixed = fix_normal_consistency(&mut result);
    if normals_fixed > 0 {
        tracing::debug!("Fixed {} inconsistent normals", normals_fixed);
    }

    // Recalculate all normals after repairs
    result.normals = Mesh::calculate_normals(&result.vertices, &result.triangles);

    Ok(result)
}

/// Task 2.1: Remove duplicate vertices using spatial hashing
/// Finds vertices at the same position (within epsilon) and merges them
fn remove_duplicate_vertices(mesh: &mut Mesh) -> usize {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return 0;
    }

    let eps = VERTEX_MERGE_EPSILON;
    let eps_sq = eps * eps;

    // Build spatial hash map for efficient duplicate detection
    // Key: quantized position, Value: first vertex index with this position
    let mut position_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut vertex_remapping: Vec<Option<usize>> = vec![None; mesh.vertices.len()];
    let mut merged_count = 0;

    // Quantize factor - ensures vertices within epsilon map to same key
    let quantize = |v: &nalgebra::Point3<f32>| -> (i64, i64, i64) {
        let scale = 1.0 / eps;
        (
            (v.x * scale).round() as i64,
            (v.y * scale).round() as i64,
            (v.z * scale).round() as i64,
        )
    };

    // First pass: build the map and find duplicates
    for (i, vertex) in mesh.vertices.iter().enumerate() {
        let key = quantize(vertex);

        if let Some(&existing_idx) = position_map.get(&key) {
            // This vertex is a duplicate - map to existing vertex
            vertex_remapping[i] = Some(existing_idx);
            merged_count += 1;
        } else {
            // First vertex at this position
            position_map.insert(key, i);
            vertex_remapping[i] = Some(i);
        }
    }

    if merged_count == 0 {
        return 0;
    }

    // Build new vertex list with remapped indices
    let mut new_vertices: Vec<nalgebra::Point3<f32>> = Vec::new();
    let mut index_remap: Vec<usize> = vec![0; mesh.vertices.len()];

    for (i, vertex) in mesh.vertices.iter().enumerate() {
        if vertex_remapping[i] == Some(i) {
            // This vertex is kept
            index_remap[i] = new_vertices.len();
            new_vertices.push(*vertex);
        }
    }

    // Update triangle indices
    for tri in &mut mesh.triangles {
        for idx in &mut tri.indices {
            *idx = index_remap[vertex_remapping[*idx].unwrap_or(*idx)];
        }
    }

    mesh.vertices = new_vertices;
    merged_count
}

/// Task 2.2: Remove degenerate triangles (zero or very small area)
fn remove_degenerate_triangles(mesh: &mut Mesh) -> usize {
    let mut to_remove: Vec<usize> = Vec::new();

    // Calculate average triangle area for relative threshold
    let mut total_area = 0.0f32;
    let mut triangle_areas: Vec<f32> = Vec::new();

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let v = tri.get_vertices(&mesh.vertices);
        let e1 = v[1] - v[0];
        let e2 = v[2] - v[0];
        let cross = e1.cross(&e2);
        let area = 0.5 * cross.magnitude();

        triangle_areas.push(area);
        total_area += area;
    }

    let avg_area = if !triangle_areas.is_empty() {
        total_area / triangle_areas.len() as f32
    } else {
        0.0
    };

    let min_area = avg_area * MIN_TRIANGLE_AREA_RATIO;
    let absolute_min = 1e-10; // Absolute minimum

    let threshold = min_area.max(absolute_min);

    for (i, area) in triangle_areas.iter().enumerate() {
        if *area < threshold {
            to_remove.push(i);
        }
    }

    // Remove degenerate triangles (in reverse to maintain indices)
    for idx in to_remove.iter().rev() {
        mesh.triangles.remove(*idx);
    }

    to_remove.len()
}

/// Task 2.3: Fix normal consistency across connected components
/// Uses BFS to traverse connected triangles and ensures consistent orientation
fn fix_normal_consistency(mesh: &mut Mesh) -> usize {
    if mesh.triangles.is_empty() || mesh.vertices.is_empty() {
        return 0;
    }

    // Build adjacency map: triangle index -> neighboring triangle indices
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); mesh.triangles.len()];

    // Build edge map: edge -> triangle indices
    let mut edge_map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let edges = [
            (tri.indices[0], tri.indices[1]),
            (tri.indices[1], tri.indices[2]),
            (tri.indices[2], tri.indices[0]),
        ];

        for (a, b) in edges {
            let key = (a.min(b), a.max(b));
            edge_map.entry(key).or_insert_with(Vec::new).push(tri_idx);
        }
    }

    // Build adjacency from shared edges
    for triangles in edge_map.values() {
        if triangles.len() == 2 {
            adjacency[triangles[0]].push(triangles[1]);
            adjacency[triangles[1]].push(triangles[0]);
        }
    }

    // Track visited triangles and those that needed flipping
    let mut visited = vec![false; mesh.triangles.len()];
    let mut flipped = 0;

    // Process each unvisited component
    for start in 0..mesh.triangles.len() {
        if visited[start] {
            continue;
        }

        // BFS to traverse the connected component
        let mut queue = vec![start];
        visited[start] = true;

        // Track the expected normal direction (flip if needed)
        let start_normal = calculate_triangle_normal(mesh, start);

        while let Some(current) = queue.pop() {
            let current_normal = calculate_triangle_normal(mesh, current);

            // Check neighbors
            for &neighbor in &adjacency[current] {
                if visited[neighbor] {
                    continue;
                }

                let neighbor_normal = calculate_triangle_normal(mesh, neighbor);

                // If normals point in opposite directions, flip the neighbor
                if current_normal.dot(&neighbor_normal) < 0.0 {
                    // Flip the triangle indices to reverse normal
                    mesh.triangles[neighbor].indices.swap(1, 2);
                    flipped += 1;
                }

                visited[neighbor] = true;
                queue.push(neighbor);
            }
        }
    }

    flipped
}

/// Calculate the normal of a specific triangle
fn calculate_triangle_normal(mesh: &Mesh, tri_idx: usize) -> nalgebra::Vector3<f32> {
    let tri = &mesh.triangles[tri_idx];
    let v = tri.get_vertices(&mesh.vertices);
    let e1 = v[1] - v[0];
    let e2 = v[2] - v[0];
    let normal = e1.cross(&e2);

    if normal.magnitude_squared() > 1e-10 {
        normal.normalize()
    } else {
        nalgebra::Vector3::zeros()
    }
}
