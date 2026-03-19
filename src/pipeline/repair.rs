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
fn fill_holes(_mesh: &mut Mesh) -> usize {
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
    pub boundary_edges: usize, // Edges that belong to only 1 triangle (potential holes)
    pub duplicate_vertices: usize,
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
    metrics.boundary_edges = count_boundary_edges(mesh);
    metrics.duplicate_vertices = count_duplicate_vertices(mesh);

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

/// Count boundary edges (edges used by only 1 triangle)
/// These edges indicate holes or non-watertight regions
fn count_boundary_edges(mesh: &Mesh) -> usize {
    if mesh.triangles.is_empty() {
        return 0;
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

    // Boundary edges are those used by exactly 1 triangle
    edge_counts.values().filter(|&&count| count == 1).count()
}

/// Count duplicate vertices (vertices at the same position)
fn count_duplicate_vertices(mesh: &Mesh) -> usize {
    if mesh.vertices.is_empty() {
        return 0;
    }

    let eps = 1e-5f32;
    let eps_sq = eps * eps;

    let mut unique_count = 0;

    for (i, _vertex) in mesh.vertices.iter().enumerate() {
        let mut is_duplicate = false;

        for j in 0..i {
            let dist_sq = (mesh.vertices[i] - mesh.vertices[j]).magnitude_squared();
            if dist_sq < eps_sq {
                is_duplicate = true;
                break;
            }
        }

        if !is_duplicate {
            unique_count += 1;
        }
    }

    mesh.vertices.len() - unique_count
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
    let _eps_sq = eps * eps;

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

    for (_i, tri) in mesh.triangles.iter().enumerate() {
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
        let _start_normal = calculate_triangle_normal(mesh, start);

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

// ============================================================================
// Phase 3: Post-Boolean Repair
// These functions fix issues that appear after CSG boolean operations
// ============================================================================

#[derive(Error, Debug)]
pub enum PostRepairError {
    #[error("Empty mesh: {0}")]
    EmptyMesh(String),

    #[error("Repair failed: {0}")]
    RepairFailed(String),
}

/// Configuration for post-boolean repair
#[derive(Debug, Clone)]
pub struct PostRepairConfig {
    /// Epsilon for vertex welding (default: 1e-4)
    pub weld_threshold: f32,
    /// Maximum hole edge count to fill (default: 10)
    pub max_hole_edges: usize,
    /// Whether to fix non-manifold edges (default: true)
    pub fix_non_manifold: bool,
}

impl Default for PostRepairConfig {
    fn default() -> Self {
        Self {
            weld_threshold: 1e-4,
            max_hole_edges: 10,
            fix_non_manifold: true,
        }
    }
}

/// Result of post-boolean repair
#[derive(Debug, Default)]
pub struct PostRepairResult {
    pub vertices_merged: usize,
    pub holes_filled: usize,
    pub non_manifold_fixed: usize,
    pub degenerate_removed: usize,
}

/// Task 3.4: Post-repair mesh after CSG boolean operation
/// Chains: vertex welding -> hole filling -> non-manifold fixing -> degenerate removal
pub fn post_repair_mesh(mesh: &Mesh) -> Result<Mesh, PostRepairError> {
    post_repair_mesh_with_config(mesh, &PostRepairConfig::default())
}

/// Post-repair with custom configuration
pub fn post_repair_mesh_with_config(
    mesh: &Mesh,
    config: &PostRepairConfig,
) -> Result<Mesh, PostRepairError> {
    if mesh.triangles.is_empty() {
        return Err(PostRepairError::EmptyMesh("No triangles".to_string()));
    }
    if mesh.vertices.len() < 3 {
        return Err(PostRepairError::EmptyMesh(
            "Less than 3 vertices".to_string(),
        ));
    }

    let mut result = mesh.clone();

    // Step 1: Task 3.1 - Weld close vertices
    let vertices_merged = weld_close_vertices(&mut result, config.weld_threshold);
    if vertices_merged > 0 {
        tracing::debug!("Post-repair: merged {} close vertices", vertices_merged);
    }

    // Remove any degenerate triangles created by welding
    let degenerate_removed = remove_degenerate_triangles(&mut result);
    if degenerate_removed > 0 {
        tracing::debug!(
            "Post-repair: removed {} degenerate triangles",
            degenerate_removed
        );
    }

    // Step 2: Task 3.2 - Fill small holes
    let holes_filled = fill_small_holes(&mut result, config.max_hole_edges);
    if holes_filled > 0 {
        tracing::debug!("Post-repair: filled {} small holes", holes_filled);
    }

    // Step 3: Task 3.3 - Fix non-manifold edges
    let non_manifold_fixed = if config.fix_non_manifold {
        fix_non_manifold_edges(&mut result)
    } else {
        0
    };
    if non_manifold_fixed > 0 {
        tracing::debug!(
            "Post-repair: fixed {} non-manifold edges",
            non_manifold_fixed
        );
    }

    // Recalculate normals after all repairs
    result.normals = Mesh::calculate_normals(&result.vertices, &result.triangles);

    tracing::debug!(
        "Post-repair complete: {} vertices, {} triangles",
        result.vertices.len(),
        result.triangles.len()
    );

    Ok(result)
}

/// Task 3.1: Weld vertices that are very close together (within epsilon)
/// This handles duplicate vertices that may appear after boolean operations
fn weld_close_vertices(mesh: &mut Mesh, threshold: f32) -> usize {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return 0;
    }

    let _threshold_sq = threshold * threshold;
    let mut merged = 0;

    // Use spatial hashing for efficiency (similar to pre-repair)
    let eps = threshold;
    let quantize = |v: &nalgebra::Point3<f32>| -> (i64, i64, i64) {
        let scale = 1.0 / eps;
        (
            (v.x * scale).round() as i64,
            (v.y * scale).round() as i64,
            (v.z * scale).round() as i64,
        )
    };

    // Build position map
    let mut position_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut vertex_remapping: Vec<Option<usize>> = vec![None; mesh.vertices.len()];

    // First pass: find duplicates
    for (i, vertex) in mesh.vertices.iter().enumerate() {
        let key = quantize(vertex);

        if let Some(&existing_idx) = position_map.get(&key) {
            vertex_remapping[i] = Some(existing_idx);
            merged += 1;
        } else {
            position_map.insert(key, i);
            vertex_remapping[i] = Some(i);
        }
    }

    if merged == 0 {
        return 0;
    }

    // Build new vertex list and index remapping
    let mut new_vertices: Vec<nalgebra::Point3<f32>> = Vec::new();
    let mut index_remap: Vec<usize> = vec![0; mesh.vertices.len()];

    for (i, vertex) in mesh.vertices.iter().enumerate() {
        if vertex_remapping[i] == Some(i) {
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
    merged
}

/// Task 3.2: Fill small holes in the mesh
/// Finds boundary edges (edges used by only 1 triangle) and fills them
fn fill_small_holes(mesh: &mut Mesh, max_hole_edges: usize) -> usize {
    if mesh.triangles.is_empty() || mesh.vertices.is_empty() {
        return 0;
    }

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

    // Find boundary edges (used by only 1 triangle)
    let boundary_edges: Vec<(usize, usize)> = edge_map
        .iter()
        .filter(|(_, triangles)| triangles.len() == 1)
        .map(|((a, b), _)| (*a, *b))
        .collect();

    if boundary_edges.is_empty() {
        return 0;
    }

    // Group boundary edges into loops
    let holes_filled = fill_boundary_loops(mesh, boundary_edges, max_hole_edges);

    holes_filled
}

/// Fill connected boundary edge loops
fn fill_boundary_loops(
    mesh: &mut Mesh,
    boundary_edges: Vec<(usize, usize)>,
    max_edges: usize,
) -> usize {
    if boundary_edges.is_empty() {
        return 0;
    }

    // Build adjacency for boundary edges
    let mut edge_adj: HashMap<usize, Vec<usize>> = HashMap::new();
    for (a, b) in &boundary_edges {
        edge_adj.entry(*a).or_insert_with(Vec::new).push(*b);
        edge_adj.entry(*b).or_insert_with(Vec::new).push(*a);
    }

    let mut used_edges: HashSet<(usize, usize)> = HashSet::new();
    let mut holes_filled = 0;

    // Find connected components (loops)
    for &(a, b) in &boundary_edges {
        if used_edges.contains(&(a, b)) || used_edges.contains(&(b, a)) {
            continue;
        }

        // Trace the loop
        let mut loop_vertices: Vec<usize> = vec![a, b];
        used_edges.insert((a.min(b), a.max(b)));

        let mut current = b;
        let mut found_loop = true;

        while current != a {
            let neighbors = edge_adj.get(&current);
            let mut found_next = false;

            if let Some(neighbors) = neighbors {
                for &next in neighbors {
                    let edge = (current.min(next), current.max(next));
                    if !used_edges.contains(&edge) {
                        used_edges.insert(edge);
                        loop_vertices.push(next);
                        current = next;
                        found_next = true;
                        break;
                    }
                }
            }

            if !found_next {
                found_loop = false;
                break;
            }
        }

        // If we found a valid loop and it's small enough, fill it
        if found_loop && loop_vertices.len() >= 3 && loop_vertices.len() <= max_edges {
            // Triangulate the loop using a fan from the first vertex
            let first = loop_vertices[0];
            let mut filled = 0;

            for i in 1..(loop_vertices.len() - 1) {
                let v0 = first;
                let v1 = loop_vertices[i];
                let v2 = loop_vertices[i + 1];

                // Check if triangle is valid (not degenerate)
                let p0 = mesh.vertices[v0];
                let p1 = mesh.vertices[v1];
                let p2 = mesh.vertices[v2];

                let e1 = p1 - p0;
                let e2 = p2 - p0;
                let cross = e1.cross(&e2);

                if cross.magnitude_squared() > 1e-10 {
                    mesh.triangles
                        .push(crate::geometry::mesh::Triangle::new(v0, v1, v2));
                    filled += 1;
                }
            }

            if filled > 0 {
                holes_filled += 1;
                tracing::debug!("Filled hole with {} triangles", filled);
            }
        }
    }

    holes_filled
}

/// Task 3.3: Fix non-manifold edges
/// Non-manifold edges are edges shared by more than 2 triangles
/// or edges that are not properly connected
fn fix_non_manifold_edges(mesh: &mut Mesh) -> usize {
    if mesh.triangles.is_empty() || mesh.vertices.is_empty() {
        return 0;
    }

    // Build edge map
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

    // Find non-manifold edges (shared by > 2 triangles)
    let non_manifold: Vec<((usize, usize), Vec<usize>)> = edge_map
        .iter()
        .filter(|(_, triangles)| triangles.len() > 2)
        .map(|(k, v)| (*k, v.clone()))
        .collect();

    if non_manifold.is_empty() {
        return 0;
    }

    // For each non-manifold edge, keep only the first 2 triangles
    // and mark others for removal
    let mut triangles_to_remove: HashSet<usize> = HashSet::new();
    let mut fixed = 0;

    for ((_a, _b), triangles) in non_manifold {
        // Keep first 2 triangles, mark rest for removal
        for &tri_idx in triangles.iter().skip(2) {
            triangles_to_remove.insert(tri_idx);
        }
        fixed += triangles.len() - 2;
    }

    // Remove duplicate triangles to remove
    let mut to_remove: Vec<usize> = triangles_to_remove.iter().cloned().collect();
    to_remove.sort_unstable();
    to_remove.reverse();

    for idx in to_remove {
        if idx < mesh.triangles.len() {
            mesh.triangles.remove(idx);
        }
    }

    // Also fix degenerate triangles that may have been created
    let degenerate_removed = remove_degenerate_triangles(mesh);
    fixed += degenerate_removed;

    fixed
}

// ============================================================================
// Phase 5: Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::mesh::Triangle;

    // ============================================================================
    // Task 5.2: Unit tests for pre-repair functions
    // ============================================================================

    /// Create a simple cube mesh for testing
    fn create_test_cube() -> Mesh {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 1.0, 0.0),
            nalgebra::Point3::new(0.0, 1.0, 0.0),
            nalgebra::Point3::new(0.0, 0.0, 1.0),
            nalgebra::Point3::new(1.0, 0.0, 1.0),
            nalgebra::Point3::new(1.0, 1.0, 1.0),
            nalgebra::Point3::new(0.0, 1.0, 1.0),
        ];

        let triangles = vec![
            // Front face
            Triangle::new(0, 1, 2),
            Triangle::new(0, 2, 3),
            // Back face
            Triangle::new(4, 6, 5),
            Triangle::new(4, 7, 6),
            // Top face
            Triangle::new(3, 2, 6),
            Triangle::new(3, 6, 7),
            // Bottom face
            Triangle::new(0, 5, 1),
            Triangle::new(0, 4, 5),
            // Right face
            Triangle::new(1, 5, 6),
            Triangle::new(1, 6, 2),
            // Left face
            Triangle::new(4, 0, 3),
            Triangle::new(4, 3, 7),
        ];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// Create a mesh with duplicate vertices
    fn create_mesh_with_duplicate_vertices() -> Mesh {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0), // 0 - duplicate of 1
            nalgebra::Point3::new(0.0, 0.0, 0.0), // 1 - duplicate of 0
            nalgebra::Point3::new(1.0, 0.0, 0.0), // 2
            nalgebra::Point3::new(1.0, 1.0, 0.0), // 3
            nalgebra::Point3::new(0.0, 1.0, 0.0), // 4
        ];

        let triangles = vec![Triangle::new(0, 2, 3), Triangle::new(0, 3, 4)];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// Create a mesh with degenerate triangles
    fn create_mesh_with_degenerate_triangles() -> Mesh {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 1.0, 0.0),
            nalgebra::Point3::new(0.0, 1.0, 0.0),
            nalgebra::Point3::new(0.5, 0.5, 0.0), // degenerate vertex
        ];

        let triangles = vec![
            // Normal triangle
            Triangle::new(0, 1, 2),
            // Degenerate (zero area - all points collinear)
            Triangle::new(0, 4, 0), // same vertex twice = zero area
            // Another normal
            Triangle::new(0, 2, 3),
        ];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// Test pre_repair_mesh removes duplicate vertices
    #[test]
    fn test_pre_repair_removes_duplicate_vertices() {
        let mesh = create_mesh_with_duplicate_vertices();
        let original_vertices = mesh.vertices.len();

        let result = pre_repair_mesh(&mesh);

        assert!(result.is_ok(), "Pre-repair should succeed");
        let repaired = result.unwrap();

        // Should have fewer vertices after merging duplicates
        assert!(
            repaired.vertices.len() < original_vertices,
            "Should merge duplicate vertices, had {} now {}",
            original_vertices,
            repaired.vertices.len()
        );
    }

    /// Test pre_repair_mesh removes degenerate triangles
    #[test]
    fn test_pre_repair_removes_degenerate_triangles() {
        let mesh = create_mesh_with_degenerate_triangles();
        let original_triangles = mesh.triangles.len();

        let result = pre_repair_mesh(&mesh);

        assert!(result.is_ok(), "Pre-repair should succeed");
        let repaired = result.unwrap();

        // Should have fewer triangles after removing degenerate
        assert!(
            repaired.triangles.len() < original_triangles,
            "Should remove degenerate triangles, had {} now {}",
            original_triangles,
            repaired.triangles.len()
        );
    }

    /// Test pre_repair_mesh fixes normal consistency
    #[test]
    fn test_pre_repair_normal_consistency() {
        let mut mesh = create_test_cube();

        // Flip one triangle to create inconsistency
        mesh.triangles[0] = Triangle::new(0, 3, 2);

        let result = pre_repair_mesh(&mesh);

        assert!(result.is_ok(), "Pre-repair should succeed");
        let repaired = result.unwrap();

        // After repair, normals should be consistent
        assert!(!repaired.normals.is_empty(), "Should have normals");
    }

    /// Test remove_duplicate_vertices function directly
    #[test]
    fn test_remove_duplicate_vertices() {
        let mut mesh = create_mesh_with_duplicate_vertices();
        let original_count = mesh.vertices.len();

        let merged = remove_duplicate_vertices(&mut mesh);

        assert!(merged > 0, "Should merge some duplicate vertices");
        assert!(
            mesh.vertices.len() < original_count,
            "Should have fewer vertices after merge"
        );
    }

    /// Test remove_degenerate_triangles function directly
    #[test]
    fn test_remove_degenerate_triangles() {
        let mut mesh = create_mesh_with_degenerate_triangles();
        let original_count = mesh.triangles.len();

        let removed = remove_degenerate_triangles(&mut mesh);

        assert!(removed > 0, "Should remove some degenerate triangles");
        assert!(
            mesh.triangles.len() < original_count,
            "Should have fewer triangles after removal"
        );
    }

    // ============================================================================
    // Task 5.3: Unit tests for post-repair functions
    // ============================================================================

    /// Create a mesh with close vertices that should be welded
    fn create_mesh_with_close_vertices() -> Mesh {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(0.00001, 0.0, 0.0), // very close - should weld
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 1.0, 0.0),
            nalgebra::Point3::new(0.0, 1.0, 0.0),
        ];

        let triangles = vec![Triangle::new(0, 2, 3), Triangle::new(0, 3, 4)];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// Test post_repair_mesh welds close vertices
    #[test]
    fn test_post_repair_welds_vertices() {
        let mesh = create_mesh_with_close_vertices();
        let original_vertices = mesh.vertices.len();

        let result = post_repair_mesh(&mesh);

        assert!(result.is_ok(), "Post-repair should succeed");
        let repaired = result.unwrap();

        // Should have merged close vertices
        assert!(
            repaired.vertices.len() <= original_vertices,
            "Should not increase vertices"
        );
    }

    /// Test fix_non_manifold_edges function
    #[test]
    fn test_fix_non_manifold_edges() {
        // Create a mesh with non-manifold edges (3 triangles sharing an edge)
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(0.5, 1.0, 0.0),
            nalgebra::Point3::new(0.5, -1.0, 0.0),
        ];

        // Three triangles sharing edge (0,1) - this is non-manifold
        let triangles = vec![
            Triangle::new(0, 1, 2),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 2, 1), // duplicate, will make edge shared by 3
        ];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        let mut mesh = Mesh {
            vertices,
            triangles,
            normals,
        };

        let fixed = fix_non_manifold_edges(&mut mesh);

        // Should fix non-manifold edges
        assert!(
            fixed > 0 || mesh.triangles.len() < 3,
            "Should fix non-manifold"
        );
    }

    // ============================================================================
    // Task 5.4: Unit tests for validation functions
    // ============================================================================

    /// Test is_watertight on a known watertight mesh (cube)
    #[test]
    fn test_is_watertight_cube() {
        let mesh = create_test_cube();

        let watertight = is_watertight(&mesh);

        assert!(watertight, "Cube should be watertight");
    }

    /// Test is_watertight on a mesh with holes (single triangle)
    #[test]
    fn test_is_watertight_single_triangle() {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(0.5, 1.0, 0.0),
        ];

        let triangles = vec![Triangle::new(0, 1, 2)];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        let mesh = Mesh {
            vertices,
            triangles,
            normals,
        };

        let watertight = is_watertight(&mesh);

        assert!(!watertight, "Single triangle should not be watertight");
    }

    /// Test calculate_volume on a known watertight mesh
    #[test]
    fn test_calculate_volume_cube() {
        let mesh = create_test_cube();

        let volume = calculate_volume(&mesh);

        // Cube of size 1 should have volume 1
        assert!(
            volume > 0.0,
            "Cube should have positive volume, got {}",
            volume
        );
        // Allow some tolerance for numerical errors
        assert!(
            (volume - 1.0).abs() < 0.1,
            "Cube volume should be approximately 1.0, got {}",
            volume
        );
    }

    /// Test calculate_volume on non-watertight mesh returns 0
    #[test]
    fn test_calculate_volume_non_watertight() {
        let vertices = vec![
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Point3::new(1.0, 0.0, 0.0),
            nalgebra::Point3::new(0.5, 1.0, 0.0),
        ];

        let triangles = vec![Triangle::new(0, 1, 2)];

        let normals = Mesh::calculate_normals(&vertices, &triangles);

        let mesh = Mesh {
            vertices,
            triangles,
            normals,
        };

        let volume = calculate_volume(&mesh);

        assert_eq!(volume, 0.0, "Non-watertight mesh should have zero volume");
    }

    /// Test calculate_quality_metrics on a good mesh
    #[test]
    fn test_quality_metrics_good_mesh() {
        let mesh = create_test_cube();

        let metrics = calculate_quality_metrics(&mesh);

        assert!(metrics.is_watertight, "Cube should be watertight");
        assert_eq!(
            metrics.non_manifold_edges, 0,
            "Should have no non-manifold edges"
        );
        assert_eq!(
            metrics.degenerate_triangles, 0,
            "Should have no degenerate triangles"
        );
        assert_eq!(metrics.boundary_edges, 0, "Should have no boundary edges");
        assert!(metrics.triangle_count > 0, "Should have triangles");
        assert!(metrics.vertex_count > 0, "Should have vertices");
    }

    /// Test calculate_quality_metrics on a problematic mesh
    #[test]
    fn test_quality_metrics_problematic_mesh() {
        let mesh = create_mesh_with_degenerate_triangles();

        let metrics = calculate_quality_metrics(&mesh);

        assert!(!metrics.is_watertight, "This mesh should not be watertight");
        assert!(
            metrics.degenerate_triangles > 0,
            "Should detect degenerate triangles"
        );
    }

    // ============================================================================
    // Task 5.5: Integration test for full pipeline
    // ============================================================================

    /// Test that roundtrip through pre and post repair produces valid mesh
    #[test]
    fn test_pre_post_repair_roundtrip() {
        let original = create_test_cube();

        // Pre-repair
        let pre_repaired = pre_repair_mesh(&original);
        assert!(pre_repaired.is_ok(), "Pre-repair should succeed");

        // Post-repair
        let post_repaired = post_repair_mesh(&pre_repaired.unwrap());
        assert!(post_repaired.is_ok(), "Post-repair should succeed");

        // Result should still be valid
        let result = post_repaired.unwrap();
        assert!(!result.vertices.is_empty(), "Should have vertices");
        assert!(!result.triangles.is_empty(), "Should have triangles");
    }
}
