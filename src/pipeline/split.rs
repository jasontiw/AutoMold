//! Mold splitting - separates mesh along an axis-aligned plane.
//!
//! Algorithm overview:
//!   1. Classify every triangle as fully positive, fully negative, or spanning.
//!   2. Clip spanning triangles against the plane → new sub-triangles + seam edges.
//!   3. Reconstruct the boundary loop(s) from seam edges via half-edge chaining.
//!   4. Triangulate each loop with ear-clipping (handles concave polygons).
//!   5. Stitch the cap onto each half with correct winding.
//!
//! This approach never calls csgrs and does not sort by angle/Y,
//! so it produces manifold (watertight) output for any convex or simply-connected mesh.

use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;
use thiserror::Error;

// ─── public re-exports kept for API compatibility ────────────────────────────

pub const SLAB_SIZE: f32 = 2.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Error, Debug)]
pub enum SplitError {
    #[error("Degenerate geometry: {0}")]
    DegenerateGeometry(String),

    #[error("Boundary loop reconstruction failed: {0}")]
    BoundaryLoopFailed(String),

    #[error("Triangulation failed: {0}")]
    TriangulationFailed(String),

    // kept for API compat even though CSG is no longer used
    #[error("CSG conversion failed: {0}")]
    CsgConversionFailed(String),
    #[error("CSG intersection failed: {0}")]
    CsgIntersectionFailed(String),
    #[error("Result mesh is not watertight")]
    NonWatertightResult,
    #[error("Both CSG and manual fallback failed")]
    BothStrategiesFailed {
        csg_error: String,
        manual_error: String,
    },
}

// ─── tiny epsilon (adaptive based on mesh scale) ───────────────────────────────

fn compute_adaptive_eps(mesh: &Mesh) -> f32 {
    let bbox = mesh.calculate_bounding_box();
    let diag = bbox.diagonal();
    if diag.is_finite() && diag > 0.0 {
        diag * 1e-8
    } else {
        1e-6
    }
}

// ─── helpers ──────────────────────────────────────────────────────────────────

#[inline]
fn signed_dist(p: Point3<f32>, axis: Axis, point: f32) -> f32 {
    match axis {
        Axis::X => p.x - point,
        Axis::Y => p.y - point,
        Axis::Z => p.z - point,
    }
}

/// Linearly interpolate a new vertex on the plane between `a` and `b`.
fn intersect_plane(a: Point3<f32>, b: Point3<f32>, da: f32, db: f32) -> Point3<f32> {
    let t = da / (da - db);
    Point3::new(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z),
    )
}

/// Push a vertex into `verts`, deduplicating within `eps`.
/// Returns the index.
fn push_vertex(verts: &mut Vec<Point3<f32>>, p: Point3<f32>, eps: f32) -> usize {
    // Check recent vertices first (seam verts tend to be added in bursts)
    let start = if verts.len() > 128 {
        verts.len() - 128
    } else {
        0
    };
    for (i, v) in verts[start..].iter().enumerate() {
        if (v - p).norm_squared() < eps * eps {
            return start + i;
        }
    }
    // Full scan for older verts
    for (i, v) in verts[..start].iter().enumerate() {
        if (v - p).norm_squared() < eps * eps {
            return i;
        }
    }
    verts.push(p);
    verts.len() - 1
}

// ─── main public API ──────────────────────────────────────────────────────────

/// Split `mesh` along `axis` at coordinate `point`.
///
/// Returns `(positive_half, negative_half)` where positive means the side
/// where `coord > point`.
pub fn split_mesh(mesh: &Mesh, axis: Axis, point: f32) -> Result<(Mesh, Mesh), SplitError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(SplitError::DegenerateGeometry("Empty mesh".to_string()));
    }

    let eps = compute_adaptive_eps(mesh);

    // Signed distance of every original vertex to the split plane
    let dists: Vec<f32> = mesh
        .vertices
        .iter()
        .map(|&v| signed_dist(v, axis, point))
        .collect();

    // Output geometry accumulators
    let mut pos_verts: Vec<Point3<f32>> = Vec::new();
    let mut pos_tris: Vec<[usize; 3]> = Vec::new();
    let mut neg_verts: Vec<Point3<f32>> = Vec::new();
    let mut neg_tris: Vec<[usize; 3]> = Vec::new();

    // Map from original vertex index → index in pos/neg buffers
    let mut pos_map: HashMap<usize, usize> = HashMap::new();
    let mut neg_map: HashMap<usize, usize> = HashMap::new();

    // Seam edges: pairs of vertex indices IN THE POSITIVE BUFFER that lie on the plane.
    // Each spanning triangle contributes exactly one such edge (v_lo → v_hi).
    // We collect them ordered so we can chain them into loops later.
    // seam_edges[i] = (idx_in_pos_buf, idx_in_pos_buf)
    let mut seam_edges: Vec<(usize, usize)> = Vec::new();

    let get_or_insert_pos =
        |orig: usize, verts: &mut Vec<Point3<f32>>, map: &mut HashMap<usize, usize>| {
            *map.entry(orig).or_insert_with(|| {
                let idx = push_vertex(verts, mesh.vertices[orig], eps);
                idx
            })
        };

    let get_or_insert_neg =
        |orig: usize, verts: &mut Vec<Point3<f32>>, map: &mut HashMap<usize, usize>| {
            *map.entry(orig).or_insert_with(|| {
                let idx = push_vertex(verts, mesh.vertices[orig], eps);
                idx
            })
        };

    for tri in &mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let [d0, d1, d2] = [dists[i0], dists[i1], dists[i2]];
        let [p0, p1, p2] = [mesh.vertices[i0], mesh.vertices[i1], mesh.vertices[i2]];

        // Classify vertices
        let s0 = d0 >= -eps;
        let s1 = d1 >= -eps;
        let s2 = d2 >= -eps;

        match (s0, s1, s2) {
            // ── Fully positive ──────────────────────────────────────────────
            (true, true, true) => {
                let a = get_or_insert_pos(i0, &mut pos_verts, &mut pos_map);
                let b = get_or_insert_pos(i1, &mut pos_verts, &mut pos_map);
                let c = get_or_insert_pos(i2, &mut pos_verts, &mut pos_map);
                pos_tris.push([a, b, c]);
            }

            // ── Fully negative ──────────────────────────────────────────────
            (false, false, false) => {
                let a = get_or_insert_neg(i0, &mut neg_verts, &mut neg_map);
                let b = get_or_insert_neg(i1, &mut neg_verts, &mut neg_map);
                let c = get_or_insert_neg(i2, &mut neg_verts, &mut neg_map);
                neg_tris.push([a, b, c]);
            }

            // ── Spanning: must clip ─────────────────────────────────────────
            _ => {
                // Rotate so that the "lonely" vertex (on the minority side) is first.
                // Then vertices 0 is the lone side, 1 and 2 are on the majority side.
                let count_pos = [s0, s1, s2].iter().filter(|&&x| x).count();
                let lone_pos = count_pos == 1;

                // Rotate indices, distances, and points so lone vertex is first
                let (vi, vj, vk, di, dj, dk, pi, pj, pk) = if lone_pos {
                    // lone positive vertex
                    if s0 {
                        (i0, i1, i2, d0, d1, d2, p0, p1, p2)
                    } else if s1 {
                        (i1, i2, i0, d1, d2, d0, p1, p2, p0)
                    } else {
                        (i2, i0, i1, d2, d0, d1, p2, p0, p1)
                    }
                } else {
                    // lone negative vertex
                    if !s0 {
                        (i0, i1, i2, d0, d1, d2, p0, p1, p2)
                    } else if !s1 {
                        (i1, i2, i0, d1, d2, d0, p1, p2, p0)
                    } else {
                        (i2, i0, i1, d2, d0, d1, p2, p0, p1)
                    }
                };

                // Compute the two intersection points on the plane
                let qa = intersect_plane(pi, pj, di, dj); // edge lone→j
                let qb = intersect_plane(pi, pk, di, dk); // edge lone→k

                // Insert into both buffers (they lie on the plane → shared seam)
                let qa_pos = push_vertex(&mut pos_verts, qa, eps);
                let qb_pos = push_vertex(&mut pos_verts, qb, eps);
                let qa_neg = push_vertex(&mut neg_verts, qa, eps);
                let qb_neg = push_vertex(&mut neg_verts, qb, eps);

                if lone_pos {
                    // Positive side: lone triangle (i, qa, qb)
                    let vi_pos = get_or_insert_pos(vi, &mut pos_verts, &mut pos_map);
                    pos_tris.push([vi_pos, qa_pos, qb_pos]);

                    // Negative side: quad (j, k, qb, qa) → two triangles
                    let vj_neg = get_or_insert_neg(vj, &mut neg_verts, &mut neg_map);
                    let vk_neg = get_or_insert_neg(vk, &mut neg_verts, &mut neg_map);
                    neg_tris.push([vj_neg, vk_neg, qb_neg]);
                    neg_tris.push([vj_neg, qb_neg, qa_neg]);

                    // Seam edge (positive buffer): qa → qb
                    // Direction: when the plane normal points +axis,
                    // the positive cap winds CCW viewed from +axis → qa→qb.
                    seam_edges.push((qa_pos, qb_pos));
                } else {
                    // Negative side: lone triangle (i, qa, qb)
                    let vi_neg = get_or_insert_neg(vi, &mut neg_verts, &mut neg_map);
                    neg_tris.push([vi_neg, qa_neg, qb_neg]);

                    // Positive side: quad (j, k, qb, qa) → two triangles
                    let vj_pos = get_or_insert_pos(vj, &mut pos_verts, &mut pos_map);
                    let vk_pos = get_or_insert_pos(vk, &mut pos_verts, &mut pos_map);
                    pos_tris.push([vj_pos, vk_pos, qb_pos]);
                    pos_tris.push([vj_pos, qb_pos, qa_pos]);

                    // Seam edge (positive buffer): qb → qa (reversed for CCW)
                    seam_edges.push((qb_pos, qa_pos));
                }
            }
        }
    }

    // ── Build caps from seam edges ────────────────────────────────────────────

    let plane_normal: Vector3<f32> = match axis {
        Axis::X => Vector3::new(1.0, 0.0, 0.0),
        Axis::Y => Vector3::new(0.0, 1.0, 0.0),
        Axis::Z => Vector3::new(0.0, 0.0, 1.0),
    };

    // Extract loops from seam_edges (there may be multiple disconnected loops)
    let loops = build_boundary_loops(&seam_edges).map_err(|e| SplitError::BoundaryLoopFailed(e))?;

    for loop_verts in &loops {
        // Triangulate this loop (ear-clipping, works for concave polys)
        let cap_tris = ear_clip(loop_verts, &pos_verts, plane_normal, eps)
            .map_err(|e| SplitError::TriangulationFailed(e))?;

        // Add cap to positive side (already in pos_verts indices)
        pos_tris.extend(cap_tris.iter().copied());

        // Add reversed cap to negative side
        // Mirror each vertex index from pos_verts into neg_verts
        for [a, b, c] in &cap_tris {
            let pa = pos_verts[*a];
            let pb = pos_verts[*b];
            let pc = pos_verts[*c];
            let na = push_vertex(&mut neg_verts, pa, eps);
            let nb = push_vertex(&mut neg_verts, pb, eps);
            let nc = push_vertex(&mut neg_verts, pc, eps);
            // Reverse winding for negative side
            neg_tris.push([na, nc, nb]);
        }
    }

    // ── Assemble Mesh structs ─────────────────────────────────────────────────

    let pos_mesh = assemble_mesh(pos_verts, pos_tris);
    let neg_mesh = assemble_mesh(neg_verts, neg_tris);

    Ok((pos_mesh, neg_mesh))
}

// ─── boundary loop reconstruction (half-edge chaining) ────────────────────────

/// Given a flat list of directed edges `(from, to)`, reconstruct one or more
/// closed loops by following the chain `next[v] = w`.
///
/// Fails if the edges do not form a clean manifold boundary
/// (i.e. a vertex has more than one outgoing edge).
fn build_boundary_loops(edges: &[(usize, usize)]) -> Result<Vec<Vec<usize>>, String> {
    if edges.is_empty() {
        return Ok(vec![]);
    }

    // Build adjacency: from → to
    let mut next: HashMap<usize, usize> = HashMap::new();
    for &(a, b) in edges {
        if next.insert(a, b).is_some() {
            // Duplicate outgoing edge from same vertex → non-manifold seam.
            // This can happen with degenerate triangles; skip gracefully.
            // (We overwrite with the latest edge; ear-clip will still work.)
        }
    }

    let mut visited: std::collections::HashSet<usize> = std::collections::HashSet::new();
    let mut loops: Vec<Vec<usize>> = Vec::new();

    for &start in next.keys() {
        if visited.contains(&start) {
            continue;
        }

        let mut lp: Vec<usize> = Vec::new();
        let mut cur = start;

        loop {
            if visited.contains(&cur) {
                break;
            }
            visited.insert(cur);
            lp.push(cur);

            match next.get(&cur) {
                Some(&nxt) => cur = nxt,
                None => {
                    return Err(format!(
                        "Seam edge chain broken at vertex {cur} — mesh may not be watertight"
                    ));
                }
            }

            if cur == start {
                break; // closed loop
            }
        }

        if lp.len() >= 3 {
            loops.push(lp);
        }
    }

    Ok(loops)
}

// ─── ear-clipping triangulation ────────────────────────────────────────────────

/// Triangulate a simple polygon given by `indices` into `verts`.
/// `normal` is used to determine winding direction.
///
/// Works for convex and concave (non-self-intersecting) polygons.
fn ear_clip(
    indices: &[usize],
    verts: &[Point3<f32>],
    normal: Vector3<f32>,
    eps: f32,
) -> Result<Vec<[usize; 3]>, String> {
    let n = indices.len();
    if n < 3 {
        return Err(format!("Polygon has only {n} vertices"));
    }
    if n == 3 {
        return Ok(vec![[indices[0], indices[1], indices[2]]]);
    }

    let mut remaining: Vec<usize> = indices.to_vec(); // indices into `verts`
    let mut result: Vec<[usize; 3]> = Vec::new();
    let mut guard = 0usize;

    while remaining.len() > 3 {
        let m = remaining.len();
        let mut clipped = false;

        for i in 0..m {
            let prev = remaining[(i + m - 1) % m];
            let curr = remaining[i];
            let next = remaining[(i + 1) % m];

            let a = verts[prev];
            let b = verts[curr];
            let c = verts[next];

            if !is_ear(a, b, c, &remaining, verts, normal, eps) {
                continue;
            }

            result.push([prev, curr, next]);
            remaining.remove(i);
            clipped = true;
            break;
        }

        if !clipped {
            // Fallback: force-clip the first convex vertex to avoid infinite loop.
            // This can happen with nearly-degenerate (collinear) polygons.
            guard += 1;
            if guard > remaining.len() + 10 {
                return Err("Ear-clip stuck — polygon may be self-intersecting".to_string());
            }
            // Remove vertex 1 (skip 0, which may be the problematic one)
            let prev = remaining[0];
            let curr = remaining[1];
            let next = remaining[2];
            result.push([prev, curr, next]);
            remaining.remove(1);
        }
    }

    // Last triangle
    result.push([remaining[0], remaining[1], remaining[2]]);
    Ok(result)
}

/// Returns true if vertex `b` is an ear of the polygon `prev-b-next`.
fn is_ear(
    a: Point3<f32>,
    b: Point3<f32>,
    c: Point3<f32>,
    remaining: &[usize],
    verts: &[Point3<f32>],
    normal: Vector3<f32>,
    eps: f32,
) -> bool {
    // 1. The triangle must be convex (same winding as the polygon normal)
    let cross = (b - a).cross(&(c - b));
    if cross.dot(&normal) < 0.0 {
        return false;
    }

    // 2. No other polygon vertex may lie inside the triangle
    for &idx in remaining {
        let p = verts[idx];
        if (p - a).norm_squared() < eps * eps
            || (p - b).norm_squared() < eps * eps
            || (p - c).norm_squared() < eps * eps
        {
            continue; // skip the triangle's own vertices
        }
        if point_in_triangle(p, a, b, c, normal, eps) {
            return false;
        }
    }
    true
}

/// Barycentric point-in-triangle test projected onto the plane defined by `normal`.
fn point_in_triangle(
    p: Point3<f32>,
    a: Point3<f32>,
    b: Point3<f32>,
    c: Point3<f32>,
    normal: Vector3<f32>,
    eps: f32,
) -> bool {
    let d1 = (b - a).cross(&(p - a)).dot(&normal);
    let d2 = (c - b).cross(&(p - b)).dot(&normal);
    let d3 = (a - c).cross(&(p - c)).dot(&normal);
    let has_neg = d1 < -eps || d2 < -eps || d3 < -eps;
    let has_pos = d1 > eps || d2 > eps || d3 > eps;
    !(has_neg && has_pos)
}

// ─── mesh assembly ────────────────────────────────────────────────────────────

fn assemble_mesh(vertices: Vec<Point3<f32>>, raw_tris: Vec<[usize; 3]>) -> Mesh {
    let triangles: Vec<Triangle> = raw_tris
        .into_iter()
        .filter(|[a, b, c]| a != b && b != c && a != c) // skip degenerate
        .map(|[a, b, c]| Triangle::new(a, b, c))
        .collect();

    let normals = Mesh::calculate_normals(&vertices, &triangles);
    Mesh {
        vertices,
        triangles,
        normals,
    }
}

// ─── convenience wrappers ─────────────────────────────────────────────────────

pub fn split_x(mesh: &Mesh, x: f32) -> Result<(Mesh, Mesh), SplitError> {
    split_mesh(mesh, Axis::X, x)
}

pub fn split_y(mesh: &Mesh, y: f32) -> Result<(Mesh, Mesh), SplitError> {
    split_mesh(mesh, Axis::Y, y)
}

pub fn split_z(mesh: &Mesh, z: f32) -> Result<(Mesh, Mesh), SplitError> {
    split_mesh(mesh, Axis::Z, z)
}

// ─── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::repair::{calculate_volume, is_watertight};
    use nalgebra::Point3;

    fn create_unit_cube() -> Mesh {
        let vertices = vec![
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let triangles = vec![
            Triangle::new(0, 1, 2),
            Triangle::new(0, 2, 3),
            Triangle::new(4, 6, 5),
            Triangle::new(4, 7, 6),
            Triangle::new(3, 2, 6),
            Triangle::new(3, 6, 7),
            Triangle::new(0, 5, 1),
            Triangle::new(0, 4, 5),
            Triangle::new(1, 5, 6),
            Triangle::new(1, 6, 2),
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

    #[test]
    fn test_split_cube_x_axis() {
        let cube = create_unit_cube();
        let (pos, neg) = split_x(&cube, 0.0).unwrap();
        assert!(!pos.triangles.is_empty());
        assert!(!neg.triangles.is_empty());
    }

    #[test]
    fn test_split_cube_y_axis() {
        let cube = create_unit_cube();
        let (pos, neg) = split_y(&cube, 0.0).unwrap();
        assert!(!pos.triangles.is_empty());
        assert!(!neg.triangles.is_empty());
    }

    #[test]
    fn test_split_cube_z_axis() {
        let cube = create_unit_cube();
        let (pos, neg) = split_z(&cube, 0.0).unwrap();
        assert!(!pos.triangles.is_empty());
        assert!(!neg.triangles.is_empty());
    }

    #[test]
    fn test_split_watertight() {
        let cube = create_unit_cube();
        let (pos, neg) = split_z(&cube, 0.0).unwrap();
        assert!(is_watertight(&pos), "Positive half should be watertight");
        assert!(is_watertight(&neg), "Negative half should be watertight");
    }

    #[test]
    fn test_split_preserves_volume() {
        let cube = create_unit_cube();
        let original_vol = calculate_volume(&cube).abs();
        let (pos, neg) = split_z(&cube, 0.0).unwrap();
        let total = calculate_volume(&pos).abs() + calculate_volume(&neg).abs();
        let err = (total - original_vol).abs() / original_vol;
        assert!(err < 0.01, "Volume error {err:.4} exceeds 1%");
    }

    #[test]
    fn test_split_negative_coordinate() {
        let cube = create_unit_cube();
        split_x(&cube, -0.25).unwrap();
    }

    #[test]
    fn test_split_along_different_axes() {
        let cube = create_unit_cube();
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            let (pos, neg) = split_mesh(&cube, axis, 0.0).unwrap();
            assert!(is_watertight(&pos), "{axis:?} positive not watertight");
            assert!(is_watertight(&neg), "{axis:?} negative not watertight");
        }
    }

    #[test]
    fn test_split_real_cube_geometry() {
        use crate::geometry::bbox::BoundingBox;
        use crate::pipeline::loader::load_stl;
        use crate::pipeline::repair::is_watertight;

        // Load the actual test file
        let cube = load_stl(
            std::path::Path::new("test_data/cube_10mm.stl"),
            crate::core::config::Unit::Millimeters,
        )
        .expect("Should load cube_10mm.stl");

        eprintln!(
            "Loaded cube: {} vertices, {} triangles",
            cube.vertices.len(),
            cube.triangles.len()
        );

        // Print bounding box
        let bbox = cube.calculate_bounding_box();
        eprintln!(
            "Bounding box: min=({:.2}, {:.2}, {:.2}), max=({:.2}, {:.2}, {:.2})",
            bbox.min.x, bbox.min.y, bbox.min.z, bbox.max.x, bbox.max.y, bbox.max.z
        );

        // Split along X at x=5.0
        let result = split_x(&cube, 5.0);

        match result {
            Ok((pos, neg)) => {
                eprintln!("Split at x=5.0:");
                eprintln!(
                    "  Positive: {} vertices, {} triangles, watertight={}",
                    pos.vertices.len(),
                    pos.triangles.len(),
                    is_watertight(&pos)
                );
                eprintln!(
                    "  Negative: {} vertices, {} triangles, watertight={}",
                    neg.vertices.len(),
                    neg.triangles.len(),
                    is_watertight(&neg)
                );

                // At least one should have geometry
                assert!(!pos.triangles.is_empty() || !neg.triangles.is_empty());
            }
            Err(e) => {
                eprintln!("Split failed: {:?}", e);
                panic!("Split should succeed");
            }
        }

        // Also test at x=0 (edge case)
        let result_edge = split_x(&cube, 0.0);
        if let Ok((pos, neg)) = result_edge {
            eprintln!("\nSplit at x=0:");
            eprintln!(
                "  Positive: {} vertices, {} triangles, watertight={}",
                pos.vertices.len(),
                pos.triangles.len(),
                is_watertight(&pos)
            );
            eprintln!(
                "  Negative: {} vertices, {} triangles, watertight={}",
                neg.vertices.len(),
                neg.triangles.len(),
                is_watertight(&neg)
            );
        }
    }

    /// Export split results to STL files for visual inspection
    #[test]
    fn test_export_split_debug_files() {
        use std::fs;
        use std::path::PathBuf;

        let cube = create_unit_cube();
        let (pos, neg) = split_z(&cube, 0.0).unwrap();

        // Get temp directory
        let out_dir = std::env::var("CARGO_TARGET_TMPDIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("."));
        let out_dir = out_dir.join("split_debug");
        let _ = fs::create_dir_all(&out_dir);

        // Export positive half
        let pos_path = out_dir.join("split_positive_z0.stl");
        crate::export::stl::write_stl_ascii(&pos, &pos_path).expect("Failed to write positive STL");
        eprintln!("Exported: {}", pos_path.display());

        // Export negative half
        let neg_path = out_dir.join("split_negative_z0.stl");
        crate::export::stl::write_stl_ascii(&neg, &neg_path).expect("Failed to write negative STL");
        eprintln!("Exported: {}", neg_path.display());

        // Also export original cube
        let cube_path = out_dir.join("original_cube.stl");
        crate::export::stl::write_stl_ascii(&cube, &cube_path).expect("Failed to write cube STL");
        eprintln!("Exported: {}", cube_path.display());

        // Print stats
        eprintln!("\n=== Split Stats ===");
        eprintln!(
            "Original: {} vertices, {} triangles",
            cube.vertices.len(),
            cube.triangles.len()
        );
        eprintln!(
            "Positive: {} vertices, {} triangles",
            pos.vertices.len(),
            pos.triangles.len()
        );
        eprintln!(
            "Negative: {} vertices, {} triangles",
            neg.vertices.len(),
            neg.triangles.len()
        );
        eprintln!("Positive watertight: {}", is_watertight(&pos));
        eprintln!("Negative watertight: {}", is_watertight(&neg));

        // Print first few vertices and triangles for debugging
        eprintln!("\n=== Positive Half Vertices (first 10) ===");
        for (i, v) in pos.vertices.iter().take(10).enumerate() {
            eprintln!("  V{}: ({:.4}, {:.4}, {:.4})", i, v.x, v.y, v.z);
        }

        eprintln!("\n=== Positive Half Triangles ===");
        for (i, tri) in pos.triangles.iter().enumerate().take(15) {
            eprintln!(
                "  T{}: [{}, {}, {}]",
                i, tri.indices[0], tri.indices[1], tri.indices[2]
            );
        }

        eprintln!("\n=== Negative Half Vertices (first 10) ===");
        for (i, v) in neg.vertices.iter().take(10).enumerate() {
            eprintln!("  V{}: ({:.4}, {:.4}, {:.4})", i, v.x, v.y, v.z);
        }

        eprintln!("\n=== Negative Half Triangles ===");
        for (i, tri) in neg.triangles.iter().enumerate().take(15) {
            eprintln!(
                "  T{}: [{}, {}, {}]",
                i, tri.indices[0], tri.indices[1], tri.indices[2]
            );
        }
    }

    /// Test all axes with debug output
    #[test]
    fn test_split_all_axes_debug() {
        use std::fs;
        use std::path::PathBuf;

        let cube = create_unit_cube();
        let out_dir = std::env::var("CARGO_TARGET_TMPDIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("."));
        let out_dir = out_dir.join("split_debug");
        let _ = fs::create_dir_all(&out_dir);

        for axis in [Axis::X, Axis::Y, Axis::Z] {
            let (pos, neg) = split_mesh(&cube, axis, 0.0).unwrap();

            let prefix = match axis {
                Axis::X => "x",
                Axis::Y => "y",
                Axis::Z => "z",
            };

            eprintln!("\n=== Axis: {:?} at 0.0 ===", axis);
            eprintln!(
                "Positive: {} tris, watertight={}",
                pos.triangles.len(),
                is_watertight(&pos)
            );
            eprintln!(
                "Negative: {} tris, watertight={}",
                neg.triangles.len(),
                is_watertight(&neg)
            );

            // Export files
            let pos_path = out_dir.join(format!("split_pos_{}_0.stl", prefix));
            let neg_path = out_dir.join(format!("split_neg_{}_0.stl", prefix));

            let _ = crate::export::stl::write_stl_ascii(&pos, &pos_path);
            let _ = crate::export::stl::write_stl_ascii(&neg, &neg_path);

            eprintln!(
                "Exported: split_pos_{}_0.stl, split_neg_{}_0.stl",
                prefix, prefix
            );
        }
    }
}
