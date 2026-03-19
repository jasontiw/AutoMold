//! Mold splitting - separates mesh along an axis-aligned plane.
//!
//! Algorithm:
//!   1. Pre-populate vertex buffers with all original vertices.
//!   2. Classify every triangle as fully positive, fully negative, or spanning.
//!   3. Clip spanning triangles → new sub-triangles + seam edges.
//!      Seam vertices are inserted AFTER the original vertices (seam_start),
//!      so they never alias original vertices that happen to lie on the plane
//!      (e.g. the circular end-caps of a cylinder).
//!   4. Reconstruct boundary loop(s) from seam edges via half-edge multimap.
//!      Handles vertices with multiple outgoing edges (non-simple meshes).
//!   5. Triangulate the cut face with triangulate_with_holes():
//!      - 1 loop  → simple ear-clip (cube, box, etc.)
//!      - 2+ loops → bridge technique: outer loop + inner loops (holes) merged
//!        into a single simple polygon, then ear-clipped.
//!      This produces the correct annular cap (solid face minus cavity opening)
//!      instead of a solid disc that would cover the cavity.
//!   6. Compact vertices (remove unreferenced ones from the pre-populated buffers).

use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;
use thiserror::Error;

// ─── public API kept for compatibility ───────────────────────────────────────

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
    // kept for API compatibility
    #[error("CSG conversion failed: {0}")]
    CsgConversionFailed(String),
    #[error("CSG intersection failed: {0}")]
    CsgIntersectionFailed(String),
    #[error("Result mesh is not watertight")]
    NonWatertightResult,
    #[error("Both CSG and manual fallback failed")]
    BothStrategiesFailed { csg_error: String, manual_error: String },
}

// ─── adaptive epsilon ─────────────────────────────────────────────────────────

fn compute_adaptive_eps(mesh: &Mesh) -> f32 {
    let bbox = mesh.calculate_bounding_box();
    let diag = bbox.diagonal();
    if diag.is_finite() && diag > 0.0 { diag * 1e-6 } else { 1e-4 }
}

// ─── geometry helpers ─────────────────────────────────────────────────────────

#[inline]
fn signed_dist(p: Point3<f32>, axis: Axis, point: f32) -> f32 {
    match axis {
        Axis::X => p.x - point,
        Axis::Y => p.y - point,
        Axis::Z => p.z - point,
    }
}

fn intersect_plane(a: Point3<f32>, b: Point3<f32>, da: f32, db: f32) -> Point3<f32> {
    let t = da / (da - db);
    Point3::new(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z),
    )
}

/// Deduplicate only within verts[seam_start..] to avoid merging seam vertices
/// with original mesh vertices that happen to lie on the cut plane.
fn push_seam_vertex(
    verts: &mut Vec<Point3<f32>>,
    p: Point3<f32>,
    eps: f32,
    seam_start: usize,
) -> usize {
    let eps2 = eps * eps;
    for (i, v) in verts[seam_start..].iter().enumerate() {
        if (v - p).norm_squared() < eps2 {
            return seam_start + i;
        }
    }
    verts.push(p);
    verts.len() - 1
}

// ─── main public API ──────────────────────────────────────────────────────────

pub fn split_mesh(mesh: &Mesh, axis: Axis, point: f32) -> Result<(Mesh, Mesh), SplitError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(SplitError::DegenerateGeometry("Empty mesh".to_string()));
    }

    let eps = compute_adaptive_eps(mesh);

    let dists: Vec<f32> = mesh.vertices.iter()
        .map(|&v| signed_dist(v, axis, point))
        .collect();

    let mut pos_verts: Vec<Point3<f32>> = Vec::new();
    let mut pos_tris:  Vec<[usize; 3]>  = Vec::new();
    let mut neg_verts: Vec<Point3<f32>> = Vec::new();
    let mut neg_tris:  Vec<[usize; 3]>  = Vec::new();
    let mut pos_map: HashMap<usize, usize> = HashMap::new();
    let mut neg_map: HashMap<usize, usize> = HashMap::new();

    // Pre-populate both buffers with ALL original vertices so that seam vertices
    // (inserted after seam_start) can never alias them by proximity.
    for (orig, &p) in mesh.vertices.iter().enumerate() {
        let pi = pos_verts.len(); pos_verts.push(p); pos_map.insert(orig, pi);
        let ni = neg_verts.len(); neg_verts.push(p); neg_map.insert(orig, ni);
    }
    let seam_start_pos = pos_verts.len();
    let seam_start_neg = neg_verts.len();

    let mut seam_edges: Vec<(usize, usize)> = Vec::new();

    let gp = |orig: usize| pos_map[&orig];
    let gn = |orig: usize| neg_map[&orig];

    for tri in &mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let [d0, d1, d2] = [dists[i0], dists[i1], dists[i2]];
        let [p0, p1, p2] = [mesh.vertices[i0], mesh.vertices[i1], mesh.vertices[i2]];

        let s0 = d0 >= -eps;
        let s1 = d1 >= -eps;
        let s2 = d2 >= -eps;

        match (s0, s1, s2) {
            // ── Fully positive ──────────────────────────────────────────────
            (true, true, true) => {
                pos_tris.push([gp(i0), gp(i1), gp(i2)]);
            }
            // ── Fully negative ──────────────────────────────────────────────
            (false, false, false) => {
                neg_tris.push([gn(i0), gn(i1), gn(i2)]);
            }
            // ── Spanning: clip against the plane ────────────────────────────
            _ => {
                let count_pos = [s0, s1, s2].iter().filter(|&&x| x).count();
                let lone_pos = count_pos == 1;

                let (vi, vj, vk, di, dj, dk, pi, pj, pk) = if lone_pos {
                    if s0      { (i0,i1,i2, d0,d1,d2, p0,p1,p2) }
                    else if s1 { (i1,i2,i0, d1,d2,d0, p1,p2,p0) }
                    else       { (i2,i0,i1, d2,d0,d1, p2,p0,p1) }
                } else {
                    if !s0      { (i0,i1,i2, d0,d1,d2, p0,p1,p2) }
                    else if !s1 { (i1,i2,i0, d1,d2,d0, p1,p2,p0) }
                    else        { (i2,i0,i1, d2,d0,d1, p2,p0,p1) }
                };

                let qa = intersect_plane(pi, pj, di, dj);
                let qb = intersect_plane(pi, pk, di, dk);

                let qa_pos = push_seam_vertex(&mut pos_verts, qa, eps, seam_start_pos);
                let qb_pos = push_seam_vertex(&mut pos_verts, qb, eps, seam_start_pos);
                let qa_neg = push_seam_vertex(&mut neg_verts, qa, eps, seam_start_neg);
                let qb_neg = push_seam_vertex(&mut neg_verts, qb, eps, seam_start_neg);

                if lone_pos {
                    pos_tris.push([gp(vi), qa_pos, qb_pos]);
                    neg_tris.push([gn(vj), gn(vk), qb_neg]);
                    neg_tris.push([gn(vj), qb_neg,  qa_neg]);
                    seam_edges.push((qa_pos, qb_pos));
                } else {
                    neg_tris.push([gn(vi), qa_neg, qb_neg]);
                    pos_tris.push([gp(vj), gp(vk), qb_pos]);
                    pos_tris.push([gp(vj), qb_pos,  qa_pos]);
                    seam_edges.push((qb_pos, qa_pos));
                }
            }
        }
    }

    // ── Build caps ────────────────────────────────────────────────────────────
    //
    // The cut plane may expose multiple boundary loops:
    //   • 1 outer loop  — the block perimeter
    //   • N inner loops — cavity cross-sections (cylinder hole, etc.)
    //
    // triangulate_with_holes() handles both cases correctly:
    //   single loop → ear-clip (solid face, fine for cubes/boxes)
    //   multi loop  → bridge inner loops into outer, then ear-clip
    //                 → annular cap with hole(s), correct for molds

    let plane_normal: Vector3<f32> = match axis {
        Axis::X => Vector3::new(1.0, 0.0, 0.0),
        Axis::Y => Vector3::new(0.0, 1.0, 0.0),
        Axis::Z => Vector3::new(0.0, 0.0, 1.0),
    };

    let loops = build_boundary_loops(&seam_edges)
        .map_err(|e| SplitError::BoundaryLoopFailed(e))?;

    if !loops.is_empty() {
        let cap_tris = triangulate_with_holes(&loops, &pos_verts, plane_normal, eps)
            .map_err(|e| SplitError::TriangulationFailed(e))?;

        pos_tris.extend(cap_tris.iter().copied());

        for [a, b, c] in &cap_tris {
            let pa = pos_verts[*a];
            let pb = pos_verts[*b];
            let pc = pos_verts[*c];
            let na = push_seam_vertex(&mut neg_verts, pa, eps, seam_start_neg);
            let nb = push_seam_vertex(&mut neg_verts, pb, eps, seam_start_neg);
            let nc = push_seam_vertex(&mut neg_verts, pc, eps, seam_start_neg);
            neg_tris.push([na, nc, nb]); // reversed winding
        }
    }

    Ok((assemble_mesh(pos_verts, pos_tris), assemble_mesh(neg_verts, neg_tris)))
}

// ─── boundary loop reconstruction ────────────────────────────────────────────

/// Reconstruct closed loops from directed seam edges.
///
/// Uses a multimap (vertex → Vec<neighbors>) to handle vertices with
/// multiple outgoing edges (e.g. cylinder end-caps sharing vertices with the
/// seam), consuming each edge exactly once via pop().
fn build_boundary_loops(edges: &[(usize, usize)]) -> Result<Vec<Vec<usize>>, String> {
    if edges.is_empty() {
        return Ok(vec![]);
    }

    // Build next-map: each directed edge (a→b) is consumed exactly once.
    // We use a Vec per source to handle the rare case where csgrs produces
    // duplicate edges from the same vertex (non-manifold seam).
    let mut next: HashMap<usize, std::collections::VecDeque<usize>> = HashMap::new();
    for &(a, b) in edges {
        next.entry(a).or_default().push_back(b);
    }

    // Track which edges have been consumed globally.
    // A vertex is "available" as a loop start if it still has outgoing edges.
    let mut loops: Vec<Vec<usize>> = Vec::new();

    // Keep trying until all edges are consumed.
    // Sort start candidates for deterministic output.
    loop {
        // Find the first vertex that still has an outgoing edge.
        let start_opt = {
            let mut candidates: Vec<usize> = next.iter()
                .filter(|(_, q)| !q.is_empty())
                .map(|(&k, _)| k)
                .collect();
            candidates.sort_unstable();
            candidates.into_iter().next()
        };

        let start = match start_opt {
            Some(s) => s,
            None => break, // all edges consumed
        };

        let mut lp: Vec<usize> = vec![start];
        let mut cur = start;
        let max_len = edges.len() + 2;

        loop {
            // Pop one outgoing edge from cur
            let nxt = match next.get_mut(&cur).and_then(|q| q.pop_front()) {
                Some(n) => n,
                None => {
                    // Dead end — this partial loop is broken, discard it
                    lp.clear();
                    break;
                }
            };

            if nxt == start {
                // Successfully closed the loop
                break;
            }

            // Detect infinite loops (should not happen with valid meshes)
            if lp.len() >= max_len {
                lp.clear();
                break;
            }

            lp.push(nxt);
            cur = nxt;
        }

        if lp.len() >= 3 {
            loops.push(lp);
        }
    }

    Ok(loops)
}

// ─── polygon-with-holes triangulation ────────────────────────────────────────

/// Triangulate a cut face that may contain holes (inner boundary loops).
///
/// Strategy:
///   1. Identify outer loop (largest area) and inner loops (holes).
///   2. Use the bridge technique to merge each inner loop into the outer,
///      producing a simple polygon that ear-clip can handle.
///   3. After triangulation, discard any triangle whose centroid lies
///      inside an inner loop — these are the "bridge fill" triangles that
///      would otherwise appear as a wall covering the cavity opening.
fn triangulate_with_holes(
    loops: &[Vec<usize>],
    verts: &[Point3<f32>],
    normal: Vector3<f32>,
    eps: f32,
) -> Result<Vec<[usize; 3]>, String> {
    if loops.is_empty() { return Ok(vec![]); }
    if loops.len() == 1 { return ear_clip(&loops[0], verts, normal, eps); }

    // Signed 2-D area projected onto the cut plane.
    // Positive = CCW from normal direction = outer loop.
    let signed_area = |lp: &[usize]| -> f32 {
        let n = lp.len();
        (0..n).map(|i| {
            let a = verts[lp[i]];
            let b = verts[lp[(i + 1) % n]];
            if normal.x.abs() > 0.5      { (a.y - b.y) * (a.z + b.z) }
            else if normal.y.abs() > 0.5 { (a.z - b.z) * (a.x + b.x) }
            else                          { (a.x - b.x) * (a.y + b.y) }
        }).sum::<f32>() * 0.5
    };

    // Outer loop = largest absolute area.
    let outer_idx = loops.iter().enumerate()
        .max_by(|(_, a), (_, b)| signed_area(a).abs()
            .partial_cmp(&signed_area(b).abs())
            .unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    let mut outer = loops[outer_idx].clone();
    if signed_area(&outer) < 0.0 { outer.reverse(); }

    // Collect inner loops (holes) with CW winding.
    let inner_loops: Vec<Vec<usize>> = loops.iter().enumerate()
        .filter(|(i, _)| *i != outer_idx)
        .map(|(_, lp)| {
            let mut h = lp.clone();
            if signed_area(&h) > 0.0 { h.reverse(); }
            h
        })
        .collect();

    // Bridge each inner loop into the outer polygon.
    for hole in &inner_loops {
        // Find closest vertex pair (outer ↔ hole).
        let (best_oi, best_hi) = (0..outer.len())
            .flat_map(|oi| (0..hole.len()).map(move |hi| (oi, hi)))
            .min_by(|&(oa, ha), &(ob, hb)| {
                let da = (verts[outer[oa]] - verts[hole[ha]]).magnitude_squared();
                let db = (verts[outer[ob]] - verts[hole[hb]]).magnitude_squared();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or((0, 0));

        let mut rotated = hole.clone();
        rotated.rotate_left(best_hi);

        let bridge_outer = outer[best_oi];
        let bridge_hole  = rotated[0];

        let mut merged = Vec::with_capacity(outer.len() + rotated.len() + 2);
        merged.extend_from_slice(&outer[..=best_oi]);
        merged.extend_from_slice(&rotated);
        merged.push(bridge_hole);
        merged.push(bridge_outer);
        if best_oi + 1 < outer.len() {
            merged.extend_from_slice(&outer[best_oi + 1..]);
        }
        outer = merged;
    }

    // Ear-clip the merged (simple) polygon.
    let all_tris = ear_clip(&outer, verts, normal, eps)?;

    // ── Post-filter: remove triangles whose centroid lies inside any hole ──
    //
    // The bridge technique leaves triangles that span the hole opening.
    // Their centroids fall inside the inner loop → they would create a wall
    // covering the cavity. We discard them here.
    let result: Vec<[usize; 3]> = all_tris
        .into_iter()
        .filter(|&[a, b, c]| {
            let cx = (verts[a].x + verts[b].x + verts[c].x) / 3.0;
            let cy = (verts[a].y + verts[b].y + verts[c].y) / 3.0;
            let cz = (verts[a].z + verts[b].z + verts[c].z) / 3.0;
            let centroid = Point3::new(cx, cy, cz);

            // Keep only if the centroid is OUTSIDE every inner loop.
            inner_loops.iter().all(|hole| {
                !point_in_loop(centroid, hole, verts, normal)
            })
        })
        .collect();

    Ok(result)
}

/// Returns true if point `p` is inside the closed polygon defined by `loop_verts`.
/// Uses the winding number test projected onto the cut plane.
fn point_in_loop(
    p: Point3<f32>,
    loop_verts: &[usize],
    verts: &[Point3<f32>],
    normal: Vector3<f32>,
) -> bool {
    let n = loop_verts.len();
    if n < 3 { return false; }

    // Project p and polygon onto the 2-D plane perpendicular to normal.
    let proj = |v: Point3<f32>| -> (f32, f32) {
        if normal.x.abs() > 0.5      { (v.y, v.z) }
        else if normal.y.abs() > 0.5 { (v.z, v.x) }
        else                          { (v.x, v.y) }
    };

    let (px, py) = proj(p);
    let mut winding = 0i32;

    for i in 0..n {
        let (ax, ay) = proj(verts[loop_verts[i]]);
        let (bx, by) = proj(verts[loop_verts[(i + 1) % n]]);

        if ay <= py {
            if by > py {
                // Upward crossing — check if p is left of edge
                let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
                if cross > 0.0 { winding += 1; }
            }
        } else if by <= py {
            // Downward crossing — check if p is right of edge
            let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
            if cross < 0.0 { winding -= 1; }
        }
    }

    winding != 0
}


// ─── ear-clipping triangulation ──────────────────────────────────────────────

fn ear_clip(
    indices: &[usize],
    verts: &[Point3<f32>],
    normal: Vector3<f32>,
    eps: f32,
) -> Result<Vec<[usize; 3]>, String> {
    let n = indices.len();
    if n < 3 { return Err(format!("Polygon has only {n} vertices")); }
    if n == 3 { return Ok(vec![[indices[0], indices[1], indices[2]]]); }

    let mut remaining: Vec<usize> = indices.to_vec();
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

            if !is_ear(a, b, c, &remaining, verts, normal, eps) { continue; }

            result.push([prev, curr, next]);
            remaining.remove(i);
            clipped = true;
            break;
        }

        if !clipped {
            guard += 1;
            if guard > remaining.len() + 10 {
                return Err("Ear-clip stuck — polygon may be self-intersecting".to_string());
            }
            let prev = remaining[0];
            let curr = remaining[1];
            let next = remaining[2];
            result.push([prev, curr, next]);
            remaining.remove(1);
        }
    }

    result.push([remaining[0], remaining[1], remaining[2]]);
    Ok(result)
}

fn is_ear(
    a: Point3<f32>,
    b: Point3<f32>,
    c: Point3<f32>,
    remaining: &[usize],
    verts: &[Point3<f32>],
    normal: Vector3<f32>,
    eps: f32,
) -> bool {
    let cross = (b - a).cross(&(c - b));
    if cross.dot(&normal) < 0.0 { return false; }

    for &idx in remaining {
        let p = verts[idx];
        if (p - a).norm_squared() < eps * eps
            || (p - b).norm_squared() < eps * eps
            || (p - c).norm_squared() < eps * eps
        { continue; }
        if point_in_triangle(p, a, b, c, normal, eps) { return false; }
    }
    true
}

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
    let has_pos = d1 >  eps || d2 >  eps || d3 >  eps;
    !(has_neg && has_pos)
}

// ─── mesh assembly with vertex compaction ────────────────────────────────────

fn assemble_mesh(vertices: Vec<Point3<f32>>, raw_tris: Vec<[usize; 3]>) -> Mesh {
    // Filter degenerate triangles.
    //
    // Two cases to catch:
    //   1. Same index: fast check catches these immediately.
    //   2. Different indices, same position: produced when a mesh vertex lies
    //      exactly on the cut plane. intersect_plane() returns a point identical
    //      to that vertex but with a different seam index. These "needle" triangles
    //      have area ≈ 0 and appear as phantom walls inside the mold cavity.
    //      Caught by the cross-product magnitude check.
    let valid_tris: Vec<[usize; 3]> = raw_tris
        .into_iter()
        .filter(|&[a, b, c]| {
            if a == b || b == c || a == c { return false; }
            let va = vertices[a];
            let vb = vertices[b];
            let vc = vertices[c];
            (vb - va).cross(&(vc - va)).magnitude_squared() > 1e-10
        })
        .collect();

    // Compact: remove vertices not referenced by any triangle
    let mut used = vec![false; vertices.len()];
    for &[a, b, c] in &valid_tris { used[a] = true; used[b] = true; used[c] = true; }

    let mut remap = vec![0usize; vertices.len()];
    let mut new_verts: Vec<Point3<f32>> = Vec::new();
    for (old, &is_used) in used.iter().enumerate() {
        if is_used {
            remap[old] = new_verts.len();
            new_verts.push(vertices[old]);
        }
    }

    let triangles: Vec<Triangle> = valid_tris
        .into_iter()
        .map(|[a, b, c]| Triangle::new(remap[a], remap[b], remap[c]))
        .collect();

    let normals = Mesh::calculate_normals(&new_verts, &triangles);
    Mesh { vertices: new_verts, triangles, normals }
}

// ─── convenience wrappers ─────────────────────────────────────────────────────

pub fn split_x(mesh: &Mesh, x: f32) -> Result<(Mesh, Mesh), SplitError> { split_mesh(mesh, Axis::X, x) }
pub fn split_y(mesh: &Mesh, y: f32) -> Result<(Mesh, Mesh), SplitError> { split_mesh(mesh, Axis::Y, y) }
pub fn split_z(mesh: &Mesh, z: f32) -> Result<(Mesh, Mesh), SplitError> { split_mesh(mesh, Axis::Z, z) }

// ─── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::repair::{calculate_volume, is_watertight};
    use nalgebra::Point3;

    fn create_unit_cube() -> Mesh {
        let vertices = vec![
            Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5,  0.5, -0.5),  Point3::new(-0.5,  0.5, -0.5),
            Point3::new(-0.5, -0.5,  0.5), Point3::new(0.5, -0.5,  0.5),
            Point3::new(0.5,  0.5,  0.5),  Point3::new(-0.5,  0.5,  0.5),
        ];
        let triangles = vec![
            Triangle::new(0,1,2), Triangle::new(0,2,3),
            Triangle::new(4,6,5), Triangle::new(4,7,6),
            Triangle::new(3,2,6), Triangle::new(3,6,7),
            Triangle::new(0,5,1), Triangle::new(0,4,5),
            Triangle::new(1,5,6), Triangle::new(1,6,2),
            Triangle::new(4,0,3), Triangle::new(4,3,7),
        ];
        let normals = Mesh::calculate_normals(&vertices, &triangles);
        Mesh { vertices, triangles, normals }
    }

    #[test]
    fn test_split_cube_watertight() {
        let cube = create_unit_cube();
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            let (pos, neg) = split_mesh(&cube, axis, 0.0).unwrap();
            assert!(is_watertight(&pos), "{axis:?} positive not watertight");
            assert!(is_watertight(&neg), "{axis:?} negative not watertight");
        }
    }

    #[test]
    fn test_split_preserves_volume() {
        let cube = create_unit_cube();
        let orig = calculate_volume(&cube).abs();
        let (pos, neg) = split_z(&cube, 0.0).unwrap();
        let total = calculate_volume(&pos).abs() + calculate_volume(&neg).abs();
        let err = (total - orig).abs() / orig;
        assert!(err < 0.01, "Volume error {err:.4} exceeds 1%");
    }

    #[test]
    fn test_split_negative_coordinate() {
        let cube = create_unit_cube();
        split_x(&cube, -0.25).unwrap();
    }

    #[test]
    fn test_slab_size_constant() {
        assert!(SLAB_SIZE >= 2.0);
    }
}