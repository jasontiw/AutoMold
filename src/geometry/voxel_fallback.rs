//! Voxelization fallback for CSG operations
//! Implements basic voxelization with marching cubes for isosurface extraction

use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::Point3;
use tracing::info;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelStrategy {
    Standard,
    HighPrecision,
    Fast,
}

impl Default for VoxelStrategy {
    fn default() -> Self {
        Self::Standard
    }
}

#[derive(Debug, Clone)]
pub struct VoxelConfig {
    pub resolution: u32,
    pub strategy: VoxelStrategy,
    pub smooth_normals: bool,
}

impl Default for VoxelConfig {
    fn default() -> Self {
        Self {
            resolution: 64,
            strategy: VoxelStrategy::Standard,
            smooth_normals: true,
        }
    }
}

#[derive(Debug)]
pub enum VoxelError {
    VoxelizationFailed(String),
    MarchingCubesFailed(String),
    MeshEmpty,
    InvalidBounds,
}

impl std::fmt::Display for VoxelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VoxelError::VoxelizationFailed(msg) => write!(f, "Voxelization failed: {}", msg),
            VoxelError::MarchingCubesFailed(msg) => write!(f, "Marching cubes failed: {}", msg),
            VoxelError::MeshEmpty => write!(f, "Mesh is empty"),
            VoxelError::InvalidBounds => write!(f, "Invalid bounding box"),
        }
    }
}

impl std::error::Error for VoxelError {}

#[derive(Clone)]
struct TriangleData {
    vertices: [Point3<f32>; 3],
}

/// Signed distance from `point` to the CLOSEST point ON `tri` (RTCD, Ericson
/// §5.1.5), with the sign taken from the triangle's plane normal. Degenerate
/// zero-area triangles are guarded BEFORE any normalize and return `f32::MAX`
/// so the min-|dist| selection in `mesh_to_sdf` skips them (S3).
fn signed_distance_to_triangle(point: Point3<f32>, tri: &TriangleData) -> f32 {
    let (v0, v1, v2) = (tri.vertices[0], tri.vertices[1], tri.vertices[2]);
    let (ab, ac, ap) = (v1 - v0, v2 - v0, point - v0);
    let n = ab.cross(&ac);
    // NaN guard BEFORE normalize: nalgebra's normalize() turns a zero cross
    // product into NaN, and Vector3 has no is_finite in 0.32.
    if !(n.x.is_finite() && n.y.is_finite() && n.z.is_finite())
        || n.magnitude_squared() <= f32::EPSILON
    {
        return f32::MAX;
    }
    let closest = closest_point_on_triangle(point, v0, v1, v2);
    let sign = if ap.dot(&n) < 0.0 { -1.0 } else { 1.0 };
    sign * (point - closest).magnitude()
}

/// Closest point on a triangle to `p` — the 7-region test from Real-Time
/// Collision Detection (Ericson, §5.1.5).
fn closest_point_on_triangle(
    p: Point3<f32>,
    a: Point3<f32>,
    b: Point3<f32>,
    c: Point3<f32>,
) -> Point3<f32> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

/// Axis-aligned bounds of the mesh inflated by a small margin. Single source
/// of truth for the sampling box, shared by `mesh_to_sdf` and the field-grid
/// volume oracle in tests.
fn mesh_bounds(mesh: &Mesh) -> (Point3<f32>, Point3<f32>) {
    let bbox = mesh.calculate_bounding_box();
    let margin = 0.01;
    let min = Point3::new(
        bbox.min.x - margin,
        bbox.min.y - margin,
        bbox.min.z - margin,
    );
    let max = Point3::new(
        bbox.max.x + margin,
        bbox.max.y + margin,
        bbox.max.z + margin,
    );
    (min, max)
}

fn mesh_to_sdf(
    mesh: &Mesh,
) -> Result<
    (
        Box<dyn Fn(Point3<f32>) -> f32 + Send + Sync>,
        Point3<f32>,
        Point3<f32>,
    ),
    VoxelError,
> {
    if mesh.triangles.is_empty() {
        return Err(VoxelError::MeshEmpty);
    }

    let triangles: Vec<TriangleData> = mesh
        .triangles
        .iter()
        .map(|tri| {
            let v = tri.get_vertices(&mesh.vertices);
            let verts: [Point3<f32>; 3] = [*v[0], *v[1], *v[2]];
            TriangleData { vertices: verts }
        })
        .collect();

    let (min, max) = mesh_bounds(mesh);

    let triangles_owned = triangles;
    let sdf = move |point: Point3<f32>| -> f32 {
        let mut min_dist = f32::MAX;

        for tri in &triangles_owned {
            let dist = signed_distance_to_triangle(point, tri);
            if dist.abs() < min_dist.abs() {
                min_dist = dist;
            }
        }

        min_dist
    };

    Ok((Box::new(sdf), min, max))
}

fn sample_grid<F>(sdf: &F, min: Point3<f32>, max: Point3<f32>, res: u32) -> Vec<f32>
where
    F: Fn(Point3<f32>) -> f32 + Send + Sync,
{
    use rayon::prelude::*;

    let mut grid = vec![0.0f32; (res * res * res) as usize];

    let step = (max - min) / (res as f32);
    let res_u = res as usize;

    // The SDF closures are Send + Sync by design; sampling is embarrassingly
    // parallel and dominates voxel cost (O(tris * res^3)), so parallelize it.
    grid.par_iter_mut().enumerate().for_each(|(idx, val)| {
        let zi = idx / (res_u * res_u);
        let yi = (idx / res_u) % res_u;
        let xi = idx % res_u;

        let x = min.x + (xi as f32) * step.x;
        let y = min.y + (yi as f32) * step.y;
        let z = min.z + (zi as f32) * step.z;

        *val = sdf(Point3::new(x, y, z));
    });

    grid
}

fn marching_cubes_grid(grid: &[f32], min: Point3<f32>, max: Point3<f32>, res: u32) -> Mesh {
    let step = (max - min) / (res as f32);
    let res_usize = res as usize;
    let res2 = res_usize * res_usize;

    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut indices: Vec<[usize; 3]> = Vec::new();

    let get_idx = |x, y, z| x + y * res_usize + z * res2;

    let lerp = |p1: Point3<f32>, p2: Point3<f32>, v1: f32, v2: f32| -> Point3<f32> {
        if (v2 - v1).abs() < 1e-6 {
            return p1;
        }
        let t = -v1 / (v2 - v1);
        p1 + (p2 - p1) * t
    };

    for z in 0..res_usize - 1 {
        for y in 0..res_usize - 1 {
            for x in 0..res_usize - 1 {
                let v = [
                    grid[get_idx(x, y, z)],
                    grid[get_idx(x + 1, y, z)],
                    grid[get_idx(x + 1, y, z + 1)],
                    grid[get_idx(x, y, z + 1)],
                    grid[get_idx(x, y + 1, z)],
                    grid[get_idx(x + 1, y + 1, z)],
                    grid[get_idx(x + 1, y + 1, z + 1)],
                    grid[get_idx(x, y + 1, z + 1)],
                ];

                let mut cube_index = 0;
                for i in 0..8 {
                    if v[i] < 0.0 {
                        cube_index |= 1 << i;
                    }
                }

                if cube_index == 0 || cube_index == 255 {
                    continue;
                }

                let px = min.x + (x as f32) * step.x;
                let py = min.y + (y as f32) * step.y;
                let pz = min.z + (z as f32) * step.z;

                let corners = [
                    Point3::new(px, py, pz),
                    Point3::new(px + step.x, py, pz),
                    Point3::new(px + step.x, py, pz + step.z),
                    Point3::new(px, py, pz + step.z),
                    Point3::new(px, py + step.y, pz),
                    Point3::new(px + step.x, py + step.y, pz),
                    Point3::new(px + step.x, py + step.y, pz + step.z),
                    Point3::new(px, py + step.y, pz + step.z),
                ];

                let mut vert_list: [Option<Point3<f32>>; 12] = [None; 12];

                const EDGES: [(usize, usize, usize); 12] = [
                    (0, 1, 0),
                    (1, 2, 1),
                    (2, 3, 2),
                    (3, 0, 3),
                    (4, 5, 4),
                    (5, 6, 5),
                    (6, 7, 6),
                    (7, 4, 7),
                    (0, 4, 8),
                    (1, 5, 9),
                    (2, 6, 10),
                    (3, 7, 11),
                ];

                for (a, b, edge_idx) in EDGES.iter() {
                    if (v[*a] < 0.0) != (v[*b] < 0.0) {
                        vert_list[*edge_idx] = Some(lerp(corners[*a], corners[*b], v[*a], v[*b]));
                    }
                }

                let triangles_to_add: [(usize, usize, usize); 16] = [
                    (0, 8, 3),
                    (0, 1, 8),
                    (1, 9, 8),
                    (2, 10, 9),
                    (3, 11, 10),
                    (0, 3, 11),
                    (4, 8, 5),
                    (5, 8, 9),
                    (4, 5, 6),
                    (6, 7, 4),
                    (8, 10, 11),
                    (9, 10, 8),
                    (1, 2, 9),
                    (2, 3, 10),
                    (0, 4, 7),
                    (4, 6, 5),
                ];

                for (a, b, c) in triangles_to_add.iter() {
                    if let (Some(v0), Some(v1), Some(v2)) =
                        (vert_list[*a], vert_list[*b], vert_list[*c])
                    {
                        let idx = vertices.len();
                        vertices.push(v0);
                        vertices.push(v1);
                        vertices.push(v2);
                        indices.push([idx, idx + 1, idx + 2]);
                    }
                }
            }
        }
    }

    let triangles: Vec<Triangle> = indices
        .iter()
        .map(|&[i0, i1, i2]| Triangle::new(i0, i1, i2))
        .collect();

    let normals = Mesh::calculate_normals(&vertices, &triangles);

    Mesh {
        vertices,
        triangles,
        normals,
    }
}

/// Canonical combined CSG-subtraction field (D1): `max(block(p), -model(p))`.
/// Field < 0 → SOLID, > 0 → VOID. Shared by the pipeline and the unit-test
/// sampler so the formula can never diverge between production and tests.
pub(crate) struct CombinedField {
    block: Box<dyn Fn(Point3<f32>) -> f32 + Send + Sync>,
    model: Box<dyn Fn(Point3<f32>) -> f32 + Send + Sync>,
}

impl CombinedField {
    pub(crate) fn new(block: &Mesh, model: &Mesh) -> Result<Self, VoxelError> {
        let (block_sdf, _, _) = mesh_to_sdf(block)?;
        let (model_sdf, _, _) = mesh_to_sdf(model)?;
        Ok(Self {
            block: block_sdf,
            model: model_sdf,
        })
    }

    pub(crate) fn sample(&self, p: Point3<f32>) -> f32 {
        (self.block)(p).max(-(self.model)(p))
    }
}

/// Deterministic single-point probe of the combined field (R3). `_config` is
/// reserved for future resolution tuning; the field does not depend on it.
/// Consumed by the voxel unit-test probes (S1/S2/S5) so the canonical formula
/// is exercised through the same entry point the pipeline uses.
#[allow(dead_code)] // test-only sampler (R3); used only from #[cfg(test)] probes
pub(crate) fn combined_sdf_at(
    block: &Mesh,
    model: &Mesh,
    _config: &VoxelConfig,
    point: Point3<f32>,
) -> Result<f32, VoxelError> {
    let field = CombinedField::new(block, model)?;
    Ok(field.sample(point))
}

pub fn voxel_boolean_subtract(
    block: &Mesh,
    model: &Mesh,
    config: &VoxelConfig,
) -> Result<Mesh, VoxelError> {
    info!(
        "Starting voxel boolean: block={}, model={}, resolution={}",
        block.triangles.len(),
        model.triangles.len(),
        config.resolution
    );

    let resolution = config.resolution;

    let field = CombinedField::new(block, model)?;
    let (block_min, block_max) = mesh_bounds(block);

    let sample = |p: Point3<f32>| field.sample(p);
    let grid = sample_grid(&sample, block_min, block_max, resolution);

    let result = marching_cubes_grid(&grid, block_min, block_max, resolution);

    info!(
        "Voxel boolean complete: {} triangles",
        result.triangles.len()
    );

    Ok(result)
}

pub fn auto_voxel_resolution(block_triangles: usize, model_triangles: usize) -> u32 {
    let total = block_triangles + model_triangles;

    if total < 10000 {
        32
    } else if total <= 50000 {
        48
    } else if total <= 120000 {
        64
    } else {
        // Clamped from 96: the SDF is O(tris * res^3) with no BVH, so the top
        // tier at 96 is several orders of magnitude slower than 64 for the
        // models that reach it (defense-in-depth for direct voxel callers).
        64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_voxel_resolution_small() {
        assert_eq!(auto_voxel_resolution(100, 100), 32);
    }

    #[test]
    fn test_auto_voxel_resolution_medium() {
        assert_eq!(auto_voxel_resolution(25000, 25000), 48);
    }

    #[test]
    fn test_auto_voxel_resolution_large() {
        assert_eq!(auto_voxel_resolution(60000, 60000), 64);
    }

    #[test]
    fn test_auto_voxel_resolution_boundary_48() {
        // total == 10_000: not < 10_000, must step up to the 48 tier.
        assert_eq!(auto_voxel_resolution(5000, 5000), 48);
    }

    #[test]
    fn test_auto_voxel_resolution_boundary_64() {
        // total == 50_001: just past the 48 tier upper bound (<= 50_000) → 64.
        assert_eq!(auto_voxel_resolution(25001, 25000), 64);
    }

    #[test]
    fn test_auto_voxel_resolution_boundary_96() {
        // total == 120_001: just past the 64 tier upper bound (<= 120_000);
        // the top tier is clamped to 64 (was 96) to bound SDF cost.
        assert_eq!(auto_voxel_resolution(60000, 60001), 64);
    }

    // ── lucas-mold T-1: voxel combined-SDF correctness (R1-R4) ────────────────
    //
    // Fixture: 100³ block (half-size 50) minus a radius-20 sphere. Sign
    // convention: field < 0 → SOLID, > 0 → VOID. The analytic cavity volume is
    // 100³ − 4/3·π·20³ = 966,489.7 mm³. These tests probe the field directly via
    // combined_sdf_at / CombinedField (R3), never through marching cubes.

    /// Closed box centered at the origin. OUTWARD-wound (standard CSG
    /// convention, matching `mold_block::generate_block`): points inside the
    /// box get a NEGATIVE SDF, which is what the combined field's sign
    /// convention (field < 0 → SOLID) requires.
    fn block_mesh(half_size: f32) -> Mesh {
        let v = |x: f32, y: f32, z: f32| Point3::new(x, y, z);
        let vertices = vec![
            v(-half_size, -half_size, -half_size),
            v(half_size, -half_size, -half_size),
            v(half_size, half_size, -half_size),
            v(-half_size, half_size, -half_size),
            v(-half_size, -half_size, half_size),
            v(half_size, -half_size, half_size),
            v(half_size, half_size, half_size),
            v(-half_size, half_size, half_size),
        ];
        let triangles = vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 3, 2),
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            Triangle::new(3, 6, 2),
            Triangle::new(3, 7, 6),
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
            Triangle::new(1, 6, 5),
            Triangle::new(1, 2, 6),
            Triangle::new(4, 3, 0),
            Triangle::new(4, 7, 3),
        ];
        let normals = Mesh::calculate_normals(&vertices, &triangles);
        Mesh {
            vertices,
            triangles,
            normals,
        }
    }

    /// UV sphere centered at the origin: `lat * lon * 2` triangles.
    fn uv_sphere(radius: f32, lat: usize, lon: usize) -> Mesh {
        let mut vertices: Vec<Point3<f32>> = Vec::new();
        for i in 0..=lat {
            let phi = std::f32::consts::PI * i as f32 / lat as f32;
            for j in 0..lon {
                let theta = 2.0 * std::f32::consts::PI * j as f32 / lon as f32;
                vertices.push(Point3::new(
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

    /// Field-grid cavity volume (C3 oracle): count SOLID (field < 0) grid
    /// samples × voxel volume. `calculate_volume` MUST NOT be used (S10).
    fn field_grid_volume(field: &CombinedField, block: &Mesh, res: u32) -> f32 {
        let (min, max) = mesh_bounds(block);
        let grid = sample_grid(&|p: Point3<f32>| field.sample(p), min, max, res);
        let step = (max - min) / (res as f32);
        let solid = grid.iter().filter(|&&v| v < 0.0).count();
        solid as f32 * (step.x * step.y * step.z)
    }

    #[test]
    fn test_s1_origin_inside_model_is_void() {
        // C1: origin is inside the sphere → the combined field must be VOID.
        // max(block, -model) = max(-50, +20) = +20, NOT the sum −70 (SOLID).
        let block = block_mesh(50.0);
        let sphere = uv_sphere(20.0, 16, 16);
        let v = combined_sdf_at(
            &block,
            &sphere,
            &VoxelConfig::default(),
            Point3::new(0.0, 0.0, 0.0),
        )
        .expect("combined_sdf_at must succeed");
        assert!(v > 0.0, "origin must be VOID (field {v}); the sum bug gives −70 (SOLID)");
        assert!(
            (v - 20.0).abs() < 1.0,
            "origin field {v} must be ≈ +20 (max(block, −model))"
        );
    }

    #[test]
    fn test_s2_block_point_outside_model_is_solid() {
        // C2: (48,0,0) is inside the block but ~28 mm outside the sphere → SOLID.
        // The plane-SDF error (m = −0.24) would flip it VOID.
        let block = block_mesh(50.0);
        let sphere = uv_sphere(20.0, 16, 16);
        let v = combined_sdf_at(
            &block,
            &sphere,
            &VoxelConfig::default(),
            Point3::new(48.0, 0.0, 0.0),
        )
        .expect("combined_sdf_at must succeed");
        assert!(v < 0.0, "(48,0,0) must be SOLID (field {v}); plane-SDF would flip it VOID");
    }

    #[test]
    fn test_s3_degenerate_pole_triangles_stay_finite() {
        // uv_sphere(20, 8, 8) collapses all 8 ring-0 vertices onto the north pole,
        // so every pole triangle has zero area; we also push an explicit
        // zero-area triangle. The field must stay finite everywhere (S3).
        let mut sphere = uv_sphere(20.0, 8, 8);
        sphere.triangles.push(Triangle::new(0, 0, 0));
        sphere.normals = Mesh::calculate_normals(&sphere.vertices, &sphere.triangles);

        let has_degenerate = sphere.triangles.iter().any(|t| {
            let v = t.get_vertices(&sphere.vertices);
            (v[1] - v[0]).cross(&(v[2] - v[0])).magnitude_squared() < 1e-10
        });
        assert!(has_degenerate, "fixture must contain zero-area triangles");

        let block = block_mesh(50.0);
        let field = CombinedField::new(&block, &sphere).expect("build combined field");
        let (min, max) = mesh_bounds(&block);
        let grid = sample_grid(&|p: Point3<f32>| field.sample(p), min, max, 16);
        assert!(!grid.is_empty());
        assert!(
            grid.iter().all(|v| v.is_finite()),
            "every sampled field value must be finite (no NaN from .normalize())"
        );
        assert!(
            grid.iter().any(|&v| v < 0.0) && grid.iter().any(|&v| v > 0.0),
            "field must contain both SOLID and VOID samples"
        );
    }

    #[test]
    fn test_s4_off_plane_projection_hits_features() {
        // Query (3, 0.5, 4) projects OUTSIDE the z=0 triangle, so the SDF must
        // be the distance to the closest feature (vertex (1,0,0)) ≈ 4.5, not
        // the plane distance 4.0 (S4).
        let tri = TriangleData {
            vertices: [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        };
        let d = signed_distance_to_triangle(Point3::new(3.0, 0.5, 4.0), &tri);
        assert!(
            d > 4.05,
            "closest-point distance {d} must exceed the 4.0 plane distance"
        );
        assert!(
            (d - 4.5).abs() < 0.1,
            "distance {d} must be ≈ dist to vertex (1,0,0) = 4.5"
        );
    }

    #[test]
    fn test_s8_cavity_volume_res64_within_5_percent() {
        // C3 res 64: cavity volume within ±5% of the analytic 966,489.7.
        let block = block_mesh(50.0);
        let sphere = uv_sphere(20.0, 16, 16);
        let field = CombinedField::new(&block, &sphere).expect("build combined field");
        let volume = field_grid_volume(&field, &block, 64);
        let analytic = 966_489.7;
        let (lo, hi) = (analytic * 0.95, analytic * 1.05);
        assert!(
            volume >= lo && volume <= hi,
            "res-64 cavity volume {volume:.0} outside [{lo:.0}, {hi:.0}] (±5% of 966,489.7)"
        );
    }

    #[test]
    fn test_s9_cavity_volume_res96_within_4_percent() {
        // C3 res 96: cavity volume within ±4% of the analytic 966,489.7.
        let block = block_mesh(50.0);
        let sphere = uv_sphere(20.0, 16, 16);
        let field = CombinedField::new(&block, &sphere).expect("build combined field");
        let volume = field_grid_volume(&field, &block, 96);
        let analytic = 966_489.7;
        let (lo, hi) = (analytic * 0.96, analytic * 1.04);
        assert!(
            volume >= lo && volume <= hi,
            "res-96 cavity volume {volume:.0} outside [{lo:.0}, {hi:.0}] (±4% of 966,489.7)"
        );
    }

    #[test]
    fn test_s10_calculate_volume_not_oracle_for_non_watertight() {
        // S10: repair::calculate_volume returns 0.0 for non-watertight output
        // (voxel MC output is not guaranteed watertight), so volume assertions
        // MUST use the field-grid sampler instead.
        let mut block = block_mesh(50.0);
        block.triangles.truncate(block.triangles.len() - 2); // open one face
        assert!(
            !crate::pipeline::repair::is_watertight(&block),
            "fixture must be non-watertight"
        );
        assert_eq!(
            crate::pipeline::repair::calculate_volume(&block),
            0.0,
            "calculate_volume must return 0.0 for non-watertight output — never the oracle"
        );
    }
}
