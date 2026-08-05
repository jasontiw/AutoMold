//! Voxelization fallback for CSG operations
//! Implements basic voxelization with marching cubes for isosurface extraction

use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::{Point3, Vector3};
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
    normal: Vector3<f32>,
}

fn signed_distance_to_triangle(point: Point3<f32>, tri: &TriangleData) -> f32 {
    let v0 = tri.vertices[0];
    let v1 = tri.vertices[1];
    let v2 = tri.vertices[2];

    let edge0 = v1 - v0;
    let edge1 = v2 - v0;
    let normal = edge0.cross(&edge1).normalize();

    let p_to_v0 = point - v0;
    let dist = p_to_v0.dot(&normal);

    dist
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
            let edge0 = v[1] - v[0];
            let edge1 = v[2] - v[0];
            let normal = edge0.cross(&edge1).normalize();
            TriangleData {
                vertices: verts,
                normal,
            }
        })
        .collect();

    let bbox = mesh.calculate_bounding_box();
    let margin = 0.01;

    let min = Point3::new(
        (bbox.min.x - margin) as f32,
        (bbox.min.y - margin) as f32,
        (bbox.min.z - margin) as f32,
    );
    let max = Point3::new(
        (bbox.max.x + margin) as f32,
        (bbox.max.y + margin) as f32,
        (bbox.max.z + margin) as f32,
    );

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

    let (block_sdf, block_min, block_max) = mesh_to_sdf(block)?;
    let (model_sdf, _, _) = mesh_to_sdf(model)?;

    let combined_sdf = move |p: Point3<f32>| -> f32 {
        let block_dist = block_sdf(p);
        let model_dist = model_sdf(p);
        block_dist - (-model_dist)
    };

    let grid = sample_grid(&combined_sdf, block_min, block_max, resolution);

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
}
