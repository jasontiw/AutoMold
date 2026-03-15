//! Mold splitting - separates mesh along a plane

use crate::geometry::mesh::{Mesh, Triangle};
use nalgebra::{Point3, Vector3};

/// Split mesh along a plane
/// Returns two meshes: positive side and negative side
pub fn split_mesh(
    mesh: &Mesh,
    axis: Vector3<f32>,
    split_point: Point3<f32>,
) -> (Option<Mesh>, Option<Mesh>) {
    let mut positive_vertices: Vec<Point3<f32>> = Vec::new();
    let mut positive_indices: Vec<[usize; 3]> = Vec::new();

    let mut negative_vertices: Vec<Point3<f32>> = Vec::new();
    let mut negative_indices: Vec<[usize; 3]> = Vec::new();

    // Normalize axis
    let axis = axis.normalize();

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);

        // Calculate distance from plane for each vertex
        let distances: Vec<f32> = v
            .iter()
            .map(|p| {
                let p_vec = Vector3::new(p.x, p.y, p.z)
                    - Vector3::new(split_point.x, split_point.y, split_point.z);
                p_vec.dot(&axis)
            })
            .collect();

        // Classify triangle
        let all_positive = distances.iter().all(|&d| d >= 0.0);
        let all_negative = distances.iter().all(|&d| d <= 0.0);

        if all_positive {
            // Entire triangle goes to positive side
            let idx = positive_vertices.len();
            positive_vertices.push(*v[0]);
            positive_vertices.push(*v[1]);
            positive_vertices.push(*v[2]);
            positive_indices.push([idx, idx + 1, idx + 2]);
        } else if all_negative {
            // Entire triangle goes to negative side
            let idx = negative_vertices.len();
            negative_vertices.push(*v[0]);
            negative_vertices.push(*v[1]);
            negative_vertices.push(*v[2]);
            negative_indices.push([idx, idx + 1, idx + 2]);
        } else {
            // Triangle crosses the plane - split it
            // Find the intersection points
            let crossing_edges = find_crossing_edges(v, &distances, axis, split_point);

            for (p1, p2, p3, side) in crossing_edges {
                let idx = if side {
                    positive_vertices.len()
                } else {
                    negative_vertices.len()
                };

                if side {
                    positive_vertices.push(p1);
                    positive_vertices.push(p2);
                    positive_vertices.push(p3);
                    positive_indices.push([idx, idx + 1, idx + 2]);
                } else {
                    negative_vertices.push(p1);
                    negative_vertices.push(p2);
                    negative_vertices.push(p3);
                    negative_indices.push([idx, idx + 1, idx + 2]);
                }
            }
        }
    }

    // Create meshes
    let positive_mesh = if !positive_vertices.is_empty() {
        Some(create_mesh(positive_vertices, positive_indices))
    } else {
        None
    };

    let negative_mesh = if !negative_vertices.is_empty() {
        Some(create_mesh(negative_vertices, negative_indices))
    } else {
        None
    };

    (positive_mesh, negative_mesh)
}

/// Find edges that cross the plane
fn find_crossing_edges(
    vertices: [&Point3<f32>; 3],
    distances: &[f32],
    axis: Vector3<f32>,
    split_point: Point3<f32>,
) -> Vec<(Point3<f32>, Point3<f32>, Point3<f32>, bool)> {
    let mut result = Vec::new();

    // Edges: (0,1), (1,2), (2,0)
    let edges = [(0, 1), (1, 2), (2, 0)];

    let mut intersection_points: Vec<Point3<f32>> = Vec::new();
    let mut positive_count = 0;
    let mut negative_count = 0;

    for &(i, j) in &edges {
        let d1 = distances[i];
        let d2 = distances[j];

        if (d1 >= 0.0 && d2 <= 0.0) || (d1 <= 0.0 && d2 >= 0.0) {
            // Edge crosses plane - find intersection
            let t = d1 / (d1 - d2);
            let p = lerp_point(vertices[i], vertices[j], t);
            intersection_points.push(p);

            if d1 > 0.0 {
                positive_count += 1;
            } else {
                negative_count += 1;
            }
        }
    }

    if intersection_points.len() == 2 {
        // Two intersections - create two triangles
        // Triangle on positive side
        if positive_count > 0 {
            result.push((
                vertices[0],
                intersection_points[0],
                intersection_points[1],
                true,
            ));
        }

        // Triangle on negative side
        if negative_count > 0 {
            result.push((
                vertices[1],
                intersection_points[1],
                intersection_points[0],
                false,
            ));
        }
    }

    result
}

/// Linear interpolation between two points
fn lerp_point(a: &Point3<f32>, b: &Point3<f32>, t: f32) -> Point3<f32> {
    Point3::new(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    )
}

/// Create mesh from vertices and indices
fn create_mesh(vertices: Vec<Point3<f32>>, indices: Vec<[usize; 3]>) -> Mesh {
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

/// Split along X axis
pub fn split_x(mesh: &Mesh, x: f32) -> (Option<Mesh>, Option<Mesh>) {
    split_mesh(mesh, Vector3::x(), Point3::new(x, 0.0, 0.0))
}

/// Split along Y axis
pub fn split_y(mesh: &Mesh, y: f32) -> (Option<Mesh>, Option<Mesh>) {
    split_mesh(mesh, Vector3::y(), Point3::new(0.0, y, 0.0))
}

/// Split along Z axis
pub fn split_z(mesh: &Mesh, z: f32) -> (Option<Mesh>, Option<Mesh>) {
    split_mesh(mesh, Vector3::z(), Point3::new(0.0, 0.0, z))
}
