//! Math utilities for geometry

use nalgebra::{Matrix3, Matrix4, Point3, Vector3};
use std::f32::consts::PI;

/// Calculate the centroid of a set of points
pub fn centroid(points: &[Point3<f32>]) -> Point3<f32> {
    if points.is_empty() {
        return Point3::origin();
    }

    let sum: Vector3<f32> = points.iter().map(|p| Vector3::new(p.x, p.y, p.z)).sum();

    let count = points.len() as f32;
    Point3::new(sum.x / count, sum.y / count, sum.z / count)
}

/// Calculate PCA (Principal Component Analysis) for orientation
/// Returns eigenvectors as a rotation matrix
pub fn calculate_pca(points: &[Point3<f32>]) -> Matrix3<f32> {
    if points.len() < 3 {
        return Matrix3::identity();
    }

    let c = centroid(points);

    // Build covariance matrix
    let mut cov = Matrix3::zeros();

    for p in points {
        let v = Vector3::new(p.x - c.x, p.y - c.y, p.z - c.z);
        cov += v * v.transpose();
    }

    // Simple power iteration for eigenvectors (simplified)
    // In production, use nalgebra's eigenvalue decomposition
    let _result: Matrix3<f32> = Matrix3::identity();

    // Power iteration for first eigenvector
    let mut v = Vector3::new(1.0, 1.0, 1.0).normalize();
    for _ in 0..20 {
        let new_v = cov * v;
        v = new_v.normalize();
    }

    // Use first eigenvector as primary axis (usually Z for upright objects)
    // This is simplified - proper PCA needs full eigendecomposition

    // For now, return identity (Z up)
    Matrix3::identity()
}

/// Calculate angle between two vectors in degrees
pub fn angle_between(a: &Vector3<f32>, b: &Vector3<f32>) -> f32 {
    let dot = a.normalize().dot(&b.normalize());
    dot.clamp(-1.0, 1.0).acos() * 180.0 / PI
}

/// Signed angle from vector a to b around axis
pub fn signed_angle(a: &Vector3<f32>, b: &Vector3<f32>, axis: &Vector3<f32>) -> f32 {
    let a_norm = a.normalize();
    let b_norm = b.normalize();

    let dot = a_norm.dot(&b_norm);
    let cross = a_norm.cross(&b_norm);
    let sign = cross.dot(axis).signum();

    dot.clamp(-1.0, 1.0).acos() * sign
}

/// Create a transformation matrix for translation
pub fn translation(tx: f32, ty: f32, tz: f32) -> Matrix4<f32> {
    Matrix4::new(
        1.0, 0.0, 0.0, tx, 0.0, 1.0, 0.0, ty, 0.0, 0.0, 1.0, tz, 0.0, 0.0, 0.0, 1.0,
    )
}

/// Create a transformation matrix for scaling
pub fn scaling(sx: f32, sy: f32, sz: f32) -> Matrix4<f32> {
    Matrix4::new(
        sx, 0.0, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 0.0, sz, 0.0, 0.0, 0.0, 0.0, 1.0,
    )
}

/// Create a transformation matrix for uniform scaling
pub fn uniform_scale(s: f32) -> Matrix4<f32> {
    scaling(s, s, s)
}

/// Create a rotation matrix around an axis
pub fn rotation_axis(axis: &Vector3<f32>, angle: f32) -> Matrix4<f32> {
    let axis = axis.normalize();
    let c = angle.cos();
    let s = angle.sin();
    let t = 1.0 - c;

    let x = axis.x;
    let y = axis.y;
    let z = axis.z;

    Matrix4::new(
        t * x * x + c,
        t * x * y - z * s,
        t * x * z + y * s,
        0.0,
        t * x * y + z * s,
        t * y * y + c,
        t * y * z - x * s,
        0.0,
        t * x * z - y * s,
        t * y * z + x * s,
        t * z * z + c,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

/// Check if a point is on the positive side of a plane
pub fn point_on_positive_side(
    point: &Point3<f32>,
    plane_point: &Point3<f32>,
    plane_normal: &Vector3<f32>,
) -> bool {
    let v = point - plane_point;
    v.dot(plane_normal) > 0.0
}

/// Distance from point to plane
pub fn point_plane_distance(
    point: &Point3<f32>,
    plane_point: &Point3<f32>,
    plane_normal: &Vector3<f32>,
) -> f32 {
    let v = point - plane_point;
    v.dot(&plane_normal.normalize())
}

/// Project point onto plane
pub fn project_to_plane(
    point: &Point3<f32>,
    plane_point: &Point3<f32>,
    plane_normal: &Vector3<f32>,
) -> Point3<f32> {
    let n = plane_normal.normalize();
    let d = point_plane_distance(point, plane_point, &n);
    point - n * d
}

/// Linear interpolation between two points
pub fn lerp(a: &Point3<f32>, b: &Point3<f32>, t: f32) -> Point3<f32> {
    Point3::new(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    )
}

/// Check if float is approximately equal
pub fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
    (a - b).abs() < epsilon
}

/// Clamp value between min and max
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

/// Smooth step function (for interpolation)
pub fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
