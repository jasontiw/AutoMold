//! Orientation analysis - determines optimal split axis using PCA and undercut detection

use crate::core::config::SplitAxis;
use crate::geometry::mesh::Mesh;
use nalgebra::Vector3;
use std::collections::HashMap;

/// Analyze mesh orientation to determine best split axis
pub fn analyze_orientation(mesh: &Mesh) -> Option<SplitAxis> {
    // Use PCA to find principal axes
    let pca = calculate_pca(&mesh.vertices);

    // Get the primary axis (usually the one with largest eigenvalue)
    // For most models, Z is up, so we prefer that
    // But we also check for undercuts

    let axes = vec![
        (SplitAxis::X, Vector3::x()),
        (SplitAxis::Y, Vector3::y()),
        (SplitAxis::Z, Vector3::z()),
    ];

    // Count undercuts for each axis
    let mut undercut_counts: Vec<(SplitAxis, usize)> = Vec::new();

    for (axis, direction) in axes {
        let count = count_undercuts(mesh, direction);
        undercut_counts.push((axis, count));
    }

    // Sort by undercuts (least is best)
    undercut_counts.sort_by_key(|(_, c)| *c);

    // Return the axis with least undercuts
    let (best_axis, min_undercuts) = undercut_counts[0].clone();

    Some(best_axis)
}

/// Calculate PCA to find principal axes
fn calculate_pca(vertices: &[nalgebra::Point3<f32>]) -> PCAResult {
    if vertices.is_empty() {
        return PCAResult::default();
    }

    // Calculate centroid
    let centroid = nalgebra::Point3::new(
        vertices.iter().map(|p| p.x).sum::<f32>() / vertices.len() as f32,
        vertices.iter().map(|p| p.y).sum::<f32>() / vertices.len() as f32,
        vertices.iter().map(|p| p.z).sum::<f32>() / vertices.len() as f32,
    );

    // Build covariance matrix
    let mut cov = nalgebra::Matrix3::zeros();

    for p in vertices {
        let v = nalgebra::Vector3::new(p.x - centroid.x, p.y - centroid.y, p.z - centroid.z);
        cov += v * v.transpose();
    }

    cov /= vertices.len() as f32;

    // For now, return identity (actual eigendecomposition would be better)
    // This is a simplified PCA that prefers Z axis
    PCAResult {
        centroid,
        axes: [Vector3::x(), Vector3::y(), Vector3::z()],
        eigenvalues: [1.0, 1.0, 1.0], // Placeholder
    }
}

/// Count undercuts for a given pull direction
fn count_undercuts(mesh: &Mesh, pull_direction: Vector3<f32>) -> usize {
    // Undercuts are faces whose normal points against the pull direction
    let pull = -pull_direction; // Faces should point in pull direction

    let mut undercuts = 0;

    for tri in &mesh.triangles {
        let v = tri.get_vertices(&mesh.vertices);

        // Calculate face normal
        let e1 = v[1] - v[0];
        let e2 = v[2] - v[0];
        let normal = e1.cross(&e2).normalize();

        // If normal points away from pull direction, it's an undercut
        if normal.dot(&pull) < -0.1 {
            undercuts += 1;
        }
    }

    undercuts
}

/// Calculate visibility ratio for an axis
pub fn calculate_visibility(mesh: &Mesh, axis: Vector3<f32>) -> f32 {
    let total_faces = mesh.triangles.len();
    if total_faces == 0 {
        return 1.0;
    }

    let visible = count_visible_faces(mesh, axis);
    visible as f32 / total_faces as f32
}

/// Count faces visible from a direction
fn count_visible_faces(mesh: &Mesh, view_direction: Vector3<f32>) -> usize {
    mesh.triangles
        .iter()
        .map(|tri| {
            let v = tri.get_vertices(&mesh.vertices);
            let e1 = v[1] - v[0];
            let e2 = v[2] - v[0];
            let normal = e1.cross(&e2).normalize();
            normal.dot(&view_direction) > 0.0
        })
        .filter(|&visible| visible)
        .count()
}

/// PCA result structure
#[derive(Debug, Default)]
pub struct PCAResult {
    pub centroid: nalgebra::Point3<f32>,
    pub axes: [Vector3<f32>; 3],
    pub eigenvalues: [f32; 3],
}

impl PCAResult {
    pub fn primary_axis(&self) -> Vector3<f32> {
        // Return the axis with largest eigenvalue
        // Simplified - assumes order is X, Y, Z
        self.axes[2]
    }
}
