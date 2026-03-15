//! Bounding box geometry

use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};

/// Axis-aligned bounding box
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self {
            min: Point3::new(f32::MAX, f32::MAX, f32::MAX),
            max: Point3::new(f32::MIN, f32::MIN, f32::MIN),
        }
    }
}

impl BoundingBox {
    /// Create bounding box from a list of points
    pub fn from_points(points: &[Point3<f32>]) -> Self {
        if points.is_empty() {
            return Self::default();
        }

        let mut min = Point3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut max = Point3::new(f32::MIN, f32::MIN, f32::MIN);

        for p in points {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        Self { min, max }
    }

    /// Get center point
    pub fn center(&self) -> Point3<f32> {
        Point3::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }

    /// Get size (width, height, depth)
    pub fn size(&self) -> Vector3<f32> {
        Vector3::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Get the maximum dimension
    pub fn max_dimension(&self) -> f32 {
        self.size().x.max(self.size().y).max(self.size().z)
    }

    /// Check if a point is inside the bounding box
    pub fn contains(&self, point: &Point3<f32>) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Expand the bounding box to include a point
    pub fn expand_by_point(&mut self, point: &Point3<f32>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    /// Expand by a margin on all sides
    pub fn expand(&mut self, margin: f32) {
        self.min.x -= margin;
        self.min.y -= margin;
        self.min.z -= margin;
        self.max.x += margin;
        self.max.y += margin;
        self.max.z += margin;
    }

    /// Get the 8 corners of the bounding box
    pub fn corners(&self) -> [Point3<f32>; 8] {
        [
            Point3::new(self.min.x, self.min.y, self.min.z),
            Point3::new(self.max.x, self.min.y, self.min.z),
            Point3::new(self.min.x, self.max.y, self.min.z),
            Point3::new(self.max.x, self.max.y, self.min.z),
            Point3::new(self.min.x, self.min.y, self.max.z),
            Point3::new(self.max.x, self.min.y, self.max.z),
            Point3::new(self.min.x, self.max.y, self.max.z),
            Point3::new(self.max.x, self.max.y, self.max.z),
        ]
    }

    /// Check intersection with another bounding box
    pub fn intersects(&self, other: &BoundingBox) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Create a bounding box that contains both this and another
    pub fn union(&self, other: &BoundingBox) -> BoundingBox {
        BoundingBox {
            min: Point3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Point3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(-1.0, -2.0, -3.0),
        ];

        let bbox = BoundingBox::from_points(&points);

        assert_eq!(bbox.min, Point3::new(-1.0, -2.0, -3.0));
        assert_eq!(bbox.max, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(bbox.size(), Vector3::new(2.0, 4.0, 6.0));
        assert_eq!(bbox.center(), Point3::new(0.0, 0.0, 0.0));
    }
}
