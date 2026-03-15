//! Memory utilities - memory detection and estimation

use sysinfo::System;

/// Get available system memory in bytes
pub fn get_available_memory() -> usize {
    let mut sys = System::new();
    sys.refresh_memory();

    // Use available memory (total - used)
    sys.available()
}

/// Get total system memory in bytes
pub fn get_total_memory() -> usize {
    let mut sys = System::new();
    sys.refresh_memory();
    sys.total_memory()
}

/// Get memory info
pub fn get_memory_info() -> MemoryInfo {
    let mut sys = System::new();
    sys.refresh_memory();

    MemoryInfo {
        total: sys.total_memory(),
        available: sys.available(),
        used: sys.used_memory(),
    }
}

/// Memory information
#[derive(Debug)]
pub struct MemoryInfo {
    pub total: usize,
    pub available: usize,
    pub used: usize,
}

impl MemoryInfo {
    pub fn available_mb(&self) -> f32 {
        self.available as f32 / (1024.0 * 1024.0)
    }

    pub fn total_mb(&self) -> f32 {
        self.total as f32 / (1024.0 * 1024.0)
    }

    pub fn used_mb(&self) -> f32 {
        self.used as f32 / (1024.0 * 1024.0)
    }
}

/// Estimate memory required for mesh processing
/// Formula: triangles * 470 bytes (per PRD)
pub fn estimate_mesh_memory(triangles: usize) -> usize {
    triangles * 470
}

/// Estimate memory for BVH construction
pub fn estimate_bvh_memory(triangles: usize) -> usize {
    triangles * 80
}

/// Estimate total memory for pipeline
pub fn estimate_pipeline_memory(triangles: usize) -> usize {
    let mesh = estimate_mesh_memory(triangles);
    let bvh = estimate_bvh_memory(triangles);
    let copy = estimate_mesh_memory(triangles);
    let result = triangles * 150;

    mesh + bvh + copy + result
}

/// Check if pipeline fits in memory budget
pub fn fits_in_memory(triangles: usize, budget_bytes: usize) -> bool {
    estimate_pipeline_memory(triangles) <= budget_bytes
}

/// Calculate optimal decimation ratio to fit memory
pub fn calculate_decimation_for_memory(triangles: usize, budget_bytes: usize) -> f32 {
    let needed = estimate_pipeline_memory(triangles);
    if needed <= budget_bytes {
        return 1.0;
    }

    // Budget / needed gives us the ratio, but with some safety margin
    let ratio = (budget_bytes as f32 / needed as f32) * 0.9;
    ratio.clamp(0.1, 1.0)
}
