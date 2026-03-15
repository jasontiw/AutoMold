//! Pipeline context - maintains state during processing

use crate::core::config::{Config, DecisionLog, SplitAxis, Stats, Unit};
use crate::geometry::mesh::Mesh;
use std::path::PathBuf;
use std::time::Instant;

/// Processing context - holds all state during mold generation
pub struct Context {
    /// Configuration
    pub config: Config,

    /// Current mesh being processed
    pub mesh: Option<Mesh>,

    /// Original mesh (before decimation for orientation analysis)
    pub original_mesh: Option<Mesh>,

    /// Statistics
    pub stats: Stats,

    /// Decision log for output
    pub decisions: DecisionLog,

    /// Bounding box of the model
    pub bounding_box: Option<crate::geometry::bbox::BoundingBox>,

    /// Detected available memory in bytes
    pub available_memory: usize,

    /// Start time for timing
    pub start_time: Option<Instant>,

    /// Suggested split axis after analysis
    pub suggested_split_axis: Option<SplitAxis>,
}

impl Context {
    pub fn new(config: Config) -> Self {
        let available_memory = crate::utils::memory::get_available_memory();

        Self {
            config,
            mesh: None,
            original_mesh: None,
            stats: Stats::default(),
            decisions: DecisionLog {
                split_axis: "Auto".to_string(),
                wall_thickness: 0.0,
                tolerance: 0.2,
                pins_enabled: true,
                pour_channel_enabled: false,
                unit: "mm".to_string(),
                memory_budget_mb: available_memory / (1024 * 1024),
                threads: 0,
                auto_decimate: None,
                auto_decimate_reason: None,
            },
            bounding_box: None,
            available_memory,
            start_time: None,
            suggested_split_axis: None,
        }
    }

    /// Calculate wall thickness based on bounding box
    pub fn calculate_wall_thickness(&self) -> f32 {
        if let Some(bbox) = &self.bounding_box {
            let size = bbox.size();
            // Rule of thumb: wall thickness = 10% of largest dimension, min 6mm, max 20mm
            let max_dim = size.x.max(size.y).max(size.z);
            (max_dim * 0.1).clamp(6.0, 20.0)
        } else {
            12.0 // default
        }
    }

    /// Calculate memory estimate for the current mesh
    pub fn estimate_memory(&self) -> usize {
        let triangles = self.mesh.as_ref().map(|m| m.triangles.len()).unwrap_or(0);
        // Formula: triangles * 470 bytes (per PRD)
        triangles * 470
    }

    /// Check if auto-decimation is needed
    pub fn needs_auto_decimate(&self) -> bool {
        let estimated = self.estimate_memory();
        let limit = self
            .config
            .memory_limit
            .unwrap_or(self.available_memory * 75 / 100);
        estimated > limit
    }

    /// Get auto-decimation ratio based on memory budget
    pub fn auto_decimate_ratio(&self) -> f32 {
        let estimated = self.estimate_memory();
        let limit = self
            .config
            .memory_limit
            .unwrap_or(self.available_memory * 75 / 100);

        if estimated <= limit {
            return 1.0;
        }

        // Calculate ratio to fit in budget
        let ratio = limit as f32 / estimated as f32;
        ratio.clamp(0.3, 0.9)
    }

    /// Record start time
    pub fn start(&mut self) {
        self.start_time = Some(Instant::now());
    }

    /// Get elapsed time in milliseconds
    pub fn elapsed_ms(&self) -> Option<u64> {
        self.start_time.map(|t| t.elapsed().as_millis() as u64)
    }

    /// Get output path for a file
    pub fn output_path(&self, name: &str, extension: &str) -> PathBuf {
        let dir = self.config.output_dir.clone().unwrap_or_else(|| {
            self.config
                .input
                .parent()
                .map(|p| p.to_path_buf())
                .unwrap_or_default()
        });

        let stem = self
            .config
            .input
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "model".to_string());

        dir.join(format!("{}_{}.{}", stem, name, extension))
    }
}

/// Exit codes as per PRD
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExitCode {
    Success = 0,
    FileNotFound = 1,
    UnsupportedFormat = 2,
    MeshUnrecoverable = 3,
    BooleanFailed = 4,
    InvalidArgument = 5,
    ScaleWarning = 6,
    OutOfMemory = 7,
}

impl ExitCode {
    pub fn code(&self) -> i32 {
        *self as i32
    }
}

impl std::fmt::Display for ExitCode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExitCode::Success => write!(f, "Success"),
            ExitCode::FileNotFound => write!(f, "File not found"),
            ExitCode::UnsupportedFormat => write!(f, "Unsupported format"),
            ExitCode::MeshUnrecoverable => write!(f, "Mesh unrecoverable"),
            ExitCode::BooleanFailed => write!(f, "Boolean operation failed"),
            ExitCode::InvalidArgument => write!(f, "Invalid argument"),
            ExitCode::ScaleWarning => write!(f, "Scale warning"),
            ExitCode::OutOfMemory => write!(f, "Out of memory"),
        }
    }
}
