//! AutoMold - Automatic mold generation from 3D models
//!
//! Core configuration module - defines runtime parameters and limits

use std::path::PathBuf;

/// Runtime configuration for AutoMold
#[derive(Debug, Clone)]
pub struct Config {
    /// Input file path
    pub input: PathBuf,

    /// Output directory (defaults to current directory)
    pub output_dir: Option<PathBuf>,

    /// Generate open mold (without lid)
    pub open_mold: bool,

    /// Axis for mold split (X, Y, or Z)
    pub split_axis: SplitAxis,

    /// Wall thickness in mm (auto-calculated if None)
    pub wall_thickness: Option<f32>,

    /// Tolerance/offset for cavity in mm
    pub tolerance: f32,

    /// Generate alignment pins
    pub generate_pins: bool,

    /// Generate pour channel
    pub pour_channel: bool,

    /// Generate hollow shell mold
    pub shell_mold: bool,

    /// Input model unit
    pub input_unit: Unit,

    /// Output format
    pub output_format: OutputFormat,

    /// Decimation ratio (0.0 - 1.0)
    pub decimate: Option<f32>,

    /// Memory limit in MB (auto-detected if None)
    pub memory_limit: Option<usize>,

    /// Number of threads (auto-detected if None)
    pub threads: Option<usize>,

    /// Force processing even with memory warnings
    pub force: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            input: PathBuf::new(),
            output_dir: None,
            open_mold: false,
            split_axis: SplitAxis::Auto,
            wall_thickness: None,
            tolerance: 0.2,
            generate_pins: true,
            pour_channel: false,
            shell_mold: false,
            input_unit: Unit::Millimeters,
            output_format: OutputFormat::Stl,
            decimate: None,
            memory_limit: None,
            threads: None,
            force: false,
        }
    }
}

/// Axis for mold splitting
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SplitAxis {
    #[default]
    Auto,
    X,
    Y,
    Z,
}

impl SplitAxis {
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "x" => Some(SplitAxis::X),
            "y" => Some(SplitAxis::Y),
            "z" => Some(SplitAxis::Z),
            "auto" => Some(SplitAxis::Auto),
            _ => None,
        }
    }
}

/// Input/Output unit
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Unit {
    #[default]
    Millimeters,
    Centimeters,
    Inches,
}

impl Unit {
    /// Convert to millimeters
    pub fn to_mm(&self) -> f32 {
        match self {
            Unit::Millimeters => 1.0,
            Unit::Centimeters => 10.0,
            Unit::Inches => 25.4,
        }
    }

    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "mm" => Some(Unit::Millimeters),
            "cm" => Some(Unit::Centimeters),
            "in" | "inch" | "inches" => Some(Unit::Inches),
            _ => None,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Unit::Millimeters => "mm",
            Unit::Centimeters => "cm",
            Unit::Inches => "in",
        }
    }
}

/// Output file format
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum OutputFormat {
    #[default]
    Stl,
    ThreeMF,
}

impl OutputFormat {
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "stl" => Some(OutputFormat::Stl),
            "3mf" => Some(OutputFormat::ThreeMF),
            _ => None,
        }
    }
}

/// Processing statistics
#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub triangles_in: usize,
    pub triangles_out: Option<usize>,
    pub holes_filled: usize,
    pub normals_fixed: usize,
    pub non_manifold_edges: usize,
    pub decimation_ratio: Option<f32>,
    pub memory_used_mb: Option<f32>,
    pub processing_time_ms: Option<u64>,
}

impl std::fmt::Display for Stats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Input triangles: {}", self.triangles_in)?;

        if let Some(out) = self.triangles_out {
            writeln!(f, "Output triangles: {}", out)?;
        }

        if self.holes_filled > 0 {
            writeln!(f, "Holes filled: {}", self.holes_filled)?;
        }

        if self.normals_fixed > 0 {
            writeln!(f, "Normals fixed: {}", self.normals_fixed)?;
        }

        if let Some(ratio) = self.decimation_ratio {
            writeln!(f, "Decimation: {:.0}%", ratio * 100.0)?;
        }

        if let Some(mem) = self.memory_used_mb {
            writeln!(f, "Memory used: {:.1} MB", mem)?;
        }

        if let Some(time) = self.processing_time_ms {
            writeln!(f, "Processing time: {:.1}s", time as f32 / 1000.0)?;
        }

        Ok(())
    }
}

/// Processing decision log - shows what decisions were made automatically
#[derive(Debug, Clone, Default)]
pub struct DecisionLog {
    pub split_axis: String,
    pub wall_thickness: f32,
    pub tolerance: f32,
    pub pins_enabled: bool,
    pub pour_channel_enabled: bool,
    pub unit: String,
    pub memory_budget_mb: usize,
    pub threads: usize,
    pub auto_decimate: Option<f32>,
    pub auto_decimate_reason: Option<String>,
    pub boolean_strategy: Option<String>,
    pub boolean_quality_score: Option<f32>,
    pub watertight: Option<bool>,
}

impl DecisionLog {
    pub fn print(&self) {
        println!("Decisions:");
        println!("  Split axis:      {} (auto)", self.split_axis);
        println!("  Wall thickness:  {:.0}mm (auto)", self.wall_thickness);
        println!("  Tolerance:       {:.1}mm (auto)", self.tolerance);
        println!(
            "  Pins:            {}",
            if self.pins_enabled {
                "enabled (auto)"
            } else {
                "disabled"
            }
        );
        println!(
            "  Pour channel:    {}",
            if self.pour_channel_enabled {
                "enabled"
            } else {
                "disabled"
            }
        );
        println!("  Unit:            {} (default)", self.unit);
        println!("  Memory budget:   {}MB available", self.memory_budget_mb);
        if let Some(ratio) = self.auto_decimate {
            println!("  Auto-decimate:  {:.0}% (budget exceeded)", ratio * 100.0);
            if let Some(ref reason) = self.auto_decimate_reason {
                println!("                  Reason: {}", reason);
            }
        }
        println!("  Threads:         {} (auto)", self.threads);
    }
}
