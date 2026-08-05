//! CLI arguments using clap

use clap::{Parser, ValueEnum};
use std::path::PathBuf;

/// AutoMold CLI arguments
#[derive(Parser, Debug)]
#[command(name = "automold")]
#[command(version = env!("CARGO_PKG_VERSION"))]
#[command(about = "Automatic mold generation from 3D models", long_about = None)]
pub struct Args {
    /// Input model file (STL or OBJ)
    #[arg(value_name = "INPUT", required = true)]
    pub input: PathBuf,

    /// Generate open mold (without lid)
    #[arg(long)]
    pub open_mold: bool,

    /// Axis for mold split (X, Y, or Z)
    #[arg(long, value_enum, default_value = "auto")]
    pub split_axis: SplitAxisArg,

    /// Wall thickness in mm (default: auto-calculated)
    #[arg(long, value_name = "mm")]
    pub wall: Option<f32>,

    /// Tolerance/offset for cavity in mm (default: 0.2)
    #[arg(long, value_name = "mm", default_value = "0.2")]
    pub tolerance: f32,

    /// Generate alignment pins
    #[arg(long)]
    pub pins: bool,

    /// Generate pour channel
    #[arg(long)]
    pub pour: bool,

    /// Generate hollow shell mold (Fase 3)
    #[arg(long)]
    pub shell: bool,

    /// Input model unit
    #[arg(long, value_enum, default_value = "mm")]
    pub unit: UnitArg,

    /// Output format
    #[arg(long, value_enum, default_value = "stl")]
    pub format: FormatArg,

    /// Decimation ratio (0.0 - 1.0)
    #[arg(long, value_name = "ratio")]
    pub decimate: Option<f32>,

    /// Memory limit in MB (default: auto-detect)
    ///
    /// The raw MB value is converted to bytes in the `From<Args>` impl, so
    /// `Config::memory_limit` always holds bytes.
    #[arg(long, value_name = "MB")]
    pub memory_limit: Option<usize>,

    /// Number of threads (default: auto)
    #[arg(long, value_name = "n")]
    pub threads: Option<usize>,

    /// Force processing even with memory warnings
    #[arg(long, short)]
    pub force: bool,

    /// Output directory
    #[arg(long, short, value_name = "DIR")]
    pub output: Option<PathBuf>,

    /// Verbose output
    #[arg(long, short)]
    pub verbose: bool,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum SplitAxisArg {
    Auto,
    X,
    Y,
    Z,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum UnitArg {
    Mm,
    Cm,
    In,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum FormatArg {
    Stl,
    #[clap(name = "3mf")]
    ThreeMF,
}

impl From<Args> for crate::core::config::Config {
    fn from(args: Args) -> Self {
        use crate::core::config::*;

        let split_axis = match args.split_axis {
            SplitAxisArg::Auto => SplitAxis::Auto,
            SplitAxisArg::X => SplitAxis::X,
            SplitAxisArg::Y => SplitAxis::Y,
            SplitAxisArg::Z => SplitAxis::Z,
        };

        let input_unit = match args.unit {
            UnitArg::Mm => Unit::Millimeters,
            UnitArg::Cm => Unit::Centimeters,
            UnitArg::In => Unit::Inches,
        };

        let output_format = match args.format {
            FormatArg::Stl => OutputFormat::Stl,
            FormatArg::ThreeMF => OutputFormat::ThreeMF,
        };

        Self {
            input: args.input,
            output_dir: args.output,
            open_mold: args.open_mold,
            split_axis,
            wall_thickness: args.wall,
            tolerance: args.tolerance,
            generate_pins: args.pins,
            pour_channel: args.pour,
            shell_mold: args.shell,
            input_unit,
            output_format,
            decimate: args.decimate,
            // The CLI flag is documented in MB; convert once here so
            // `Config::memory_limit` is always in bytes.
            memory_limit: args.memory_limit.map(|mb| mb * 1024 * 1024),
            threads: args.threads,
            force: args.force,
        }
    }
}

impl From<&Args> for crate::core::config::Config {
    fn from(args: &Args) -> Self {
        // Clone and use the owned From implementation
        let owned_args = Args {
            input: args.input.clone(),
            open_mold: args.open_mold,
            split_axis: args.split_axis.clone(),
            wall: args.wall,
            tolerance: args.tolerance,
            pins: args.pins,
            pour: args.pour,
            shell: args.shell,
            output: args.output.clone(),
            unit: args.unit.clone(),
            format: args.format.clone(),
            decimate: args.decimate,
            // Do NOT convert MB -> bytes here: this is the raw Args value that
            // the owned `From<Args>` impl converts. Multiplying again would
            // double-convert (MB -> bytes^2).
            memory_limit: args.memory_limit,
            threads: args.threads,
            verbose: args.verbose,
            force: args.force,
        };
        crate::core::config::Config::from(owned_args)
    }
}

/// Convert CLI args to runtime config
impl Args {
    pub fn to_config(&self) -> crate::core::config::Config {
        crate::core::config::Config::from(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_with_memory_limit() -> Args {
        Args::parse_from(["automold", "in.stl", "--memory-limit", "8192"])
    }

    /// The owned `From<Args>` impl must convert the documented MB value to bytes.
    #[test]
    fn test_memory_limit_mb_conversion_owned() {
        let config = crate::core::config::Config::from(parse_with_memory_limit());
        assert_eq!(
            config.memory_limit,
            Some(8192 * 1024 * 1024),
            "--memory-limit 8192 must be stored as 8192 MiB in bytes"
        );
    }

    /// The `From<&Args>` impl delegates to the owned impl and must NOT
    /// double-convert (MB -> bytes applied twice).
    #[test]
    fn test_memory_limit_mb_conversion_ref() {
        let args = parse_with_memory_limit();
        let config = crate::core::config::Config::from(&args);
        assert_eq!(
            config.memory_limit,
            Some(8192 * 1024 * 1024),
            "&Args conversion must produce the same byte budget as owned conversion"
        );
    }
}
