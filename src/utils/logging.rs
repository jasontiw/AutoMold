//! Logging utilities

use std::path::Path;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

/// Initialize logging with default settings
pub fn init_logging() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_target(true))
        .init();
}

/// Initialize logging with custom level
pub fn init_logging_with_level(level: &str) {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(level));

    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_target(true))
        .init();
}

/// Initialize logging to file
pub fn init_logging_to_file<P: AsRef<Path>>(path: P) -> Result<(), std::io::Error> {
    use tracing_subscriber::fmt::writer::MakeWriterExt;

    let file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path.as_ref())?;

    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    tracing_subscriber::registry()
        .with(filter)
        .with(
            tracing_subscriber::fmt::layer()
                .with_writer(file.and(std::io::stdout))
                .with_target(true),
        )
        .init();

    Ok(())
}

/// Progress indicator for long operations
pub struct Progress {
    current: usize,
    total: usize,
    message: String,
}

impl Progress {
    pub fn new(total: usize, message: &str) -> Self {
        Self {
            current: 0,
            total,
            message: message.to_string(),
        }
    }

    pub fn tick(&mut self) {
        self.current += 1;
        if self.current % 1000 == 0 || self.current == self.total {
            println!("{}: {}/{}", self.message, self.current, self.total);
        }
    }

    pub fn set(&mut self, current: usize) {
        self.current = current;
    }
}

/// Log configuration info
pub fn log_config(config: &crate::core::config::Config) {
    tracing::info!("Configuration:");
    tracing::info!("  Input: {:?}", config.input);
    if let Some(dir) = &config.output_dir {
        tracing::info!("  Output dir: {:?}", dir);
    }
    tracing::info!("  Open mold: {}", config.open_mold);
    tracing::info!("  Split axis: {:?}", config.split_axis);
    if let Some(wall) = config.wall_thickness {
        tracing::info!("  Wall thickness: {} mm", wall);
    }
    tracing::info!("  Tolerance: {} mm", config.tolerance);
    tracing::info!("  Generate pins: {}", config.generate_pins);
    tracing::info!("  Pour channel: {}", config.pour_channel);
    tracing::info!("  Shell mold: {}", config.shell_mold);
    tracing::info!("  Input unit: {}", config.input_unit.as_str());
    tracing::info!("  Output format: {:?}", config.output_format);
    if let Some(dec) = config.decimate {
        tracing::info!("  Decimate: {}", dec);
    }
    if let Some(mem) = config.memory_limit {
        tracing::info!("  Memory limit: {} MB", mem);
    }
    if let Some(threads) = config.threads {
        tracing::info!("  Threads: {}", threads);
    }
    tracing::info!("  Force: {}", config.force);
}

/// Log system info
pub fn log_system_info() {
    let mem = crate::utils::memory::get_memory_info();
    tracing::info!(
        "System memory: {:.0} MB total, {:.0} MB available",
        mem.total_mb(),
        mem.available_mb()
    );

    let threads = rayon::current_num_threads();
    tracing::info!("Available threads: {}", threads);
}

/// Format file size
pub fn format_size(bytes: usize) -> String {
    const KB: usize = 1024;
    const MB: usize = KB * 1024;
    const GB: usize = MB * 1024;

    if bytes >= GB {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.2} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.2} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}

/// Format duration in milliseconds
pub fn format_duration(ms: u64) -> String {
    if ms < 1000 {
        format!("{} ms", ms)
    } else {
        format!("{:.2} s", ms as f64 / 1000.0)
    }
}
