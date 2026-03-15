//! AutoMold - Automatic mold generation from 3D models
//!
//! Main entry point

#![allow(unused)]

mod cli;
mod core;
mod export;
mod geometry;
mod pipeline;
mod utils;

use clap::Parser;
use cli::args::Args;
use core::context::{Context, ExitCode};
use pipeline::pipeline_core::run_pipeline;
use std::process;
use tracing::{error, info};
use utils::logging;

fn main() {
    // Parse command line arguments
    let args = Args::parse();

    // Initialize logging
    if args.verbose {
        logging::init_logging_with_level("debug");
    } else {
        logging::init_logging();
    }

    // Convert to config
    let config = args.to_config();

    // Log system info
    info!("AutoMold v{}", env!("CARGO_PKG_VERSION"));
    logging::log_system_info();

    // Create processing context
    let mut ctx = Context::new(config);

    // Print decision log
    ctx.decisions.print();

    // Run pipeline
    match run_pipeline(&mut ctx) {
        Ok(()) => {
            info!("Mold generation completed successfully");
            process::exit(ExitCode::Success.code() as i32);
        }
        Err(err_tuple) => {
            let (code, message): (ExitCode, String) = err_tuple;
            error!("Error: {}", message);
            eprintln!("ERROR: {}", message);
            let exit_code: i32 = code.code();
            process::exit(exit_code);
        }
    }
}
