//! AutoMold - Automatic mold generation from 3D models
//! 
//! Main entry point

#![allow(unused)]

mod cli;
mod core;
mod geometry;
mod pipeline;
mod export;
mod utils;

use cli::args::Args;
use core::context::{Context, ExitCode};
use core::pipeline;
use utils::logging;
use clap::Parser;
use std::process;
use tracing::{info, error};

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
    match pipeline::run_pipeline(&mut ctx) {
        Ok(()) => {
            info!("Mold generation completed successfully");
            process::exit(ExitCode::Success.code());
        }
        Err((code, message)) => {
            error!("Error: {}", message);
            eprintln!("ERROR: {}", message);
            process::exit(code.code());
        }
    }
}