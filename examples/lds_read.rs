//
// Copyright (c) 2022 Gabriele Baldoni
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   Gabriele Baldoni, <gabriele@gabrielebaldoni.com>
//

use clap::Parser;
use ctrlc;
use hls_lfcd_lds_driver::{LFCDLaser, DEFAULT_BAUD_RATE, DEFAULT_PORT};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Parser, Debug)]
struct Args {
    #[clap(short, long, default_value = DEFAULT_PORT)]
    port: String,
    #[clap(short, long, default_value = DEFAULT_BAUD_RATE)]
    baud_rate: u32,
}

#[cfg(feature = "async_tokio")]
#[tokio::main]
async fn main() -> tokio_serial::Result<()> {
    let flag = Arc::new(AtomicBool::new(true));
    let c_flag = flag.clone();
    ctrlc::set_handler(move || c_flag.store(false, Ordering::Relaxed))
        .expect("Error when setting Ctrl-C handler");

    let args = Args::parse();
    println!(
        "Going to open LDS01 on {} with {}",
        args.port, args.baud_rate
    );

    let mut port = LFCDLaser::new(args.port, args.baud_rate)?;

    while flag.load(Ordering::Relaxed) {
        let reading = port.read().await?;
        println!("Reading: {reading:?}")
    }

    Ok(())
}

#[cfg(feature = "sync")]
fn main() -> serialport::Result<()> {
    let args = Args::parse();

    let flag = Arc::new(AtomicBool::new(true));
    let c_flag = flag.clone();
    ctrlc::set_handler(move || c_flag.store(false, Ordering::Relaxed))
        .expect("Error when setting Ctrl-C handler");

    println!(
        "Going to open LDS01 on {} with {}",
        args.port, args.baud_rate
    );

    let mut port = LFCDLaser::new(args.port, args.baud_rate)?;

    while flag.load(Ordering::Relaxed) {
        let reading = port.read()?;
        println!("Reading: {reading:?}")
    }

    Ok(())
}

#[cfg(feature = "async_smol")]
#[async_std::main]
async fn main() -> mio_serial::Result<()> {
    let args = Args::parse();
    let flag = Arc::new(AtomicBool::new(true));
    let c_flag = flag.clone();
    ctrlc::set_handler(move || c_flag.store(false, Ordering::Relaxed))
        .expect("Error when setting Ctrl-C handler");

    println!(
        "Going to open LDS01 on {} with {}",
        args.port, args.baud_rate
    );

    let mut port = LFCDLaser::new(args.port, args.baud_rate)?;

    while flag.load(Ordering::Relaxed) {
        let reading = port.read().await?;
        println!("Reading: {reading:?}")
    }

    Ok(())
}
