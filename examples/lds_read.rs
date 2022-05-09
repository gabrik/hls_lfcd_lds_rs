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
use hls_lfcd_lds_driver::{LFCDLaser, DEFAULT_BAUD_RATE, DEFAULT_PORT};

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
    let args = Args::parse();
    println!(
        "Going to open LDS01 on {} with {}",
        args.port, args.baud_rate
    );

    let mut port = LFCDLaser::new(args.port, args.baud_rate)?;

    loop {
        let reading = port.read().await?;
        println!("Reading: {reading:?}")
    }
}

#[cfg(feature = "sync")]
fn main() -> serialport::Result<()> {
    let args = Args::parse();
    println!(
        "Going to open LDS01 on {} with {}",
        args.port, args.baud_rate
    );

    let mut port = LFCDLaser::new(args.port, args.baud_rate)?;

    loop {
        let reading = port.read()?;
        println!("Reading: {reading:?}")
    }
}


#[cfg(feature = "async_mio")]
#[async_std::main]
async fn main() -> mio_serial::Result<()> {
    let args = Args::parse();
    println!(
        "Going to open LDS01 on {} with {}",
        args.port, args.baud_rate
    );

    let mut port = LFCDLaser::new(args.port, args.baud_rate)?;

    loop {
        let reading = port.read().await?;
        println!("Reading: {reading:?}")
    }
}