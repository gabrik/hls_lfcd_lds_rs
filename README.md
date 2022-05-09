# Rust HLDS HLS-LFCD-LDS (LDS-01)



[![Crates](https://img.shields.io/crates/v/hls_lfcd_lds_driver.svg?style=flat-square)](https://crates.io/crates/hls_lfcd_lds_driver)
[![docs.rs](https://img.shields.io/badge/docs-latest-blue.svg?style=flat-square)](https://docs.rs/hls_lfcd_lds_driver)


This is a rust version of ROBOTIS HLDS HLS-LFCD-LDS (LDS-01) driver.
Please refer to the [ROBOTIS repository](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver) for more information.

## Example
Reading data from the lidar.


```rust
use clap::Parser;
use hls_lfcd_lds_driver::{LFCDLaser, DEFAULT_BAUD_RATE, DEFAULT_PORT};

#[derive(Parser, Debug)]
struct Args {
    #[clap(short, long, default_value = DEFAULT_PORT)]
    port: String,
    #[clap(short, long, default_value = DEFAULT_BAUD_RATE)]
    baud_rate: u32,
}

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
```