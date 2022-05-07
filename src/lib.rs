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

use tokio::io::AsyncReadExt;
use tokio_serial::SerialPortBuilderExt;
use tokio_serial::SerialStream;

pub static DEFAULT_PORT: &str = "/dev/ttyUSB0";
pub static DEFAULT_BAUD_RATE: &str = "230400";

#[derive(Debug)]
pub struct LaserReading {
    pub ranges: [u16; 360],
    pub intensities: [u16; 360],
    pub rpms: u16,
}

impl LaserReading {
    pub fn new() -> Self {
        Self {
            ranges: [0u16; 360],
            intensities: [0u16; 360],
            rpms: 0,
        }
    }
}

impl Default for LaserReading {
    fn default() -> Self {
        Self::new()
    }
}

pub struct LFCDLaser {
    port: String,
    baud_rate: u32,
    shutting_down: bool,
    motor_speed: u16,
    rpms: u16,
    serial: SerialStream,
    buff: [u8; 2520],
}

impl LFCDLaser {
    pub fn new(port: String, baud_rate: u32) -> tokio_serial::Result<Self> {
        let mut serial = tokio_serial::new(port.clone(), baud_rate).open_native_async()?;

        #[cfg(unix)]
        serial.set_exclusive(false)?;

        Ok(Self {
            port,
            baud_rate,
            shutting_down: false,
            motor_speed: 0,
            rpms: 0,
            serial,
            buff: [0u8; 2520],
        })
    }

    pub fn close(&mut self) {
        self.shutting_down = true;
    }

    pub fn speed(&self) -> u16 {
        self.motor_speed
    }

    pub fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    pub fn port(&self) -> String {
        self.port.clone()
    }

    pub fn rpms(&self) -> u16 {
        self.rpms
    }

    pub async fn read(&mut self) -> tokio_serial::Result<LaserReading> {
        let mut start_count: usize = 0;
        let mut good_sets: u8 = 0;

        let mut scan = LaserReading::new();

        if self.shutting_down {
            return Err(tokio_serial::Error::new(
                tokio_serial::ErrorKind::Unknown,
                "Driver is closed",
            ));
        }

        loop {
            // Wait for data sync of frame: 0xFA, 0XA0

            // Read one byte
            self.serial
                .read_exact(std::slice::from_mut(&mut self.buff[start_count]))
                .await?;
            //println!("start_count : {start_count} = Read {:02X?}", buff[start_count]);
            if start_count == 0 {
                if self.buff[start_count] == 0xFA {
                    start_count = 1
                }
            } else if start_count == 1 {
                if self.buff[start_count] == 0xA0 {
                    self.serial.read_exact(&mut self.buff[2..]).await?;

                    //read data in sets of 6

                    for i in (0..self.buff.len()).step_by(42) {
                        if self.buff[i] == 0xFA && usize::from(self.buff[i + 1]) == (0xA0 + i / 42)
                        {
                            good_sets = good_sets.wrapping_add(1);

                            let b_rmp0: u16 = self.buff[i + 3] as u16;
                            let b_rmp1: u16 = self.buff[i + 2] as u16;

                            // motor_speed = motor_speed.wrapping_add((b_rmp0 as u32) << 8 + (b_rmp1 as u32)); // accumulate count for avg. time increment
                            let rpms = (b_rmp0 << 8 | b_rmp1) / 10;
                            scan.rpms = rpms;
                            self.rpms = rpms;

                            for j in ((i + 4)..(i + 40)).step_by(6) {
                                let index = 6 * (i / 42) + (j - 4 - i) / 6;
                                // Four bytes `per reading
                                let b0: u16 = self.buff[j] as u16;
                                let b1: u16 = self.buff[j + 1] as u16;
                                let b2: u16 = self.buff[j + 2] as u16;
                                let b3: u16 = self.buff[j + 3] as u16;

                                // Remaining bits are the range in mm
                                let range: u16 = (b3 << 8) + b2;

                                // Last two bytes represents the uncertanity or intensity, might also
                                // be pixel area of target...
                                // let intensity = (b3 << 8) + b2;
                                let intensity: u16 = (b1 << 8) + b0;

                                scan.ranges[359 - index] = range;
                                scan.intensities[359 - index] = intensity;
                            }
                        }
                    }

                    // self.time_increment = motor_speed/good_sets/1e8;
                    return Ok(scan);
                } else {
                    start_count = 0;
                }
            }
        }
    }
}
