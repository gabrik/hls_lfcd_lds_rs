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

//! hls_lfcd_lds_driver provides a rust version of the LDS01 driver from robotis.
//! This crate facilitates reading information from that specific lidar.

#[cfg(feature = "async_tokio")]
use tokio::io::AsyncReadExt;
#[cfg(feature = "async_tokio")]
use tokio_serial::{SerialPortBuilderExt, SerialStream};

#[cfg(feature = "async_smol")]
use futures::prelude::*;
#[cfg(feature = "async_smol")]
use mio_serial::{SerialPortBuilderExt, SerialStream};
#[cfg(feature = "async_smol")]
use smol::Async;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "serde")]
use serde_big_array::BigArray;

#[cfg(feature = "sync")]
use std::io::Read;

#[cfg(feature = "sync")]
use serialport::TTYPort;

/// Default serial port of the lidar
pub static DEFAULT_PORT: &str = "/dev/ttyUSB0";
/// Default baud_rate of the lidar
pub static DEFAULT_BAUD_RATE: &str = "230400";

/// Byte sent to stop the lidar, 101 = ASCII 'e'
static STOP_BYTE: u8 = 101;

/// Byte sent to start the lidar, 98 = ASCII 'b'
static START_BYTE: u8 = 98;

/// This struct contains the reading from the lidar.
/// The `ranges` array contains 360 elements, one for each degree,
/// with a value from 0 to 1000, indicating the distance.
///
/// The `intensites` array contains 360 elements, one for each degree,
/// with a value, indicating accuracy of the reading
///
/// The `rmps` field gets the lidar RPMs
#[cfg(feature = "ser_de")]
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct LaserReading {
    #[serde(with = "BigArray")]
    pub ranges: [u16; 360],
    #[serde(with = "BigArray")]
    pub intensities: [u16; 360],
    pub rpms: u16,
}

/// This struct contains the reading from the lidar.
/// The `ranges` array contains 360 elements, one for each degree,
/// with a value from 0 to 1000, indicating the distance.
///
/// The `intensites` array contains 360 elements, one for each degree,
/// with a value, indicating accuracy of the reading
///
/// The `rmps` field gets the lidar RPMs
#[cfg(not(feature = "ser_de"))]
#[derive(Debug, Clone)]
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

/// This struct allows to read lidar information and to "shutdown" the driver

pub struct LFCDLaser {
    port: String,
    baud_rate: u32,
    shutting_down: bool,
    motor_speed: u16,
    rpms: u16,
    #[cfg(feature = "async_tokio")]
    serial: SerialStream,
    #[cfg(feature = "async_smol")]
    serial: Async<SerialStream>,
    #[cfg(feature = "sync")]
    serial: TTYPort,
    buff: [u8; 2520],
}

impl LFCDLaser {
    /// Creates the `LFCDLaser`
    pub fn close(&mut self) {
        self.shutting_down = true;

        // Stopping the Lidar, ignoring the result.
        #[cfg(not(feature = "async_smol"))]
        std::io::Write::write_all(&mut self.serial, &[STOP_BYTE]).ok();
        #[cfg(feature = "async_smol")]
        std::io::Write::write_all(&mut self.serial.get_mut(), &[STOP_BYTE]).ok();
    }

    /// Gets lidar speed.
    pub fn speed(&self) -> u16 {
        self.motor_speed
    }

    /// Gets the configured baud rate
    pub fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    /// Gets the configured serial port
    pub fn port(&self) -> String {
        self.port.clone()
    }

    /// Gets the lidars rmp from the last reading
    pub fn rpms(&self) -> u16 {
        self.rpms
    }

    // Starts the Lidar
    pub fn start(&mut self) {
        // Starting the Lidar
        #[cfg(not(feature = "async_smol"))]
        std::io::Write::write_all(&mut self.serial, &[START_BYTE]).ok();

        #[cfg(feature = "async_smol")]
        std::io::Write::write_all(&mut self.serial.get_mut(), &[START_BYTE]).ok();

        self.shutting_down = false;
    }
}

impl Drop for LFCDLaser {
    fn drop(&mut self) {
        self.close();
    }
}

#[cfg(feature = "async_tokio")]
impl LFCDLaser {
    /// Creates a new `LFCDLaser` with the given parameters.
    ///
    /// # Errors
    /// An error variant is returned in case of:
    /// - unable to open the specified serial port
    /// - unable to set the port to non-exclusive (only on unix)
    pub fn new(port: String, baud_rate: u32) -> tokio_serial::Result<Self> {
        let mut serial = tokio_serial::new(port.clone(), baud_rate).open_native_async()?;

        #[cfg(unix)]
        serial.set_exclusive(false)?;

        let mut lidar = Self {
            port,
            baud_rate,
            shutting_down: false,
            motor_speed: 0,
            rpms: 0,
            serial,
            buff: [0u8; 2520],
        };

        lidar.start();

        Ok(lidar)
    }

    /// Gets a reading from the lidar, returing a `LaserReading` object.
    ///
    /// # Errors
    /// An error variant is returned in case of:
    /// - unable to read form the serial port
    /// - the driver is closed
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

#[cfg(feature = "sync")]
impl LFCDLaser {
    /// Creates a new `LFCDLaser` with the given parameters.
    ///
    /// # Errors
    /// An error variant is returned in case of:
    /// - unable to open the specified serial port
    /// - unable to set the port to non-exclusive (only on unix)
    pub fn new(port: String, baud_rate: u32) -> serialport::Result<Self> {
        let mut serial = serialport::new(port.clone(), baud_rate).open_native()?;

        #[cfg(unix)]
        serial.set_exclusive(false)?;

        let mut lidar = Self {
            port,
            baud_rate,
            shutting_down: false,
            motor_speed: 0,
            rpms: 0,
            serial,
            buff: [0u8; 2520],
        };

        lidar.start();

        Ok(lidar)
    }

    /// Gets a reading from the lidar, returing a `LaserReading` object.
    ///
    /// # Errors
    /// An error variant is returned in case of:
    /// - unable to read form the serial port
    /// - the driver is closed
    pub fn read(&mut self) -> serialport::Result<LaserReading> {
        let mut start_count: usize = 0;
        let mut good_sets: u8 = 0;

        let mut scan = LaserReading::new();

        if self.shutting_down {
            return Err(serialport::Error::new(
                serialport::ErrorKind::Unknown,
                "Driver is closed",
            ));
        }

        loop {
            // Wait for data sync of frame: 0xFA, 0XA0

            // Read one byte
            self.serial
                .read_exact(std::slice::from_mut(&mut self.buff[start_count]))?;
            //println!("start_count : {start_count} = Read {:02X?}", buff[start_count]);
            if start_count == 0 {
                if self.buff[start_count] == 0xFA {
                    start_count = 1
                }
            } else if start_count == 1 {
                if self.buff[start_count] == 0xA0 {
                    self.serial.read_exact(&mut self.buff[2..])?;

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

#[cfg(feature = "async_smol")]
impl LFCDLaser {
    /// Creates a new `LFCDLaser` with the given parameters.
    ///
    /// # Errors
    /// An error variant is returned in case of:
    /// - unable to open the specified serial port
    /// - unable to set the port to non-exclusive (only on unix)
    pub fn new(port: String, baud_rate: u32) -> mio_serial::Result<Self> {
        let mut serial = mio_serial::new(port.clone(), baud_rate).open_native_async()?;

        #[cfg(unix)]
        serial.set_exclusive(false)?;

        // Wrapping into smol::Async to make it "async", similar to what tokio-serial does.
        let serial = Async::new(serial).map_err(|e| {
            mio_serial::Error::new(
                mio_serial::ErrorKind::Unknown,
                format!("Unable to wrap mio-serial in smol::Async: {e}"),
            )
        })?;

        let mut lidar = Self {
            port,
            baud_rate,
            shutting_down: false,
            motor_speed: 0,
            rpms: 0,
            serial,
            buff: [0u8; 2520],
        };

        lidar.start();

        Ok(lidar)
    }

    /// Gets a reading from the lidar, returing a `LaserReading` object.
    ///
    /// # Errors
    /// An error variant is returned in case of:
    /// - unable to read form the serial port
    /// - the driver is closed
    pub async fn read(&mut self) -> mio_serial::Result<LaserReading> {
        let mut start_count: usize = 0;
        let mut good_sets: u8 = 0;

        let mut scan = LaserReading::new();

        if self.shutting_down {
            return Err(mio_serial::Error::new(
                mio_serial::ErrorKind::Unknown,
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
