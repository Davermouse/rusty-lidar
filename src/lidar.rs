use core::cell::{Cell, RefCell};

use defmt::*;
use embassy_sync::{blocking_mutex, mutex};
use embassy_time::{Duration, Timer};
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config};
use embassy_rp::peripherals::{DMA_CH0, PIO0, UART0};
use embedded_io_async::{Read, Write};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub struct LidarData {
    pub rpm: u16,
    pub distances: [u16; 360],
    pub intensities: [u16; 360],
}

pub static LIDAR_DATA: blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<LidarData>> =
    blocking_mutex::Mutex::new(RefCell::new(LidarData { rpm: 0, distances: [0 ; 360], intensities: [0 ; 360] }));

pub struct LidarManager<'d> {
    buffer: [u8;42],
    rx: BufferedUartRx<'d, UART0>,
    tx: BufferedUartTx<'d, UART0>
}

impl<'d> LidarManager<'d> {
    pub fn new(mut uart: BufferedUart<'d, UART0>) -> LidarManager<'d> {
        let (mut tx, mut rx) = uart.split();

        let mut lidar = Self {
            buffer: [0 ; 42],
            tx: tx,
            rx: rx,
        };

        return lidar;
    }

    async fn reset_lidar(&mut self) {
        self.tx.write("e".as_bytes()).await.unwrap();
        Timer::after_millis(500).await;
        self.tx.write("b".as_bytes()).await.unwrap();
    }

    async fn run_lidar(mut self) {
        self.reset_lidar().await;

        let mut buffer = [0; 42];

        loop {
            match self.rx.read_exact(&mut buffer).await {
                Err(err) => {
                    warn!("Error reading from LIDAR {}", err);
                    self.reset_lidar().await;
                    continue;
                }
                Ok(_) => {
                    if buffer[0] != 250 {
                        warn!("Unexpected buffer content");
                        continue;
                    }

                    let degree = (buffer[1] as usize - 0xA0) * 6;

                //  info!("Packet degree: {}", degree);

                    if buffer[41] != buffer[40] || buffer[40] == 0 {
                        warn!("Invalid data for angle {}", degree);
                        continue;
                    }

                    LIDAR_DATA.lock(|x| {
                        let mut lidar_readings = x.borrow_mut();

                        lidar_readings.rpm = ((buffer[3] as u16) << 8) | buffer[2] as u16;

                        for i in 0..6 {
                            let offset = 4 + i * 6;
        
                            let dist = ((buffer[offset + 3] as u16) << 8) | (buffer[offset + 2] as u16);
                            let intensity = ((buffer[offset + 1] as u16) << 8) | (buffer[offset] as u16);
        
                            lidar_readings.distances[degree + i] = dist;
                            lidar_readings.intensities[degree + i] = intensity;
                        }
                    });

                    LIDAR_DATA.lock(|x| {
                        let reading = x.borrow();
            
                        info!("Distance 270: {} {}", reading.distances[270], reading.intensities[270]);
                    });
                },
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lidar_task(mut uart: BufferedUart<'static, UART0>) {
    let lidar = LidarManager::new(uart);

    lidar.run_lidar().await;
}