use core::cell::RefCell;

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex;
use embassy_time::Timer;
use embassy_rp::uart::{self, BufferedUart, BufferedUartRx, BufferedUartTx, Instance};
use embassy_rp::peripherals::UART0;
use embedded_io_async::{Read, Write};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;

use crate::Irqs;

pub struct LidarData {
    pub rpm: u16,
    pub distances: [u16; 360],
    pub intensities: [u16; 360],
}

pub static LIDAR_DATA: blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<LidarData>> =
    blocking_mutex::Mutex::new(RefCell::new(LidarData { rpm: 0, distances: [0 ; 360], intensities: [0 ; 360] }));

pub struct LidarManager<'d, UART>
where UART: Instance {
    buffer: [u8;42],
    rx: BufferedUartRx<'d, UART>,
    tx: BufferedUartTx<'d, UART>
}

impl<'d, UART> LidarManager<'d, UART>
where UART: Instance {
    pub fn new(uart: BufferedUart<'d, UART>) -> LidarManager<'d, UART> {
        let (tx, rx) = uart.split();

        let lidar = Self {
            buffer: [0 ; 42],
            tx: tx,
            rx: rx,
        };

        return lidar;
    }

    async fn reset_lidar(&mut self) {
        warn!("Resetting LIDAR");

        self.tx.write("e".as_bytes()).await.unwrap();
        Timer::after_millis(1000).await;
        self.tx.write("b".as_bytes()).await.unwrap();
    }

    async fn run_lidar(mut self) {
        self.reset_lidar().await;

        loop {
            match self.rx.read_exact(&mut self.buffer).await {
                Err(err) => {
                    warn!("Error reading from LIDAR {}", err);
                    self.reset_lidar().await;
                    continue;
                }
                Ok(_) => {
                    if self.buffer[0] != 250 {
                        warn!("Unexpected buffer content");
                        self.reset_lidar().await;

                        continue;
                    }

                    let degree = (self.buffer[1] as usize - 0xA0) * 6;

                    if self.buffer[41] != self.buffer[40] || self.buffer[40] == 0 {
                        warn!("Invalid data for angle {}", degree);
                        continue;
                    }

                    LIDAR_DATA.lock(|x| {
                        let mut lidar_readings = x.borrow_mut();

                        lidar_readings.rpm = ((self.buffer[3] as u16) << 8) | self.buffer[2] as u16;

                        for i in 0..6 {
                            let offset = 4 + i * 6;
        
                            let dist = ((self.buffer[offset + 3] as u16) << 8) | (self.buffer[offset + 2] as u16);
                            let intensity = ((self.buffer[offset + 1] as u16) << 8) | (self.buffer[offset] as u16);
        
                            lidar_readings.distances[degree + i] = dist;
                            lidar_readings.intensities[degree + i] = intensity;
                        }
                    });
                },
            }
        }
    }
}

pub fn setup_lidar(r: crate::LidarResources, irqs: Irqs, spawner: Spawner) {
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 230400;

    static TX_BUF: StaticCell<[u8; 8]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 8])[..];
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 256])[..];
    let uart = BufferedUart::new(r.uart, irqs, r.tx, r.rx, tx_buf, rx_buf, uart_config);

    unwrap!(spawner.spawn(lidar_task(uart)));
}

#[embassy_executor::task]
pub async fn lidar_task(uart: BufferedUart<'static, UART0>) {
    info!("Starting LIDAR task");

    let lidar = LidarManager::new(uart);

    lidar.run_lidar().await;
}