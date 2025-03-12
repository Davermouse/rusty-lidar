//! This example test the RP Pico W on board LED.
//!
//! It does not work with the RP Pico board. See blinky.rs.

#![no_std]
#![no_main]
#![feature(array_chunks)]
#![feature(iter_array_chunks)]

mod leds;
mod lidar;

use cyw43_pio::PioSpi;
use defmt::*;
use assign_resources::assign_resources;
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals;
use embassy_rp::peripherals::{ DMA_CH0, PIO0, PIO1, UART0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::uart::BufferedInterruptHandler;
use embassy_time::{Duration, Timer};
use leds::led_task;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

assign_resources! {
    wifi: WifiResources {
        pwr: PIN_23,
        cs: PIN_25,
        pio: PIO0,
        dio: PIN_24,
        clk: PIN_29,
        dma: DMA_CH0,
    },
    lidar: LidarResources {
        uart: UART0,
        rx: PIN_17,
        tx: PIN_16,
    },
    leds: LedResources {
        pio: PIO1,
        dtr: PIN_15,
        dma: DMA_CH1,
    },
    buttons: ButtonResources {

    },
    sound: SoundResources {
        bit_clock: PIN_18,
        lr_clock_pin: PIN_19,
        data: PIN_20,
        dma: DMA_CH2,
    },
}

#[cfg(not(feature = "pico_w"))]
async fn setup_wifi(r: WifiResources, spawner: Spawner) -> Option<cyw43::Control<'static>> {
    None
}

#[cfg(feature = "pico_w")]
async fn setup_wifi(r: WifiResources, spawner: Spawner) -> Option<cyw43::Control<'static>> {
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download ../../cyw43-firmware/43439A0.bin --binary-format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download ../../cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(r.pwr, Level::Low);
    let cs = Output::new(r.cs, Level::High);
    let mut pio = Pio::new(r.pio, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, r.dio, r.clk, r.dma);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    return Some(control);
}


#[cfg(feature = "pico_w")]
#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
   
    let r = split_resources!(p);

    let mut control = setup_wifi(r.wifi, spawner).await;

    spawner.spawn(led_task(r.leds)).unwrap();

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) }, 
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| lidar::setup_lidar(r.lidar, Irqs, spawner));
        });

    let delay = Duration::from_secs(1);
    loop {
        info!("led on!");
        match control {
            None => debug!("LED ON"),
            Some(ref mut c) => c.gpio_set(0, true).await
        };

        Timer::after(delay).await;

        info!("led off!");
        match control {
            None => debug!("LED OFF"),
            Some(ref mut c) => c.gpio_set(0, false).await
        };
        Timer::after(delay).await;

        lidar::LIDAR_DATA.lock(|x| {
            let reading = x.borrow();

            info!("Reset count: {} success count: {} last_reading: {} - distance 0: {} {}", reading.reset_count, reading.success_count, reading.last_reading, reading.distances[0], reading.intensities[0]);
        });
    }
}
