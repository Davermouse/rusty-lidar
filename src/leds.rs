use embassy_rp::{peripherals::PIO1, pio::Pio, pio_programs::ws2812::{PioWs2812, PioWs2812Program}, Peripheral};

use embassy_time::{Duration, Ticker};
use smart_leds::RGB8;

use defmt::*;

use crate::lidar;

static LED_COUNT: usize = 60;

pub struct LEDManager{

}

impl<'d> LEDManager {
    pub fn new() -> LEDManager {

        let led_manager = Self {

        };

        return led_manager;
    }
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

#[embassy_executor::task]
pub async fn led_task(p: crate::LedResources) {
    info!("Starting LED task");

    let Pio { mut common, sm0, .. } = Pio::new(p.pio, crate::Irqs);

    let program = PioWs2812Program::new(&mut common);
    let mut ws2812: PioWs2812<'_, PIO1, 0, LED_COUNT> = 
        PioWs2812::new(&mut common, sm0, p.dma, p.dtr, &program);

    let led_manager = LEDManager::new();

    let mut data = [RGB8::default(); LED_COUNT];

    let mut ticker = Ticker::every(Duration::from_millis(20));
    loop {
        debug!("New Colors:");

        lidar::LIDAR_DATA.lock(|x| {
            let reading = x.borrow();

            let degrees_per_led = 360 / LED_COUNT;
            let max_dist = 7000;

            for (i, r) in reading.distances.array_chunks::<6>().enumerate() {
                let total = r.iter().map(|e| *e as u32).sum::<u32>();

                data[i] = ((total / max_dist * 255) as u8, 0, 0).into();
            }

       //     info!("Distance 270: {} {}", reading.distances[270], reading.intensities[270]);
        });
        /*
        for i in 0..LED_COUNT {
            data[i] = wheel((((i * 256) as u16 / LED_COUNT as u16 + j as u16) & 255) as u8);
            debug!("R: {} G: {} B: {}", data[i].r, data[i].g, data[i].b);
        }
        */
        ws2812.write(&data).await;

        ticker.next().await;
    }
}