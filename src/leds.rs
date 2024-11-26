use core::{cmp, u16};

use embassy_rp::{peripherals::PIO1, pio::Pio, pio_programs::ws2812::{PioWs2812, PioWs2812Program}};

use embassy_time::{Duration, Ticker};
use smart_leds::RGB8;

use defmt::*;

use crate::lidar;

static LED_COUNT: usize = 60;
static LEDS_PER_DEGREE: usize = 360 / LED_COUNT;
static REFRESH_TIME: u64 = 20;

static MAX_DISTANCE: i32 = 500;
static MIN_DISTANCE: f32 = 60.0;

static MIN_INTENSITY: f32 = 100.0;

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

    let Pio { mut common, sm0, sm1, .. } = Pio::new(p.pio, crate::Irqs);

    let program = PioWs2812Program::new(&mut common);
    let mut ws2812: PioWs2812<'_, PIO1, 0, LED_COUNT> = 
        PioWs2812::new(&mut common, sm0, p.dma, p.dtr, &program);

    let led_manager = LEDManager::new();

    let mut data = [RGB8::default(); LED_COUNT];
    let mut led_distances = [0;360];
    let mut led_intensities = [0;360];

    let mut ticker: Ticker = Ticker::every(Duration::from_millis(REFRESH_TIME));

    let degrees_per_led = 360 / LED_COUNT;
    
    loop {
        debug!("New Colors:");

        lidar::LIDAR_DATA.lock(|x| {
            let reading = x.borrow();

            led_distances.copy_from_slice(&reading.distances);
            led_intensities.copy_from_slice(&reading.intensities);
        });

        for (i, r) in 
                led_intensities.iter().zip(led_distances).array_chunks::<LEDS_PER_DEGREE>().enumerate() {
                    let total_intensity = r.iter().map(|e| *e.0 as i32).sum::<i32>();
                    let total_distance = r.iter().map(|e| e.1 as i32).sum::<i32>();

                    let avg_intensity = total_intensity as f32 / LEDS_PER_DEGREE as f32;
                    let mut avg_distance = total_distance as f32 / LEDS_PER_DEGREE as f32;

                    // If we can't get a reading, we end up with a distance of 0
                    // but a very low intensity, so treat as very far
                    if avg_intensity < MIN_INTENSITY as f32 || 
                        avg_distance > MAX_DISTANCE as f32 || 
                        avg_distance < MIN_DISTANCE {
                        avg_distance = MAX_DISTANCE as f32;
                    }

                    let flipped_distance = MAX_DISTANCE as f32 - avg_distance;

                    let scaled_dist = flipped_distance as f32 / MAX_DISTANCE as f32;

                    let brightness = (scaled_dist * 50.0) as u8;

                    let scaled_intensity = avg_intensity / u16::MAX as f32;

                    data[i] = (brightness, (scaled_intensity * 50.0) as u8, 0).into();

                    if i == 0 {
                        info!("Total intensity {} Total distance {} flip {} Scaled {} Brightness {}", avg_intensity, avg_distance, flipped_distance, scaled_dist, brightness);
                    }
                }
                
        ws2812.write(&data).await;

        ticker.next().await;
    }
}