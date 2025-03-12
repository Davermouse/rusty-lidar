use core::{cmp, u16};

use embassy_rp::{peripherals::PIO1, pio::Pio, pio_programs::ws2812::{PioWs2812, PioWs2812Program}};

use embassy_time::{Duration, Ticker};
use smart_leds::RGB8;

use defmt::*;

use crate::lidar;

static LED_COUNT: usize = 60;
static LEDS_PER_DEGREE: usize = 360 / LED_COUNT;
static REFRESH_TIME: u64 = 20;

static MAX_DISTANCE: i32 = 400;
static MIN_DISTANCE: f32 = 60.0;

static MIN_INTENSITY: f32 = 100.0;

enum LEDState {
    LIDAR,
    Loading,
}

pub struct LEDManager<'d> {
    ticks: u32,
    state: LEDState,
    leds: PioWs2812<'d, PIO1, 0, LED_COUNT>,
    target_leds: [RGB8 ; LED_COUNT],
}

impl<'d> LEDManager<'d> {
    pub fn new(leds: PioWs2812<'d, PIO1, 0, LED_COUNT>) -> LEDManager {

        let led_manager = Self {
            ticks: 0,
            state: LEDState::LIDAR,
            leds,
            target_leds: [RGB8::default() ; LED_COUNT]
        };

        return led_manager;
    }

    pub async fn run(mut self) {
        let mut curr_leds = [RGB8::default(); LED_COUNT];
     //   let mut target_leds = [RGB8::default(); LED_COUNT];
        let mut led_distances = [0;360];
        let mut led_intensities = [0;360];

        let mut ticker: Ticker = Ticker::every(Duration::from_millis(REFRESH_TIME));

        let degrees_per_led = 360 / LED_COUNT;
        
        loop {
            debug!("New Colors:");

            match self.state {
                LEDState::LIDAR => {
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

                                //hsv_to_rgb(scaled_dist, scaled_intensity, 0.2f) 
                                self.target_leds[i] = (brightness, (scaled_intensity * 50.0) as u8, 0).into();

                                if i == 0 {
                                    //info!("Total intensity {} Total distance {} flip {} Scaled {} Brightness {}", avg_intensity, avg_distance, flipped_distance, scaled_dist, brightness);
                                }
                            }

                  /*   for n in 0..60 {
                        let dr = curr_leds[n].r as i16 - target_leds[n].r as i16;

                        curr_leds[n].r += (0.1 * dr as f32) as u8;
                        curr_leds[n].g = self.target_leds[n].g;
                    }*/
            },
                LEDState::Loading => {
                   self.update_loading();
            }
        }
                    
            self.leds.write(&self.target_leds).await;

            self.ticks += 1;
/* 
            if self.ticks % 500  == 0{
                info!("New state");

                match self.state {
                    LEDState::LIDAR => self.start_loading(),
                    LEDState::Loading => self.state = LEDState::LIDAR,
                }
            }
*/
            ticker.next().await;
        }
    }

    fn start_loading(&mut self) {
        self.state = LEDState::Loading;
    }

    fn update_loading(&mut self) {
        self.target_leds.fill((0, 0, 0).into());

        let colour = wheel((self.ticks % 255) as u8);

        self.target_leds[(self.ticks % 60) as usize] = colour;
     //   self.target_leds[(self.ticks - 1 % 60) as usize] = (0, 0, 40).into();
        self.target_leds[((self.ticks + 15) % 60) as usize] = colour;
        self.target_leds[((self.ticks + 30) % 60) as usize] = colour;
        self.target_leds[((self.ticks + 45) % 60) as usize] = colour;
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

pub fn hsv_to_rgb(hue: f32, saturation: f32, value: f32) -> RGB8 {
    fn is_between(value: f32, min: f32, max: f32) -> bool {
        min <= value && value < max
    }

    //check_bounds(hue, saturation, value);

    let c = value * saturation;
    let h = hue / 60.0;
    let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
    let m = value - c;

    let (r, g, b): (f32, f32, f32) = if is_between(h, 0.0, 1.0) {
        (c, x, 0.0).into()
    } else if is_between(h, 1.0, 2.0) {
        (x, c, 0.0).into()
    } else if is_between(h, 2.0, 3.0) {
        (0.0, c, x).into()
    } else if is_between(h, 3.0, 4.0) {
        (0.0, x, c).into()
    } else if is_between(h, 4.0, 5.0) {
        (x, 0.0, c).into()
    } else {
        (c, 0.0, x).into()
    };

    (
        ((r + m) * 255.0) as u8,
        ((g + m) * 255.0) as u8,
        ((b + m) * 255.0) as u8,
    ).into()
}

#[embassy_executor::task]
pub async fn led_task(p: crate::LedResources) {
    info!("Starting LED task");

    let Pio { mut common, sm0, sm1, .. } = Pio::new(p.pio, crate::Irqs);

    let program = PioWs2812Program::new(&mut common);
    let mut ws2812: PioWs2812<'_, PIO1, 0, LED_COUNT> = 
        PioWs2812::new(&mut common, sm0, p.dma, p.dtr, &program);

    let led_manager = LEDManager::new(ws2812);

    led_manager.run().await;
}