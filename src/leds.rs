use embassy_rp::{pio::Pio, pio_programs::ws2812::{PioWs2812, PioWs2812Program}, Peripheral};

use smart_leds::RGB8;

use defmt::*;

pub struct LEDManager<'d> {

}

impl<'d> LEDManager<'d> {
    pub fn new() -> LEDManager<'d> {
        let program = PioWs2812Program::new(&mut common);
        let mut ws2812 = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_16, &program);


        let led_manager = Self {

        };

        return led_manager;
    }
}

#[embassy_executor::task]
pub async fn led_task(p: Peripherals, ) {
    info!("Starting LED task");

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);


    let led_manager = LEDManager::new();
}