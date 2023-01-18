#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    peripherals::{PA8, PB0},
};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

struct RfSwitch<'a> {
    ctrl1: Output<'a, PB0>,
    ctrl3: Output<'a, PA8>,
}

impl<'a> RfSwitch<'a> {
    fn new(ctrl1: Output<'a, PB0>, ctrl3: Output<'a, PA8>) -> Self {
        Self { ctrl1, ctrl3 }
    }
}

impl<'a> embassy_lora::stm32wl::RadioSwitch for RfSwitch<'a> {
    fn set_rx(&mut self) {
        self.ctrl1.set_high();
        self.ctrl3.set_low();
    }

    fn set_tx(&mut self) {
        self.ctrl1.set_low();
        self.ctrl3.set_high();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PB4, Level::High, Speed::Low);

    loop {
        info!("high");
        led.set_high();
        Timer::after(Duration::from_millis(200)).await;

        info!("low");
        led.set_low();
        Timer::after(Duration::from_millis(200)).await;
    }
}
