#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::UartTx;
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::MSI(embassy_stm32::rcc::MSIRange::Range11); // 48MHz
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    // Default: 115200 8N1
    let mut usart = UartTx::new(p.USART2, p.PA2, NoDma, Default::default());

    let mut led = Output::new(p.PB4, Level::High, Speed::Low);

    let mut msg: String<64> = String::new();
    loop {
        info!("tick");
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;

        led.set_low();

        let i = Instant::now();
        core::write!(msg, "Hello world, device time: {}\r\n", i.as_millis()).unwrap();
        usart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();

        Timer::after(Duration::from_millis(1000)).await;
    }
}
