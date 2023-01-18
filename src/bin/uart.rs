#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::usart::UartTx;
use embassy_time::{Duration, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    // Default: 115200
    let mut usart = UartTx::new(p.USART2, p.PA2, NoDma, Default::default());

    let mut msg: String<64> = String::new();

    // PB2: ADC_IN4
    let mut led = Output::new(p.PB4, Level::High, Speed::Low);

    loop {
        //let sample = adc.read(&mut p.PB2);
        let sample = 10;

        writeln!(&mut msg, "ADC read: {} \r", sample).unwrap();

        led.set_high();
        Timer::after(Duration::from_millis(200)).await;

        info!("low");
        led.set_low();

        usart.blocking_write(msg.as_bytes()).unwrap();

        //usart.write(msg.as_bytes()).await.unwrap();

        Timer::after(Duration::from_millis(200)).await;

        msg.clear();
    }
}
