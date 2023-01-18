#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::rcc::{ClockSrc, MSIRange};
use embassy_stm32::usart::{Uart, UartTx};
use embassy_time::{Duration, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = ClockSrc::MSI(MSIRange::Range11);     // 48MHz; default 4MHz
    let p = embassy_stm32::init(config);

    info!("Starting system....");

    // Default: 115200
    let irq = interrupt::take!(USART2);

    let mut usart2 = Uart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        irq,
        p.DMA1_CH2,
        p.DMA1_CH4,
        Default::default(),
    );

    unwrap!(usart2.write(b"Starting system....\r\n").await);

    let mut msg: String<64> = String::new();

    // PB2: ADC_IN4
    let mut led = Output::new(p.PB4, Level::High, Speed::Low);

    let mut buf = [0u8; 300];
    loop {
        //let sample = adc.read(&mut p.PB2);
        let sample = 10;

        writeln!(&mut msg, "ADC read: {} \r", sample).unwrap();

        let result = usart2.read_until_idle(&mut buf).await;
        match result {
            Ok(size) => {
                match usart2.write(&buf[0..size]).await {
                    Ok(()) => {
                        //Write suc.
                    }
                    Err(..) => {
                        //Wasnt able to write
                    }
                }
            }
            Err(_err) => {
                //Ignore eg. framing errors
            }
        }
        led.toggle();
        msg.clear();
    }
}
