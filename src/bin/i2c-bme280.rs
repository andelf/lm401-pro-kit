//! BME280 sensor demo

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::UartTx;
use embassy_time::{Delay, Duration, Timer};
use heapless::String;
use lm401_pro_kit::bme280::BME280;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::MSI(embassy_stm32::rcc::MSIRange::Range11); // 48MHz
    let p = embassy_stm32::init(config);

    info!("I2C BME280 demo!");

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    // Default: 115200 8N1
    let mut uart = UartTx::new(p.USART2, p.PA2, NoDma, Default::default());

    let irq = interrupt::take!(I2C2_EV);
    let i2c = I2c::new(
        p.I2C2,
        p.PA12,
        p.PA11,
        irq,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    );
    let mut delay = Delay;

    let mut bmp280 = BME280::new_primary(i2c);
    unwrap!(bmp280.init(&mut delay));

    let mut msg: String<128> = String::new();
    loop {
        info!("measure tick");
        let raw = unwrap!(bmp280.measure(&mut delay));

        info!("BME280: {:?}", raw);
        core::write!(
            msg,
            "temperature:{},pressure:{},humidity:{}\r\n",
            raw.temperature,
            raw.pressure,
            raw.humidity
        )
        .unwrap();
        uart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();

        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
