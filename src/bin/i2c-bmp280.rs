#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Error, I2c, TimeoutI2c};
use embassy_stm32::interrupt;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Duration, Timer};
use lm401_pro_kit::bme280::BME280;
use {defmt_rtt as _, panic_probe as _};

const ADDRESS: u8 = 0x76; // primary. 0x77 for secondary

const BMX280_CHIP_ID_ADDR: u8 = 0xD0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::MSI(embassy_stm32::rcc::MSIRange::Range11); // 48MHz
    let p = embassy_stm32::init(config);

    info!("I2C BMP280 demo!");

    let mut led = Output::new(p.PB4, Level::High, Speed::Low);

    let irq = interrupt::take!(I2C2_EV);
    let mut i2c = I2c::new(
        p.I2C2,
        p.PA12,
        p.PA11,
        irq,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    );

    // I2C bus can freeze if SCL line is shorted or due to a broken device that clock stretches for too long.
    // TimeoutI2c allows recovering from such errors by throwing `Error::Timeout` after a given delay.
    // let mut timeout_i2c = TimeoutI2c::new(&mut i2c, Duration::from_millis(1000));

    let mut delay = Delay;

    let mut bmp280 = BME280::new_primary(i2c);
    bmp280.init(&mut delay);

    loop {
        info!("measure tick");
        let raw = bmp280.measure(&mut delay).unwrap();

        info!("measurements: {:?}", raw);

        led.toggle();
        Timer::after(Duration::from_millis(2000)).await;
    }
}