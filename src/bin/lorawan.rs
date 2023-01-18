#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_lora::stm32wl::{SubGhzRadio, SubGhzRadioConfig};
use embassy_lora::LoraTimer;
use embassy_stm32::subghz::*;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    interrupt, pac,
    peripherals::{PA8, PB0},
    rng::Rng,
};
use embassy_time::{Duration, Timer};
use lorawan::default_crypto::DefaultFactory as Crypto;
use lorawan_device::JoinMode;
use lorawan_device::{async_device::Device, region};
use {defmt_rtt as _, panic_probe as _};

struct RadioSwitch<'a> {
    ctrl1: Output<'a, PB0>,
    ctrl3: Output<'a, PA8>,
}

impl<'a> RadioSwitch<'a> {
    fn new(ctrl1: Output<'a, PB0>, ctrl3: Output<'a, PA8>) -> Self {
        Self { ctrl1, ctrl3 }
    }
}

impl<'a> embassy_lora::stm32wl::RadioSwitch for RadioSwitch<'a> {
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
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSI16; //
    config.rcc.enable_lsi = true;

    let p = embassy_stm32::init(config);
    info!("Hello World!");

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }

    let ctrl1 = Output::new(p.PB0, Level::High, Speed::Low);
    let ctrl3 = Output::new(p.PA8, Level::High, Speed::Low);
    let rfs = RadioSwitch::new(ctrl1, ctrl3);

    let radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);
    let irq = interrupt::take!(SUBGHZ_RADIO);

    let mut radio_config = SubGhzRadioConfig::default();
    radio_config.calibrate_image = CalibrateImage::ISM_470_510;
    let radio = SubGhzRadio::new(radio, rfs, irq, radio_config).unwrap();

    let mut region: region::Configuration = region::CN470::default().into();
    // NOTE: This is specific for TTN, as they have a special RX1 delay
    region.set_receive_delay1(5000);

    let mut device: Device<_, Crypto, _, _> =
        Device::new(region, radio, LoraTimer::new(), Rng::new(p.RNG));

    //info!("datarate: {:?}", device.get_datarate().);
    device.set_datarate(region::DR::_0);

    defmt::info!("Joining LoRaWAN network");
    // TODO: Adjust the EUI and Keys according to your network credentials
    device
        .join(&JoinMode::OTAA {
            deveui: [0, 0, 0, 0, 0, 0, 0, 0],
            appeui: [0, 0, 0, 0, 0, 0, 0, 0],
            appkey: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        })
        .await
        .ok()
        .unwrap();

    defmt::info!("LoRaWAN network joined");

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    let mut rx: [u8; 255] = [0; 255];
    defmt::info!("Sending 'PING'");
    loop {
        info!("high");

        let len = device
            .send_recv(b"PINGING ...", &mut rx[..], 1, true)
            .await
            .ok()
            .unwrap();
        if len > 0 {
            defmt::info!("Message sent, received downlink: {:?}", &rx[..len]);
        } else {
            defmt::info!("Message sent!");
        }
        led.set_high();
        Timer::after(Duration::from_millis(200)).await;

        info!("low");
        led.set_low();
        Timer::after(Duration::from_millis(200)).await;
    }
}
