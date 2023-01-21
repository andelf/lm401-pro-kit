#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_lora::stm32wl::{RadioSwitch, SubGhzRadio, SubGhzRadioConfig};
use embassy_lora::LoraTimer;
use embassy_stm32::interrupt::{Interrupt, InterruptExt};
use embassy_stm32::rcc::{ClockSrc, MSIRange};
use embassy_stm32::subghz::*;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    interrupt, pac,
    peripherals::{PA8, PB0},
    rng::Rng,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use lorawan::default_crypto::DefaultFactory as Crypto;
use lorawan_device::JoinMode;
use lorawan_device::{async_device::Device, region};
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
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = ClockSrc::MSI(MSIRange::Range11); // 48MHz; default 4MHz
    config.rcc.enable_lsi = true;

    let p = embassy_stm32::init(config);
    info!("Hello World!");

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }

    let mut rfs = {
        let ctrl1 = Output::new(p.PB0, Level::High, Speed::Low);
        let ctrl3 = Output::new(p.PA8, Level::High, Speed::Low);
        RfSwitch::new(ctrl1, ctrl3)
    };

    static IRQ_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    let radio_irq = interrupt::take!(SUBGHZ_RADIO);
    radio_irq.set_handler(|_| {
        IRQ_SIGNAL.signal(());
        unsafe { interrupt::SUBGHZ_RADIO::steal() }.disable();
    });

    let mut radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);

    unwrap!(radio.set_standby(StandbyClk::Rc));
    const TCXO_MODE: TcxoMode = TcxoMode::new()
        .set_txco_trim(TcxoTrim::Volts1pt7)
        .set_timeout(Timeout::from_duration_sat(
            core::time::Duration::from_millis(10),
        ));
    radio.set_tcxo_mode(&TCXO_MODE);
    unwrap!(radio.set_standby(StandbyClk::Hse));
    radio.set_regulator_mode(RegMode::Ldo);
    const TX_BUF_OFFSET: u8 = 128;
    const RX_BUF_OFFSET: u8 = 0;
    radio.set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET);
    // only tx hp
    const PA_CONFIG: PaConfig = PaConfig::new()
        .set_pa_duty_cycle(0x1)
        .set_hp_max(0x1)
        .set_pa(PaSel::Hp);
    radio.set_pa_config(&PA_CONFIG);
    radio.set_pa_ocp(Ocp::Max60m);
    const TX_PARAMS: TxParams = TxParams::new()
        .set_power(0x0D)
        .set_ramp_time(RampTime::Micros40);
    radio.set_tx_params(&TX_PARAMS);

    radio.set_packet_type(PacketType::LoRa);
    radio.set_lora_sync_word(LoRaSyncWord::Public);
    const LORA_MOD_PARAMS: LoRaModParams = LoRaModParams::new()
        .set_bw(LoRaBandwidth::Bw125)
        .set_cr(CodingRate::Cr45)
        .set_ldro_en(true)
        .set_sf(SpreadingFactor::Sf7);
    radio.set_lora_mod_params(&LORA_MOD_PARAMS);
    const PREAMBLE_LEN: u16 = 5 * 8;
    const PING_DATA: &str = "PING";
    const DATA_LEN: u8 = PING_DATA.len() as u8;
    const LORA_PACKET_PARAMS: LoRaPacketParams = LoRaPacketParams::new()
        .set_crc_en(true)
        .set_preamble_len(PREAMBLE_LEN)
        .set_payload_len(DATA_LEN)
        .set_invert_iq(false)
        .set_header_type(HeaderType::Fixed);
    radio.set_lora_packet_params(&LORA_PACKET_PARAMS);
    radio.calibrate_image(CalibrateImage::ISM_470_510);
    const RF_FREQ: RfFreq = RfFreq::from_frequency(490_150_000);
    radio.set_rf_frequency(&RF_FREQ);

    defmt::info!("Status: {:?}", unwrap!(radio.status()));

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    let mut rx: [u8; 255] = [0; 255];
    defmt::info!("Sending 'PING'");
    const PING_DATA_BYTES: &[u8] = PING_DATA.as_bytes();

    loop {
        info!("debug 1");

        unwrap!(radio.set_irq_cfg(&CfgIrq::new().irq_enable_all(Irq::TxDone)));
        info!("debug 2");

        unwrap!(radio.write_buffer(TX_BUF_OFFSET, PING_DATA_BYTES));
        info!("debug 3");

        rfs.set_tx();
        rfs.set_rx();

        unwrap!(radio.set_tx(Timeout::DISABLED));
        info!("debug 4");

        radio_irq.enable();
        info!("debug 5");

        IRQ_SIGNAL.wait().await;
        info!("debug 6");

        let (_, irq_status) = unwrap!(radio.irq_status());
        if irq_status & Irq::TxDone.mask() != 0 {
            defmt::info!("TX done");
        }
        unwrap!(radio.clear_irq_status(irq_status));
        led.toggle();
        info!("toggle");
    }
}
