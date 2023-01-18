#![no_std]
#![no_main]
#![macro_use]
#![allow(dead_code)]
#![feature(type_alias_impl_trait)]

use core::time::Duration;

use defmt::*;
use embassy_executor::Spawner;
use embassy_lora::stm32wl::RadioSwitch as _RadioSwitchTrait;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{Interrupt, InterruptExt};
use embassy_stm32::subghz::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

const PING_DATA: &str = "PING";
const DATA_LEN: u8 = PING_DATA.len() as u8;
const PING_DATA_BYTES: &[u8] = PING_DATA.as_bytes();
const PREAMBLE_LEN: u16 = 5 * 8;

const RF_FREQ: RfFreq = RfFreq::from_frequency(490_500_000);

const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
const SYNC_WORD_LEN: u8 = SYNC_WORD.len() as u8;
const SYNC_WORD_LEN_BITS: u8 = SYNC_WORD_LEN * 8;

const TX_BUF_OFFSET: u8 = 128;
const RX_BUF_OFFSET: u8 = 0;
const LORA_PACKET_PARAMS: LoRaPacketParams = LoRaPacketParams::new()
    .set_crc_en(true)
    .set_preamble_len(PREAMBLE_LEN)
    .set_payload_len(DATA_LEN)
    .set_invert_iq(false)
    .set_header_type(HeaderType::Fixed);

const LORA_MOD_PARAMS: LoRaModParams = LoRaModParams::new()
    .set_bw(LoRaBandwidth::Bw125)
    .set_cr(CodingRate::Cr45)
    .set_ldro_en(true)
    .set_sf(SpreadingFactor::Sf7);

// configuration for +10 dBm output power
// see table 35 "PA optimal setting and operating modes"
const PA_CONFIG: PaConfig = PaConfig::new()
    .set_pa_duty_cycle(0x4)
    .set_hp_max(0x2)
    .set_pa(PaSel::Hp); // RFO_HP

/*
const TCXO_MODE: TcxoMode = TcxoMode::new()
    .set_txco_trim(TcxoTrim::Volts1pt7)
    .set_timeout(Timeout::from_duration_sat(
        core::time::Duration::from_millis(100),
    ));
*/

const TX_PARAMS: TxParams = TxParams::new()
    .set_power(0x00) // 0 DB
    .set_ramp_time(RampTime::Micros200);
//.set_power(0x16) // +22dB
//.set_ramp_time(RampTime::Micros40);

/// via BSP_RADIO_ConfigRFSwitch
struct RadioSwitch<'a> {
    ctrl1: Output<'a, AnyPin>,
    ctrl2: Output<'a, AnyPin>,
    ctrl3: Output<'a, AnyPin>,
}

impl<'a> RadioSwitch<'a> {
    fn new(
        ctrl1: Output<'a, AnyPin>,
        ctrl2: Output<'a, AnyPin>,
        ctrl3: Output<'a, AnyPin>,
    ) -> Self {
        Self {
            ctrl1,
            ctrl2,
            ctrl3,
        }
    }

    fn set_off(&mut self) {
        self.ctrl3.set_low();
        self.ctrl1.set_low();
        self.ctrl2.set_low();
    }
}

impl<'a> embassy_lora::stm32wl::RadioSwitch for RadioSwitch<'a> {
    fn set_rx(&mut self) {
        self.ctrl3.set_low();
        self.ctrl1.set_high();
        self.ctrl2.set_low();
    }

    // For both
    fn set_tx(&mut self) {
        self.ctrl3.set_high();
        self.ctrl1.set_low();
        self.ctrl2.set_low();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    // 48MHz; default 4MHz
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::MSI(embassy_stm32::rcc::MSIRange::Range11);
    let p = embassy_stm32::init(config);

    let ctrl1 = Output::new(p.PB0.degrade(), Level::Low, Speed::VeryHigh);
    let ctrl2 = Output::new(p.PA15.degrade(), Level::Low, Speed::VeryHigh);
    let ctrl3 = Output::new(p.PA8.degrade(), Level::Low, Speed::VeryHigh);
    let mut rfs = RadioSwitch::new(ctrl1, ctrl2, ctrl3);

    let mut led1 = Output::new(p.PB5, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PB4, Level::Low, Speed::Low);
    let mut led3 = Output::new(p.PB3, Level::Low, Speed::Low);

    let button = Input::new(p.PA0, Pull::Up);
    let mut pin = ExtiInput::new(button, p.EXTI0);

    static IRQ_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    let radio_irq = interrupt::take!(SUBGHZ_RADIO);
    radio_irq.set_handler(|_| {
        info!("handle irq");
        IRQ_SIGNAL.signal(());
        unsafe { interrupt::SUBGHZ_RADIO::steal() }.disable();
    });

    let mut radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);

    //defmt::info!("Radio ready for use");

    led1.set_low();
    led2.set_high();

    // from demo code: Radio_SMPS_Set
    unwrap!(radio.set_smps_clock_det_en(true));
    unwrap!(radio.set_smps_drv(SmpsDrv::Milli40));

    unwrap!(radio.set_standby(StandbyClk::Rc));

    // TCXO not enbaled
    // unwrap!(radio.set_tcxo_mode(&TCXO_MODE)); // TCXO not supported on this board
    // unwrap!(radio.set_standby(StandbyClk::Hse));

    // in XO mode, set internal capacitor (from 0x00 to 0x2F starting 11.2pF with 0.47pF steps)
    unwrap!(radio.set_hse_in_trim(HseTrim::from_raw(0x20)));
    unwrap!(radio.set_hse_out_trim(HseTrim::from_raw(0x20)));

    unwrap!(radio.set_regulator_mode(RegMode::Smps)); // Use DCDC

    unwrap!(radio.set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET));

    unwrap!(radio.set_pa_config(&PA_CONFIG));
    unwrap!(radio.set_pa_ocp(Ocp::Max60m)); // current max
    unwrap!(radio.set_tx_params(&TX_PARAMS));

    unwrap!(radio.set_packet_type(PacketType::LoRa));
    unwrap!(radio.set_lora_sync_word(LoRaSyncWord::Public));
    unwrap!(radio.set_lora_mod_params(&LORA_MOD_PARAMS));
    unwrap!(radio.set_lora_packet_params(&LORA_PACKET_PARAMS));
    unwrap!(radio.calibrate_image(CalibrateImage::ISM_470_510));
    unwrap!(radio.set_rf_frequency(&RF_FREQ));

    unwrap!(radio.set_rx_gain(PMode::Boost2));

    defmt::info!("Radio Status: {:?}", unwrap!(radio.status()));

    led2.set_low();

    let mut buf = [0u8; 256];
    // let mut msg: String<256> = String::;
    loop {
        // LED1: received data indicator
        led1.set_low();
        // pin.wait_for_rising_edge().await;
        led3.set_high();
        rfs.set_rx();

        unwrap!(radio.set_fs());

        info!("begin rx...");

        unwrap!(radio.set_irq_cfg(
            &CfgIrq::new().irq_enable_all(Irq::RxDone) //   .irq_enable_all(Irq::Timeout)
        ));
        unwrap!(radio.read_buffer(RX_BUF_OFFSET, &mut buf));
        unwrap!(radio.set_rx(Timeout::from_duration_sat(Duration::from_millis(3000))));

        //unwrap!(radio.write_buffer(TX_BUF_OFFSET, PING_DATA_BYTES));
        //unwrap!(radio.set_tx(Timeout::DISABLED));

        radio_irq.enable();
        IRQ_SIGNAL.wait().await;
        rfs.set_off();

        let (st, irq_status) = unwrap!(radio.irq_status());
        info!("irq_status={:?}", st);
        unwrap!(radio.clear_irq_status(irq_status));
        unwrap!(radio.set_fs());
        if irq_status & Irq::RxDone.mask() != 0 {
            led1.set_high();
            let (st, len, offset) = unwrap!(radio.rx_buffer_status());
            info!(
                "RX done: rx_buf_st: {:?}, len={} offset={}",
                st, len, offset
            );
            let payload = &buf[offset as usize..offset as usize + len as usize];
            info!("received: {:?}", unsafe {
                core::str::from_utf8_unchecked(payload)
            });
        } else {
            info!("W: no data received, irq_status={}", irq_status);
        }

        led3.set_low();

        led2.set_high();

        Timer::after(embassy_time::Duration::from_millis(5000)).await;
        led2.set_low();

        //       info!("buffer {:?}", buf);
        //msg.clear();
    }
}
