#![no_std]
#![no_main]
#![macro_use]
#![allow(dead_code)]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use core::time::Duration;

use defmt::*;
use embassy_executor::Spawner;
use embassy_lora::stm32wl::RadioSwitch as _RadioSwitchTrait;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{Interrupt, InterruptExt};
use embassy_stm32::subghz::*;
use embassy_stm32::usart::UartTx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

const DATA_LEN: u8 = 20;
const PREAMBLE_LEN: u16 = 0x8;

const RF_FREQ: RfFreq = RfFreq::from_frequency(490_500_000);

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

const TX_PARAMS: TxParams = TxParams::new()
    .set_power(0x00) // 0 DB
    .set_ramp_time(RampTime::Micros200);
//.set_power(0x16) // +22dB
//.set_ramp_time(RampTime::Micros40);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::MSI(embassy_stm32::rcc::MSIRange::Range11);
    let p = embassy_stm32::init(config);

    let mut rfs = lm401_pro_kit::RadioSwitch::new_from_pins(p.PB0, p.PA15, p.PA8);

    let mut led_booting = Output::new(p.PB5, Level::High, Speed::Low);
    let mut led_rx = Output::new(p.PB4, Level::Low, Speed::Low);
    let mut led_error = Output::new(p.PB3, Level::Low, Speed::Low);

    let button = Input::new(p.PA0, Pull::Up);
    let mut pin = ExtiInput::new(button, p.EXTI0);

    // Default Config: 115200 8N1
    let mut usart = UartTx::new(p.USART2, p.PA2, NoDma, Default::default());

    static IRQ_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

    let radio_irq = interrupt::take!(SUBGHZ_RADIO);
    radio_irq.disable();
    radio_irq.set_handler(|_| {
        IRQ_SIGNAL.signal(());
        unsafe { interrupt::SUBGHZ_RADIO::steal() }.disable();
    });

    let mut radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);

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
    unwrap!(radio.set_pa_ocp(Ocp::Max140m)); // current max
    unwrap!(radio.set_tx_params(&TX_PARAMS));

    unwrap!(radio.set_packet_type(PacketType::LoRa));
    unwrap!(radio.set_lora_sync_word(LoRaSyncWord::Public));
    unwrap!(radio.set_lora_mod_params(&LORA_MOD_PARAMS));
    unwrap!(radio.set_lora_packet_params(&LORA_PACKET_PARAMS));
    unwrap!(radio.calibrate(0x7f));

    unwrap!(radio.calibrate_image(CalibrateImage::ISM_470_510));
    unwrap!(radio.set_rf_frequency(&RF_FREQ));

    unwrap!(radio.set_rx_gain(PMode::Boost2));

    defmt::info!("Radio Status: {:?}", unwrap!(radio.status()));
    defmt::info!("Radio ready for use");

    led_booting.set_low();

    let mut buf = [0u8; 256];
    let mut message: String<256> = String::new();

    pin.wait_for_any_edge().await;
    info!("button pressed, begin rx");
    loop {
        led_rx.set_high();
        rfs.set_rx();

        info!("begin rx...");

        unwrap!(radio.set_irq_cfg(
            &CfgIrq::new()
                .irq_enable_all(Irq::RxDone)
                .irq_enable_all(Irq::Timeout)
                .irq_enable_all(Irq::Err)
        ));
        unwrap!(radio.read_buffer(RX_BUF_OFFSET, &mut buf));
        unwrap!(radio.set_rx(Timeout::from_duration_sat(Duration::from_millis(5000))));

        radio_irq.unpend();
        radio_irq.enable();

        IRQ_SIGNAL.wait().await;
        led_rx.set_low();
        let (_, irq_status) = unwrap!(radio.irq_status());
        unwrap!(radio.clear_irq_status(irq_status));

        if irq_status & Irq::RxDone.mask() != 0 {
            led_error.set_low();

            let (_st, len, offset) = unwrap!(radio.rx_buffer_status());
            let packet_status = unwrap!(radio.lora_packet_status());
            let rssi = packet_status.rssi_pkt().to_integer();
            let snr = packet_status.snr_pkt().to_integer();
            info!(
                "RX done: rssi={}dBm snr={}dB len={} offset={}",
                rssi, snr, len, offset
            );
            let payload = &buf[offset as usize..offset as usize + len as usize];

            debug!("got BMP280 node raw={:?} ", payload);
            if let Some((addr, temp, pressure, humidity)) = parse_payload(payload) {
                info!(
                    "dev addr={:x} temp={} pressure={} humidity={}",
                    addr, temp, pressure, humidity
                );

                // to UART
                core::write!(
                    &mut message,
                    "addr={:x},rssi={},snr={},temperature={},pressure={},humidity={}\r\n",
                    addr,
                    rssi,
                    snr,
                    temp,
                    pressure,
                    humidity
                )
                .unwrap();
                unwrap!(usart.blocking_write(message.as_bytes()));
                message.clear();
            } else {
                led_error.set_high();
                warn!("invalid payload");
            }
        } else if irq_status & Irq::Timeout.mask() != 0 {
            led_error.set_high();
            warn!("timeout");
        } else {
            led_error.set_high();

            warn!("W: no data received, irq_status={}", irq_status);
            let (st, len, offset) = unwrap!(radio.rx_buffer_status());
            warn!("status {:?} len {} offset {}", st, len, offset);
        }

        rfs.set_off();
    }
}

/// addr, temp, pressure, humidity
fn parse_payload(payload: &[u8]) -> Option<(u32, f32, f32, f32)> {
    if payload.len() != 20 {
        return None;
    }
    if &payload[0..2] != b"MM" {
        return None;
    }
    let addr = u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
    let temp = f32::from_le_bytes([payload[6], payload[7], payload[8], payload[9]]);
    let pressure = f32::from_le_bytes([payload[10], payload[11], payload[12], payload[13]]);
    let humidity = f32::from_le_bytes([payload[14], payload[15], payload[16], payload[17]]);

    let checksum = payload[2..18]
        .iter()
        .fold(0u16, |acc, x| acc.wrapping_add(*x as u16));
    let checksum = checksum.to_le_bytes();
    if checksum != [payload[18], payload[19]] {
        return None;
    }

    Some((addr, temp, pressure, humidity))
}
