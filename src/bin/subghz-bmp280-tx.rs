#![no_std]
#![no_main]
#![macro_use]
#![allow(dead_code)]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_lora::stm32wl::RadioSwitch as _RadioSwitchTrait;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{Interrupt, InterruptExt};
use embassy_stm32::subghz::*;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer};
use lm401_pro_kit::bme280::BME280;
use {defmt_rtt as _, panic_probe as _};

const DATA_LEN: u8 = 28_u8;
const PREAMBLE_LEN: u16 = 0x8 * 8;

const RF_FREQ: RfFreq = RfFreq::from_frequency(490_500_000);

const TX_BUF_OFFSET: u8 = 128;
const RX_BUF_OFFSET: u8 = 0;
const LORA_PACKET_PARAMS: LoRaPacketParams = LoRaPacketParams::new()
    .set_crc_en(false)
    .set_preamble_len(PREAMBLE_LEN)
    .set_payload_len(DATA_LEN)
    .set_invert_iq(false)
    .set_header_type(HeaderType::Fixed);

// SF7, 125 kHz, 4/5 coding rate, low data rate optimization
const LORA_MOD_PARAMS: LoRaModParams = LoRaModParams::new()
    .set_bw(LoRaBandwidth::Bw125)
    .set_cr(CodingRate::Cr45)
    .set_ldro_en(false) // for compatibility with LoRaWAN
    .set_sf(SpreadingFactor::Sf7);

// configuration for +10 dBm output power
// see table 35 "PA optimal setting and operating modes"
const PA_CONFIG: PaConfig = PaConfig::new()
    .set_pa_duty_cycle(0x3)
    .set_hp_max(0x7)
    .set_pa(PaSel::Hp);

const TX_PARAMS: TxParams = TxParams::new()
    .set_power(0x16) // +22dB
    .set_ramp_time(RampTime::Micros200);
//.set_power(0x16) // +22dB
//.set_ramp_time(RampTime::Micros40);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // enable LSI, use MSI as clock sourc, set to 48MHz
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::MSI(embassy_stm32::rcc::MSIRange::Range11);
    let p = embassy_stm32::init(config);

    let mut delay = Delay;

    // Board specific setup
    let mut rfs = lm401_pro_kit::RadioSwitch::new_from_pins(p.PB0, p.PA15, p.PA8);

    let mut led_booting = Output::new(p.PB5, Level::High, Speed::Low);
    let mut led_tx = Output::new(p.PB4, Level::Low, Speed::Low);
    let mut led_rx = Output::new(p.PB3, Level::Low, Speed::Low);

    let button = Input::new(p.PA0, Pull::Up);
    let mut pin = ExtiInput::new(button, p.EXTI0);

    // BMP280 setup
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

    let mut bmp280 = BME280::new_primary(i2c);
    unwrap!(bmp280.init(&mut delay));

    // LoRa radio setup
    static IRQ_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    let radio_irq = interrupt::take!(SUBGHZ_RADIO);
    radio_irq.set_handler(|_| {
        IRQ_SIGNAL.signal(());
        unsafe { interrupt::SUBGHZ_RADIO::steal() }.disable();
    });

    let mut radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);

    // from demo code: Radio_SMPS_Set
    unwrap!(radio.set_smps_clock_det_en(true));
    unwrap!(radio.set_smps_drv(SmpsDrv::Milli40));

    unwrap!(radio.set_standby(StandbyClk::Rc));

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

    defmt::info!("Radio Status: {:?}", unwrap!(radio.status()));
    //defmt::info!("Radio ready for use");
    let chip_id = lm401_pro_kit::chip_id();
    let dev_addr = chip_id[0] ^ chip_id[1] ^ chip_id[2];
    info!(
        "Chip ID: {:08x} {:08x} {:08x}",
        chip_id[0], chip_id[1], chip_id[2]
    );
    info!("Dev Addr: {:08x}", dev_addr);

    led_booting.set_low();

    // mearure -> send -> receive -> measure -> send -> receive -> ...
    let mut payload = [0u8; 28];
    loop {
        // pin.wait_for_rising_edge().await;

        let now = Instant::now();
        let measurements = unwrap!(bmp280.measure(&mut delay));

        info!("BMP280 measure: {:?}", measurements);

        payload[0] = b'M';
        payload[1] = b'M';

        payload[2..6].copy_from_slice(dev_addr.to_be_bytes().as_slice());
        payload[6..14].copy_from_slice(now.as_millis().to_be_bytes().as_slice());
        payload[14..18].copy_from_slice(measurements.temperature.to_be_bytes().as_slice());
        payload[18..22].copy_from_slice(measurements.pressure.to_be_bytes().as_slice());
        payload[22..26].copy_from_slice(measurements.pressure.to_be_bytes().as_slice());

        let checksum = payload[2..26]
            .iter()
            .fold(0u16, |acc, x| acc.wrapping_add(*x as u16));
        info!("checksum: {:04x}", checksum);
        payload[26..28].copy_from_slice(checksum.to_be_bytes().as_slice());

        led_tx.set_high();
        rfs.set_tx();
        unwrap!(radio.set_irq_cfg(&CfgIrq::new().irq_enable_all(Irq::TxDone)));
        unwrap!(radio.write_buffer(TX_BUF_OFFSET, &payload[..]));
        unwrap!(radio.set_tx(Timeout::DISABLED));

        radio_irq.enable();
        IRQ_SIGNAL.wait().await;
        rfs.set_off();

        let (_, irq_status) = unwrap!(radio.irq_status());
        if irq_status & Irq::TxDone.mask() != 0 {
            defmt::info!("TX done");
        }
        unwrap!(radio.clear_irq_status(irq_status));
        led_tx.set_low();

        //radio.read_buffer(offset, buf)
        led_rx.set_high();

        Timer::after(Duration::from_millis(1000)).await;
        led_rx.set_low();
        Timer::after(Duration::from_millis(1000)).await;
    }
}
