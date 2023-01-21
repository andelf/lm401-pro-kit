#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m::delay::Delay;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::pwm::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::pwm::Channel;
use embassy_stm32::rcc::{ClockSrc, MSIRange};
use embassy_stm32::time::hz;
use embassy_stm32::Config;
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    peripherals::{PA8, PB0},
};
use embassy_time::{Duration, Timer}; // Delay from embassy-time is not working
use embedded_hal::blocking::delay::DelayMs;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = ClockSrc::MSI(MSIRange::Range11); // 48MHz;
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    // Channel 2 of TIM2
    // PB3: TIM2_CH2
    // NOTE: conflicts with Timer
    let buzz_pin = PwmPin::new_ch2(p.PB3);
    let mut pwm = SimplePwm::new(p.TIM2, None, Some(buzz_pin), None, None, hz(2000));

    let max_duty = pwm.get_max_duty();
    //pwm.set_duty(Channel::Ch2, max_duty / 2); // 50duty
    //pwm.enable(Channel::Ch2);
    //pwm.set_freq(2000)
    info!("max duty: {}", max_duty);

    // hclk = 48MHz
    let mut delay = Delay::new(cp.SYST, 48_000_000);

    loop {
        for i in 0..50 {
            pwm.set_duty(Channel::Ch2, 20 * i);
            pwm.enable(Channel::Ch2);
            delay.delay_ms(10u32);
        }
        for i in (0..50).rev() {
            pwm.set_duty(Channel::Ch2, 20 * i);
            pwm.enable(Channel::Ch2);
            delay.delay_ms(10u32);
        }
        info!("buzz");

        // PWM resets instant
        let i = embassy_time::Instant::now();
        info!("instant => {:?}", i);
    }
}
