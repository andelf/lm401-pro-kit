#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::gpio::{Input, Pin, Pull};
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::rcc::{ClockSrc, MSIRange};
use embassy_stm32::time::hz;
use embassy_stm32::Config;
use embassy_time::{Duration, Timer};
use embedded_hal::blocking::delay::DelayMs;
use {defmt_rtt as _, panic_probe as _};

static CYCLE_STEP_MS: AtomicU32 = AtomicU32::new(100);

// (pool_size = 3) required for multiple same task
#[embassy_executor::task(pool_size = 3)]
async fn blinky_task(led: AnyPin, init_level: Level) {
    let mut led = Output::new(led, init_level, Speed::Low);
    loop {
        let dur = CYCLE_STEP_MS.load(Ordering::Relaxed);
        Timer::after(Duration::from_millis(dur.into())).await;
        led.toggle();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.enable_lsi = true;
    config.rcc.mux = ClockSrc::MSI(MSIRange::Range11); // 48MHz;
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    let button_pin = Input::new(p.PA0, Pull::Up);
    let mut button = ExtiInput::new(button_pin, p.EXTI0);

    let mut delay_ms = 100;
    CYCLE_STEP_MS.store(delay_ms, Ordering::Relaxed);

    spawner
        .spawn(blinky_task(p.PB3.degrade(), Level::Low))
        .unwrap();
    spawner
        .spawn(blinky_task(p.PB4.degrade(), Level::High))
        .unwrap();
    spawner
        .spawn(blinky_task(p.PB5.degrade(), Level::Low))
        .unwrap();

    loop {
        button.wait_for_falling_edge().await;

        delay_ms += 100;
        if delay_ms > 1000 {
            delay_ms = 100;
        }
        info!("update delay = {}ms", delay_ms);

        CYCLE_STEP_MS.store(delay_ms, Ordering::Relaxed)
    }
}
