#![no_std]

use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Pin, Speed},
    peripherals::{PA15, PA8, PB0},
};

pub mod bme280;

/// via BSP_RADIO_ConfigRFSwitch
pub struct RadioSwitch<'a> {
    ctrl1: Output<'a, AnyPin>,
    ctrl2: Output<'a, AnyPin>,
    ctrl3: Output<'a, AnyPin>,
}
impl<'a> RadioSwitch<'a> {
    pub fn new_from_pins(ctrl1: PB0, ctrl2: PA15, ctrl3: PA8) -> Self {
        Self {
            ctrl1: Output::new(ctrl1.degrade(), Level::Low, Speed::VeryHigh),
            ctrl2: Output::new(ctrl2.degrade(), Level::Low, Speed::VeryHigh),
            ctrl3: Output::new(ctrl3.degrade(), Level::Low, Speed::VeryHigh),
        }
    }

    pub fn new(
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

    pub fn set_off(&mut self) {
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

// Device ID in STM32L4(WL) microcontrollers
pub fn chip_id() -> [u32; 3] {
    unsafe {
        [
            core::ptr::read_volatile(0x1FFF7590 as *const u32),
            core::ptr::read_volatile(0x1FFF7594 as *const u32),
            core::ptr::read_volatile(0x1FFF7598 as *const u32),
        ]
    }
}
