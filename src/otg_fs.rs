//! USB OTG full-speed peripheral
//!
//! Requires the `usb_fs` feature.

use crate::stm32;

use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::{Alternate, PushPull};

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

pub struct USB {
    pub usb_global: stm32::OTG_FS_GLOBAL,
    pub usb_device: stm32::OTG_FS_DEVICE,
    pub usb_pwrclk: stm32::OTG_FS_PWRCLK,
    pub pin_dm: PA11<Alternate<PushPull>>,
    pub pin_dp: PA12<Alternate<PushPull>>,
}

unsafe impl Sync for USB {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = stm32::OTG_FS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320;

    fn enable() {
        let rcc = unsafe { &*stm32::RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.ahbenr.modify(|_, w| w.otgfsen().set_bit());

            // Reset USB peripheral
            rcc.ahbrstr.modify(|_, w| w.otgfsrst().set_bit());
            rcc.ahbrstr.modify(|_, w| w.otgfsrst().clear_bit());
        });
    }
}

pub type UsbBusType = UsbBus<USB>;
