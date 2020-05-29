//! # Reset & Control Clock

use cast::u32;
use core::cmp;

use crate::backup_domain::BackupDomain;
use crate::flash::ACR;
use crate::pac;
use crate::pac::{rcc, PWR, RCC};
use crate::time::Hertz;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                adcclk: None,
            },
            bkp: BKP { _0: () },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers
    pub ahb: AHB,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    pub cfgr: CFGR,
    pub bkp: BKP,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcc::AHBENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbenr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

impl APB1 {
    /// Set power interface clock (PWREN) bit in RCC_APB1ENR
    pub fn set_pwren(&mut self) {
        self.enr().modify(|_r, w| w.pwren().set_bit())
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

const HSI: u32 = 8_000_000; // Hz

pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

impl CFGR {
    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    pub fn use_hse<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the HCLK clock
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the PCKL1 clock
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the PCLK2 clock
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the SYSCLK clock
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Sets the desired frequency for the ADCCLK clock
    pub fn adcclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.adcclk = Some(freq.into().0);
        self
    }

    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let pllsrcclk = self.hse.unwrap_or(HSI / 2);

        let pllmul = self.sysclk.unwrap_or(pllsrcclk) / pllsrcclk;

        let (pllmul_bits, sysclk) = if pllmul == 1 {
            (None, self.hse.unwrap_or(HSI))
        } else {
            #[cfg(not(feature = "connectivity"))]
            let pllmul = cmp::min(cmp::max(pllmul, 1), 16);

            #[cfg(feature = "connectivity")]
            let pllmul = cmp::min(cmp::max(pllmul, 4), 9);

            (Some(pllmul as u8 - 2), pllsrcclk * pllmul)
        };

        assert!(sysclk <= 72_000_000);

        let hpre_bits = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3..=5 => 0b1001,
                6..=11 => 0b1010,
                12..=39 => 0b1011,
                40..=95 => 0b1100,
                96..=191 => 0b1101,
                192..=383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = if hpre_bits >= 0b1100 {
            sysclk / (1 << (hpre_bits - 0b0110))
        } else {
            sysclk / (1 << (hpre_bits - 0b0111))
        };

        assert!(hclk <= 72_000_000);

        let ppre1_bits = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= 36_000_000);

        let ppre2_bits = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= 72_000_000);

        // adjust flash wait states
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        unsafe {
            acr.acr().write(|w| {
                w.latency().bits(if sysclk <= 24_000_000 {
                    0b000
                } else if sysclk <= 48_000_000 {
                    0b001
                } else {
                    0b010
                })
            })
        }

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let (usbpre, usbclk_valid) = match (self.hse, pllmul_bits, sysclk) {
            (Some(_), Some(_), 72_000_000) => (false, true),
            (Some(_), Some(_), 48_000_000) => (true, true),
            _ => (true, false),
        };

        let apre_bits: u8 = self
            .adcclk
            .map(|adcclk| match pclk2 / adcclk {
                0..=2 => 0b00,
                3..=4 => 0b01,
                5..=7 => 0b10,
                _ => 0b11,
            })
            .unwrap_or(0b11);

        let apre = (apre_bits + 1) << 1;
        let adcclk = pclk2 / u32(apre);

        assert!(adcclk <= 14_000_000);

        let rcc = unsafe { &*RCC::ptr() };

        if self.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr.modify(|_, w| w.hseon().set_bit());

            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        if let Some(pllmul_bits) = pllmul_bits {
            // enable PLL and wait for it to be ready

            #[allow(unused_unsafe)]
            rcc.cfgr.modify(|_, w| unsafe {
                w.pllmul()
                    .bits(pllmul_bits)
                    .pllsrc()
                    .bit(self.hse.is_some())
            });

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

        // set prescalers and clock source
        #[cfg(feature = "connectivity")]
        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().bits(apre_bits);
            w.ppre2()
                .bits(ppre2_bits)
                .ppre1()
                .bits(ppre1_bits)
                .hpre()
                .bits(hpre_bits)
                .otgfspre()
                .bit(usbpre)
                .sw()
                .bits(if pllmul_bits.is_some() {
                    // PLL
                    0b10
                } else if self.hse.is_some() {
                    // HSE
                    0b1
                } else {
                    // HSI
                    0b0
                })
        });

        #[cfg(feature = "stm32f103")]
        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().bits(apre_bits);
            w.ppre2()
                .bits(ppre2_bits)
                .ppre1()
                .bits(ppre1_bits)
                .hpre()
                .bits(hpre_bits)
                .usbpre()
                .bit(usbpre)
                .sw()
                .bits(if pllmul_bits.is_some() {
                    // PLL
                    0b10
                } else if self.hse.is_some() {
                    // HSE
                    0b1
                } else {
                    // HSI
                    0b0
                })
        });

        #[cfg(any(feature = "stm32f100", feature = "stm32f101"))]
        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().bits(apre_bits);
            w.ppre2()
                .bits(ppre2_bits)
                .ppre1()
                .bits(ppre1_bits)
                .hpre()
                .bits(hpre_bits)
                .sw()
                .bits(if pllmul_bits.is_some() {
                    // PLL
                    0b10
                } else if self.hse.is_some() {
                    // HSE
                    0b1
                } else {
                    // HSI
                    0b0
                })
        });

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
            adcclk: Hertz(adcclk),
            usbclk_valid,
        }
    }

    #[cfg(feature = "stm32f107")]
    pub fn freeze_explicit(
        self,
        acr: &mut ACR,
        hse_frequency: u32,
        hse_prediv: rcc::cfgr2::PREDIV1_A,
        hse_prediv2: rcc::cfgr2::PREDIV2_A,
        pll2_mul: pac::rcc::cfgr2::PLL2MUL_A,
        pll3_mul: pac::rcc::cfgr2::PLL3MUL_A,
        prediv1_src: pac::rcc::cfgr2::PREDIV1SRC_A,
        pll1_input_clock: pac::rcc::cfgr::PLLSRC_A,
        pll1_mul: pac::rcc::cfgr::PLLMUL_A,
        flash_latency: pac::flash::acr::LATENCY_A,
        ahb_prescaler: pac::rcc::cfgr::HPRE_A,
        system_clock_src: pac::rcc::cfgr::SW_A,
        apb1_prescaler: pac::rcc::cfgr::PPRE1_A,
        apb2_prescaler: pac::rcc::cfgr::PPRE2_A,
        adc_prescaler: pac::rcc::cfgr::ADCPRE_A,
        usb_prescaler: pac::rcc::cfgr::OTGFSPRE_A,
    ) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };

        {
            rcc.cr.modify(|_, w| w.hseon().set_bit());
            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        {
            // PLL2

            {
                // disable PLL2 and wait for it to be off
                rcc.cr.modify(|_, w| w.pll2on().clear_bit());
                while !rcc.cr.read().pll2rdy().bit_is_clear() {}
            }

            rcc.cfgr2.modify(|_, w| w.prediv2().variant(hse_prediv2));
            rcc.cfgr2.modify(|_, w| w.pll2mul().variant(pll2_mul));

            {
                // enable PLL2 and wait for it to be ready
                rcc.cr.modify(|_, w| w.pll2on().set_bit());
                while rcc.cr.read().pll2rdy().bit_is_clear() {}
            }
        }

        {
            // PLL3

            {
                // disable PLL3 and wait for it to be off
                rcc.cr.modify(|_, w| w.pll3on().clear_bit());
                while !rcc.cr.read().pll3rdy().bit_is_clear() {}
            }

            rcc.cfgr2.modify(|_, w| w.pll3mul().variant(pll3_mul));

            {
                // enable PLL3 and wait for it to be ready
                rcc.cr.modify(|_, w| w.pll3on().set_bit());
                while rcc.cr.read().pll3rdy().bit_is_clear() {}
            }
        }

        {
            // Main PLL

            {
                // disable Main PLL and wait for it to be off
                rcc.cr.modify(|_, w| w.pllon().clear_bit());
                while !rcc.cr.read().pllrdy().bit_is_clear() {}
            }

            rcc.cfgr2.modify(|_, w| {
                w.prediv1src()
                    .variant(prediv1_src)
                    .prediv1()
                    .variant(hse_prediv)
            });
            rcc.cfgr.modify(|_, w| {
                w.pllsrc()
                    .variant(pll1_input_clock)
                    .pllmul()
                    .variant(pll1_mul)
            });

            {
                // enable Main PLL and wait for it to be ready
                rcc.cr.modify(|_, w| w.pllon().set_bit());
                while rcc.cr.read().pllrdy().bit_is_clear() {}
            }
        }

        {
            // Peripheral clocks config

            acr.acr().modify(|_, w| w.latency().variant(flash_latency));

            {
                // "Set the highest APBx dividers in order to ensure that we do not go through
                // a non-spec phase whatever we decrease or increase HCLK."
                rcc.cfgr.modify(|_, w| w.ppre1().div16());
                rcc.cfgr.modify(|_, w| w.ppre2().div16());
            }

            rcc.cfgr.modify(|_, w| w.hpre().variant(ahb_prescaler));

            {
                // set sysclock src and wait for it to take effect
                rcc.cfgr.modify(|_, w| w.sw().variant(system_clock_src));
                while rcc.cfgr.read().sws() != system_clock_src {}
            }

            // set flash latency again (maybe it can change with clocks?)
            acr.acr().modify(|_, w| w.latency().variant(flash_latency));

            rcc.cfgr.modify(|_, w| {
                w.ppre1()
                    .variant(apb1_prescaler)
                    .ppre2()
                    .variant(apb2_prescaler)
                    .adcpre()
                    .variant(adc_prescaler)
                    .otgfspre()
                    .variant(usb_prescaler)
            });

            {
                // Peripheral clock enable and wait for it to happen
                rcc.apb2enr.modify(|_, w| w.afioen().enabled());
                while rcc.apb2enr.read().afioen().is_disabled() {}
            }

            {
                // CRC clock enable, and wait for it to happen
                rcc.ahbenr.modify(|_, w| w.crcen().enabled());
                while rcc.ahbenr.read().crcen().is_disabled() {}
            }

            {
                // Enable DMA2 clock, and wait for it to be ready
                rcc.ahbenr.modify(|_, w| w.dma2en().enabled());
                while rcc.ahbenr.read().dma2en().is_disabled() {}
            }

            {
                // Ethernet clock enable and wait for it to happen
                rcc.ahbenr.modify(|_, w| {
                    w.ethmacen()
                        .enabled()
                        .ethmactxen()
                        .enabled()
                        .ethmacrxen()
                        .enabled()
                });
                while rcc.ahbenr.read().ethmacen().is_disabled() {}
            }
        }

        let vco_input_2 = match hse_prediv2 {
            rcc::cfgr2::PREDIV2_A::DIV1 => hse_frequency / 1,
            rcc::cfgr2::PREDIV2_A::DIV2 => hse_frequency / 2,
            rcc::cfgr2::PREDIV2_A::DIV3 => hse_frequency / 3,
            rcc::cfgr2::PREDIV2_A::DIV4 => hse_frequency / 4,
            rcc::cfgr2::PREDIV2_A::DIV5 => hse_frequency / 5,
            rcc::cfgr2::PREDIV2_A::DIV6 => hse_frequency / 6,
            rcc::cfgr2::PREDIV2_A::DIV7 => hse_frequency / 7,
            rcc::cfgr2::PREDIV2_A::DIV8 => hse_frequency / 8,
            rcc::cfgr2::PREDIV2_A::DIV9 => hse_frequency / 9,
            rcc::cfgr2::PREDIV2_A::DIV10 => hse_frequency / 10,
            rcc::cfgr2::PREDIV2_A::DIV11 => hse_frequency / 11,
            rcc::cfgr2::PREDIV2_A::DIV12 => hse_frequency / 12,
            rcc::cfgr2::PREDIV2_A::DIV13 => hse_frequency / 13,
            rcc::cfgr2::PREDIV2_A::DIV14 => hse_frequency / 14,
            rcc::cfgr2::PREDIV2_A::DIV15 => hse_frequency / 15,
            rcc::cfgr2::PREDIV2_A::DIV16 => hse_frequency / 16,
        };
        let pll2_clk = match pll2_mul {
            pac::rcc::cfgr2::PLL2MUL_A::MUL8 => vco_input_2 * 8,
            pac::rcc::cfgr2::PLL2MUL_A::MUL9 => vco_input_2 * 9,
            pac::rcc::cfgr2::PLL2MUL_A::MUL10 => vco_input_2 * 10,
            pac::rcc::cfgr2::PLL2MUL_A::MUL11 => vco_input_2 * 11,
            pac::rcc::cfgr2::PLL2MUL_A::MUL12 => vco_input_2 * 12,
            pac::rcc::cfgr2::PLL2MUL_A::MUL13 => vco_input_2 * 13,
            pac::rcc::cfgr2::PLL2MUL_A::MUL14 => vco_input_2 * 14,
            pac::rcc::cfgr2::PLL2MUL_A::MUL16 => vco_input_2 * 16,
            pac::rcc::cfgr2::PLL2MUL_A::MUL20 => vco_input_2 * 20,
        };
        let pll = match pll1_input_clock {
            pac::rcc::cfgr::PLLSRC_A::HSI_DIV2 => 4_000_000,
            pac::rcc::cfgr::PLLSRC_A::HSE_DIV_PREDIV => {
                let prediv1_val = match hse_prediv {
                    rcc::cfgr2::PREDIV1_A::DIV1 => 1,
                    rcc::cfgr2::PREDIV1_A::DIV2 => 2,
                    rcc::cfgr2::PREDIV1_A::DIV3 => 3,
                    rcc::cfgr2::PREDIV1_A::DIV4 => 4,
                    rcc::cfgr2::PREDIV1_A::DIV5 => 5,
                    rcc::cfgr2::PREDIV1_A::DIV6 => 6,
                    rcc::cfgr2::PREDIV1_A::DIV7 => 7,
                    rcc::cfgr2::PREDIV1_A::DIV8 => 8,
                    rcc::cfgr2::PREDIV1_A::DIV9 => 9,
                    rcc::cfgr2::PREDIV1_A::DIV10 => 10,
                    rcc::cfgr2::PREDIV1_A::DIV11 => 11,
                    rcc::cfgr2::PREDIV1_A::DIV12 => 12,
                    rcc::cfgr2::PREDIV1_A::DIV13 => 13,
                    rcc::cfgr2::PREDIV1_A::DIV14 => 14,
                    rcc::cfgr2::PREDIV1_A::DIV15 => 15,
                    rcc::cfgr2::PREDIV1_A::DIV16 => 16,
                };

                match prediv1_src {
                    pac::rcc::cfgr2::PREDIV1SRC_A::HSE => hse_frequency / prediv1_val,
                    pac::rcc::cfgr2::PREDIV1SRC_A::PLL2 => pll2_clk / prediv1_val,
                }
            }
        };
        let sysclk = match system_clock_src {
            pac::rcc::cfgr::SW_A::HSI => 8_000_000,
            pac::rcc::cfgr::SW_A::HSE => hse_frequency,
            pac::rcc::cfgr::SW_A::PLL => match pll1_mul {
                pac::rcc::cfgr::PLLMUL_A::MUL4 => pll * 4,
                pac::rcc::cfgr::PLLMUL_A::MUL5 => pll * 5,
                pac::rcc::cfgr::PLLMUL_A::MUL6 => pll * 6,
                pac::rcc::cfgr::PLLMUL_A::MUL6_5 => (pll * 13) / 2,
                pac::rcc::cfgr::PLLMUL_A::MUL7 => pll * 7,
                pac::rcc::cfgr::PLLMUL_A::MUL8 => pll * 8,
                pac::rcc::cfgr::PLLMUL_A::MUL9 => pll * 9,
            },
        };
        let hclk = match ahb_prescaler {
            pac::rcc::cfgr::HPRE_A::DIV1 => sysclk / 1,
            pac::rcc::cfgr::HPRE_A::DIV2 => sysclk / 2,
            pac::rcc::cfgr::HPRE_A::DIV4 => sysclk / 4,
            pac::rcc::cfgr::HPRE_A::DIV8 => sysclk / 8,
            pac::rcc::cfgr::HPRE_A::DIV16 => sysclk / 16,
            pac::rcc::cfgr::HPRE_A::DIV64 => sysclk / 64,
            pac::rcc::cfgr::HPRE_A::DIV128 => sysclk / 128,
            pac::rcc::cfgr::HPRE_A::DIV256 => sysclk / 256,
            pac::rcc::cfgr::HPRE_A::DIV512 => sysclk / 512,
        };
        let apb1_prescaler_val = match apb1_prescaler {
            pac::rcc::cfgr::PPRE1_A::DIV1 => 1,
            pac::rcc::cfgr::PPRE1_A::DIV2 => 2,
            pac::rcc::cfgr::PPRE1_A::DIV4 => 4,
            pac::rcc::cfgr::PPRE1_A::DIV8 => 8,
            pac::rcc::cfgr::PPRE1_A::DIV16 => 16,
        };
        let pclk1 = hclk / apb1_prescaler_val;
        let apb2_prescaler_val = match apb2_prescaler {
            pac::rcc::cfgr::PPRE2_A::DIV1 => 1,
            pac::rcc::cfgr::PPRE2_A::DIV2 => 2,
            pac::rcc::cfgr::PPRE2_A::DIV4 => 4,
            pac::rcc::cfgr::PPRE2_A::DIV8 => 8,
            pac::rcc::cfgr::PPRE2_A::DIV16 => 16,
        };
        let pclk2 = hclk / apb2_prescaler_val;
        let adcclk = match adc_prescaler {
            pac::rcc::cfgr::ADCPRE_A::DIV2 => (hclk / apb2_prescaler_val) / 2,
            pac::rcc::cfgr::ADCPRE_A::DIV4 => (hclk / apb2_prescaler_val) / 4,
            pac::rcc::cfgr::ADCPRE_A::DIV6 => (hclk / apb2_prescaler_val) / 6,
            pac::rcc::cfgr::ADCPRE_A::DIV8 => (hclk / apb2_prescaler_val) / 8,
        };

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1: apb1_prescaler_val as u8,
            ppre2: apb2_prescaler_val as u8,
            sysclk: Hertz(sysclk),
            adcclk: Hertz(adcclk),
            usbclk_valid: true,
        }
    }
}

pub struct BKP {
    _0: (),
}

impl BKP {
    /// Enables write access to the registers in the backup domain
    pub fn constrain(self, bkp: crate::pac::BKP, apb1: &mut APB1, pwr: &mut PWR) -> BackupDomain {
        // Enable the backup interface by setting PWREN and BKPEN
        apb1.enr()
            .modify(|_r, w| w.bkpen().set_bit().pwren().set_bit());

        // Enable access to the backup registers
        pwr.cr.modify(|_r, w| w.dbp().set_bit());

        BackupDomain { _regs: bkp }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    adcclk: Hertz,
    usbclk_valid: bool,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the frequency of the APB1 Timers
    pub fn pclk1_tim(&self) -> Hertz {
        Hertz(self.pclk1.0 * if self.ppre1() == 1 { 1 } else { 2 })
    }

    /// Returns the frequency of the APB2 Timers
    pub fn pclk2_tim(&self) -> Hertz {
        Hertz(self.pclk2.0 * if self.ppre2() == 1 { 1 } else { 2 })
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the adc clock frequency
    pub fn adcclk(&self) -> Hertz {
        self.adcclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    pub fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}

pub trait GetBusFreq {
    fn get_frequency(clocks: &Clocks) -> Hertz;
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        Self::get_frequency(clocks)
    }
}

impl GetBusFreq for AHB {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl GetBusFreq for APB1 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk1
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk1_tim()
    }
}

impl GetBusFreq for APB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk2
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk2_tim()
    }
}

pub(crate) mod sealed {
    /// Bus associated to peripheral
    pub trait RccBus {
        /// Bus type;
        type Bus;
    }
}
use sealed::RccBus;

/// Enable/disable peripheral
pub trait Enable: RccBus {
    fn enable(apb: &mut Self::Bus);
    fn disable(apb: &mut Self::Bus);
}

/// Reset peripheral
pub trait Reset: RccBus {
    fn reset(apb: &mut Self::Bus);
}

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $peren:ident, $perrst:ident),)+) => {
        $(
            impl RccBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().set_bit());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().clear_bit());
                }
            }
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset(apb: &mut Self::Bus) {
                    apb.rstr().modify(|_, w| w.$perrst().set_bit());
                    apb.rstr().modify(|_, w| w.$perrst().clear_bit());
                }
            }
        )+
    }
}

macro_rules! ahb_bus {
    ($($PER:ident => ($peren:ident),)+) => {
        $(
            impl RccBus for crate::pac::$PER {
                type Bus = AHB;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().set_bit());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().clear_bit());
                }
            }
        )+
    }
}

#[cfg(feature = "stm32f103")]
bus! {
    ADC2 => (APB2, adc2en, adc2rst),
    CAN1 => (APB1, canen, canrst),
}
#[cfg(all(feature = "stm32f103", feature = "high",))]
bus! {
    ADC3 => (APB2, adc3en, adc3rst),
}
bus! {
    ADC1 => (APB2, adc1en, adc1rst),
    AFIO => (APB2, afioen, afiorst),
    GPIOA => (APB2, iopaen, ioparst),
    GPIOB => (APB2, iopben, iopbrst),
    GPIOC => (APB2, iopcen, iopcrst),
    GPIOD => (APB2, iopden, iopdrst),
    GPIOE => (APB2, iopeen, ioperst),
    I2C1 => (APB1, i2c1en, i2c1rst),
    I2C2 => (APB1, i2c2en, i2c2rst),
    SPI1 => (APB2, spi1en, spi1rst),
    SPI2 => (APB1, spi2en, spi2rst),
    USART1 => (APB2, usart1en, usart1rst),
    USART2 => (APB1, usart2en, usart2rst),
    USART3 => (APB1, usart3en, usart3rst),
    WWDG => (APB1, wwdgen, wwdgrst),
}

#[cfg(any(feature = "high", feature = "connectivity"))]
bus! {
    SPI3 => (APB1, spi3en, spi3rst),
}

ahb_bus! {
    CRC => (crcen),
    DMA1 => (dma1en),
    DMA2 => (dma2en),
}

#[cfg(feature = "high")]
ahb_bus! {
    FSMC => (fsmcen),
}

bus! {
    TIM2 => (APB1, tim2en, tim2rst),
    TIM3 => (APB1, tim3en, tim3rst),
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
bus! {
    TIM1 => (APB2, tim1en, tim1rst),
}

#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity"))]
bus! {
    TIM6 => (APB1, tim6en, tim6rst),
}

#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103")),
    any(feature = "stm32f100", feature = "connectivity")
))]
bus! {
    TIM7 => (APB1, tim7en, tim7rst),
}

#[cfg(feature = "stm32f100")]
bus! {
    TIM15 => (APB2, tim15en, tim15rst),
    TIM16 => (APB2, tim16en, tim16rst),
    TIM17 => (APB2, tim17en, tim17rst),
}

#[cfg(feature = "medium")]
bus! {
    TIM4 => (APB1, tim4en, tim4rst),
}

#[cfg(any(feature = "high", feature = "connectivity"))]
bus! {
    TIM5 => (APB1, tim5en, tim5rst),
}

#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high",)))]
bus! {
    TIM12 => (APB1, tim12en, tim12rst),
    TIM13 => (APB1, tim13en, tim13rst),
    TIM14 => (APB1, tim14en, tim14rst),
}

#[cfg(all(feature = "stm32f103", feature = "high",))]
bus! {
    TIM8 => (APB2, tim8en, tim8rst),
}

#[cfg(feature = "xl")]
bus! {
    TIM9 => (APB2, tim9en, tim9rst),
    TIM10 => (APB2, tim10en, tim10rst),
    TIM11 => (APB2, tim11en, tim11rst),
}

#[cfg(any(feature = "stm32f102", feature = "stm32f103",))]
bus! {
    USB => (APB1, usben, usbrst),
}
