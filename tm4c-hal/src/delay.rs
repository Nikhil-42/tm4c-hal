//! Code for busy-waiting

use crate::{sysctl::Clocks, time::Hertz};
use cortex_m::peripheral::{syst::SystClkSource, SYST};
use embedded_hal::delay::DelayNs;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    sysclk: Hertz,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay {
            syst,
            sysclk: clocks.sysclk,
        }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // * : u32 x u32 => u64, so do the multiplication in u64 to avoid overflow
        let rvr: u32 = (((ns as u64) * (self.sysclk.0 as u64)) / 1_000_000_000) as u32;


        assert!(rvr < (1 << 24));
        self.syst.set_reload(rvr);
        self.syst.clear_current();
        self.syst.enable_counter();
        while !self.syst.has_wrapped() {}
        self.syst.disable_counter();
    }
}
