//! Analog Input
//! 

use crate::Sealed;
use crate::sysctl;
use crate::gpio::*;
use tm4c123x::{ADC0, ADC1};

/// Analog-to-Digital Converter (ADC) peripheral
pub struct AdcSingle<ADC, PIN> where PIN: AdcPin {
    /// Underlying ADC peripheral
    pub adc: ADC,
    /// Underlying GPIO pin used by peripheral
    pub pins: PIN,
}

/// ADC pin
pub trait AdcPin: Sealed {
    /// Returns the associated channel number for this pin
    fn channel(&self) -> u8;
}

macro_rules! adc_pin {
    ([
        $($($PXi: ident)::*: $ch:expr,)+
    ]) => {
        $(
            impl<MODE> AdcPin for $($PXi)::*<Analog<Input<MODE>>> where MODE: InputMode {
                fn channel(&self) -> u8 {
                    $ch
                }
            }
        )+
    }
}

adc_pin!([
    gpiob::PB4: 10,
    gpiob::PB5: 11,
    gpiod::PD0: 7,
    gpiod::PD1: 6,
    gpiod::PD2: 5,
    gpiod::PD3: 4,
    gpioe::PE0: 3,
    gpioe::PE1: 2,
    gpioe::PE2: 1,
    gpioe::PE3: 0,
    gpioe::PE4: 9,
    gpioe::PE5: 8,
]);

macro_rules! adc {
    ($ADCx:ident, $adcx:ident, $($Adcx:ident)::*) => {
        impl<PIN> AdcSingle<$ADCx, PIN> where PIN: AdcPin {
            /// Create a new ADC peripheral using the given pin
            pub fn $adcx(adc: $ADCx, pin: PIN, pc: &sysctl::PowerControl) -> Self {
                sysctl::control_power(pc, $($Adcx)::*, sysctl::RunMode::Run, crate::sysctl::PowerState::On);
                sysctl::reset(pc, $($Adcx)::*);

                adc.actss.write(|w| w.asen0().clear_bit());
                adc.emux.write(|w| w.em0().processor());
                // adc.sspri.write(|w| unsafe { w.ss3().bits(0b00).ss2().bits(0b01).ss1().bits(0b10).ss0().bits(0b11) });
                adc.ssmux0.write(|w| unsafe { w.mux0().bits(pin.channel()) });
                adc.ssctl0.write(|w| w.ie0().set_bit().end0().set_bit());
                adc.actss.write(|w| w.asen0().set_bit());
                adc.isc.write(|w| w.in3().set_bit());
                AdcSingle { adc, pins: pin }
            }

            /// Read a single value from the ADC
            pub fn read(&mut self) -> u16 {
                // Start a conversion
                self.adc.pssi.write(|w| w.ss0().set_bit());

                // Wait for conversion to complete
                while self.adc.ris.read().inr0().bit_is_clear() {}
                self.adc.isc.write(|w| w.in0().set_bit());

                // Read the result
                let mut result = 0xFFFF;
                while self.adc.ssfstat0.read().empty().bit_is_clear() {
                    result = self.adc.ssfifo0.read().data().bits();
                }
                result
            } 
        }
    }
}

        
adc!(ADC0, adc0, sysctl::Domain::Adc0);
adc!(ADC1, adc1, sysctl::Domain::Adc1);
