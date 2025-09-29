//! Inter-Integrated Circuit (I2C) bus

use crate::{
    gpio::*,
    sysctl::{self, Clocks},
    time::Hertz,
    Sealed,
};

use tm4c129x::{I2C0, I2C1, I2C2, I2C3};
use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, I2c, Operation, SevenBitAddress};

pub use tm4c_hal::{i2c_busy_wait, i2c_hal, i2c_pins};

/// I2C peripheral operating in master mode
pub struct I2C<I2Cx, PINS> {
    /// Underlying I2C peripheral
    pub i2c: I2Cx,
    /// Underlying GPIO pins used by periphI2C3eral
    pub pins: PINS,
}

/// SCL pin
pub trait SclPin<I2C>: Sealed {}

/// SDA pin
pub trait SdaPin<I2C>: Sealed {}

i2c_pins!(I2C0, scl: [(gpiob::PB2, AF2)], sda: [(gpiob::PB3, AF2)],);
i2c_pins!(I2C1, scl: [(gpiog::PG0, AF2)], sda: [(gpiog::PG1, AF2)],);
i2c_pins!(I2C2,
    scl: [(gpiol::PL1, AF2),(gpiop::PP5, AF2), (gpion::PN5, AF3)],
    sda: [(gpiol::PL0, AF2), (gpion::PN4, AF3)],
);

i2c_hal!(I2C0, I2c0);
i2c_hal!(I2C1, I2c1);
i2c_hal!(I2C2, I2c2);
i2c_hal!(I2C3, I2c3);

