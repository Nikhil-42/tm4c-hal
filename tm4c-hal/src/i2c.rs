//! Common I2C code for TM4C123 and TM4C129

#[macro_export]
/// Implements the traits for an I2C peripheral
macro_rules! i2c_pins {
    ($I2Cx:ident,
        scl: [$(($($sclgpio: ident)::*, $sclaf: ident)),*],
        sda: [$(($($sdagpio: ident)::*, $sdaaf: ident)),*],
    ) => {
        $(
            impl SclPin<$I2Cx> for $($sclgpio)::*<AlternateFunction<$sclaf, PushPull>>
            {}
        )*

        $(
            impl SdaPin<$I2Cx> for $($sdagpio)::*<AlternateFunction<$sdaaf, OpenDrain<PullUp>>>
            {}
        )*
    }
}

#[macro_export]
/// Spins until the controler is ready (mcs.busy is clear) and optionally on
/// another field of the mcs register until it is clear or set (depending on op
/// parameter).
macro_rules! i2c_busy_wait {
    ($i2c:expr $(, $field:ident, $op:ident)? ) => {{
        // in 'release' builds, the time between setting the `run` bit and checking the `busy`
        // bit is too short and the `busy` bit is not reliably set by the time you get there,
        // it can take up to 8 clock cycles for the `run` to begin so this delay allows time
        // for that hardware synchronization
        cortex_m::asm::delay(8);

        // Allow 1,000 clock cycles before we timeout. At 100 kHz, this is 10 ms.
        $i2c.mclkocnt
            .write(|w| unsafe { w.cntl().bits((1_000 >> 4) as u8) });

        let mcs = loop {
            let mcs = $i2c.mcs.read();
            if mcs.busy().bit_is_clear() {
                break mcs;
            }
        };

        if mcs.clkto().bit_is_set() {
            return Err(ErrorKind::Other)
        } else if mcs.error().bit_is_set() {
            if mcs.arblst().bit_is_set() {
                return Err(ErrorKind::ArbitrationLoss)
            } else {
                $i2c.mcs.write(|w| w.stop().set_bit());
                if mcs.adrack().bit_is_set() {
                    return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                } else { // if mcs.datack().bit_is_set() {
                    return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                }
            }
        }

        $( if !mcs.$field().$op() {
            loop {
                let mcs = loop {
                    let mcs = $i2c.mcs.read();

                    if mcs.busy().bit_is_clear() {
                        break mcs;
                    }
                };

                if mcs.clkto().bit_is_set() {
                    return Err(ErrorKind::Other)
                } else if mcs.error().bit_is_set() {
                    if mcs.arblst().bit_is_set() {
                        return Err(ErrorKind::ArbitrationLoss)
                    } else {
                        $i2c.mcs.write(|w| w.stop().set_bit());
                        if mcs.adrack().bit_is_set() {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        } else { // if mcs.datack().bit_is_set() {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                        }
                    }
                } else if mcs.$field().$op() {
                    break;
                }
                // else keep waiting
            }
        };)?

        Ok(())
    }};
}

#[macro_export]
/// Implements embedded-hal for an TM4C I2C peripheral
macro_rules! i2c_hal {
    ($I2Cx:ident, $powerDomain:ident) => {
        impl<SCL: SclPin<$I2Cx>, SDA: SdaPin<$I2Cx>> I2C<$I2Cx, (SCL, SDA)> {
            /// Configures the I2C peripheral to work in master mode
            pub fn new<F: Into<Hertz>>(
                i2c: $I2Cx,
                pins: (SCL, SDA),
                freq: F,
                clocks: &Clocks,
                pc: &sysctl::PowerControl,
            ) -> Self {
                sysctl::control_power(
                    pc,
                    sysctl::Domain::$powerDomain,
                    sysctl::RunMode::Run,
                    sysctl::PowerState::On,
                );
                sysctl::reset(pc, sysctl::Domain::$powerDomain);

                // set Master Function Enable, and clear other bits.
                i2c.mcr.write(|w| w.mfe().set_bit());

                // Write TimerPeriod configuration and clear other bits.
                let freq = freq.into().0;
                let tpr = ((clocks.sysclk.0 / (2 * 10 * freq)) - 1) as u8;

                i2c.mtpr.write(|w| unsafe { w.tpr().bits(tpr) });

                I2C { i2c, pins }
            }

            /// Releases the I2C peripheral and associated pins
            pub fn free(self) -> ($I2Cx, (SCL, SDA)) {
                (self.i2c, self.pins)
            }
        }

        impl<PINS> ErrorType for I2C<$I2Cx, PINS> {
            type Error = ErrorKind;
        }

        impl<PINS> I2c for I2C<$I2Cx, PINS> {
            fn transaction(
                &mut self,
                addr: SevenBitAddress,
                operations: &mut [Operation<'_>],
            ) -> Result<(), Self::Error> {
                // Nothing to do
                if operations.is_empty() {
                    return Ok(());
                }

                // Operations must not be empty
                for op in operations.iter_mut() {
                    match op {
                        Operation::Write(bytes) => {
                            if bytes.is_empty() {
                                return Err(ErrorKind::Other);
                            }
                        }
                        Operation::Read(bytes) => {
                            if bytes.is_empty() {
                                return Err(ErrorKind::Other);
                            }
                        }
                    }
                }

                let mut start = true;
                let mut op_i = 0;
                let mut byte_i = 0;

                let one_op = operations.len() == 1;
                let op_change = !one_op 
                    && core::mem::discriminant(&operations[op_i])
                        != core::mem::discriminant(&operations[op_i + 1]);
                let op = &mut operations[op_i];
                let op_len = match op {
                    Operation::Write(bytes) => {
                        self.i2c
                            .msa
                            .write(|w| unsafe { w.sa().bits(addr).rs().clear_bit() });
                        self.i2c
                            .mdr
                            .write(|w| unsafe { w.data().bits(bytes[byte_i]) });

                        if one_op && bytes.len() == 1 {
                            // Special case for single byte write transaction
                            i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                            self.i2c
                                .mcs
                                .write(|w| w.start().set_bit().run().set_bit().stop().set_bit());
                            i2c_busy_wait!(self.i2c)?;
                            return Ok(());
                        } else {
                            i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                            self.i2c.mcs.write(|w| w.start().set_bit().run().set_bit())
                        }
                        i2c_busy_wait!(self.i2c)?;

                        bytes.len()
                    }
                    Operation::Read(buffer) => {
                        self.i2c
                            .msa
                            .write(|w| unsafe { w.sa().bits(addr).rs().set_bit() });

                        if one_op && buffer.len() == 1 {
                            // Special case for single byte read transaction
                            i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                            self.i2c
                                .mcs
                                .write(|w| w.start().set_bit().run().set_bit().stop().set_bit());
                            i2c_busy_wait!(self.i2c)?;
                            buffer[0] = self.i2c.mdr.read().data().bits();
                            return Ok(());
                        } else if op_change && buffer.len() == 1 {
                            // NACK on op_change
                            i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                            self.i2c.mcs.write(|w| w.start().set_bit().run().set_bit())
                        } else {
                            i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                            self.i2c
                                .mcs
                                .write(|w| w.start().set_bit().run().set_bit().ack().set_bit());
                        }

                        i2c_busy_wait!(self.i2c)?;
                        buffer[byte_i] = self.i2c.mdr.read().data().bits();

                        buffer.len()
                    }
                };

                // Move to next byte
                byte_i += 1;
                start = false;

                // Move to next operation
                if byte_i == op_len {
                    op_i += 1;
                    byte_i = 0;
                    start = op_change;
                };

                loop {
                    let op_len = match &operations[op_i] {
                        Operation::Write(bytes) => bytes.len(),
                        Operation::Read(buffer) => buffer.len(),
                    };

                    let op_change = if op_i == operations.len() - 1 {
                        if byte_i == op_len - 1 {
                            // If this is the last byte of the last operation, handle it specially
                            break;
                        }
                        false
                    } else {
                        // If this is not the last operation, check if the next operation is different
                        core::mem::discriminant(&operations[op_i])
                            != core::mem::discriminant(&operations[op_i + 1])
                    };

                    // Process current operation
                    match &mut operations[op_i] {
                        Operation::Write(bytes) => {
                            if start {
                                self.i2c
                                    .msa
                                    .write(|w| unsafe { w.sa().bits(addr).rs().clear_bit() });
                                self.i2c
                                    .mdr
                                    .write(|w| unsafe { w.data().bits(bytes[byte_i]) });
                                self.i2c.mcs.write(|w| w.start().set_bit().run().set_bit());
                            } else {
                                self.i2c
                                    .mdr
                                    .write(|w| unsafe { w.data().bits(bytes[byte_i]) });
                                self.i2c.mcs.write(|w| w.run().set_bit())
                            }
                            i2c_busy_wait!(self.i2c)?;
                        }
                        Operation::Read(buffer) => {
                            if start {
                                self.i2c
                                    .msa
                                    .write(|w| unsafe { w.sa().bits(addr).rs().set_bit() });

                                if op_change && byte_i == buffer.len() - 1 {
                                    // NACK on op_change
                                    self.i2c.mcs.write(|w| w.start().set_bit().run().set_bit());
                                } else {
                                    self.i2c.mcs.write(|w| {
                                        w.start().set_bit().run().set_bit().ack().set_bit()
                                    });
                                }
                            } else {
                                if op_change && byte_i == buffer.len() - 1 {
                                    // NACK on op_change
                                    self.i2c.mcs.write(|w| w.run().set_bit());
                                } else {
                                    self.i2c.mcs.write(|w| w.run().set_bit().ack().set_bit());
                                }
                            }
                            i2c_busy_wait!(self.i2c)?;
                            buffer[byte_i] = self.i2c.mdr.read().data().bits();
                        }
                    };

                    // Move to next byte
                    byte_i += 1;
                    start = false;

                    // Move to next operation
                    if byte_i == op_len {
                        op_i += 1;
                        byte_i = 0;
                        start = op_change;
                    }
                }

                // Final operation
                match &mut operations[op_i] {
                    Operation::Write(bytes) => {
                        if start {
                            self.i2c
                                .msa
                                .write(|w| unsafe { w.sa().bits(addr).rs().clear_bit() });
                            self.i2c
                                .mdr
                                .write(|w| unsafe { w.data().bits(bytes[byte_i]) });
                            self.i2c.mcs.write(|w| unsafe {
                                w.start().set_bit().run().set_bit().stop().set_bit()
                            });
                        } else {
                            self.i2c
                                .mdr
                                .write(|w| unsafe { w.data().bits(bytes[byte_i]) });
                            self.i2c.mcs.write(|w| w.run().set_bit().stop().set_bit());
                        }
                    },
                    Operation::Read(buffer) => {
                        if start {
                            self.i2c
                                .msa
                                .write(|w| unsafe { w.sa().bits(addr).rs().set_bit() });
                            self.i2c.mcs.write(|w| w.start().set_bit().run().set_bit().stop().set_bit() );
                        } else {
                            self.i2c.mcs.write(|w| w.run().set_bit().stop().set_bit() );
                        }

                        i2c_busy_wait!(self.i2c)?;
                        buffer[byte_i] = self.i2c.mdr.read().data().bits();
                    }
                };

                i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;
                Ok(())
            }
        }
    };
}
