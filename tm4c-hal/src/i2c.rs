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
        delay(8);

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
        } else if mcs.arblst().bit_is_set() {
            return Err(ErrorKind::ArbitrationLoss)
        } else if mcs.error().bit_is_set() {
            if mcs.adrack().bit_is_set() {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
            } else { // if mcs.datack().bit_is_set() {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
            }
        }

        $( loop {
            if mcs.clkto().bit_is_set() {
                return Err(ErrorKind::Other)
            } else if mcs.arblst().bit_is_set() {
                return Err(ErrorKind::ArbitrationLoss)
            } else if mcs.error().bit_is_set() {
                if mcs.adrack().bit_is_set() {
                    return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                } else { // if mcs.datack().bit_is_set() {
                    return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                }
            } else if mcs.$field().$op() {
                break;
            } else {
                // try again
            }
        };)?

        Ok(())
    }};
}

#[macro_export]
/// Implements embedded-hal for an TM4C I2C peripheral
macro_rules! i2c_hal {
    ($I2Cx:ident, $powerDomain:ident) => {
            impl<SCL, SDA> I2C<$I2Cx, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn new<F>(
                    i2c: $I2Cx,
                    pins: (SCL, SDA),
                    freq: F,
                    clocks: &Clocks,
                    pc: &sysctl::PowerControl,
                ) -> Self
                where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2Cx>,
                    SDA: SdaPin<$I2Cx>,
                {
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
                    // First operation always needs a start
                    let mut send_start = true;

                    // Iterate through operations, skipping any that have zero length
                    let mut ops = operations
                        .iter_mut()
                        .filter(|op| match op {
                            Operation::Read(buffer) => !buffer.is_empty(),
                            Operation::Write(bytes) => !bytes.is_empty(),
                        })
                        .peekable();

                    while let Some(op) = ops.next() {
                        let (op_change, send_stop) = if let Some(next_op) = ops.peek() {
                            (
                                match (&op, next_op) {
                                    (Operation::Read(_), Operation::Read(_))
                                    | (Operation::Write(_), Operation::Write(_)) => false,
                                    _ => true,
                                },
                                false,
                            )
                        } else {
                            (true, true)
                        };

                        match op {
                            Operation::Write(bytes) => {
                                if send_start {
                                    // Write Slave address and clear Receive bit
                                    self.i2c.msa.write(|w| unsafe { w.sa().bits(addr) });
                                }

                                let sz = bytes.len();

                                // Put first byte in data register
                                self.i2c.mdr.write(|w| unsafe { w.data().bits(bytes[0]) });

                                i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;

                                // Send START + RUN
                                // If single byte transfer, set STOP
                                if sz == 1 {
                                    self.i2c.mcs.write(|w| {
                                        if send_stop {
                                            w.stop().set_bit();
                                        };
                                        if send_start {
                                            w.start().set_bit();
                                        };
                                        w.run().set_bit()
                                    });
                                } else {
                                    for (i, byte) in (&bytes[1..sz - 1]).iter().enumerate() {
                                        i2c_busy_wait!(self.i2c)?;

                                        // Put next byte in data register
                                        self.i2c.mdr.write(|w| unsafe { w.data().bits(*byte) });

                                        // Send RUN command (Burst continue)
                                        // Set STOP on last byte
                                        self.i2c.mcs.write(|w| w.run().set_bit());
                                    }

                                    i2c_busy_wait!(self.i2c)?;

                                    // Put last byte in data register
                                    self.i2c
                                        .mdr
                                        .write(|w| unsafe { w.data().bits(bytes[sz - 1]) });

                                    // Send RUN command (Burst continue)
                                    // Set STOP on last byte
                                    self.i2c.mcs.write(|w| {
                                        if send_stop {
                                            w.stop().set_bit();
                                        };
                                        w.run().set_bit()
                                    });
                                }
                            }
                            Operation::Read(buffer) => {
                                if send_start {
                                    // Write Slave address and set Receive bit
                                    self.i2c
                                        .msa
                                        .write(|w| unsafe { w.sa().bits(addr).rs().set_bit() });
                                }

                                let sz = buffer.len();

                                i2c_busy_wait!(self.i2c, busbsy, bit_is_clear)?;

                                if sz == 1 {
                                    // Single receive
                                    self.i2c.mcs.write(|w| {
                                        if send_stop {
                                            w.stop().set_bit();
                                        };
                                        if send_start {
                                            w.start().set_bit();
                                        };
                                        if !op_change {
                                            w.ack().set_bit();
                                        };
                                        w.run().set_bit()
                                    });

                                    i2c_busy_wait!(self.i2c)?;
                                    buffer[0] = self.i2c.mdr.read().data().bits();
                                } else {
                                    self.i2c.mcs.write(|w| {
                                        if send_start {
                                            w.start().set_bit();
                                        };
                                        w.run().set_bit().ack().set_bit()
                                    });

                                    i2c_busy_wait!(self.i2c)?;
                                    buffer[0] = self.i2c.mdr.read().data().bits();

                                    for byte in &mut buffer[1..sz - 1] {
                                        self.i2c.mcs.write(|w| w.run().set_bit().ack().set_bit());
                                        i2c_busy_wait!(self.i2c)?;
                                        *byte = self.i2c.mdr.read().data().bits();
                                    }

                                    self.i2c.mcs.write(|w| {
                                        if send_stop {
                                            w.stop().set_bit();
                                        };
                                        if !op_change {
                                            w.ack().set_bit();
                                        };
                                        w.run().set_bit()
                                    });

                                    i2c_busy_wait!(self.i2c)?;
                                    buffer[sz - 1] = self.i2c.mdr.read().data().bits();
                                }
                            }
                        }

                        send_start = op_change;
                    }

                    i2c_busy_wait!(self.i2c)?;
                    Ok(())
                }
            }
    };
}
