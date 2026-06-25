use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntStatus {
    pub(crate) bits: u8,
}

impl IntStatus {
    pub const fn data_ready(self) -> bool {
        self.bits & registers::INT_STATUS_DATA_RDY != 0
    }

    pub const fn fifo_overflow(self) -> bool {
        self.bits & registers::INT_STATUS_FIFO_OFLOW != 0
    }
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn enable_data_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::INT_ENABLE,
            registers::INT_ENABLE_DATA_RDY,
            registers::INT_ENABLE_DATA_RDY,
        )
    }

    pub fn enable_fifo_overflow_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::INT_ENABLE,
            registers::INT_ENABLE_FIFO_OFLOW,
            registers::INT_ENABLE_FIFO_OFLOW,
        )
    }

    pub fn int_status(&mut self) -> Result<IntStatus, I2C::Error> {
        self.read_register(registers::INT_STATUS)
            .map(|bits| IntStatus { bits })
    }
}
