use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn wake(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::PWR_MGMT_1, 0x00)
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::PWR_MGMT_1, 0x80)
    }
}
