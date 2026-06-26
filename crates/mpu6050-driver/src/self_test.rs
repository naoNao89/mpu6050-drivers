use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn set_accel_self_test(&mut self, enabled: bool) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::ACCEL_CONFIG,
            registers::SELF_TEST_MASK,
            if enabled {
                registers::SELF_TEST_MASK
            } else {
                0
            },
        )
    }

    pub fn set_gyro_self_test(&mut self, enabled: bool) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::GYRO_CONFIG,
            registers::SELF_TEST_MASK,
            if enabled {
                registers::SELF_TEST_MASK
            } else {
                0
            },
        )
    }
}
