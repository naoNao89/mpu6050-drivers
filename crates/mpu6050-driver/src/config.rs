use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

/// MPU6050 I2C address selected by the AD0 pin.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Address {
    Ad0Low = 0x68,
    Ad0High = 0x69,
}

impl Address {
    pub(crate) const fn as_u8(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Identity {
    Mpu6050,
    Mpu6500Compatible,
    Unknown(u8),
}

impl Identity {
    pub(crate) const fn from_who_am_i(id: u8) -> Self {
        decode_identity(id)
    }
}

const fn decode_identity(id: u8) -> Identity {
    match id {
        0x68 => Identity::Mpu6050,
        0x70 => Identity::Mpu6500Compatible,
        other => Identity::Unknown(other),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccelRange {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelRange {
    const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroRange {
    Dps250 = 0,
    Dps500 = 1,
    Dps1000 = 2,
    Dps2000 = 3,
}

impl GyroRange {
    const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn who_am_i(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(registers::WHO_AM_I)
    }

    pub fn identity(&mut self) -> Result<Identity, I2C::Error> {
        self.who_am_i().map(Identity::from_who_am_i)
    }

    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::ACCEL_CONFIG,
            registers::ACCEL_RANGE_MASK,
            range.bits(),
        )
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::GYRO_CONFIG,
            registers::GYRO_RANGE_MASK,
            range.bits(),
        )
    }
}
