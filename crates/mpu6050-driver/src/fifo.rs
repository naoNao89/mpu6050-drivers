use embedded_hal::i2c::I2c;

use crate::{Mpu6050, registers};

/// Bytes per FIFO frame for accel XYZ + gyro XYZ, no temp/ext slaves.
pub const FIFO_ACCEL_GYRO_FRAME_BYTES: usize = 12;

/// Low-level FIFO diagnostics for streaming/debug burst reads.
///
/// Applications should prefer `frame_usable` and `should_reset_fifo` over
/// interpreting every field directly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FifoReadDiagnostics {
    pub fifo_count_before_bytes: u16,
    pub fifo_bytes_requested: u16,
    pub fifo_count_after_bytes: u16,
    /// True only when this diagnostics call successfully read `INT_STATUS`
    /// and observed `FIFO_OFLOW` set.
    ///
    /// This is not persistent overflow history. A previous `INT_STATUS` read
    /// may consume the overflow flag before this method observes it.
    pub fifo_overflow_seen: bool,
    /// True when INT_STATUS was read successfully.
    pub int_status_read_ok: bool,
    pub read_len_frame_aligned: bool,
    pub fifo_count_before_frame_aligned: bool,
    pub fifo_count_after_frame_aligned: bool,
    pub had_requested_bytes_before_read: bool,
    pub fifo_count_delta_ok: bool,
}

impl FifoReadDiagnostics {
    /// True when the requested read is non-empty, no overflow was confirmed,
    /// read length and FIFO counts are frame-aligned, and enough bytes were
    /// present before the read.
    ///
    /// Does not require `fifo_count_delta_ok` because FIFO may refill while enabled.
    pub const fn frame_usable(&self) -> bool {
        !self.fifo_overflow_seen
            && self.read_len_frame_aligned
            && self.fifo_count_before_frame_aligned
            && self.fifo_count_after_frame_aligned
            && self.had_requested_bytes_before_read
            && self.fifo_bytes_requested > 0
    }

    /// True when confirmed overflow or bad frame alignment suggests FIFO reset.
    ///
    /// FIFO reset should follow the device-safe sequence, disabling FIFO/sources
    /// before reset as appropriate.
    pub const fn should_reset_fifo(&self) -> bool {
        self.fifo_overflow_seen
            || !self.fifo_count_before_frame_aligned
            || !self.fifo_count_after_frame_aligned
    }
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn reset_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::USER_CTRL, registers::USER_CTRL_FIFO_RESET)
    }

    pub fn enable_motion_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_register(
            registers::FIFO_EN,
            registers::FIFO_SOURCES_ACCEL_XYZ_GYRO_XYZ,
        )
    }

    pub fn disable_fifo_sources(&mut self) -> Result<(), I2C::Error> {
        self.write_register(registers::FIFO_EN, 0)
    }

    pub fn enable_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(
            registers::USER_CTRL,
            registers::USER_CTRL_FIFO_EN,
            registers::USER_CTRL_FIFO_EN,
        )
    }

    pub fn disable_fifo(&mut self) -> Result<(), I2C::Error> {
        self.write_masked(registers::USER_CTRL, registers::USER_CTRL_FIFO_EN, 0)
    }

    pub fn fifo_count(&mut self) -> Result<u16, I2C::Error> {
        let mut bytes = [0_u8; 2];
        self.i2c
            .write_read(self.address.as_u8(), &[registers::FIFO_COUNTH], &mut bytes)?;
        Ok(u16::from_be_bytes(bytes))
    }

    pub fn read_fifo_bytes(&mut self, bytes: &mut [u8]) -> Result<(), I2C::Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.i2c
            .write_read(self.address.as_u8(), &[registers::FIFO_R_W], bytes)
    }

    /// Reads FIFO bytes and returns diagnostics around the burst read.
    ///
    /// `INT_STATUS` is read best-effort. Reading it clears interrupt status
    /// bits on supported MPU devices. Callers should avoid reading
    /// `INT_STATUS` immediately before this method when they expect
    /// `fifo_overflow_seen` to report a pending overflow event.
    ///
    /// `fifo_overflow_seen == false` does not prove that overflow never
    /// occurred, because the flag may have been consumed by an earlier read.
    /// Frame-alignment diagnostics provide an additional recovery signal.
    pub fn read_fifo_bytes_with_diagnostics(
        &mut self,
        bytes: &mut [u8],
    ) -> Result<FifoReadDiagnostics, I2C::Error> {
        self.read_fifo_bytes_with_diagnostics_frame_size(bytes, FIFO_ACCEL_GYRO_FRAME_BYTES)
    }

    fn read_fifo_bytes_with_diagnostics_frame_size(
        &mut self,
        bytes: &mut [u8],
        frame_size: usize,
    ) -> Result<FifoReadDiagnostics, I2C::Error> {
        let requested_len = bytes.len();
        let fifo_count_before_bytes = self.fifo_count()?;
        let (int_status_read_ok, fifo_overflow_seen) = match self.int_status() {
            Ok(status) => (true, status.fifo_overflow()),
            Err(_) => (false, false),
        };
        self.read_fifo_bytes(bytes)?;
        let fifo_count_after_bytes = self.fifo_count()?;
        let fifo_bytes_requested = requested_len.min(u16::MAX as usize) as u16;
        let read_len_frame_aligned = frame_size != 0 && requested_len.is_multiple_of(frame_size);
        let fifo_count_before_frame_aligned =
            frame_size != 0 && (fifo_count_before_bytes as usize).is_multiple_of(frame_size);
        let fifo_count_after_frame_aligned =
            frame_size != 0 && (fifo_count_after_bytes as usize).is_multiple_of(frame_size);
        let had_requested_bytes_before_read = fifo_count_before_bytes as usize >= requested_len;
        let fifo_count_delta_ok = (fifo_count_before_bytes as usize)
            .checked_sub(requested_len)
            .map(|count| count == fifo_count_after_bytes as usize)
            .unwrap_or(false);

        Ok(FifoReadDiagnostics {
            fifo_count_before_bytes,
            fifo_bytes_requested,
            fifo_count_after_bytes,
            fifo_overflow_seen,
            int_status_read_ok,
            read_len_frame_aligned,
            fifo_count_before_frame_aligned,
            fifo_count_after_frame_aligned,
            had_requested_bytes_before_read,
            fifo_count_delta_ok,
        })
    }
}
