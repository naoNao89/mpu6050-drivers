use mpu6050_driver::{
    ACCEL_LSB_PER_G_2G, AccelRange, Address, FIFO_ACCEL_GYRO_FRAME_BYTES, FifoReadDiagnostics,
    GYRO_LSB_PER_DPS_250DPS, GyroRange, Identity, IntStatus, Mpu6050, RawAccelGyroTemp,
    RawReadOutcome, RawRetryPolicy, RawSampleSuspicion, TEMP_LSB_PER_DEG_C, TEMP_OFFSET_DEG_C,
    raw_to_imu_sample,
};

#[test]
fn crate_root_public_api_still_imports() {
    let raw = RawAccelGyroTemp::new([0, 0, 16_384], 0, [0, 0, 131]);
    let _sample = raw_to_imu_sample(raw);
    Mpu6050::new((), Address::Ad0Low).release();

    let diagnostics = FifoReadDiagnostics {
        fifo_count_before_bytes: FIFO_ACCEL_GYRO_FRAME_BYTES as u16,
        fifo_bytes_requested: FIFO_ACCEL_GYRO_FRAME_BYTES as u16,
        fifo_count_after_bytes: 0,
        fifo_overflow_seen: false,
        int_status_read_ok: true,
        read_len_frame_aligned: true,
        fifo_count_before_frame_aligned: true,
        fifo_count_after_frame_aligned: true,
        had_requested_bytes_before_read: true,
        fifo_count_delta_ok: true,
    };

    assert!(diagnostics.frame_usable());
    assert!(!diagnostics.should_reset_fifo());
    assert!(!raw.is_suspicious());
    assert_eq!(raw.temp_degrees_c(), TEMP_OFFSET_DEG_C);
    assert_eq!(ACCEL_LSB_PER_G_2G, 16_384.0);
    assert_eq!(GYRO_LSB_PER_DPS_250DPS, 131.0);
    assert_eq!(TEMP_LSB_PER_DEG_C, 340.0);

    let _ = AccelRange::G2;
    let _ = GyroRange::Dps250;
    let _ = Identity::Mpu6050;
    let _ = RawRetryPolicy::reject_after_retries(0);
    let _ = RawRetryPolicy::accept_after_retries(1);
    let _ = RawReadOutcome::<()>::Clean { raw };
    let _ = RawSampleSuspicion::GyroPartialMinusOne;

    fn takes_int_status(_status: IntStatus) {}
    let _ = takes_int_status;
}
