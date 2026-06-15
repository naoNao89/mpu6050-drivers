#![no_std]
#![no_main]

use core::fmt;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{BusTimeout, Config as I2cConfig, I2c, SoftwareTimeout},
    main,
    time::{Duration, Instant, Rate},
};
use esp_println::println;

const MPU_ADDR_AD0_LOW: u8 = 0x68;
const MPU_ADDR_AD0_HIGH: u8 = 0x69;

const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_FIFO_EN: u8 = 0x23;
const REG_INT_ENABLE: u8 = 0x38;
const REG_INT_STATUS: u8 = 0x3A;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_FIFO_COUNTH: u8 = 0x72;
const REG_FIFO_R_W: u8 = 0x74;
const REG_WHO_AM_I: u8 = 0x75;

const ACCEL_RANGE_MASK: u8 = 0x18;
const GYRO_RANGE_MASK: u8 = 0x18;
const SELF_TEST_MASK: u8 = 0xE0;
const USER_CTRL_FIFO_EN: u8 = 1 << 6;
const USER_CTRL_FIFO_RESET: u8 = 1 << 2;
const FIFO_EN_ACCEL_XYZ_GYRO_XYZ: u8 = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3);
const INT_ENABLE_DATA_RDY: u8 = 1 << 0;
const INT_ENABLE_FIFO_OFLOW: u8 = 1 << 4;
const INT_STATUS_DATA_RDY: u8 = 1 << 0;
const INT_STATUS_FIFO_OFLOW: u8 = 1 << 4;
const FIFO_ACCEL_GYRO_FRAME_BYTES: u16 = 12;
const RAW_STREAM_PERIOD_MS: u32 = 100;

// Reference dev-board wiring used by this bring-up firmware.
//
// The repo's board-under-test is an ESP32-C3 SuperMini-class board connected to
// a GY-521/MPU6050 module. Keep these constants aligned with the README so the
// firmware is explicitly a board sample that exercises this MPU6050 driver
// stack, rather than an anonymous ESP32-C3 snippet.
const BOARD_NAME: &str = "ESP32-C3 SuperMini-class dev board";
const I2C_BUS_NAME: &str = "I2C0";
const I2C_FREQUENCY_KHZ: u32 = 100;
const SCL_PIN_NAME: &str = "GPIO0";
const SDA_PIN_NAME: &str = "GPIO1";
const AD0_PIN_NAME: &str = "GPIO5";
const INT_PIN_NAME: &str = "GPIO6";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ImuIdentity {
    Mpu6050,
    Mpu6500Compatible,
    Unknown(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IdentityVerdict {
    ClassicMpu6050,
    Mpu6500Compatible,
    Unknown,
}

impl IdentityVerdict {
    fn from_identity(identity: ImuIdentity) -> Self {
        match identity {
            ImuIdentity::Mpu6050 => Self::ClassicMpu6050,
            ImuIdentity::Mpu6500Compatible => Self::Mpu6500Compatible,
            ImuIdentity::Unknown(_) => Self::Unknown,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::ClassicMpu6050 => "ClassicMpu6050",
            Self::Mpu6500Compatible => "Mpu6500CompatibleNonClassic",
            Self::Unknown => "Unknown",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VerificationLevel {
    MarkingOnly,
    I2cResponsive,
    RegisterCompatible,
    MotionVerified,
    AdvancedVerified,
}

impl VerificationLevel {
    fn from_score(score: u8) -> Self {
        match score {
            0..=5 => Self::MarkingOnly,
            6..=12 => Self::I2cResponsive,
            13..=22 => Self::RegisterCompatible,
            23..=35 => Self::MotionVerified,
            _ => Self::AdvancedVerified,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::MarkingOnly => "MarkingOnly",
            Self::I2cResponsive => "I2cResponsiveCompatibleDevice",
            Self::RegisterCompatible => "FunctionalRegisterCompatibleImu",
            Self::MotionVerified => "MotionVerifiedCompatibleImu",
            Self::AdvancedVerified => "AdvancedVerifiedCompatibleImu",
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct VerificationEvidence {
    package_marking_matches: bool,
    i2c_ack: bool,
    identity: Option<ImuIdentity>,
    pwr_mgmt_1_readable: bool,
    raw_block_readable: bool,
}

impl VerificationEvidence {
    fn score(self) -> u8 {
        let mut score = 0;
        if self.package_marking_matches {
            score += 1;
        }
        if self.i2c_ack {
            score += 2;
        }
        match self.identity {
            Some(ImuIdentity::Mpu6050) => score += 4,
            Some(ImuIdentity::Mpu6500Compatible) => score += 2,
            Some(ImuIdentity::Unknown(_)) | None => {}
        }
        if self.pwr_mgmt_1_readable {
            score += 3;
        }
        if self.raw_block_readable {
            score += 3;
        }
        score
    }

    fn identity_verdict(self) -> IdentityVerdict {
        self.identity
            .map(IdentityVerdict::from_identity)
            .unwrap_or(IdentityVerdict::Unknown)
    }

    fn level(self) -> VerificationLevel {
        VerificationLevel::from_score(self.score())
    }
}

#[derive(Debug, Clone, Copy)]
struct ProbeResult {
    address: u8,
    who_am_i: Option<u8>,
    pwr_mgmt_1: Option<u8>,
    raw_block_readable: bool,
}

#[derive(Debug, Clone, Copy)]
struct RawAverage {
    ax: i32,
    ay: i32,
    az: i32,
    gx: i32,
    gy: i32,
    gz: i32,
}

struct HexOpt(Option<u8>);
struct U16Opt(Option<u16>);

impl fmt::Display for HexOpt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "0x{:02x}", value),
            None => f.write_str("unreadable"),
        }
    }
}

impl fmt::Display for U16Opt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(value) => write!(f, "{}", value),
            None => f.write_str("unreadable"),
        }
    }
}

impl ImuIdentity {
    fn from_who_am_i(id: u8) -> Self {
        match id {
            0x68 => Self::Mpu6050,
            0x70 => Self::Mpu6500Compatible,
            other => Self::Unknown(other),
        }
    }

    fn description(self) -> &'static str {
        match self {
            Self::Mpu6050 => "MPU-6050-class IMU",
            Self::Mpu6500Compatible => "MPU-6500-compatible / clone / relabeled variant",
            Self::Unknown(_) => "unknown IMU identity",
        }
    }
}

#[main]
fn main() -> ! {
    let mut peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    println!("MPU6050 ESP32-C3 esp-hal I2C bring-up started");
    println!("Board profile: {}", BOARD_NAME);
    println!(
        "Wiring: VCC=3V GND=GND SCL={} SDA={} AD0={} INT={}",
        SCL_PIN_NAME, SDA_PIN_NAME, AD0_PIN_NAME, INT_PIN_NAME
    );

    let scl_probe = Input::new(
        peripherals.GPIO0.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    let sda_probe = Input::new(
        peripherals.GPIO1.reborrow(),
        InputConfig::default().with_pull(Pull::Up),
    );
    println!(
        "Pre-I2C idle check: SCL={} SDA={}",
        if scl_probe.is_high() { "HIGH" } else { "LOW" },
        if sda_probe.is_high() { "HIGH" } else { "LOW" }
    );
    drop(scl_probe);
    drop(sda_probe);

    let _ad0 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    println!("AD0 driven LOW on GPIO5; expected 7-bit address is 0x68");

    let config = I2cConfig::default()
        .with_frequency(Rate::from_khz(I2C_FREQUENCY_KHZ))
        .with_timeout(BusTimeout::Maximum)
        .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(50)));

    let mut i2c = I2c::new(peripherals.I2C0, config)
        .expect("failed to initialize I2C0")
        .with_scl(peripherals.GPIO0)
        .with_sda(peripherals.GPIO1);

    println!(
        "{} initialized at {} kHz: SCL={} SDA={}",
        I2C_BUS_NAME, I2C_FREQUENCY_KHZ, SCL_PIN_NAME, SDA_PIN_NAME
    );

    scan_candidates(&mut i2c);
    let primary_probe = probe_imu(&mut i2c, MPU_ADDR_AD0_LOW);
    probe_imu(&mut i2c, MPU_ADDR_AD0_HIGH);
    log_verification_summary(primary_probe);
    run_advanced_validation(&mut i2c, &delay, MPU_ADDR_AD0_LOW);

    println!(
        "Repeating raw read from 0x68 every {}ms",
        RAW_STREAM_PERIOD_MS
    );
    let mut raw_sequence: u64 = 0;
    loop {
        read_motion_sample(&mut i2c, MPU_ADDR_AD0_LOW, &mut raw_sequence);
        delay.delay_millis(RAW_STREAM_PERIOD_MS);
    }
}

fn run_advanced_validation(i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &Delay, address: u8) {
    println!("advanced_validation_begin");
    reset_wake_configure(i2c, delay, address);
    validate_scale_registers(i2c, address);
    validate_self_test_coarse(i2c, delay, address);
    validate_fifo_timing(i2c, delay, address);
    validate_int_status(i2c, address);
    println!("advanced_validation_end");
}

fn reset_wake_configure(i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &Delay, address: u8) {
    println!("advanced reset_wake_begin");
    let reset_ok = write_reg(i2c, address, REG_PWR_MGMT_1, 0x80).is_ok();
    delay.delay_millis(100);
    let wake_ok = write_reg(i2c, address, REG_PWR_MGMT_1, 0x01).is_ok();
    delay.delay_millis(20);
    let config_ok = write_reg(i2c, address, REG_CONFIG, 0x03).is_ok();
    let sample_ok = write_reg(i2c, address, REG_SMPLRT_DIV, 9).is_ok();
    let accel_ok = write_masked(
        i2c,
        address,
        REG_ACCEL_CONFIG,
        ACCEL_RANGE_MASK | SELF_TEST_MASK,
        0x00,
    )
    .is_ok();
    let gyro_ok = write_masked(
        i2c,
        address,
        REG_GYRO_CONFIG,
        GYRO_RANGE_MASK | SELF_TEST_MASK,
        0x00,
    )
    .is_ok();
    let pwr = read_reg(i2c, address, REG_PWR_MGMT_1).ok();
    let config = read_reg(i2c, address, REG_CONFIG).ok();
    let smplrt = read_reg(i2c, address, REG_SMPLRT_DIV).ok();
    println!(
        "advanced reset_wake reset_ok={} wake_ok={} config_ok={} sample_ok={} accel_cfg_ok={} gyro_cfg_ok={} pwr_mgmt_1={} config={} smplrt_div={}",
        reset_ok,
        wake_ok,
        config_ok,
        sample_ok,
        accel_ok,
        gyro_ok,
        fmt_opt_hex(pwr),
        fmt_opt_hex(config),
        fmt_opt_hex(smplrt)
    );
    println!("advanced reset_wake_end");
}

fn validate_scale_registers(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8) {
    println!("advanced scale_range_begin");
    for setting in 0..=3u8 {
        let accel_bits = setting << 3;
        let gyro_bits = setting << 3;
        let accel_write =
            write_masked(i2c, address, REG_ACCEL_CONFIG, ACCEL_RANGE_MASK, accel_bits).is_ok();
        let gyro_write =
            write_masked(i2c, address, REG_GYRO_CONFIG, GYRO_RANGE_MASK, gyro_bits).is_ok();
        let accel_read = read_reg(i2c, address, REG_ACCEL_CONFIG).ok();
        let gyro_read = read_reg(i2c, address, REG_GYRO_CONFIG).ok();
        let accel_match = accel_read
            .map(|v| v & ACCEL_RANGE_MASK == accel_bits)
            .unwrap_or(false);
        let gyro_match = gyro_read
            .map(|v| v & GYRO_RANGE_MASK == gyro_bits)
            .unwrap_or(false);
        println!(
            "advanced scale_range setting={} accel_write={} accel_reg={} accel_match={} gyro_write={} gyro_reg={} gyro_match={}",
            setting,
            accel_write,
            fmt_opt_hex(accel_read),
            accel_match,
            gyro_write,
            fmt_opt_hex(gyro_read),
            gyro_match
        );
    }
    let _ = write_masked(i2c, address, REG_ACCEL_CONFIG, ACCEL_RANGE_MASK, 0x00);
    let _ = write_masked(i2c, address, REG_GYRO_CONFIG, GYRO_RANGE_MASK, 0x00);
    println!("advanced scale_range_end");
}

fn validate_self_test_coarse(i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &Delay, address: u8) {
    println!("advanced self_test_begin");
    let _ = write_masked(
        i2c,
        address,
        REG_ACCEL_CONFIG,
        ACCEL_RANGE_MASK | SELF_TEST_MASK,
        0x00,
    );
    let _ = write_masked(
        i2c,
        address,
        REG_GYRO_CONFIG,
        GYRO_RANGE_MASK | SELF_TEST_MASK,
        0x00,
    );
    delay.delay_millis(50);
    let baseline = average_raw(i2c, delay, address, 8);
    let accel_st_ok = write_masked(
        i2c,
        address,
        REG_ACCEL_CONFIG,
        SELF_TEST_MASK,
        SELF_TEST_MASK,
    )
    .is_ok();
    let gyro_st_ok = write_masked(
        i2c,
        address,
        REG_GYRO_CONFIG,
        SELF_TEST_MASK,
        SELF_TEST_MASK,
    )
    .is_ok();
    delay.delay_millis(100);
    let self_test = average_raw(i2c, delay, address, 8);
    let _ = write_masked(i2c, address, REG_ACCEL_CONFIG, SELF_TEST_MASK, 0x00);
    let _ = write_masked(i2c, address, REG_GYRO_CONFIG, SELF_TEST_MASK, 0x00);
    delay.delay_millis(50);

    if let (Some(base), Some(st)) = (baseline, self_test) {
        let accel_delta = abs3_sum(st.ax - base.ax, st.ay - base.ay, st.az - base.az);
        let gyro_delta = abs3_sum(st.gx - base.gx, st.gy - base.gy, st.gz - base.gz);
        println!(
            "advanced self_test accel_st_write={} gyro_st_write={} baseline_accel=({},{},{}) selftest_accel=({},{},{}) baseline_gyro=({},{},{}) selftest_gyro=({},{},{}) accel_delta_sum={} gyro_delta_sum={} coarse_response={}",
            accel_st_ok,
            gyro_st_ok,
            base.ax,
            base.ay,
            base.az,
            st.ax,
            st.ay,
            st.az,
            base.gx,
            base.gy,
            base.gz,
            st.gx,
            st.gy,
            st.gz,
            accel_delta,
            gyro_delta,
            accel_delta > 100 || gyro_delta > 100
        );
    } else {
        println!(
            "advanced self_test accel_st_write={} gyro_st_write={} baseline_readable={} selftest_readable={} coarse_response=false",
            accel_st_ok,
            gyro_st_ok,
            baseline.is_some(),
            self_test.is_some()
        );
    }
    println!("advanced self_test_end");
}

fn validate_fifo_timing(i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &Delay, address: u8) {
    println!("advanced fifo_timing_begin");
    let disable_fifo_ok = write_reg(i2c, address, REG_FIFO_EN, 0x00).is_ok();
    let user_reset_ok = write_reg(i2c, address, REG_USER_CTRL, USER_CTRL_FIFO_RESET).is_ok();
    delay.delay_millis(20);
    let enable_sources_ok =
        write_reg(i2c, address, REG_FIFO_EN, FIFO_EN_ACCEL_XYZ_GYRO_XYZ).is_ok();
    let enable_fifo_ok = write_reg(i2c, address, REG_USER_CTRL, USER_CTRL_FIFO_EN).is_ok();
    let count0 = read_fifo_count(i2c, address);
    delay.delay_millis(250);
    let count1 = read_fifo_count(i2c, address);
    let mut frame_read_ok = false;
    if let Some(count) = count1 {
        if count >= FIFO_ACCEL_GYRO_FRAME_BYTES {
            frame_read_ok =
                read_fifo_bytes(i2c, address, FIFO_ACCEL_GYRO_FRAME_BYTES as usize).is_ok();
        }
    }
    let disable_after_ok = write_reg(i2c, address, REG_FIFO_EN, 0x00).is_ok()
        && write_reg(i2c, address, REG_USER_CTRL, 0x00).is_ok();
    println!(
        "advanced fifo_timing disable_fifo_ok={} user_reset_ok={} enable_sources_ok={} enable_fifo_ok={} count0={} count1={} frame_bytes={} frame_read_ok={} disable_after_ok={}",
        disable_fifo_ok,
        user_reset_ok,
        enable_sources_ok,
        enable_fifo_ok,
        fmt_opt_u16(count0),
        fmt_opt_u16(count1),
        FIFO_ACCEL_GYRO_FRAME_BYTES,
        frame_read_ok,
        disable_after_ok
    );
    println!("advanced fifo_timing_end");
}

fn validate_int_status(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8) {
    println!("advanced int_status_begin");
    let enable_ok = write_reg(
        i2c,
        address,
        REG_INT_ENABLE,
        INT_ENABLE_DATA_RDY | INT_ENABLE_FIFO_OFLOW,
    )
    .is_ok();
    let status = read_reg(i2c, address, REG_INT_STATUS).ok();
    let data_ready = status
        .map(|v| v & INT_STATUS_DATA_RDY != 0)
        .unwrap_or(false);
    let fifo_overflow = status
        .map(|v| v & INT_STATUS_FIFO_OFLOW != 0)
        .unwrap_or(false);
    println!(
        "advanced int_status enable_ok={} int_status={} data_ready={} fifo_overflow={}",
        enable_ok,
        fmt_opt_hex(status),
        data_ready,
        fifo_overflow
    );
    println!("advanced int_status_end");
}

fn scan_candidates(i2c: &mut I2c<'_, esp_hal::Blocking>) {
    println!("I2C candidate scan: 0x68, 0x69");
    for address in [MPU_ADDR_AD0_LOW, MPU_ADDR_AD0_HIGH] {
        match read_reg(i2c, address, REG_WHO_AM_I) {
            Ok(value) => println!("ACK/read at 0x{:02x}: WHO_AM_I=0x{:02x}", address, value),
            Err(error) => println!("No read at 0x{:02x}: {:?}", address, error),
        }
    }
}

fn probe_imu(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8) -> ProbeResult {
    println!("Probing bus_address=0x{:02x}", address);
    let mut who_am_i = None;
    let mut pwr_mgmt_1 = None;

    match read_reg(i2c, address, REG_WHO_AM_I) {
        Ok(value) => {
            who_am_i = Some(value);
            let identity = ImuIdentity::from_who_am_i(value);
            println!(
                "bus_address=0x{:02x} who_am_i=0x{:02x} identity={}",
                address,
                value,
                identity.description()
            );
            if let ImuIdentity::Unknown(id) = identity {
                println!(
                    "bus_address=0x{:02x}: unknown WHO_AM_I=0x{:02x}; raw reads will still be attempted",
                    address, id
                );
            }
        }
        Err(error) => println!("0x{:02x}: WHO_AM_I read failed: {:?}", address, error),
    }

    match read_reg(i2c, address, REG_PWR_MGMT_1) {
        Ok(value) => {
            pwr_mgmt_1 = Some(value);
            println!("bus_address=0x{:02x} pwr_mgmt_1=0x{:02x}", address, value)
        }
        Err(error) => println!("0x{:02x}: PWR_MGMT_1 read failed: {:?}", address, error),
    }

    let raw_block_readable = read_raw_block(i2c, address).is_ok();
    println!(
        "bus_address=0x{:02x} raw_block_0x3b_readable={}",
        address, raw_block_readable
    );

    ProbeResult {
        address,
        who_am_i,
        pwr_mgmt_1,
        raw_block_readable,
    }
}

fn log_verification_summary(probe: ProbeResult) {
    let identity = probe.who_am_i.map(ImuIdentity::from_who_am_i);
    let evidence = VerificationEvidence {
        package_marking_matches: true,
        i2c_ack: probe.who_am_i.is_some() || probe.pwr_mgmt_1.is_some(),
        identity,
        pwr_mgmt_1_readable: probe.pwr_mgmt_1.is_some(),
        raw_block_readable: probe.raw_block_readable,
    };
    let score = evidence.score();
    println!("verification_summary_begin");
    println!("bus_address=0x{:02x}", probe.address);
    match probe.who_am_i {
        Some(value) => println!("who_am_i=0x{:02x}", value),
        None => println!("who_am_i=unreadable"),
    }
    match probe.pwr_mgmt_1 {
        Some(value) => println!("pwr_mgmt_1=0x{:02x}", value),
        None => println!("pwr_mgmt_1=unreadable"),
    }
    println!("identity_verdict={}", evidence.identity_verdict().as_str());
    println!("verification_score={}", score);
    println!("verification_level={}", evidence.level().as_str());
    println!(
        "non_classic_identity={}",
        matches!(identity, Some(ImuIdentity::Mpu6500Compatible))
    );
    println!(
        "pending_tests=six_face,accel_scale_range,gyro_scale_range,gyro_bias,temp_sanity,self_test,fifo_interrupt,timing_noise"
    );
    println!("verification_summary_end");
}

fn read_motion_sample(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8, raw_sequence: &mut u64) {
    match read_raw_block(i2c, address) {
        Ok(data) => {
            let timestamp_us = Instant::now().duration_since_epoch().as_micros();
            let ax = be_i16(data[0], data[1]);
            let ay = be_i16(data[2], data[3]);
            let az = be_i16(data[4], data[5]);
            let temp = be_i16(data[6], data[7]);
            let gx = be_i16(data[8], data[9]);
            let gy = be_i16(data[10], data[11]);
            let gz = be_i16(data[12], data[13]);
            println!(
                "RAW 0x{:02x}: accel=({}, {}, {}) temp_raw={} gyro=({}, {}, {}) timestamp_us={} sequence={} timestamp_source=device_instant",
                address, ax, ay, az, temp, gx, gy, gz, timestamp_us, *raw_sequence
            );
            *raw_sequence = raw_sequence.wrapping_add(1);
        }
        Err(error) => println!("RAW 0x{:02x}: read failed: {:?}", address, error),
    }
}

fn read_raw_block(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
) -> Result<[u8; 14], esp_hal::i2c::master::Error> {
    let mut data = [0u8; 14];
    i2c.write_read(address, &[REG_ACCEL_XOUT_H], &mut data)?;
    Ok(data)
}

fn read_reg(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
    register: u8,
) -> Result<u8, esp_hal::i2c::master::Error> {
    let mut data = [0u8; 1];
    i2c.write_read(address, &[register], &mut data)?;
    Ok(data[0])
}

fn write_reg(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
    register: u8,
    value: u8,
) -> Result<(), esp_hal::i2c::master::Error> {
    i2c.write(address, &[register, value])
}

fn write_masked(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
    register: u8,
    mask: u8,
    value: u8,
) -> Result<(), esp_hal::i2c::master::Error> {
    let current = read_reg(i2c, address, register)?;
    write_reg(i2c, address, register, (current & !mask) | (value & mask))
}

fn read_fifo_count(i2c: &mut I2c<'_, esp_hal::Blocking>, address: u8) -> Option<u16> {
    let mut data = [0u8; 2];
    i2c.write_read(address, &[REG_FIFO_COUNTH], &mut data)
        .ok()?;
    Some(u16::from_be_bytes(data))
}

fn read_fifo_bytes(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    address: u8,
    len: usize,
) -> Result<(), esp_hal::i2c::master::Error> {
    let mut data = [0u8; FIFO_ACCEL_GYRO_FRAME_BYTES as usize];
    i2c.write_read(address, &[REG_FIFO_R_W], &mut data[..len])
}

fn average_raw(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    delay: &Delay,
    address: u8,
    samples: i32,
) -> Option<RawAverage> {
    let mut ax = 0i32;
    let mut ay = 0i32;
    let mut az = 0i32;
    let mut gx = 0i32;
    let mut gy = 0i32;
    let mut gz = 0i32;
    for _ in 0..samples {
        let data = read_raw_block(i2c, address).ok()?;
        ax += be_i16(data[0], data[1]) as i32;
        ay += be_i16(data[2], data[3]) as i32;
        az += be_i16(data[4], data[5]) as i32;
        gx += be_i16(data[8], data[9]) as i32;
        gy += be_i16(data[10], data[11]) as i32;
        gz += be_i16(data[12], data[13]) as i32;
        delay.delay_millis(10);
    }
    Some(RawAverage {
        ax: ax / samples,
        ay: ay / samples,
        az: az / samples,
        gx: gx / samples,
        gy: gy / samples,
        gz: gz / samples,
    })
}

fn abs3_sum(x: i32, y: i32, z: i32) -> i32 {
    x.abs() + y.abs() + z.abs()
}

fn fmt_opt_hex(value: Option<u8>) -> HexOpt {
    HexOpt(value)
}

fn fmt_opt_u16(value: Option<u16>) -> U16Opt {
    U16Opt(value)
}

fn be_i16(high: u8, low: u8) -> i16 {
    i16::from_be_bytes([high, low])
}
