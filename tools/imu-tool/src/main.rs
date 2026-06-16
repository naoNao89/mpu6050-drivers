use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

#[derive(Clone, Debug)]
enum ExpectedIdentityArg {
    ClassicMpu6050,
    Mpu6500Compatible,
    AnyKnown,
    Exact(u8),
}

impl From<ExpectedIdentityArg> for imu_tool::ExpectedIdentity {
    fn from(value: ExpectedIdentityArg) -> Self {
        match value {
            ExpectedIdentityArg::ClassicMpu6050 => Self::ClassicMpu6050,
            ExpectedIdentityArg::Mpu6500Compatible => Self::Mpu6500Compatible,
            ExpectedIdentityArg::AnyKnown => Self::AnyKnown,
            ExpectedIdentityArg::Exact(v) => Self::Exact(v),
        }
    }
}

impl ExpectedIdentityArg {
    fn parse(s: &str) -> Result<Self, String> {
        match s.trim().to_ascii_lowercase().as_str() {
            "classic_mpu6050" | "classic-mpu6050" | "mpu6050" => Ok(Self::ClassicMpu6050),
            "mpu6500_compatible" | "mpu6500-compatible" | "mpu6500" => Ok(Self::Mpu6500Compatible),
            "any_known" | "any-known" | "any" => Ok(Self::AnyKnown),
            _ => Ok(Self::Exact(parse_hex_u8(s)?)),
        }
    }
}

#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Capture {
        #[arg(long)]
        port: String,
        #[arg(long, default_value_t = 30.0)]
        seconds: f64,
        #[arg(long, default_value_t = 115200)]
        baud: u32,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, value_enum, default_value = "text")]
        mode: StreamModeArg,
    },
    Monitor {
        #[arg(long)]
        port: String,
        #[arg(long, default_value_t = 115200)]
        baud: u32,
        #[arg(long)]
        out: Option<PathBuf>,
        #[arg(long, value_enum, default_value = "text")]
        mode: StreamModeArg,
    },
    OrientationCapture {
        #[arg(long)]
        port: String,
        #[arg(long, default_value_t = 60.0)]
        seconds: f64,
        #[arg(long, default_value_t = 115200)]
        baud: u32,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value_t = false)]
        stop_when_covered: bool,
        #[arg(long, default_value_t = 3)]
        min_samples_per_axis: usize,
        #[arg(long, default_value_t = 0.80)]
        mag_min: f64,
        #[arg(long, default_value_t = 1.20)]
        mag_max: f64,
        #[arg(long, default_value_t = 0.70)]
        dominance: f64,
    },
    SixfaceCapture {
        #[arg(long)]
        port: String,
        #[arg(long, default_value_t = 8.0)]
        seconds_per_face: f64,
        #[arg(long, default_value_t = 115200)]
        baud: u32,
        #[arg(long)]
        out: PathBuf,
    },
    StationarySuite {
        #[arg(long)]
        port: String,
        #[arg(long, default_value_t = 600.0)]
        seconds: f64,
        #[arg(long, default_value_t = 115200)]
        baud: u32,
        #[arg(long, default_value_t = 10.0)]
        sample_rate_hz: f64,
        #[arg(long, default_value = "stationary")]
        label: String,
        #[arg(long, default_value = "logs")]
        out_dir: PathBuf,
        #[arg(long, value_enum, default_value = "report")]
        validation_mode: ValidationModeArg,
        #[arg(long)]
        noise_psd_band_low_hz: Option<f64>,
        #[arg(long)]
        noise_psd_band_high_hz: Option<f64>,
    },
    Analyze {
        log: PathBuf,
        #[arg(long, default_value_t = 20)]
        min_samples: usize,
        #[arg(long, default_value_t = 10)]
        min_stationary_samples: usize,
        #[arg(long, default_value = "0x68")]
        expected_address: String,
        #[arg(long = "expected-identity", value_parser = ExpectedIdentityArg::parse, default_value = "any-known")]
        expected_identity: ExpectedIdentityArg,
        #[arg(long = "expected-whoami", hide = true, value_parser = ExpectedIdentityArg::parse)]
        expected_whoami_legacy: Option<ExpectedIdentityArg>,
    },
    OrientationAnalyze {
        log: PathBuf,
        #[arg(long, default_value_t = 3)]
        min_samples_per_axis: usize,
        #[arg(long, default_value_t = 0.80)]
        mag_min: f64,
        #[arg(long, default_value_t = 1.20)]
        mag_max: f64,
        #[arg(long, default_value_t = 0.70)]
        dominance: f64,
    },
    SixfaceAnalyze {
        log: PathBuf,
        #[arg(long, default_value_t = 5)]
        min_samples_per_face: usize,
        #[arg(long)]
        mapping: Option<PathBuf>,
    },
    ExportCsv {
        log: PathBuf,
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value_t = 10.0)]
        sample_rate_hz: f64,
    },
    AllanAnalyze {
        csv: PathBuf,
        #[arg(long, default_value_t = 10.0)]
        sample_rate_hz: f64,
        #[arg(long)]
        out: PathBuf,
    },
    PsdAnalyze {
        csv: PathBuf,
        #[arg(long, default_value_t = 10.0)]
        sample_rate_hz: f64,
        #[arg(long)]
        out: PathBuf,
    },
    SixfaceCalibration {
        log: PathBuf,
        #[arg(long)]
        out: PathBuf,
    },
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum ValidationModeArg {
    Report,
    Strict,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum StreamModeArg {
    Text,
    Binary,
}

impl From<StreamModeArg> for imu_tool::StreamMode {
    fn from(value: StreamModeArg) -> Self {
        match value {
            StreamModeArg::Text => Self::Text,
            StreamModeArg::Binary => Self::Binary,
        }
    }
}

impl From<ValidationModeArg> for imu_tool::ValidationMode {
    fn from(value: ValidationModeArg) -> Self {
        match value {
            ValidationModeArg::Report => imu_tool::ValidationMode::Report,
            ValidationModeArg::Strict => imu_tool::ValidationMode::Strict,
        }
    }
}

fn parse_hex_u8(s: &str) -> Result<u8, String> {
    u8::from_str_radix(s.trim_start_matches("0x"), 16).map_err(|_| format!("invalid hex byte: {s}"))
}

fn parse_hex(s: &str) -> i32 {
    parse_hex_u8(s).unwrap() as i32
}

fn main() -> std::process::ExitCode {
    let cli = Cli::parse();
    let result: Result<i32, Box<dyn std::error::Error>> = match cli.command {
        Command::Capture {
            port,
            seconds,
            baud,
            out,
            mode,
        } => imu_tool::capture(&port, baud, seconds, &out, mode.into()),
        Command::Monitor {
            port,
            baud,
            out,
            mode,
        } => imu_tool::monitor(&port, baud, out.as_deref(), mode.into()),
        Command::OrientationCapture {
            port,
            seconds,
            baud,
            out,
            stop_when_covered,
            min_samples_per_axis,
            mag_min,
            mag_max,
            dominance,
        } => imu_tool::orientation_capture(
            &port,
            baud,
            seconds,
            &out,
            stop_when_covered,
            min_samples_per_axis,
            mag_min,
            mag_max,
            dominance,
        ),
        Command::SixfaceCapture {
            port,
            seconds_per_face,
            baud,
            out,
        } => imu_tool::sixface_capture(&port, baud, seconds_per_face, &out),
        Command::StationarySuite {
            port,
            seconds,
            baud,
            sample_rate_hz,
            label,
            out_dir,
            validation_mode,
            noise_psd_band_low_hz,
            noise_psd_band_high_hz,
        } => imu_tool::stationary_suite(
            &port,
            baud,
            seconds,
            sample_rate_hz,
            &label,
            &out_dir,
            validation_mode.into(),
            noise_psd_band_low_hz,
            noise_psd_band_high_hz,
        ),
        Command::Analyze {
            log,
            min_samples,
            min_stationary_samples,
            expected_address,
            expected_identity,
            expected_whoami_legacy,
        } => imu_tool::analyze(
            &log,
            min_samples,
            min_stationary_samples,
            parse_hex(&expected_address),
            expected_whoami_legacy.unwrap_or(expected_identity).into(),
        )
        .map_err(Into::into),
        Command::OrientationAnalyze {
            log,
            min_samples_per_axis,
            mag_min,
            mag_max,
            dominance,
        } => imu_tool::orientation_analyze(&log, min_samples_per_axis, mag_min, mag_max, dominance)
            .map_err(Into::into),
        Command::SixfaceAnalyze {
            log,
            min_samples_per_face,
            mapping,
        } => imu_tool::sixface_analyze(&log, min_samples_per_face, mapping.as_deref())
            .map_err(Into::into),
        Command::ExportCsv {
            log,
            out,
            sample_rate_hz,
        } => imu_tool::export_csv(&log, &out, sample_rate_hz).map_err(Into::into),
        Command::AllanAnalyze {
            csv,
            sample_rate_hz,
            out,
        } => imu_tool::allan_analyze(&csv, sample_rate_hz, &out).map_err(Into::into),
        Command::PsdAnalyze {
            csv,
            sample_rate_hz,
            out,
        } => imu_tool::psd_analyze(&csv, sample_rate_hz, &out).map_err(Into::into),
        Command::SixfaceCalibration { log, out } => {
            imu_tool::sixface_calibration(&log, &out).map_err(Into::into)
        }
    };
    let rc = result.unwrap_or_else(|e| {
        eprintln!("error: {e}");
        2
    });
    std::process::ExitCode::from(rc as u8)
}
