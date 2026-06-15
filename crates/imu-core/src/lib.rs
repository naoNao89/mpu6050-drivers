#![no_std]

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImuSample {
    pub accel_g: [f64; 3],
    pub gyro_dps: [f64; 3],
    pub timestamp_s: Option<f64>,
    pub sequence: Option<u64>,
}

impl ImuSample {
    pub fn from_g_dps(accel_g: [f64; 3], gyro_dps: [f64; 3]) -> Self {
        Self {
            accel_g,
            gyro_dps,
            timestamp_s: None,
            sequence: None,
        }
    }

    pub fn from_si(accel_mps2: [f64; 3], gyro_radps: [f64; 3]) -> Self {
        const STANDARD_GRAVITY_MPS2: f64 = 9.80665;
        Self::from_g_dps(
            accel_mps2.map(|v| v / STANDARD_GRAVITY_MPS2),
            gyro_radps.map(f64::to_degrees),
        )
    }

    pub fn new(accel_g: [f64; 3], gyro_dps: [f64; 3]) -> Self {
        Self::from_g_dps(accel_g, gyro_dps)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AccelCalibration {
    pub offset_g: [f64; 3],
    pub scale: [f64; 3],
}

impl AccelCalibration {
    pub const fn identity() -> Self {
        Self {
            offset_g: [0.0; 3],
            scale: [1.0; 3],
        }
    }
}

impl Default for AccelCalibration {
    fn default() -> Self {
        Self::identity()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GyroCalibration {
    pub bias_dps: [f64; 3],
}

impl GyroCalibration {
    pub const fn identity() -> Self {
        Self { bias_dps: [0.0; 3] }
    }
}

impl Default for GyroCalibration {
    fn default() -> Self {
        Self::identity()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImuCalibration {
    pub accel: AccelCalibration,
    pub gyro: GyroCalibration,
}

impl ImuCalibration {
    pub const fn identity() -> Self {
        Self {
            accel: AccelCalibration::identity(),
            gyro: GyroCalibration::identity(),
        }
    }

    pub fn apply(&self, sample: &ImuSample) -> ImuSample {
        let mut accel_g = sample.accel_g;
        let mut gyro_dps = sample.gyro_dps;
        for i in 0..3 {
            let raw = sample.accel_g[i];
            let offset = self.accel.offset_g[i];
            let scale = self.accel.scale[i];
            let corrected = (raw - offset) / scale;
            if raw.is_finite()
                && offset.is_finite()
                && scale.is_finite()
                && scale != 0.0
                && corrected.is_finite()
            {
                accel_g[i] = corrected;
            }
            let raw_g = sample.gyro_dps[i];
            let bias = self.gyro.bias_dps[i];
            let corrected_g = raw_g - bias;
            if raw_g.is_finite() && bias.is_finite() && corrected_g.is_finite() {
                gyro_dps[i] = corrected_g;
            }
        }
        ImuSample {
            accel_g,
            gyro_dps,
            timestamp_s: sample.timestamp_s,
            sequence: sample.sequence,
        }
    }
}

impl Default for ImuCalibration {
    fn default() -> Self {
        Self::identity()
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructors_and_si_conversion() {
        let s = ImuSample::from_g_dps([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]);
        assert_eq!(s.accel_g, [1.0, 2.0, 3.0]);
        let si = ImuSample::from_si([9.80665, 0.0, 0.0], [core::f64::consts::PI, 0.0, 0.0]);
        assert_eq!(si.accel_g[0], 1.0);
        assert!((si.gyro_dps[0] - 180.0).abs() < f64::EPSILON);
    }

    #[test]
    fn calibration_identity_and_corrections() {
        let mut s = ImuSample::new([2.0, 4.0, 6.0], [10.0, 20.0, 30.0]);
        s.timestamp_s = Some(1.25);
        s.sequence = Some(7);
        assert_eq!(ImuCalibration::identity().apply(&s).accel_g, s.accel_g);
        let cal = ImuCalibration {
            accel: AccelCalibration {
                offset_g: [1.0, 2.0, 3.0],
                scale: [1.0, 2.0, 3.0],
            },
            gyro: GyroCalibration {
                bias_dps: [1.0, 2.0, 3.0],
            },
        };
        let out = cal.apply(&s);
        assert_eq!(out.accel_g, [1.0, 1.0, 1.0]);
        assert_eq!(out.gyro_dps, [9.0, 18.0, 27.0]);
        assert_eq!(out.timestamp_s, Some(1.25));
        assert_eq!(out.sequence, Some(7));
    }

    #[test]
    fn invalid_scale_falls_back_without_nan() {
        let s = ImuSample::new([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]);
        let cal = ImuCalibration {
            accel: AccelCalibration {
                offset_g: [10.0, f64::NAN, 1.0],
                scale: [0.0, 1.0, f64::INFINITY],
            },
            gyro: GyroCalibration {
                bias_dps: [f64::NAN, 1.0, f64::INFINITY],
            },
        };
        let out = cal.apply(&s);
        assert_eq!(out.accel_g, s.accel_g);
        assert_eq!(out.gyro_dps[0], 1.0);
        assert_eq!(out.gyro_dps[1], 1.0);
        assert_eq!(out.gyro_dps[2], 3.0);
        assert!(
            out.accel_g
                .iter()
                .chain(out.gyro_dps.iter())
                .all(|v| v.is_finite())
        );
    }
}
