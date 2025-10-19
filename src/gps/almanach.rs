use crate::{
    gps::{GPS_WORD_BITS, GPS_WORD_BYTES},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// [GpsQzssAlmanach] frame found in some reconfigured Frame-4 pages (when reconfigured),
/// or Frame-5 page 1 to 24.
#[derive(Debug, Copy, Clone, Default)]
pub struct GpsQzssAlmanach {
    /// Eccentricity (16 bit)
    pub eccentricity: f32,

    /// Time of issue of Almanach (in seconds)
    pub toa_seconds: u8,

    /// Mean motion difference from computed value (in semi-circles)
    pub di: f32,

    /// Omega_dot (in semi circles.s⁻¹)
    pub omega_dot: f32,

    /// SV health (8-bit)
    pub sv_health: u8,

    /// Square root of semi-major axis, in square root of meters.
    pub sqrt_a: f32,

    /// Longitude of ascending node of orbit plane at weekly epoch (in semi circles)
    pub omega0: f32,

    /// Omega (in semi circles)
    pub omega: f32,

    /// Mean anomaly at reference time (in semi-circles)
    pub m0: f32,

    /// af1 (in seconds per second)
    pub af1: f64,

    /// 22-bit af0 (in seconds)
    pub af0: f64,
}

impl PartialEq for GpsQzssAlmanach {
    fn eq(&self, rhs: &Self) -> bool {
        if self.toa_seconds != rhs.toa_seconds {
            return false;
        }

        if self.sv_health != rhs.sv_health {
            return false;
        }

        if (self.di - rhs.di).abs() < 1e-9 {
            return false;
        }

        if (self.omega - rhs.omega).abs() < 1e-9 {
            return false;
        }

        if (self.omega0 - rhs.omega0).abs() < 1e-9 {
            return false;
        }

        if (self.omega_dot - rhs.omega_dot).abs() < 1e-9 {
            return false;
        }

        if (self.sqrt_a - rhs.sqrt_a).abs() < 1e-9 {
            return false;
        }

        if (self.m0 - rhs.m0).abs() < 1e-9 {
            return false;
        }

        if (self.af0 - rhs.af0).abs() < 1e-9 {
            return false;
        }

        if (self.af1 - rhs.af1).abs() < 1e-9 {
            return false;
        }

        true
    }
}

impl Message<BigEndian> for GpsQzssAlmanach {
    fn encoding_size(&self) -> usize {
        8 * GPS_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssAlmanach {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssAlmanach {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        Ok(Self::default())
    }
}

impl GpsQzssAlmanach {
    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
    }

    pub fn with_eccentricity(mut self, e: f32) -> Self {
        self.eccentricity = e;
        self
    }
}

#[cfg(test)]
mod frame1 {
    use super::*;

    #[test]
    fn encoding() {
        for (eccentricity,) in [(0.01,), (0.001,)] {
            let frame = GpsQzssAlmanach::default().with_eccentricity(eccentricity);

            let mut buffer = [0; 1024];

            frame.encode(&mut buffer).unwrap_or_else(|e| {
                panic!("failed to encode frame: {}", e);
            });

            let decoded = GpsQzssAlmanach::decode(&buffer).unwrap_or_else(|e| {
                panic!("failed to decode frame: {}", e);
            });

            assert_eq!(decoded.eccentricity, eccentricity);
        }
    }
}
