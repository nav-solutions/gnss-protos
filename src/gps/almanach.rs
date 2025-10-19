use crate::{
    gps::{GPS_WORD_BITS, GPS_WORD_BYTES},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// [GpsQzssAlmanach] frame found in some reconfigured Frame-4 pages (when reconfigured),
/// or Frame-5 page 1 to 24.
#[derive(Copy, Clone, Default)]
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
}

#[cfg(test)]
mod frame1 {
    use super::*;

    #[test]
    fn encoding() {
        for (
            week,
            ca_or_p_l2,
            ura,
            health,
            iodc,
            toc,
            tgd,
            af0,
            af1,
            af2,
            l2_p_data_flag,
            reserved_word4,
            reserved_word5,
            reserved_word6,
            reserved_word7,
        ) in [
            (
                350, 1, 2, 3, 10, 50_000, 6.0E-9, 7.0E-9, 8.0E-13, 9.0E-15, true, 10, 11, 12, 13,
            ),
            (
                900, 1, 2, 3, 10, 24_992, -1.0E-9, -7.0E-9, 8.0E-13, 9.0E-15, true, 10, 11, 12, 13,
            ),
        ] {
            let frame1 = GpsQzssFrame1 {
                week,
                ca_or_p_l2,
                ura,
                health,
                iodc,
                toc,
                tgd: tgd,
                af0: af0,
                af1: af1,
                af2: af2,
                l2_p_data_flag,
                reserved_word4,
                reserved_word5,
                reserved_word6,
                reserved_word7,
            };

            let words = frame1.to_words();

            let decoded = GpsQzssFrame1::from_words(&words);

            assert_eq!(decoded.ura, frame1.ura);
            assert_eq!(decoded.week, frame1.week);
            assert_eq!(decoded.toc, frame1.toc);
            assert_eq!(decoded.ca_or_p_l2, frame1.ca_or_p_l2);
            assert_eq!(decoded.l2_p_data_flag, frame1.l2_p_data_flag);
            assert_eq!(decoded.reserved_word4, frame1.reserved_word4);
            assert_eq!(decoded.reserved_word5, frame1.reserved_word5);
            assert_eq!(decoded.reserved_word6, frame1.reserved_word6);
            assert_eq!(decoded.reserved_word7, frame1.reserved_word7);

            assert!((decoded.af0 - frame1.af0).abs() < 1E-10);
            assert!((decoded.af1 - frame1.af1).abs() < 1E-14);
            assert!((decoded.af2 - frame1.af2).abs() < 1E-14);
        }
    }

    #[test]
    fn user_range_accuracy() {
        for (value_m, encoded_ura) in [
            (0.1, 0),
            (1.0, 0),
            (2.4, 0),
            (2.5, 1),
            (2.6, 1),
            (3.4, 1),
            (3.5, 2),
            (95.0, 8),
            (96.0, 8),
            (96.1, 9),
            (3071.0, 13),
            (3072.0, 13),
            (3072.1, 14),
            (4000.1, 14),
        ] {
            let ura = GpsQzssFrame1::compute_ura(value_m);
            assert_eq!(ura, encoded_ura, "encoded incorrect URA from {}m", value_m);

            let mut frame1 = GpsQzssFrame1::default().with_user_range_accuracy_m(value_m);

            assert_eq!(frame1.ura, encoded_ura);

            let mut expected = GpsQzssFrame1::default();
            expected.ura = encoded_ura;

            // assert_eq!(
            //     frame1.with_nominal_user_range_accuracy_m(value_m).ura,
            //     encoded_ura,
            //     "failed for value={}m, encoded={}", value_m, encoded_ura,
            // );
        }
    }
}
