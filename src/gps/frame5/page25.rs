use crate::gps::{GpsQzssSatelliteHealth, GPS_WORD_BITS, GPS_WORD_BYTES};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame5Page25 {
    /// TOA
    pub toa: u8,

    /// WNa
    pub wna: u8,

    /// reserved (3 bits)
    pub reserved: u8,

    /// Spare (19 bits)
    pub spare: u32,

    /// [GpsQzssSatelliteHealth] array for satellites #1 through #24
    /// (both included).
    pub sat_healths: [GpsQzssSatelliteHealth; 24],
}

impl GpsQzssFrame5Page25 {
    pub fn with_toa_seconds(mut self, toa_sec: u8) -> Self {
        self.toa = toa_sec;
        self
    }

    pub fn with_weeks(mut self, wna: u8) -> Self {
        self.wna = wna;
        self
    }

    pub fn with_spare(mut self, spare: u32) -> Self {
        self.spare = spare & 0x7_ffff;
        self
    }

    pub fn with_satellite_health(
        mut self,
        satellite: usize,
        health: GpsQzssSatelliteHealth,
    ) -> Self {
        if satellite < 24 {
            self.sat_healths[satellite] = health;
        }
        self
    }

    pub fn with_satellites_health(mut self, healths: [GpsQzssSatelliteHealth; 24]) -> Self {
        self.sat_healths = healths;
        self
    }

    pub fn with_reserved_bits(mut self, reserved: u8) -> Self {
        self.reserved = reserved & 0x07;
        self
    }

    #[cfg(test)]
    pub fn model() -> Self {
        Self {
            toa: 10,
            wna: 100,
            reserved: 1,
            spare: 0x1_2345,
            sat_healths: Default::default(),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn reciprocal() {
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
