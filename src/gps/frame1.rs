use crate::{
    gps::{GpsQzssSatelliteHealth, GPS_WORD_BITS, GPS_WORD_BYTES},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// [GpsQzssFrame1] Ephemeris #1 frame interpretation.
#[derive(Debug, Default, Copy, Clone)]
pub struct GpsQzssFrame1 {
    /// Rolling week counter.
    pub week: u16,

    /// C/A or P ON L2.  
    /// When asserted, indicates the NAV data stream was commanded OFF on the L2 channel P-code.
    pub ca_or_p_l2: u8,

    /// User Range Accuracy (URA) indication:
    /// the lower the better, interpret as follow (error in meters)
    ///
    /// - 0:  0 < ura <= 2.4m
    /// - 1:  2.4 < ura <= 3.4m
    /// - 2:  3.4 < ura <= 4.85
    /// - 3:  4.85 < ura <= 6.85
    /// - 4:  6.85 < ura <= 9.65
    /// - 5:  9.65 < ura <= 13.65
    /// - 6:  13.65 < ura <= 24.00
    /// - 7:  24.00 < ura <= 48.00
    /// - 8:  48.00 < ura <= 96.00
    /// - 9:  96.00 < ura <= 192.00
    /// - 10: 192.00 < ura <=  384.00
    /// - 11: 384.00 < ura <=  768.00
    /// - 12: 768.00 < ura <= 1536.00
    /// - 13: 1536.00 < ura <= 3072.00
    /// - 14: 3072.00 < ura <= 6144.00
    /// - 15: 6144.00 < ura
    pub ura: u8,

    /// [GpsQzssSatelliteHealth] flags.
    pub health: GpsQzssSatelliteHealth,

    /// (MSB) IODC message identifier.
    iodc_msb: u8,

    /// L2/P flag
    pub l2_p_data_flag: bool,

    /// Word #4 reserved bits
    pub reserved_word4: u32,

    /// Word #5 reserved bits
    pub reserved_word5: u32,

    /// Word #6 reserved bits
    pub reserved_word6: u32,

    /// Word #7 reserved bits
    pub reserved_word7: u16,

    /// TGD in seconds
    pub tgd: f64,

    /// (LSB) IODC message identifier.
    iodc_lsb: u8,

    /// Time of clock (in seconds)
    pub toc: u32,

    /// af2, in seconds per squared second.
    pub af2: f64,

    /// af1, in seconds per second.
    pub af1: f64,

    /// af0, in seconds.
    pub af0: f64,
}

impl PartialEq for GpsQzssFrame1 {
    fn eq(&self, rhs: &Self) -> bool {
        if rhs.week != self.week {
            return false;
        }

        if rhs.ca_or_p_l2 != self.ca_or_p_l2 {
            return false;
        }

        if rhs.ura != self.ura {
            return false;
        }

        if rhs.health != self.health {
            return false;
        }

        if rhs.iodc() != self.iodc() {
            return false;
        }

        if rhs.toc != self.toc {
            return false;
        }

        if (rhs.tgd - self.tgd).abs() > 1e-9 {
            return false;
        }

        if (rhs.af2 - self.af2).abs() > 1E-13 {
            return false;
        }

        if (rhs.af1 - self.af1).abs() > 1E-12 {
            return false;
        }

        if (rhs.af0 - self.af0).abs() > 1E-9 {
            return false;
        }

        if rhs.reserved_word4 != self.reserved_word4 {
            return false;
        }

        if rhs.l2_p_data_flag != self.l2_p_data_flag {
            return false;
        }

        if rhs.reserved_word5 != self.reserved_word5 {
            return false;
        }

        if rhs.reserved_word6 != self.reserved_word6 {
            return false;
        }

        if rhs.reserved_word7 != self.reserved_word7 {
            return false;
        }

        true
    }
}

impl Message<BigEndian> for GpsQzssFrame1 {
    fn encoding_size(&self) -> usize {
        8 * GPS_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame1 {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.week, 10)?;
        stream.write_int(self.ca_or_p_l2, 2)?;
        stream.write_int(self.ura, 4)?;
        stream.write(&self.health)?;
        stream.write_int(self.iodc_msb, 2)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_bool(self.l2_p_data_flag)?;
        stream.write_int(self.reserved_word4, 23)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(self.reserved_word5, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(self.reserved_word6, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let tgd = (self.tgd * 2.0_f64.powi(31)).round() as i8;
        stream.write_int(self.reserved_word7, 16)?;
        stream.write_int(tgd, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let toc = (self.toc / 16) as u16;
        stream.write_int(self.iodc_lsb, 8)?;
        stream.write_int(toc, 16)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let af1 = (self.af1 * 2.0_f64.powi(43)).round() as i16;
        let af2 = (self.af2 * 2.0_f64.powi(55)).round() as i8;
        stream.write_int(af2, 8)?;
        stream.write_int(af1, 16)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let af0 = (self.af0 * 2.0_f64.powi(31)).round() as i32;
        stream.write_int(af0 & 0x3f_ffff, 22)?;
        stream.write_int(0, 2)?; // TODO (parity)
        stream.write_int(0, 6) // TODO (parity)
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame1 {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let week = stream.read_int::<u16>(10)?;
        let ca_or_p_l2 = stream.read_int::<u8>(2)?;
        let ura = stream.read_int::<u8>(4)?;
        let health = stream.read::<GpsQzssSatelliteHealth>()?;
        let iodc_msb = stream.read_int::<u8>(2)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let l2_p_data_flag = stream.read_bool()?;
        let reserved_word4 = stream.read_int::<u32>(23)?;
        let parity = stream.read_int::<u8>(6)?;

        let reserved_word5 = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?;

        let reserved_word6 = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?;

        let reserved_word7 = stream.read_int::<u16>(16)?;
        let tgd = stream.read_int::<i8>(8)?;
        let tgd = (tgd as f64) * 2.0_f64.powi(-31);
        let parity = stream.read_int::<u8>(6)?;

        let iodc_lsb = stream.read_int::<u8>(8)?;
        let toc = stream.read_int::<u32>(16)? * 16;
        let parity = stream.read_int::<u8>(6)?;

        let af2 = stream.read_int::<i8>(8)?;
        let af2 = (af2 as f64) * 2.0_f64.powi(-55);
        let af1 = stream.read_int::<i16>(16)?;
        let af1 = (af1 as f64) * 2.0_f64.powi(-43);
        let parity = stream.read_int::<u8>(6)?;

        let af0 = stream.read_int::<i32>(22)?;
        let af0 = (af0 as f64) * 2.0_f64.powi(-31);
        let bits = stream.read_int::<u8>(2)?;
        let parity = stream.read_int::<u8>(6)?;

        Ok(Self {
            week,
            ca_or_p_l2,
            ura,
            health,
            iodc_msb,
            iodc_lsb,
            l2_p_data_flag,
            reserved_word4,
            reserved_word5,
            reserved_word6,
            reserved_word7,
            tgd,
            toc,
            af2,
            af1,
            af0,
        })
    }
}

impl GpsQzssFrame1 {
    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
            .with_week(0x123)
            .with_iodc(0x1)
            .healthy()
            .with_l2p_flag()
            .with_clock_offset_nanoseconds(1.0)
            .with_clock_drift_seconds_s(1E-12)
            .with_clock_drift_rate_seconds_s2(1E-15)
            .with_reserved23_word(0x12_3456)
            .with_reserved24_word1(0x34_5678)
            .with_reserved24_word2(0x98_7654)
            .with_total_group_delay_nanos(5.0)
            .with_ca_or_p_l2_mask(0x3)
            .with_user_range_accuracy_m(4.0)
    }

    /// Computes binary URA from value in meters
    fn compute_ura(value_m: f64) -> u8 {
        if value_m <= 2.4 {
            0
        } else if value_m <= 3.4 {
            1
        } else if value_m <= 4.85 {
            2
        } else if value_m <= 6.85 {
            3
        } else if value_m <= 9.65 {
            4
        } else if value_m <= 13.65 {
            5
        } else if value_m <= 24.0 {
            6
        } else if value_m <= 48.0 {
            7
        } else if value_m <= 96.0 {
            8
        } else if value_m <= 192.0 {
            9
        } else if value_m <= 384.0 {
            10
        } else if value_m <= 768.0 {
            11
        } else if value_m <= 1536.0 {
            12
        } else if value_m <= 3072.0 {
            13
        } else if value_m <= 6144.0 {
            14
        } else {
            15
        }
    }

    /// Returns the IODC message identifier
    pub fn iodc(&self) -> u16 {
        let mut value = self.iodc_msb as u16;
        value <<= 8;
        value |= self.iodc_lsb as u16;
        value
    }

    /// Calculates nominal User Range Accuracy in meters
    pub fn nominal_user_range_accuracy(&self) -> f64 {
        // For each URA index, users may compute a nominal URA value (x)
        //  - ura < 6: 2**(1+N/2)
        //  - ura > 6: 2**(N-2)
        if self.ura <= 6 {
            2.0_f64.powi((1 + self.ura / 2) as i32)
        } else {
            2.0_f64.powi((self.ura / 2) as i32)
        }
    }

    /// Copies and returns [GpsQzssFrame1] with updated Week number
    pub fn with_week(mut self, week: u16) -> Self {
        self.week = week & 0x3ff;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated 10-bit IODC mask
    pub fn with_iodc(mut self, iodc: u16) -> Self {
        self.iodc_lsb = iodc as u8;
        self.iodc_msb = (iodc >> 8) as u8;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with asserted L2P data flag
    pub fn with_l2p_flag(mut self) -> Self {
        self.l2_p_data_flag = true;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with deasserted L2P data flag
    pub fn without_l2p_flag(mut self) -> Self {
        self.l2_p_data_flag = false;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated 23-bit reserved word
    pub fn with_reserved23_word(mut self, reserved: u32) -> Self {
        self.reserved_word4 = reserved & 0x7f_ffff;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated (first) 24-bit reserved word
    pub fn with_reserved24_word1(mut self, reserved: u32) -> Self {
        self.reserved_word5 = reserved & 0xff_ffff;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated (second) 24-bit reserved word
    pub fn with_reserved24_word2(mut self, reserved: u32) -> Self {
        self.reserved_word6 = reserved & 0xff_ffff;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated 16-bit reserved word
    pub fn with_reserved16_word(mut self, reserved: u16) -> Self {
        self.reserved_word7 = reserved;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated Healthy flags
    pub fn healthy(mut self) -> Self {
        self.health = GpsQzssSatelliteHealth::healthy();
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated [GpsQzssSatelliteHealth] flags.
    pub fn with_health(mut self, health: GpsQzssSatelliteHealth) -> Self {
        self.health = health;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated time of clock in seconds.
    /// Provided value must be a multiple of 16 to be perfectly encoded.
    pub fn with_time_of_clock_seconds(mut self, toc_s: u32) -> Self {
        self.toc = toc_s;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated Total Group Delay (TGD) in seconds
    pub fn with_total_group_delay_seconds(mut self, tgd_s: f64) -> Self {
        self.tgd = tgd_s;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated Total Group Delay (TGD) in nanoseconds
    pub fn with_total_group_delay_nanos(mut self, tgd_nanos: f64) -> Self {
        self.tgd = tgd_nanos * 1e-9;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated User Range Accuracy
    /// in meters.
    pub fn with_user_range_accuracy_m(mut self, ura_m: f64) -> Self {
        self.ura = Self::compute_ura(ura_m);
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated 2-bit C/A or P ON L2 mask.
    pub fn with_ca_or_p_l2_mask(mut self, mask: u8) -> Self {
        self.ca_or_p_l2 = mask & 0x3;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated User Range Accuracy
    /// from a nominal User Range Accuracy in meters.
    pub fn with_nominal_user_range_accuracy_m(mut self, ura_m: f64) -> Self {
        // For each URA index, users may compute a nominal URA value (x)
        //  - ura < 6: 2**(1+N/2)
        //  - ura > 6: 2**(N-2)
        let ura = if ura_m <= 24.0 {
            2.0 * (ura_m.log2() - 1.0)
        } else {
            2.0 + ura_m.log2()
        };

        self.ura = ura.round() as u8;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (0) term in seconds.
    pub fn with_clock_offset_seconds(mut self, a0_seconds: f64) -> Self {
        self.af0 = a0_seconds;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (0) term in nanoseconds.
    pub fn with_clock_offset_nanoseconds(mut self, a0_nanos: f64) -> Self {
        self.af0 = a0_nanos * 1E-9;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (1) term (in seconds per second).
    pub fn with_clock_drift_seconds_s(mut self, af1: f64) -> Self {
        self.af1 = af1;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (2) term (in seconds per squared second).
    pub fn with_clock_drift_rate_seconds_s2(mut self, af2: f64) -> Self {
        self.af2 = af2;
        self
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{GpsQzssFrame1, GpsQzssSatelliteHealth},
        Buffer, Message, StaticBuffer,
    };

    use bitbuffer::{BigEndian, BitReadStream};

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
            let iodc_lsb = (iodc & 0xff) as u8;
            let iodc_msb = (iodc >> 8) as u8;

            let frame = GpsQzssFrame1 {
                week,
                ca_or_p_l2,
                ura,
                health: GpsQzssSatelliteHealth::from(health),
                iodc_lsb,
                iodc_msb,
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
            };

            let mut buf = [0; 1024];

            frame.encode(&mut buf).unwrap_or_else(|e| {
                panic!("failed to encode frame: {}", e);
            });

            let decoded = GpsQzssFrame1::decode(&buf).unwrap_or_else(|e| {
                panic!("failed to decode GPS EPH-1: {}", e);
            });

            assert_eq!(decoded.ura, frame.ura);
            assert_eq!(decoded.week, frame.week);
            assert_eq!(decoded.toc, frame.toc);
            assert_eq!(decoded.ca_or_p_l2, frame.ca_or_p_l2);
            assert_eq!(decoded.l2_p_data_flag, frame.l2_p_data_flag);
            assert_eq!(decoded.reserved_word4, frame.reserved_word4);
            assert_eq!(decoded.reserved_word5, frame.reserved_word5);
            assert_eq!(decoded.reserved_word6, frame.reserved_word6);
            assert_eq!(decoded.reserved_word7, frame.reserved_word7);

            assert!(
                (decoded.af0 - frame.af0).abs() < 1E-10,
                "expecting {:.3E} got {:.3E}",
                frame.af0,
                decoded.af0
            );

            assert!(
                (decoded.af1 - frame.af1).abs() < 1E-14,
                "expecting {:.3E} got {:.3E}",
                frame.af1,
                decoded.af1
            );

            assert!(
                (decoded.af2 - frame.af2).abs() < 1E-14,
                "expecting {:.3E} got {:.3E}",
                frame.af2,
                decoded.af2
            );
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
