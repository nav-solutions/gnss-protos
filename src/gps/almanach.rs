use crate::{
    gps::{GpsDataWord, GPS_WORDS_PER_FRAME},
    twos_complement,
};

/// [GpsQzssAlmanach] frame found in some reconfigured Frame-4 pages (when reconfigured),
/// or Frame-5 page 1 to 24.
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

        if rhs.iodc != self.iodc {
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

impl GpsQzssAlmanach {

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
        self.iodc = iodc & 0x3ff;
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

    /// Returns true if [GpsQzssFrame1] indicates all-signals are OK.
    pub fn healthy(&self) -> bool {
        self.health == 0
    }

    /// Returns true if [GpsQzssFrame1] indicates this satellite is temporarily
    /// out of service
    pub fn unavailable(&self) -> bool {
        self.health == 0x1C
    }

    /// Returns true if [GpsQzssFrame1] indicates this satellite has a pending
    /// maintenance operation (should be used with caution)
    pub fn pending_maintenance(&self) -> bool {
        self.health == 0x1D
    }

    /// Returns true if [GpsQzssFrame1] indicates this satellite is experiencing
    /// code modulation or tranmission issues.
    pub fn transmission_issues(&self) -> bool {
        !self.healthy()
            && !self.pending_maintenance()
            && !self.unavailable()
            && self.health != 0x1E
            && self.health != 0x1F
    }

    /// Copies and returns [GpsQzssFrame1] with all-signals marked as OK.
    pub fn with_all_signals_ok(mut self) -> Self {
        self.health = 0;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with special status code marking
    /// temporary unavailability (under maintenance operation).
    pub fn with_unavailable_access(mut self) -> Self {
        self.health = 0x1C;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with special status code marking
    /// future (scheduled) unavailability (pending maintenance operation).
    pub fn with_pending_maintenance(mut self) -> Self {
        self.health = 0x1D;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with special status code marking
    /// a transmission issue.
    pub fn with_transmission_issue(mut self) -> Self {
        self.health = 0x0C;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated 6-bit health mask.
    /// The MSB can be used to mask the non-healthiness.
    /// The 5-LSB are health mask for each signal components.
    pub fn with_health_mask(mut self, health: u8) -> Self {
        self.health = health & 0x3f;
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

    /// Decodes [Self] from 8 [GpsDataWord]s.
    /// This method does not care for frames parity.
    pub(crate) fn from_words(words: &[GpsDataWord]) -> Self {
        let mut s = Self::default();

        for i in 0..GPS_WORDS_PER_FRAME - 2 {
            match i {
                0 => s.set_word3(Word3::from_word(words[i])),
                1 => s.set_word4(Word4::from_word(words[i])),
                2 => s.set_word5(Word5::from_word(words[i])),
                3 => s.set_word6(Word6::from_word(words[i])),
                4 => s.set_word7(Word7::from_word(words[i])),
                5 => s.set_word8(Word8::from_word(words[i])),
                6 => s.set_word9(Word9::from_word(words[i])),
                7 => s.set_word10(Word10::from_word(words[i])),
                _ => unreachable!("expecting 8 data words"),
            }
        }

        s
    }

    /// Updates scaled content from [Word3]
    fn set_word3(&mut self, word: Word3) {
        self.week = word.week;
        self.ura = word.ura;
        self.ca_or_p_l2 = word.ca_or_p_l2;
        self.health = word.health;
        self.iodc = (word.iodc_msb as u16) << 8;
    }

    /// Encodes a [Word3] from [GpsQzssFrame1]
    fn word3(&self) -> Word3 {
        Word3 {
            week: self.week,
            ura: self.ura,
            health: self.health,
            ca_or_p_l2: self.ca_or_p_l2,
            iodc_msb: ((self.iodc & 0x300) >> 8) as u8,
        }
    }

    /// Updates scaled content from [Word4]
    fn set_word4(&mut self, word: Word4) {
        self.l2_p_data_flag = word.l2_p_data_flag;
        self.reserved_word4 = word.reserved;
    }

    /// Encodes a [Word4] from [GpsQzssFrame1]
    fn word4(&self) -> Word4 {
        Word4 {
            reserved: self.reserved_word4,
            l2_p_data_flag: self.l2_p_data_flag,
        }
    }

    /// Updates scaled content from [Word5]
    fn set_word5(&mut self, word: Word5) {
        self.reserved_word5 = word.reserved;
    }

    /// Encodes a [Word5] from [GpsQzssFrame1]
    fn word5(&self) -> Word5 {
        Word5 {
            reserved: self.reserved_word5,
        }
    }

    /// Updates scaled content from [Word6]
    fn set_word6(&mut self, word: Word6) {
        self.reserved_word6 = word.reserved;
    }

    /// Encodes a [Word6] from [GpsQzssFrame1]
    fn word6(&self) -> Word6 {
        Word6 {
            reserved: self.reserved_word6,
        }
    }

    /// Updates scaled content from [Word7]
    fn set_word7(&mut self, word: Word7) {
        self.reserved_word7 = word.reserved;
        self.tgd = (word.tgd as f64) / 2.0_f64.powi(31);
    }

    /// Encodes a [Word7] from [GpsQzssFrame1]
    fn word7(&self) -> Word7 {
        Word7 {
            reserved: self.reserved_word7,
            tgd: (self.tgd * 2.0_f64.powi(31)).round() as i8,
        }
    }

    /// Updates scaled content from [Word8]
    fn set_word8(&mut self, word: Word8) {
        self.toc = (word.toc as u32) * 16;
        self.iodc |= word.iodc_lsb as u16;
    }

    /// Encodes a [Word8] from [GpsQzssFrame1]
    fn word8(&self) -> Word8 {
        Word8 {
            toc: (self.toc / 16) as u16,
            iodc_lsb: (self.iodc & 0xff) as u8,
        }
    }

    /// Updates scaled content from [Word9]
    fn set_word9(&mut self, word: Word9) {
        self.af2 = (word.af2 as f64) * 2.0_f64.powi(-55);
        self.af1 = (word.af1 as f64) * 2.0_f64.powi(-43);
    }

    /// Encodes a [Word9] from [GpsQzssFrame1]
    fn word9(&self) -> Word9 {
        Word9 {
            af2: (self.af2 * 2.0_f64.powi(55)).round() as i8,
            af1: (self.af1 * 2.0_f64.powi(43)).round() as i16,
        }
    }

    /// Updates scaled content from [Word10]
    fn set_word10(&mut self, word: Word10) {
        self.af0 = (word.af0 as f64) * 2.0_f64.powi(-31);
    }

    /// Encodes a [Word10] from [GpsQzssFrame1]
    fn word10(&self) -> Word10 {
        Word10 {
            af0: (self.af0 * 2.0_f64.powi(31)).round() as i32,
        }
    }

    /// Encodes this [GpsQzssFrame1] as a burst of 8 [GpsDataWord]s.
    pub(crate) fn to_words(&self) -> [GpsDataWord; GPS_WORDS_PER_FRAME - 2] {
        [
            self.word3().to_word(),
            self.word4().to_word(),
            self.word5().to_word(),
            self.word6().to_word(),
            self.word7().to_word(),
            self.word8().to_word(),
            self.word9().to_word(),
            self.word10().to_word(),
        ]
    }
}

#[derive(Debug, Copy, Default, Clone, PartialEq)]
pub(crate) struct Word3 {
    /// 10-bit week counter
    pub week: u16,

    /// 2 bits C/A or P ON L2
    pub ca_or_p_l2: u8,

    /// 4-bit URA index
    pub ura: u8,

    /// 6-bit SV Health
    pub health: u8,

    /// 2-bit (MSB) IODC, you will have to associate this to Word # 8
    pub iodc_msb: u8,
}

impl Word3 {
    /// Interprets this [GpsDataWord] as [Word3].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();

        let week = ((value & WORD3_WEEK_MASK) >> WORD3_WEEK_SHIFT) as u16;
        let ca_or_p_l2 = ((value & WORD3_CA_P_L2_MASK) >> WORD3_CA_P_L2_SHIFT) as u8;
        let ura = ((value & WORD3_URA_MASK) >> WORD3_URA_SHIFT) as u8;
        let health = ((value & WORD3_HEALTH_MASK) >> WORD3_HEALTH_SHIFT) as u8;
        let iodc_msb = ((value & WORD3_IODC_MASK) >> WORD3_IODC_SHIFT) as u8;

        Self {
            week,
            ca_or_p_l2,
            ura,
            health,
            iodc_msb,
        }
    }

    /// Encodes this [Word3] as [GpsDataWord].
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;

        value |= ((self.week & 0x3ff) as u32) << WORD3_WEEK_SHIFT;
        value |= ((self.ca_or_p_l2 & 0x3) as u32) << WORD3_CA_P_L2_SHIFT;
        value |= ((self.ura & 0x07) as u32) << WORD3_URA_SHIFT;
        value |= ((self.health & 0x3f) as u32) << WORD3_HEALTH_SHIFT;
        value |= ((self.iodc_msb & 0x03) as u32) << WORD3_IODC_SHIFT;

        value <<= 2;

        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word4 {
    pub l2_p_data_flag: bool,
    pub reserved: u32,
}

impl Word4 {
    /// Interprets this [GpsDataWord] as [Word4].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let l2_p_data_flag = (value & WORD4_L2P_DATA_MASK) > 0;
        let reserved = (value & WORD4_RESERVED_MASK) >> WORD4_RESERVED_SHIFT;

        Self {
            l2_p_data_flag,
            reserved,
        }
    }

    /// Encodes this [Word4] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;

        if self.l2_p_data_flag {
            value |= WORD4_L2P_DATA_MASK;
        }

        value |= (self.reserved & 0x7fffff) << WORD4_RESERVED_SHIFT;
        value <<= 2;

        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word5 {
    /// 24-bit reserved
    pub reserved: u32,
}

impl Word5 {
    /// Interprets this [GpsDataWord] as [Word5].
    pub fn from_word(word: GpsDataWord) -> Self {
        let reserved = (word.value() & WORD5_RESERVED_MASK) >> WORD5_RESERVED_SHIFT;
        Self { reserved }
    }

    /// Encodes this [Word5] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= (self.reserved & 0x0ffffff) << WORD5_RESERVED_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub(crate) struct Word6 {
    /// 24-bit reserved
    pub reserved: u32,
}

impl Word6 {
    /// Interprets this [GpsDataWord] as [Word6].
    pub fn from_word(word: GpsDataWord) -> Self {
        let reserved = (word.value() & WORD6_RESERVED_MASK) >> WORD6_RESERVED_SHIFT;
        Self { reserved }
    }

    /// Encodes this [Word6] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= (self.reserved & 0x0ffffff) << WORD6_RESERVED_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word7 {
    /// 16-bit reserved
    pub reserved: u16,

    /// TGD
    pub tgd: i8,
}

impl Word7 {
    /// Interprets this [GpsDataWord] as [Word7].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let reserved = ((value & WORD7_RESERVED_MASK) >> WORD7_RESERVED_SHIFT) as u16;
        let tgd = ((value & WORD7_TGD_MASK) >> WORD7_TGD_SHIFT) as i8;
        Self { reserved, tgd }
    }

    /// Encodes this [Word7] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= ((self.reserved as u32) & 0x0ffff) << WORD7_RESERVED_SHIFT;
        value |= ((self.tgd as u32) & 0xff) << WORD7_TGD_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word8 {
    /// 8-bit IODC LSB to associate with Word # 3
    pub iodc_lsb: u8,

    /// 16 bit ToC
    pub toc: u16,
}

impl Word8 {
    /// Interprets this [GpsDataWord] as [Word8].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let iodc_lsb = ((value & WORD8_IODC_MASK) >> WORD8_IODC_SHIFT) as u8;
        let toc = ((value & WORD8_TOC_MASK) >> WORD8_TOC_SHIFT) as u16;
        Self { iodc_lsb, toc }
    }

    /// Encodes this [Word8] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= ((self.iodc_lsb as u32) & 0xff) << WORD8_IODC_SHIFT;
        value |= ((self.toc as u32) & 0x0ffff) << WORD8_TOC_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word9 {
    /// 8 bit af2
    pub af2: i8,

    /// 16 bit af1
    pub af1: i16,
}

impl Word9 {
    /// Interprets this [GpsDataWord] as [Word9].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let af2 = ((value & WORD9_AF2_MASK) >> WORD9_AF2_SHIFT) as i8;
        let af1 = ((value & WORD9_AF1_MASK) >> WORD9_AF1_SHIFT) as i16;
        Self { af2, af1 }
    }

    /// Encodes this [Word9] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= ((self.af2 as u32) & 0x0ff) << WORD9_AF2_SHIFT;
        value |= ((self.af1 as u32) & 0x0ffff) << WORD9_AF1_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word10 {
    /// 22-bit af0
    pub af0: i32,
}

impl Word10 {
    /// Interprets this [GpsDataWord] as [Word10].
    pub fn from_word(word: GpsDataWord) -> Self {
        let af0 = (word.value() & WORD10_AF0_MASK) >> WORD10_AF0_SHIFT;
        let af0 = twos_complement(af0, 0x3fffff, 0x200000);
        Self { af0 }
    }

    /// Encodes this [Word10] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = ((self.af0 & 0x3fffff) as u32) << WORD10_AF0_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[cfg(test)]
mod frame1 {
    use super::*;

    #[test]
    fn dword3() {
        for dword3 in [
            Word3 {
                week: 1,
                ca_or_p_l2: 0,
                ura: 0,
                health: 0,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 1,
                ura: 0,
                health: 0,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 0,
                ura: 1,
                health: 0,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 0,
                ura: 0,
                health: 1,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 0,
                ura: 0,
                health: 0,
                iodc_msb: 1,
            },
            Word3 {
                week: 1,
                ca_or_p_l2: 2,
                ura: 5,
                health: 1,
                iodc_msb: 0,
            },
        ] {
            let gps_word = dword3.to_word();
            let decoded = Word3::from_word(gps_word);
            assert_eq!(decoded, dword3);
            assert_eq!(
                decoded.to_word(),
                gps_word,
                "Reciprocal failed for {:?}",
                dword3
            );
        }
    }

    #[test]
    fn dword4() {
        for dword4 in [
            Word4 {
                l2_p_data_flag: true,
                reserved: 0,
            },
            Word4 {
                l2_p_data_flag: false,
                reserved: 1,
            },
            Word4 {
                l2_p_data_flag: true,
                reserved: 123,
            },
        ] {
            let gps_word = dword4.to_word();
            let decoded = Word4::from_word(gps_word);
            assert_eq!(decoded, dword4);
            assert_eq!(
                decoded.to_word(),
                gps_word,
                "Reciprocal failed for {:?}",
                dword4
            );
        }
    }

    #[test]
    fn dword5() {
        for dword5 in [Word5 { reserved: 0 }, Word5 { reserved: 120 }] {
            let gps_word = dword5.to_word();
            let decoded = Word5::from_word(gps_word);
            assert_eq!(decoded, dword5);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword6() {
        for dword6 in [Word6 { reserved: 0 }, Word6 { reserved: 120 }] {
            let gps_word = dword6.to_word();
            let decoded = Word6::from_word(gps_word);
            assert_eq!(decoded, dword6);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword7() {
        for dword7 in [
            Word7 {
                reserved: 0,
                tgd: 1,
            },
            Word7 {
                reserved: 120,
                tgd: 0,
            },
            Word7 {
                reserved: 120,
                tgd: 23,
            },
            Word7 {
                reserved: 120,
                tgd: -1,
            },
            Word7 {
                reserved: 127,
                tgd: -10,
            },
            Word7 {
                reserved: 127,
                tgd: -100,
            },
            Word7 {
                reserved: 127,
                tgd: -128,
            },
            Word7 {
                reserved: 127,
                tgd: 127,
            },
        ] {
            let gps_word = dword7.to_word();
            let decoded = Word7::from_word(gps_word);
            assert_eq!(decoded, dword7);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword8() {
        for dword8 in [
            Word8 {
                iodc_lsb: 10,
                toc: 30,
            },
            Word8 {
                iodc_lsb: 30,
                toc: 10,
            },
        ] {
            let gps_word = dword8.to_word();
            let decoded = Word8::from_word(gps_word);
            assert_eq!(decoded, dword8);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword9() {
        for dword9 in [Word9 { af2: 10, af1: 9 }, Word9 { af2: 9, af1: 100 }] {
            let gps_word = dword9.to_word();
            let decoded = Word9::from_word(gps_word);
            assert_eq!(decoded, dword9);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword10() {
        for dword10 in [
            Word10 { af0: 0 },
            Word10 { af0: 100 },
            Word10 { af0: -1230 },
            Word10 { af0: -3140 },
        ] {
            let gps_word = dword10.to_word();
            let decoded = Word10::from_word(gps_word);
            assert_eq!(decoded, dword10);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

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
