use crate::{
    gps::{GpsDataWord, GPS_WORDS_PER_FRAME},
    twos_complement,
};

const WORD3_DID_MASK: u32 = 0x3000_0000;
const WORD3_DID_SHIFT: usize = 24;
const WORD3_SVID_MASK: u32 = 0x0fc0_0000;
const WORD3_SVID_SHIFT: usize = 22;
const WORD3_ECC_MASK: u32 = 0x003f_ffc0;
const WORD3_ECC_SHIFT: usize = 6;

const WORD4_TOA_MASK: u32 = 0x3fc0_0000;
const WORD4_TOA_SHIFT: usize = 22;
const WORD4_DI_MASK: u32 = 0x003f_ffc0;
const WORD4_DI_SHIFT: usize = 6;

const WORD5_OMEGADOT_MASK: u32 = 0x3fff_c000;
const WORD5_OMEGADOT_SHIFT: usize = 14;
const WORD5_HEALTH_MASK: u32 = 0x0000_3fc0;
const WORD5_HEALTH_SHIFT: usize = 6;

const WORD6_SQRT_MASK: u32 = 0x3fff_ffc0;
const WORD6_SQRT_SHIFT: usize = 6;

const WORD7_OMEGA0_MASK: u32 = 0x3fff_ffc0;
const WORD7_OMEGA0_SHIFT: usize = 6;

const WORD8_OMEGA_MASK: u32 = 0x3fff_ffc0;
const WORD8_OMEGA_SHIFT: usize = 6;

const WORD9_M0_MASK: u32 = 0x3fff_ffc0;
const WORD9_M0_SHIFT: usize = 6;

const WORD10_AF0_MASK: u32 = 0x0ffe_0000;
const WORD10_AF0_SHIFT: usize = 17;
const WORD10_AF1_MASK: u32 = 0x0001_ffc0;
const WORD10_AF1_SHIFT: usize = 6;

/// [GpsQzssAlmanach] frames come from Frame-5 page 1 to 24
/// and Frame-4 pages when reconfigured (not orginally intended).
#[derive(Debug, Copy, Default, Clone)]
pub struct GpsQzssAlmanach {
    /// 6-bit SVID. "0" here means dummy satellite,
    /// when "0" is set, the data-id should be interprated as SVID.
    pub sv_id: u8,

    /// 2-bit Data ID
    pub data_id: u8,

    /// Eccentricity
    pub eccentricity: f64,

    /// Time of issue of Almanach (in seconds).
    /// The almanach reference time is nominally the multiple of 2^12
    /// seconds truncated from 3.5 days after the first valid
    /// transmission time for this almanach set.
    pub toa_seconds: u16,

    /// Mean motion difference from computed value (in semi-circles)
    pub di: f64,

    /// Omega_dot (in semi circles.s⁻¹)
    pub omega_dot: f64,

    /// SV health (8-bit)
    pub sv_health: u8,

    /// Square root of semi-major axis, in square root of meters.
    pub sqrt_a: f64,

    /// Longitude of ascending node of orbit plane at weekly epoch (in semi circles)
    pub omega0: f64,

    /// Omega (in semi circles)
    pub omega: f64,

    /// Mean anomaly at reference time (in semi-circles)
    pub m0: f64,

    /// 22-bit af0 (in seconds).
    /// The almanac time parameters consist of both af0 (constant term) and af1 (first order term)
    pub af0: f64,

    /// af1 (in seconds per second)
    pub af1: f64,
}

impl PartialEq for GpsQzssAlmanach {
    fn eq(&self, rhs: &Self) -> bool {
        if self.sv_id != rhs.sv_id {
            return false;
        }

        if self.data_id != rhs.data_id {
            return false;
        }

        if (self.eccentricity - rhs.eccentricity).abs() < 1E-3 {
            return false;
        }

        if self.toa_seconds != rhs.toa_seconds {
            return false;
        }

        if (self.di - rhs.di).abs() < 1E-9 {
            return false;
        }

        if (self.omega_dot - rhs.omega_dot).abs() < 1E-11 {
            return false;
        }

        if self.sv_health != rhs.sv_health {
            return false;
        }

        if (self.sqrt_a - rhs.sqrt_a).abs() < 1E-6 {
            return false;
        }

        if (self.omega0 - rhs.omega0).abs() < 1E-8 {
            return false;
        }

        if (self.omega - rhs.omega).abs() < 1E-8 {
            return false;
        }

        if (self.m0 - rhs.m0).abs() < 1E-11 {
            return false;
        }

        if (self.af0 - rhs.af0).abs() < 1E-9 {
            return false;
        }

        if (self.af1 - rhs.af1).abs() < 1E-12 {
            return false;
        }

        true
    }
}

impl GpsQzssAlmanach {
    /// Copies and returns updated [GpsQzssAlmanach] with
    /// desired 6-bit SV ID
    pub fn with_sv_id(mut self, id: u8) -> Self {
        self.sv_id = id & 0x03;
        self
    }

    /// Copies and returns updated [GpsQzssAlmanach] with
    /// desired 2-bit data ID
    pub fn with_data_id(mut self, id: u8) -> Self {
        self.data_id = id & 0x03;
        self
    }

    /// Copies and returns updated [GpsQzssAlmanach] with
    /// desired 6-bit SV health
    pub fn with_sv_health(mut self, health: u8) -> Self {
        self.sv_health = health & 0x3f;
        self
    }

    /// Copies and returns updated [GpsQzssAlmanach] with
    /// desired clock offset (in seconds).
    pub fn with_clock_offset_seconds_s(mut self, offset: f64) -> Self {
        self.af0 = offset;
        self
    }

    /// Copies and returns updated [GpsQzssAlmanach] with
    /// desired clock offset (in seconds per second).
    pub fn with_clock_drift_seconds_s(mut self, drift_sec_s: f64) -> Self {
        self.af1 = drift_sec_s;
        self
    }

    /// Copies and returns updated [GpsQzssAlmanach] with
    /// desired semi-major axis (in meters)
    pub fn with_semi_major_axis_meters(mut self, semi_major_m: f64) -> Self {
        self.sqrt_a = semi_major_m.sqrt();
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
        self.sv_id = word.sv_id;
        self.data_id = word.data_id;
        self.eccentricity = word.eccentricity as f64 * 2.0_f64.powi(-21);
    }

    /// Encodes a [Word3] from [GpsQzssAlmanach]
    fn word3(&self) -> Word3 {
        Word3 {
            sv_id: self.sv_id,
            data_id: self.data_id,
            eccentricity: (self.eccentricity * 2.0_f64.powi(21)).round() as i16,
        }
    }

    /// Updates scaled content from [Word4]
    fn set_word4(&mut self, word: Word4) {
        self.toa_seconds = word.toa as u16 * 4096;
        self.di = word.delta_i as f64 * 2.0_f64.powi(-19);
    }

    /// Encodes a [Word4] from [GpsQzssAlmanach]
    fn word4(&self) -> Word4 {
        Word4 {
            toa: (self.toa_seconds / 4096) as u8,
            delta_i: (self.di * 2.0_f64.powi(19)).round() as i16,
        }
    }

    /// Updates scaled content from [Word5]
    fn set_word5(&mut self, word: Word5) {
        self.omega_dot = word.omega_dot as f64 * 2.0_f64.powi(-38);
        self.sv_health = word.health;
    }

    /// Encodes a [Word5] from [GpsQzssAlmanach]
    fn word5(&self) -> Word5 {
        Word5 {
            health: self.sv_health,
            omega_dot: (self.omega_dot * 2.0_f64.powi(38)).round() as i16,
        }
    }

    /// Updates scaled content from [Word6]
    fn set_word6(&mut self, word: Word6) {
        self.sqrt_a = word.sqrt_a as f64 * 2.0_f64.powi(-11);
    }

    /// Encodes a [Word6] from [GpsQzssAlmanach]
    fn word6(&self) -> Word6 {
        Word6 {
            sqrt_a: {
                let value = (self.sqrt_a * 2.0_f64.powi(11)).round() as i32;
                (value & 0xff_ffff) as i32
            },
        }
    }

    /// Updates scaled content from [Word7]
    fn set_word7(&mut self, word: Word7) {
        self.omega0 = word.omega0 as f64 * 2.0_f64.powi(-23);
    }

    /// Encodes a [Word7] from [GpsQzssAlmanach]
    fn word7(&self) -> Word7 {
        Word7 {
            omega0: {
                let value = (self.omega0 * 2.0_f64.powi(23)).round() as u32;
                (value & 0xff_ffff) as i32
            },
        }
    }

    /// Updates scaled content from [Word8]
    fn set_word8(&mut self, word: Word8) {
        self.omega = word.omega as f64 * 2.0_f64.powi(-23);
    }

    /// Encodes a [Word8] from [GpsQzssAlmanach]
    fn word8(&self) -> Word8 {
        Word8 {
            omega: {
                let value = (self.omega * 2.0_f64.powi(23)).round() as u32;
                (value & 0xff_ffff) as i32
            },
        }
    }

    /// Updates scaled content from [Word9]
    fn set_word9(&mut self, word: Word9) {
        self.m0 = word.m0 as f64 * 2.0_f64.powi(-23);
    }

    /// Encodes a [Word9] from [GpsQzssAlmanach]
    fn word9(&self) -> Word9 {
        Word9 {
            m0: {
                let value = (self.m0 * 2.0_f64.powi(23)).round() as u32;
                (value & 0xff_ffff) as i32
            },
        }
    }

    /// Updates scaled content from [Word10]
    fn set_word10(&mut self, word: Word10) {
        self.af0 = word.af0 as f64 * 2.0_f64.powi(-20);
        self.af1 = word.af1 as f64 * 2.0_f64.powi(-38);
    }

    /// Encodes a [Word10] from [GpsQzssAlmanach]
    fn word10(&self) -> Word10 {
        Word10 {
            af0: {
                let value = (self.af0 * 2.0_f64.powi(20)).round() as u32;
                (value & 0x7ff) as i16
            },
            af1: {
                let value = (self.af0 * 2.0_f64.powi(38)).round() as u32;
                (value & 0x7ff) as i16
            },
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
struct Word3 {
    /// 2-bit DATID
    pub data_id: u8,

    /// 6-bit SVID
    pub sv_id: u8,

    /// 16 bit eccentricity
    pub eccentricity: i16,
}

impl Word3 {
    /// Interprets this [GpsDataWord] as [Word3].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let data_id = ((value & WORD3_DID_MASK) >> WORD3_DID_SHIFT) as u8;
        let sv_id = ((value & WORD3_SVID_MASK) >> WORD3_SVID_SHIFT) as u8;
        let eccentricity = ((value & WORD3_ECC_MASK) >> WORD3_ECC_SHIFT) as i16;
        Self {
            data_id,
            sv_id,
            eccentricity,
        }
    }

    /// Encodes this [Word3] as [GpsDataWord].
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value |= ((self.data_id & 0x03) as u32) << WORD3_DID_SHIFT;
        value |= ((self.sv_id & 0x3f) as u32) << WORD3_SVID_SHIFT;
        value |= (self.eccentricity as u32) << WORD3_ECC_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word4 {
    /// 8-bit TOA
    pub toa: u8,

    /// 16-bit delta_i
    pub delta_i: i16,
}

impl Word4 {
    /// Interprets this [GpsDataWord] as [Word4].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let toa = ((value & WORD4_TOA_MASK) >> WORD4_TOA_SHIFT) as u8;
        let delta_i = ((value & WORD4_DI_MASK) >> WORD4_DI_SHIFT) as i16;
        Self { toa, delta_i }
    }

    /// Encodes this [Word4] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value |= (self.toa as u32) << WORD4_TOA_SHIFT;
        value |= ((self.delta_i as u32) & 0xffff) << WORD4_DI_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word5 {
    /// 16-bit omgea dot
    pub omega_dot: i16,

    /// 8-bit health
    pub health: u8,
}

impl Word5 {
    /// Interprets this [GpsDataWord] as [Word5].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let omega_dot = ((value & WORD5_OMEGADOT_MASK) >> WORD5_OMEGADOT_SHIFT) as i16;
        let health = ((value & WORD5_HEALTH_MASK) >> WORD5_HEALTH_SHIFT) as u8;
        Self { omega_dot, health }
    }

    /// Encodes this [Word5] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value |= (self.omega_dot as u32) << WORD5_OMEGADOT_SHIFT;
        value |= (self.health as u32) << WORD5_HEALTH_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
struct Word6 {
    /// 24-bit sqrt(a)
    pub sqrt_a: i32,
}

impl Word6 {
    /// Interprets this [GpsDataWord] as [Word6].
    pub fn from_word(word: GpsDataWord) -> Self {
        let sqrt_a = (word.value() & WORD6_SQRT_MASK) >> WORD6_SQRT_SHIFT;
        let sqrt_a = twos_complement(sqrt_a, 0xffffff, 0x800000);
        Self { sqrt_a }
    }

    /// Encodes this [Word6] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word7 {
    /// 24-bit omega0
    pub omega0: i32,
}

impl Word7 {
    /// Interprets this [GpsDataWord] as [Word7].
    pub fn from_word(word: GpsDataWord) -> Self {
        let omega0 = (word.value() & WORD7_OMEGA0_MASK) >> WORD7_OMEGA0_SHIFT;
        let omega0 = twos_complement(omega0, 0xffffff, 0x800000);
        Self { omega0 }
    }

    /// Encodes this [Word7] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word8 {
    /// 24-bit omega
    pub omega: i32,
}

impl Word8 {
    /// Interprets this [GpsDataWord] as [Word8].
    pub fn from_word(word: GpsDataWord) -> Self {
        let omega = (word.value() & WORD8_OMEGA_MASK) >> WORD8_OMEGA_SHIFT;
        let omega = twos_complement(omega, 0xffffff, 0x800000);
        Self { omega }
    }

    /// Encodes this [Word8] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word9 {
    /// 24-bit M0
    pub m0: i32,
}

impl Word9 {
    /// Interprets this [GpsDataWord] as [Word9].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let m0 = (value & WORD9_M0_MASK) >> WORD9_M0_SHIFT;
        let m0 = twos_complement(m0, 0xffffff, 0x800000);
        Self { m0 }
    }

    /// Encodes this [Word9] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word10 {
    /// 11-bit AF0
    pub af0: i16,

    /// 11-bit AF1
    pub af1: i16,
}

impl Word10 {
    /// Interprets this [GpsDataWord] as [Word10].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();

        let mut af0 = (value & 0x0000_00c0) >> 6;

        let af1 = (value & 0x0007_ff00) >> 8;
        af0 |= (value & 0x07f8_0000) >> 17;

        let af0 = twos_complement(af0, 0x0000_07ff, 0x0000_0080) as i16;
        let af1 = twos_complement(af1, 0x0000_07ff, 0x0000_0080) as i16;

        Self { af0, af1 }
    }

    /// Encodes this [Word10] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = ((self.af0 & 0x7ff) as u32) << WORD10_AF0_SHIFT;
        value |= ((self.af1 & 0x7ff) as u32) << WORD10_AF1_SHIFT;
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
                data_id: 0,
                sv_id: 0,
                eccentricity: 0,
            },
            Word3 {
                data_id: 1,
                sv_id: 2,
                eccentricity: 3,
            },
            Word3 {
                data_id: 2,
                sv_id: 28,
                eccentricity: -3,
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
            Word4 { toa: 0, delta_i: 0 },
            Word4 {
                toa: 10,
                delta_i: 20,
            },
            Word4 {
                toa: 100,
                delta_i: -20,
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
        for dword5 in [
            Word5 {
                omega_dot: 0,
                health: 0,
            },
            Word5 {
                omega_dot: 10,
                health: 10,
            },
        ] {
            let gps_word = dword5.to_word();
            let decoded = Word5::from_word(gps_word);
            assert_eq!(decoded, dword5);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword6() {
        for dword6 in [
            Word6 { sqrt_a: 0 },
            Word6 { sqrt_a: 10 },
            Word6 { sqrt_a: -10 },
        ] {
            let gps_word = dword6.to_word();
            let decoded = Word6::from_word(gps_word);
            assert_eq!(decoded, dword6);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword7() {
        for dword7 in [Word7 { omega0: 10 }, Word7 { omega0: -10 }] {
            let gps_word = dword7.to_word();
            let decoded = Word7::from_word(gps_word);
            assert_eq!(decoded, dword7);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword8() {
        for dword8 in [
            Word8 { omega: 0 },
            Word8 { omega: 10 },
            Word8 { omega: -10 },
        ] {
            let gps_word = dword8.to_word();
            let decoded = Word8::from_word(gps_word);
            assert_eq!(decoded, dword8);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword9() {
        for dword9 in [Word9 { m0: 0 }, Word9 { m0: 12 }, Word9 { m0: -10 }] {
            let gps_word = dword9.to_word();
            let decoded = Word9::from_word(gps_word);
            assert_eq!(decoded, dword9);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn dword10() {
        for dword10 in [
            Word10 { af0: 0, af1: 0 },
            Word10 { af0: 100, af1: 200 },
            Word10 {
                af0: -1230,
                af1: -200,
            },
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
            sv_id,
            data_id,
            eccentricity,
            toa_seconds,
            di,
            omega_dot,
            omega,
            sv_health,
            m0,
            af0,
            af1,
            sqrt_a,
            omega0,
        ) in [
            (
                1, 2, 0.01, 10, 0.1E-6, 0.1E-6, 1E-9, 0x0, 0.03, 1e-3, 1e-9, 5151.0, 1E-4,
            ),
            (
                2, 4, 0.02, 20, 0.2E-6, 0.2E-6, 2e-9, 0x1, 0.04, 2e-3, 1e-10, 5153.1, 1E-5,
            ),
            (
                1, 2, 0.01, 10, 0.3E-6, 0.1E-6, 1E-9, 0xfe, 0.03, 1e-3, 1e-9, 5152.4, 2e-5,
            ),
            (
                1, 2, 0.01, 10, 0.4E-6, 0.1E-6, 1E-9, 0xff, 0.03, 1e-3, 1e-9, 5153.6, 3e-5,
            ),
        ] {
            let frame1 = GpsQzssAlmanach {
                sv_id,
                data_id,
                eccentricity,
                toa_seconds,
                di,
                omega_dot,
                omega,
                sv_health,
                sqrt_a,
                m0,
                af0,
                af1,
                omega0,
            };

            let words = frame1.to_words();

            let decoded = GpsQzssAlmanach::from_words(&words);

            assert_eq!(decoded.sv_id, frame1.sv_id);
            assert_eq!(decoded.data_id, frame1.data_id);
            assert_eq!(decoded.toa_seconds, frame1.toa_seconds);

            assert!((decoded.m0 - frame1.m0).abs() < 1E-7);
            assert!((decoded.omega_dot - frame1.omega_dot).abs() < 1E-11);
            assert!((decoded.omega - frame1.omega).abs() < 1E-6);
            assert!((decoded.eccentricity - frame1.eccentricity).abs() < 1E-6);
            assert!((decoded.af0 - frame1.af0).abs() < 1E-10);
            assert!((decoded.af1 - frame1.af1).abs() < 1E-14);
        }
    }
}
