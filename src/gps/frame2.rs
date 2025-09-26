use crate::{
    gps::{GpsDataByte, GpsDataWord, GpsError, GPS_WORDS_PER_FRAME},
    twos_complement,
};

use core::f64::consts::PI;

const WORD3_IODE_MASK: u32 = 0x3fc00000;
const WORD3_IODE_SHIFT: u32 = 22;
const WORD3_CRS_MASK: u32 = 0x003fffc0;
const WORD3_CRS_SHIFT: u32 = 6;

const WORD4_DELTA_N_MASK: u32 = 0x3fffc000;
const WORD4_DELTA_N_SHIFT: u32 = 14;
const WORD4_M0_MSB_MASK: u32 = 0x00003fc0;
const WORD4_M0_MSB_SHIFT: u32 = 6;

const WORD5_M0_LSB_MASK: u32 = 0x3fffffc0;
const WORD5_M0_LSB_SHIFT: u32 = 6;

const WORD6_CUC_MASK: u32 = 0x3fffc000;
const WORD6_CUC_SHIFT: u32 = 14;
const WORD6_E_MSB_MASK: u32 = 0x00003fc0;
const WORD6_E_MSB_SHIFT: u32 = 6;

const WORD7_E_LSB_MASK: u32 = 0x3fffffc0;
const WORD7_E_LSB_SHIFT: u32 = 6;

const WORD8_CUS_MASK: u32 = 0x3fffc000;
const WORD8_CUS_SHIFT: u32 = 14;
const WORD8_SQRTA_MSB_MASK: u32 = 0x00003fc0;
const WORD8_SQRTA_MSB_SHIFT: u32 = 6;

const WORD9_SQRTA_LSB_MASK: u32 = 0x3fffffc0;
const WORD9_SQRTA_LSB_SHIFT: u32 = 6;

const WORD10_TOE_MASK: u32 = 0x3fffc000;
const WORD10_TOE_SHIFT: u32 = 14;
const WORD10_FITINT_MASK: u32 = 0x00002000;
const WORD10_AODO_MASK: u32 = 0x00001f00;
const WORD10_AODO_SHIFT: u32 = 8;

/// [GpsQzssFrame2] Ephemeris #2 frame interpretation.
#[derive(Debug, Default, Copy, Clone)]
pub struct GpsQzssFrame2 {
    /// Time of issue of ephemeris (in seconds of week)
    /// at instant of transmission of the next MSB.
    /// Must be a multiple of 16 to correctly be encoded.
    pub toe: u32,

    /// 8-bit IODE (Issue of Data)
    pub iode: u8,

    /// Mean anomaly at reference time (in semi-circles)
    pub m0: f64,

    /// Mean motion difference from computed value (in semi-circles)
    pub dn: f64,

    /// Latitude (cosine harmonic) in radians.
    pub cuc: f64,

    /// Latitude (sine harmonic) in radians.
    pub cus: f64,

    /// Orbit radius (sine harmonic) in meters.
    pub crs: f64,

    /// Orbit eccentricity.
    pub e: f64,

    /// Square root of semi-major axis, in square root of meters.
    pub sqrt_a: f64,

    /// Fit interval flag
    pub fit_int_flag: bool,

    /// 5-bit AODO
    pub aodo: u8,
}

impl PartialEq for GpsQzssFrame2 {
    fn eq(&self, rhs: &Self) -> bool {
        if self.toe != rhs.toe {
            return false;
        }

        if self.iode != rhs.iode {
            return false;
        }

        if (self.m0 - rhs.m0).abs() > 1e-3 {
            return false;
        }

        if (self.dn - rhs.dn).abs() > 1e-8 {
            return false;
        }

        if (self.cuc - rhs.cuc).abs() > 1e-9 {
            return false;
        }

        if (self.cus - rhs.cus).abs() > 1e-9 {
            return false;
        }

        if (self.crs - rhs.crs).abs() > 1e-9 {
            return false;
        }

        if (self.e - rhs.e).abs() > 1e-10 {
            return false;
        }

        if (self.sqrt_a - rhs.sqrt_a).abs() > 1e-5 {
            return false;
        }

        if self.fit_int_flag != rhs.fit_int_flag {
            return false;
        }

        if self.aodo != rhs.aodo {
            return false;
        }

        true
    }
}

impl GpsQzssFrame2 {
    /// Copies and returns [GpsQzssFrame2] with updated time of issue of Ephemeris
    /// in seconds of week.
    pub fn with_toe_seconds(mut self, toe_seconds: u32) -> Self {
        self.toe = toe_seconds;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated IODE value.
    pub fn with_iode(mut self, iode: u8) -> Self {
        self.iode = iode;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean anomaly at reference time, expressed
    /// in semi-circles.
    pub fn with_mean_anomaly_semi_circles(mut self, m0_semi_circles: f64) -> Self {
        self.m0 = m0_semi_circles;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean anomaly at reference time, expressed
    /// in radians.
    pub fn with_mean_anomaly_radians(mut self, m0_rad: f64) -> Self {
        self.m0 = m0_rad * PI / 2.0f64.powi(31);
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean motion difference (in semi circles)
    pub fn with_mean_motion_difference_semi_circles(mut self, dn_semi_circles: f64) -> Self {
        self.dn = dn_semi_circles;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean motion difference (in radians)
    pub fn with_mean_motion_difference_radians(mut self, dn_rad: f64) -> Self {
        self.dn = dn_rad * PI / 2.0f64.powi(31);
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated semi-major axis (in meters)
    pub fn with_semi_major_axis_meters(mut self, semi_major_m: f64) -> Self {
        self.sqrt_a = semi_major_m.sqrt();
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated square root of semi-major axis (in square root meters)
    pub fn with_square_root_semi_major_axis(mut self, sqrt_semi_major_m: f64) -> Self {
        self.sqrt_a = sqrt_semi_major_m;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated orbit eccentricity.
    pub fn with_eccentricity(mut self, e: f64) -> Self {
        self.e = e;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated 5-bit AODO mask.
    pub fn with_aodo(mut self, aodo: u8) -> Self {
        self.aodo = aodo & 0x1f;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated radius (sine component) in meters.
    pub fn with_crs_meters(mut self, crs_m: f64) -> Self {
        self.crs = crs_m;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated latitude cosine harmnoic correction term.
    pub fn with_cuc_radians(mut self, cuc_rad: f64) -> Self {
        self.cuc = cuc_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated latitude sine harmnoic correction term.
    pub fn with_cus_radians(mut self, cus_rad: f64) -> Self {
        self.cus = cus_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with fit interval flag asserted.
    pub fn with_fit_interval_flag(mut self) -> Self {
        self.fit_int_flag = true;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with fit interval flag deasserted.
    pub fn without_fit_interval_flag(mut self) -> Self {
        self.fit_int_flag = false;
        self
    }

    /// Decodes [Self] from a burst of 8 [GpsDataWord]s
    pub(crate) fn from_words(words: &[GpsDataWord]) -> Self {
        let mut extra = 0;
        let mut s = Self::default();

        for i in 0..GPS_WORDS_PER_FRAME - 2 {
            match i {
                0 => s.set_word3(Word3::from_word(words[i])),
                1 => s.set_word4(Word4::from_word(words[i]), &mut extra),
                2 => s.set_word5(Word5::from_word(words[i]), extra),
                3 => s.set_word6(Word6::from_word(words[i]), &mut extra),
                4 => s.set_word7(Word7::from_word(words[i]), extra),
                5 => s.set_word8(Word8::from_word(words[i]), &mut extra),
                6 => s.set_word9(Word9::from_word(words[i]), extra),
                7 => s.set_word10(Word10::from_word(words[i])),
                _ => unreachable!("expecting 8 data words"),
            }
        }

        s
    }

    fn word3(&self) -> Word3 {
        Word3 {
            iode: self.iode,
            crs: (self.crs * 2.0_f64.powi(5)).round() as i32,
        }
    }

    fn set_word3(&mut self, word: Word3) {
        self.crs = (word.crs as f64) / 2.0_f64.powi(5);
        self.iode = word.iode;
    }

    fn word4(&self) -> Word4 {
        let m0 = (self.m0 * 2.0_f64.powi(31)).round() as u32;
        let dn = (self.dn * 2.0_f64.powi(43)).round() as i16;
        Word4 {
            dn,
            m0_msb: ((m0 & 0xff00_0000) >> 24) as u8,
        }
    }

    fn set_word4(&mut self, word: Word4, extra: &mut u32) {
        *extra = word.m0_msb as u32;
        self.dn = (word.dn as f64) / 2.0_f64.powi(43);
    }

    fn word5(&self) -> Word5 {
        let m0 = (self.m0 * 2.0_f64.powi(31)).round() as i32;

        Word5 {
            m0_lsb: (m0 & 0x00ffffff) as u32,
        }
    }

    fn set_word5(&mut self, word: Word5, m0_msb: u32) {
        let mut m0 = m0_msb << 24;
        m0 |= word.m0_lsb as u32;
        self.m0 = ((m0 as i32) as f64) / 2.0_f64.powi(31);
    }

    fn word6(&self) -> Word6 {
        let e = (self.e * 2.0_f64.powi(33)).round() as u32;

        Word6 {
            e_msb: ((e & 0xff000000) >> 24) as u8,
            cuc: (self.cuc * 2.0_f64.powi(29)).round() as i16,
        }
    }

    fn set_word6(&mut self, word: Word6, extra: &mut u32) {
        *extra = word.e_msb as u32;
        self.cuc = (word.cuc as f64) / 2.0_f64.powi(29);
    }

    fn word7(&self) -> Word7 {
        let e = (self.e * 2.0_f64.powi(33)).round() as i32;
        Word7 {
            e_lsb: (e & 0x00ffffff) as u32,
        }
    }

    fn set_word7(&mut self, word: Word7, e_msb: u32) {
        let mut e = e_msb << 24;
        e |= word.e_lsb;

        self.e = (e as f64) / 2.0_f64.powi(33);
    }

    fn word8(&self) -> Word8 {
        let sqrt_a = (self.sqrt_a * 2.0_f64.powi(19)).round() as u32;

        Word8 {
            sqrt_a_msb: ((sqrt_a & 0xff000000) >> 24) as u8,
            cus: (self.cus * 2.0_f64.powi(29)).round() as i32,
        }
    }

    fn set_word8(&mut self, word: Word8, extra: &mut u32) {
        *extra = word.sqrt_a_msb as u32;
        self.cus = (word.cus as f64) / 2.0_f64.powi(29);
    }

    fn word9(&self) -> Word9 {
        let sqrt_a = (self.sqrt_a * 2.0_f64.powi(19)).round() as u32;
        Word9 {
            sqrt_a_lsb: sqrt_a & 0x00ffffff,
        }
    }

    fn set_word9(&mut self, word: Word9, sqrt_a_msb: u32) {
        let mut sqrt_a = sqrt_a_msb << 24;
        sqrt_a |= word.sqrt_a_lsb;
        self.sqrt_a = (sqrt_a as f64) / 2.0_f64.powi(19);
    }

    fn word10(&self) -> Word10 {
        Word10 {
            aodo: self.aodo,
            fitint: self.fit_int_flag,
            toe: (self.toe / 16) as u16,
        }
    }

    fn set_word10(&mut self, word: Word10) {
        self.aodo = word.aodo;
        self.fit_int_flag = word.fitint;
        self.toe = (word.toe as u32) * 16;
    }

    /// Encodes this [GpsQzssFrame2] as a burst of 8 [GpsDataWord]s.
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

#[derive(Debug, Default, Clone, PartialEq)]
struct Word3 {
    /// IODE (8 LSB)
    pub iode: u8,

    /// Cr (sine)
    pub crs: i32,
}

impl Word3 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();

        let iode = ((value & WORD3_IODE_MASK) >> WORD3_IODE_SHIFT) as u8;
        let crs = ((value & WORD3_CRS_MASK) >> WORD3_CRS_SHIFT) as u32;
        let crs = twos_complement(crs, 0xffff, 0x8000);

        Self { iode, crs }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;

        value |= (self.iode as u32) << WORD3_IODE_SHIFT;
        value |= ((self.crs as u32) & 0xffff) << WORD3_CRS_SHIFT;
        value <<= 2;

        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
struct Word4 {
    /// Delta n
    pub dn: i16,

    /// M0 (8) msb, you need to associate this to Subframe #2 Word #5
    pub m0_msb: u8,
}

impl Word4 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let dn = ((value & WORD4_DELTA_N_MASK) >> WORD4_DELTA_N_SHIFT) as i16;
        let m0_msb = ((value & WORD4_M0_MSB_MASK) >> WORD4_M0_MSB_SHIFT) as u8;
        Self { dn, m0_msb }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= (self.dn as u32) << WORD4_DELTA_N_SHIFT;
        value |= (self.m0_msb as u32) << WORD4_M0_MSB_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word5 {
    /// M0 (24) lsb, you need to associate this to Subframe #2 Word #4
    pub m0_lsb: u32,
}

impl Word5 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let m0_lsb = ((value & WORD5_M0_LSB_MASK) >> WORD5_M0_LSB_SHIFT) as u32;
        Self { m0_lsb }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= ((self.m0_lsb & 0x00ffffff) as u32) << WORD5_M0_LSB_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word6 {
    pub cuc: i16,

    /// MSB(8) eccentricity, you need to associate this to Subframe #2 Word #7
    pub e_msb: u8,
}

impl Word6 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let cuc = ((value & WORD6_CUC_MASK) >> WORD6_CUC_SHIFT) as i16;
        let e_msb = ((value & WORD6_E_MSB_MASK) >> WORD6_E_MSB_SHIFT) as u8;
        Self { cuc, e_msb }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= (self.cuc as u32) << WORD6_CUC_SHIFT;
        value |= (self.e_msb as u32) << WORD6_E_MSB_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word7 {
    /// LSB(24) eccentricity, you need to associate this to Subframe #2 Word #6
    pub e_lsb: u32,
}

impl Word7 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let e_lsb = ((value & WORD7_E_LSB_MASK) >> WORD7_E_LSB_SHIFT) as u32;
        Self { e_lsb }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = (self.e_lsb as u32) << WORD7_E_LSB_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word8 {
    pub cus: i32,

    /// MSB(8) A⁻¹: you need to associate this to Subframe #2 Word #9
    pub sqrt_a_msb: u8,
}

impl Word8 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let cus = ((value & WORD8_CUS_MASK) >> WORD8_CUS_SHIFT) as u32;

        let cus = twos_complement(cus, 0xffff, 0x8000);
        let sqrt_a_msb = ((value & WORD8_SQRTA_MSB_MASK) >> WORD8_SQRTA_MSB_SHIFT) as u8;
        Self { cus, sqrt_a_msb }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= (self.sqrt_a_msb as u32) << WORD8_SQRTA_MSB_SHIFT;
        value |= (self.cus as u32) << WORD8_CUS_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word9 {
    /// LSB(24) A⁻¹: you need to associate this to Subframe #2 Word #8
    pub sqrt_a_lsb: u32,
}

impl Word9 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let sqrt_a_lsb = ((value & WORD9_SQRTA_LSB_MASK) >> WORD9_SQRTA_LSB_SHIFT) as u32;
        Self { sqrt_a_lsb }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = ((self.sqrt_a_lsb as u32) & 0x00ffffff) << WORD9_SQRTA_LSB_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
struct Word10 {
    /// Time of issue of Ephemeris (u16)
    pub toe: u16,

    /// Fit interval, differs between GPS and QZSS
    pub fitint: bool,

    /// 5-bit AODO
    pub aodo: u8,
}

impl Word10 {
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let toe = ((value & WORD10_TOE_MASK) >> WORD10_TOE_SHIFT) as u16;
        let fitint = (value & WORD10_FITINT_MASK) > 0;
        let aodo = ((value & WORD10_AODO_MASK) >> WORD10_AODO_SHIFT) as u8;
        Self { toe, fitint, aodo }
    }

    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= ((self.aodo & 0x1f) as u32) << WORD10_AODO_SHIFT;

        if self.fitint {
            value |= WORD10_FITINT_MASK;
        }

        value |= (self.toe as u32) << WORD10_TOE_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[cfg(test)]
mod frame2 {
    use super::*;

    #[test]
    fn dword3() {
        for dword3 in [
            Word3 { crs: 1, iode: 0 },
            Word3 { crs: 2, iode: 1 },
            Word3 { crs: 10, iode: 2 },
            Word3 { crs: 10, iode: 100 },
            Word3 {
                crs: -10,
                iode: 100,
            },
            Word3 {
                crs: -100,
                iode: 254,
            },
            Word3 {
                crs: -8000,
                iode: 255,
            },
            Word3 {
                crs: 8000,
                iode: 255,
            },
        ] {
            let encoded = dword3.to_word();
            let decoded = Word3::from_word(encoded);
            assert_eq!(decoded, dword3);
        }
    }

    #[test]
    fn dword4() {
        for dword4 in [
            Word4 { dn: 0, m0_msb: 1 },
            Word4 { dn: 1, m0_msb: 3 },
            Word4 { dn: -10, m0_msb: 3 },
            Word4 {
                dn: -100,
                m0_msb: 100,
            },
            Word4 {
                dn: 100,
                m0_msb: 100,
            },
            Word4 {
                dn: 0xf123u16 as i16,
                m0_msb: 100,
            },
            Word4 {
                dn: 0xffffu16 as i16,
                m0_msb: 100,
            },
        ] {
            let encoded = dword4.to_word();
            let decoded = Word4::from_word(encoded);
            assert_eq!(decoded, dword4);
        }
    }

    #[test]
    fn dword5() {
        for dword5 in [
            Word5 { m0_lsb: 1 },
            Word5 { m0_lsb: 10 },
            Word5 { m0_lsb: 0 },
            Word5 { m0_lsb: 100 },
            Word5 { m0_lsb: 10000 },
        ] {
            let encoded = dword5.to_word();
            let decoded = Word5::from_word(encoded);
            assert_eq!(decoded, dword5);
        }
    }

    #[test]
    fn dword6() {
        for dword6 in [
            Word6 { e_msb: 0, cuc: 10 },
            Word6 {
                e_msb: 1,
                cuc: -100,
            },
        ] {
            let encoded = dword6.to_word();
            let decoded = Word6::from_word(encoded);
            assert_eq!(decoded, dword6);
        }
    }

    #[test]
    fn dword7() {
        for dword7 in [
            Word7 { e_lsb: 0 },
            Word7 { e_lsb: 1 },
            Word7 { e_lsb: 10 },
            Word7 { e_lsb: 100 },
        ] {
            let encoded = dword7.to_word();
            let decoded = Word7::from_word(encoded);
            assert_eq!(decoded, dword7);
        }
    }

    #[test]
    fn dword8() {
        for dword8 in [
            Word8 {
                cus: 0,
                sqrt_a_msb: 10,
            },
            Word8 {
                cus: 10,
                sqrt_a_msb: 1,
            },
            Word8 {
                cus: 10,
                sqrt_a_msb: 0,
            },
            Word8 {
                cus: 0,
                sqrt_a_msb: 12,
            },
            Word8 {
                cus: -10,
                sqrt_a_msb: 250,
            },
            Word8 {
                cus: -10,
                sqrt_a_msb: 255,
            },
        ] {
            let encoded = dword8.to_word();
            let decoded = Word8::from_word(encoded);
            assert_eq!(decoded, dword8);
        }
    }

    #[test]
    fn dword9() {
        for dword9 in [
            Word9 { sqrt_a_lsb: 0 },
            Word9 { sqrt_a_lsb: 1 },
            Word9 { sqrt_a_lsb: 10 },
            Word9 { sqrt_a_lsb: 255 },
            Word9 {
                sqrt_a_lsb: 0x0f_1234,
            },
            Word9 {
                sqrt_a_lsb: 0x1f_1234,
            },
            Word9 {
                sqrt_a_lsb: 0x3f_1234,
            },
            Word9 {
                sqrt_a_lsb: 0xff_1234,
            },
        ] {
            let encoded = dword9.to_word();
            let decoded = Word9::from_word(encoded);
            assert_eq!(decoded, dword9);
        }
    }

    #[test]
    fn dword10() {
        for dword10 in [
            Word10 {
                toe: 0,
                fitint: false,
                aodo: 0,
            },
            Word10 {
                toe: 0,
                fitint: false,
                aodo: 1,
            },
            Word10 {
                toe: 10,
                fitint: true,
                aodo: 10,
            },
        ] {
            let encoded = dword10.to_word();
            let decoded = Word10::from_word(encoded);
            assert_eq!(decoded, dword10);
        }
    }

    #[test]
    fn encoding() {
        for (toe, iode, m0, dn, cuc, cus, crs, e, sqrt_a, fit_int_flag, aodo) in [
            (
                345_600, 10, 9.76E-1, 4.0e-9, 9.3e-7, 2.8E-6, -88.0, 0.01, 5153.639, false, 10,
            ),
            (
                2320, 10, 9.76E-1, 5.0e-9, 9.4e-7, 2.9e-6, -87.0, 0.010234, 5153.64, false, 10,
            ),
            (
                4800, 11, 9.78E-1, 6.0e-9, 9.8e-7, 3.0e-6, 87.0, 0.02, 5153.65, true, 0x1f,
            ),
            (
                4800, 11, 9.78E-1, 6.0e-9, 9.8e-7, 3.0e-6, 87.0, 0.02, 5153.65, true, 0x0f,
            ),
        ] {
            let frame2 = GpsQzssFrame2 {
                toe,
                iode,
                m0,
                dn,
                cuc,
                cus,
                crs,
                e,
                sqrt_a,
                fit_int_flag,
                aodo,
            };

            let encoded = frame2.to_words();
            let decoded = GpsQzssFrame2::from_words(&encoded);
            assert_eq!(decoded, frame2);
        }
    }
}
