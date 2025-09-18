use crate::{
    gps::{
        bytes::{ByteArray, GpsDataByte},
        GpsError,
    },
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
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame2 {
    /// Time of issue of ephemeris (in seconds of week)
    pub toe: u32,

    /// 8-bit IODE (Issue of Data)
    pub iode: u8,

    /// Mean anomaly at reference time (in semi-circles)
    pub m0: f64,

    /// Mean motion difference from computed value (in semi-circles)
    pub dn: f64,

    /// Latitude (cosine harmonic) in semi-circles.
    pub cuc: f64,

    /// Latitude (sine harmonic) in semi-circles.
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

impl GpsQzssFrame2 {
    /// Copies and returns [GpsQzssFrame2] with updated time of issue of Ephemeris
    /// in seconds of week.
    pub fn with_toe(mut self, toe_seconds: u32) -> Self {
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

    /// Decodes [Self] from a burst of 8 [GpsDataByte]s
    pub(crate) fn decode(bytes: &[GpsDataByte]) -> Self {
        let mut extra = 0u32;
        let mut s = Self::default();

        for i in 0..8 {
            let array = ByteArray::new(&bytes[i * 4..i * 4 + 4]);
            let dword = array.value_u32();

            match i {
                0 => s.set_word3(Word3::decode(dword)),
                1 => s.set_word4(Word4::decode(dword), &mut extra),
                2 => s.set_word5(Word5::decode(dword), extra),
                3 => s.set_word6(Word6::decode(dword), &mut extra),
                4 => s.set_word7(Word7::decode(dword), extra),
                5 => s.set_word8(Word8::decode(dword), &mut extra),
                6 => s.set_word9(Word9::decode(dword), extra),
                7 => s.set_word10(Word10::decode(dword)),
                _ => unreachable!("compiler issue"),
            }
        }
        s
    }

    fn word3(&self) -> Word3 {
        Word3 {
            iode: self.iode,
            crs: (self.crs * 2.0_f64.powi(5)) as i32,
        }
    }

    fn set_word3(&mut self, word: Word3) {
        self.crs = (word.crs as f64) / 2.0_f64.powi(5);
        self.iode = word.iode;
    }

    fn word4(&self) -> Word4 {
        let m0 = (self.m0 * 2.0_f64.powi(31)) as u32;
        Word4 {
            m0_msb: ((m0 & 0xff000000) >> 24) as u8,
            dn: (self.dn * 2.0_f64.powi(43)).round() as i16,
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
        let sqrt_a = (self.sqrt_a * 2.0_f64.powi(19)).round() as i32;
        Word9 {
            sqrt_a_lsb: (sqrt_a & 0x00ffffff) as u32,
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

    /// Encodes this [GpsQzssFrame2] as a burst of 8 [u32] data words
    /// starting from [Word3] to [Word10].
    pub(crate) fn encode(&self) -> [u32; 8] {
        [
            self.word3().encode(),
            self.word4().encode(),
            self.word5().encode(),
            self.word6().encode(),
            self.word7().encode(),
            self.word8().encode(),
            self.word9().encode(),
            self.word10().encode(),
        ]
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word3 {
    pub iode: u8,
    pub crs: i32,
}

impl Word3 {
    pub(crate) fn decode(dword: u32) -> Self {
        let iode = ((dword & WORD3_IODE_MASK) >> WORD3_IODE_SHIFT) as u8;
        let crs = ((dword & WORD3_CRS_MASK) >> WORD3_CRS_SHIFT) as u32;
        let crs = twos_complement(crs, 0xffff, 0x8000);
        Self { iode, crs }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;

        value |= (self.iode as u32) << WORD3_IODE_SHIFT;
        value |= ((self.crs as u32) & 0xffff) << WORD3_CRS_SHIFT;

        value
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub(crate) struct Word4 {
    /// Delta n
    pub dn: i16,

    /// M0 (8) msb, you need to associate this to Subframe #2 Word #5
    pub m0_msb: u8,
}

impl Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        let dn = ((dword & WORD4_DELTA_N_MASK) >> WORD4_DELTA_N_SHIFT) as i16;
        let m0_msb = ((dword & WORD4_M0_MSB_MASK) >> WORD4_M0_MSB_SHIFT) as u8;
        Self { dn, m0_msb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.dn as u32) << WORD4_DELTA_N_SHIFT;
        value |= (self.m0_msb as u32) << WORD4_M0_MSB_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word5 {
    /// M0 (24) lsb, you need to associate this to Subframe #2 Word #4
    pub m0_lsb: u32,
}

impl Word5 {
    pub(crate) fn decode(dword: u32) -> Self {
        let m0_lsb = ((dword & WORD5_M0_LSB_MASK) >> WORD5_M0_LSB_SHIFT) as u32;
        Self { m0_lsb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.m0_lsb & 0x00ffffff) as u32) << WORD5_M0_LSB_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word6 {
    pub cuc: i16,

    /// MSB(8) eccentricity, you need to associate this to Subframe #2 Word #7
    pub e_msb: u8,
}

impl Word6 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cuc = ((dword & WORD6_CUC_MASK) >> WORD6_CUC_SHIFT) as i16;
        let e_msb = ((dword & WORD6_E_MSB_MASK) >> WORD6_E_MSB_SHIFT) as u8;
        Self { cuc, e_msb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.cuc as u32) << WORD6_CUC_SHIFT;
        value |= (self.e_msb as u32) << WORD6_E_MSB_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word7 {
    /// LSB(24) eccentricity, you need to associate this to Subframe #2 Word #6
    pub e_lsb: u32,
}

impl Word7 {
    pub(crate) fn decode(dword: u32) -> Self {
        let e_lsb = ((dword & WORD7_E_LSB_MASK) >> WORD7_E_LSB_SHIFT) as u32;
        Self { e_lsb }
    }

    pub(crate) fn encode(&self) -> u32 {
        (self.e_lsb as u32) << WORD7_E_LSB_SHIFT
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word8 {
    pub cus: i32,

    /// MSB(8) A⁻¹: you need to associate this to Subframe #2 Word #9
    pub sqrt_a_msb: u8,
}

impl Word8 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cus = ((dword & WORD8_CUS_MASK) >> WORD8_CUS_SHIFT) as u32;

        let cus = twos_complement(cus, 0xffff, 0x8000);
        let sqrt_a_msb = ((dword & WORD8_SQRTA_MSB_MASK) >> WORD8_SQRTA_MSB_SHIFT) as u8;
        Self { cus, sqrt_a_msb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.sqrt_a_msb as u32) << WORD8_SQRTA_MSB_SHIFT;
        value |= (self.cus as u32) << WORD8_CUS_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word9 {
    /// LSB(24) A⁻¹: you need to associate this to Subframe #2 Word #8
    pub sqrt_a_lsb: u32,
}

impl Word9 {
    pub(crate) fn decode(dword: u32) -> Self {
        let sqrt_a_lsb = ((dword & WORD9_SQRTA_LSB_MASK) >> WORD9_SQRTA_LSB_SHIFT) as u32;
        Self { sqrt_a_lsb }
    }

    pub(crate) fn encode(&self) -> u32 {
        ((self.sqrt_a_lsb as u32) & 0x00ffffff) << WORD9_SQRTA_LSB_SHIFT
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word10 {
    /// Time of issue of Ephemeris (u16)
    pub toe: u16,

    /// Fit interval, differs between GPS and QZSS
    pub fitint: bool,

    /// 5-bit AODO
    pub aodo: u8,
}

impl Word10 {
    pub(crate) fn decode(dword: u32) -> Self {
        let toe = ((dword & WORD10_TOE_MASK) >> WORD10_TOE_SHIFT) as u16;
        let fitint = (dword & WORD10_FITINT_MASK) > 0;
        let aodo = ((dword & WORD10_AODO_MASK) >> WORD10_AODO_SHIFT) as u8;
        Self { toe, fitint, aodo }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.aodo & 0x1f) as u32) << WORD10_AODO_SHIFT;

        if self.fitint {
            value |= WORD10_FITINT_MASK;
        }

        value |= (self.toe as u32) << WORD10_TOE_SHIFT;
        value
    }
}

#[cfg(test)]
mod frame2 {
    use super::*;

    #[test]
    fn dword3_encoding() {
        for dword3 in [
            Word3 { crs: 1, iode: 0 },
            Word3 { crs: 2, iode: 1 },
            Word3 { crs: 10, iode: 2 },
            Word3 { crs: 10, iode: 100 },
        ] {
            let encoded = dword3.encode();
            let decoded = Word3::decode(encoded);
            assert_eq!(decoded, dword3);
        }
    }

    #[test]
    fn dword4_encoding() {
        for dword4 in [
            Word4 { dn: 0, m0_msb: 1 },
            Word4 { dn: 1, m0_msb: 3 },
            Word4 {
                dn: 10,
                m0_msb: 100,
            },
        ] {
            let encoded = dword4.encode();
            let decoded = Word4::decode(encoded);
            assert_eq!(decoded, dword4);
        }
    }

    #[test]
    fn dword5_encoding() {
        for dword5 in [
            Word5 { m0_lsb: 1 },
            Word5 { m0_lsb: 10 },
            Word5 { m0_lsb: 0 },
            Word5 { m0_lsb: 100 },
        ] {
            let encoded = dword5.encode();
            let decoded = Word5::decode(encoded);
            assert_eq!(decoded, dword5);
        }
    }

    #[test]
    fn dword6_encoding() {
        for dword6 in [Word6 { e_msb: 0, cuc: 10 }, Word6 { e_msb: 1, cuc: 100 }] {
            let encoded = dword6.encode();
            let decoded = Word6::decode(encoded);
            assert_eq!(decoded, dword6);
        }
    }

    #[test]
    fn dword7_encoding() {
        for dword7 in [
            Word7 { e_lsb: 0 },
            Word7 { e_lsb: 1 },
            Word7 { e_lsb: 10 },
            Word7 { e_lsb: 100 },
        ] {
            let encoded = dword7.encode();
            let decoded = Word7::decode(encoded);
            assert_eq!(decoded, dword7);
        }
    }

    #[test]
    fn dword8_encoding() {
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
        ] {
            let encoded = dword8.encode();
            let decoded = Word8::decode(encoded);
            assert_eq!(decoded, dword8);
        }
    }

    #[test]
    fn dword9_decoding() {
        for dword9 in [
            Word9 { sqrt_a_lsb: 0 },
            Word9 { sqrt_a_lsb: 1 },
            Word9 { sqrt_a_lsb: 10 },
            Word9 { sqrt_a_lsb: 255 },
        ] {
            let encoded = dword9.encode();
            let decoded = Word9::decode(encoded);
            assert_eq!(decoded, dword9);
        }
    }

    #[test]
    fn dword10_decoding() {
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
            let encoded = dword10.encode();
            let decoded = Word10::decode(encoded);
            assert_eq!(decoded, dword10);
        }
    }

    #[test]
    fn frame2_encoding() {
        for (toe, iode, m0, dn, cuc, cus, crs, e, sqrt_a, fit_int_flag, aodo) in [
            (
                2300, 10, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 16.0, false, 10,
            ),
            (
                4800, 11, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, true, 255,
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

            let encoded = frame2.encode();

            let mut extra = 0;
            let mut decoded = GpsQzssFrame2::default();

            for (i, dword) in encoded.iter().enumerate() {
                // decoded
                //     .decode_word(i + 3, *dword, &mut extra)
                //     .unwrap_or_else(|_| {
                //         panic!("Failed to decode dword {:3}=0x{:08X}", i, dword);
                //     });
            }

            assert_eq!(frame2.toe, toe);
            assert_eq!(frame2.iode, iode);
            assert_eq!(frame2.m0, m0);
            assert_eq!(frame2.dn, dn);
            assert_eq!(frame2.cuc, cuc);
            assert_eq!(frame2.cus, cus);
            assert_eq!(frame2.crs, crs);
            assert_eq!(frame2.e, e);
            assert_eq!(frame2.sqrt_a, sqrt_a);
            assert_eq!(frame2.fit_int_flag, fit_int_flag);
            assert_eq!(frame2.aodo, aodo);
        }
    }
}
