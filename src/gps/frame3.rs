use crate::{gps::GpsError, twos_complement};

const WORD3_CIC_MASK: u32 = 0x3fffc000;
const WORD3_CIC_SHIFT: u32 = 14;
const WORD3_OMEGA0_MASK: u32 = 0x00003fc0;
const WORD3_OMEGA0_SHIFT: u32 = 6;

const WORD4_OMEGA0_MASK: u32 = 0x3fffffc0;
const WORD4_OMEGA0_SHIFT: u32 = 6;

const WORD5_CIS_MASK: u32 = 0x3fffc000;
const WORD5_CIS_SHIFT: u32 = 14;
const WORD5_I0_MASK: u32 = 0x00003fc0;
const WORD5_I0_SHIFT: u32 = 6;

const WORD6_I0_MASK: u32 = 0x3fffffc0;
const WORD6_I0_SHIFT: u32 = 6;

const WORD7_CRC_MASK: u32 = 0x3fffc000;
const WORD7_CRC_SHIFT: u32 = 14;
const WORD7_OMEGA_MASK: u32 = 0x00003fc0;
const WORD7_OMEGA_SHIFT: u32 = 6;

const WORD8_OMEGA_MASK: u32 = 0x3fffffc0;
const WORD8_OMEGA_SHIFT: u32 = 6;

const WORD9_OMEGADOT_MASK: u32 = 0x3fffffc0;
const WORD9_OMEGADOT_SHIFT: u32 = 6;

const WORD10_IODE_MASK: u32 = 0x3fc00000;
const WORD10_IODE_SHIFT: u32 = 22;
const WORD10_IDOT_MASK: u32 = 0x003fff00;
const WORD10_IDOT_SHIFT: u32 = 8;

/// [GpsQzssFrame3] Ephemeris #3 frame interpretation.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame3 {
    /// Inclination angle cosine harmonic correction term
    pub cic: f64,

    /// Inclination angle sine harmonic correction term
    pub cis: f64,

    /// Orbit radius cosine harmonic correction term
    pub crc: f64,

    /// Inclination angle at reference time  (in semi circles)
    pub i0: f64,

    /// IODE: Issue of Data (Ephemeris)
    pub iode: u8,

    /// Rate of inclination angle (in semi circles.s⁻¹)
    pub idot: f64,

    /// Longitude of ascending node of orbit plane at weekly epoch (in semi circles)
    pub omega0: f64,

    /// Omega (in semi circles)
    pub omega: f64,

    /// Omega_dot (in semi circles.s⁻¹)
    pub omega_dot: f64,
}

impl GpsQzssFrame3 {
    /// Copies and returns [GpsQzssFrame3] with updated Cic correction term
    pub fn with_cic_radians(mut self, cic_rad: f64) -> Self {
        self.cic = cic_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated Cis correction term
    pub fn with_cis_radians(mut self, cis_rad: f64) -> Self {
        self.cis = cis_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated inclination angle at reference time (in semi circles)
    pub fn with_i0_semi_circles(mut self, i0: f64) -> Self {
        self.i0 = i0;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated orbit radius cosine harmonic term (meters)
    pub fn with_crc_meters(mut self, crc: f64) -> Self {
        self.crc = crc;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated longitude of ascending node (in semi circles)
    pub fn with_omega0_semi_circles(mut self, omega0: f64) -> Self {
        self.omega0 = omega0;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega (in semi circles)
    pub fn with_omega_semi_circles(mut self, omega: f64) -> Self {
        self.omega = omega;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega velocity (in semi circles.s⁻¹)
    pub fn with_omega_dot_semi_circles_sec(mut self, omega_dot: f64) -> Self {
        self.omega_dot = omega_dot;
        self
    }

    pub(crate) fn decode_word(
        &mut self,
        ptr: usize,
        word: u32,
        extra: &mut u32,
    ) -> Result<(), GpsError> {
        match ptr {
            3 => {
                let word = Word3::decode(word);
                self.set_word3(word, extra);
            },
            4 => {
                let word = Word4::decode(word);
                self.set_word4(word, *extra);
            },
            5 => {
                let word = Word5::decode(word);
                self.set_word5(word, extra);
            },
            6 => {
                let word = Word6::decode(word);
                self.set_word6(word, *extra);
            },
            7 => {
                let word = Word7::decode(word);
                self.set_word7(word, extra);
            },
            8 => {
                let word = Word8::decode(word);
                self.set_word8(word, *extra);
            },
            9 => {
                let word = Word9::decode(word);
                self.set_word9(word);
            },
            10 => {
                let word = Word10::decode(word);
                self.set_word10(word);
            },
            _ => return Err(GpsError::InternalFSM),
        }
        Ok(())
    }

    fn set_word3(&mut self, word: Word3, extra: &mut u32) {
        *extra = word.omega0_msb as u32;
        self.cic = (word.cic as f64) / 2.0_f64.powi(29);
    }

    fn word3(&self) -> Word3 {
        let omega0 = (self.omega0 * 2.0_f64.powi(31)).round() as u32;
        Word3 {
            omega0_msb: ((omega0 & 0xff000000) >> 24) as u8,
            cic: (self.cic * 2.0_f64.powi(29)).round() as i32,
        }
    }

    fn set_word4(&mut self, word: Word4, omega0_msb: u32) {
        let mut omega0 = omega0_msb << 24;
        omega0 |= word.omega0_lsb;
        self.omega0 = ((omega0 as i32) as f64) / 2.0_f64.powi(31);
    }

    fn word4(&self) -> Word4 {
        let omega0 = (self.omega0 * 2.0_f64.powi(31)).round() as u32;
        Word4 {
            omega0_lsb: (omega0 & 0x00ffffff) as u32,
        }
    }

    fn set_word5(&mut self, word: Word5, extra: &mut u32) {
        *extra = word.i0_msb as u32;
        self.cis = (word.cis as f64) / 2.0_f64.powi(29);
    }

    fn word5(&self) -> Word5 {
        let i0 = (self.i0 * 2.0_f64.powi(31)).round() as u32;
        Word5 {
            i0_msb: ((i0 & 0xff000000) >> 24) as u8,
            cis: (self.cis * 2.0_f64.powi(29)) as i32,
        }
    }

    fn set_word6(&mut self, word: Word6, i0_msb: u32) {
        let mut i0 = i0_msb << 24;
        i0 |= word.i0_lsb;
        self.i0 = (i0 as f64) / 2.0_f64.powi(31);
    }

    fn word6(&self) -> Word6 {
        let i0 = (self.i0 * 2.0_f64.powi(31)).round() as u32;
        Word6 {
            i0_lsb: (i0 & 0x00ffffff) as u32,
        }
    }

    fn set_word7(&mut self, word: Word7, extra: &mut u32) {
        *extra = word.omega_msb as u32;
        self.crc = (word.crc as f64) / 2.0_f64.powi(5);
    }

    fn word7(&self) -> Word7 {
        let omega = (self.omega * 2.0_f64.powi(31)).round() as u32;
        Word7 {
            crc: (self.crc * 2.0_f64.powi(5)) as i32,
            omega_msb: ((omega & 0xff000000) >> 24) as u8,
        }
    }

    fn set_word8(&mut self, word: Word8, omega_msb: u32) {
        let mut omega = omega_msb << 24;
        omega |= word.omega_lsb;
        self.omega = ((omega as i32) as f64) / 2.0_f64.powi(31);
    }

    fn word8(&self) -> Word8 {
        let omega = (self.omega * 2.0_f64.powi(31)).round() as u32;
        Word8 {
            omega_lsb: (omega & 0x00ffffff) as u32,
        }
    }

    fn set_word9(&mut self, word: Word9) {
        self.omega_dot = (word.omega_dot as f64) / 2.0_f64.powi(43);
    }

    fn word9(&self) -> Word9 {
        Word9 {
            omega_dot: (self.omega_dot * 2.0_f64.powi(43)).round() as i32,
        }
    }

    fn set_word10(&mut self, word: Word10) {
        self.idot = (word.idot as f64) / 2.0_f64.powi(43);
        self.iode = word.iode;
    }

    fn word10(&self) -> Word10 {
        Word10 {
            iode: self.iode,
            idot: (self.idot * 2.0_f64.powi(43)).round() as i32,
        }
    }

    /// Encodes this [GpsQzssFrame3] as burst of 8 32-bit data words
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
    pub cic: i32,

    /// Omega0 (8) MSB, you will have to associate this to Word #4
    pub omega0_msb: u8,
}

impl Word3 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cic = ((dword & WORD3_CIC_MASK) >> WORD3_CIC_SHIFT) as u32;
        let cic = twos_complement(cic, 0xffff, 0x8000);
        let omega0_msb = ((dword & WORD3_OMEGA0_MASK) >> WORD3_OMEGA0_SHIFT) as u8;
        Self { cic, omega0_msb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.cic as u32) << WORD3_CIC_SHIFT;
        value |= (self.omega0_msb as u32) << WORD3_OMEGA0_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word4 {
    /// Omega0 (24) LSB, you will have to associate this to Word #3
    pub omega0_lsb: u32,
}

impl Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega0_lsb = ((dword & WORD4_OMEGA0_MASK) >> WORD4_OMEGA0_SHIFT) as u32;
        Self { omega0_lsb }
    }

    pub(crate) fn encode(&self) -> u32 {
        (self.omega0_lsb as u32) << WORD4_OMEGA0_SHIFT
    }
}
#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word5 {
    pub cis: i32,

    /// I0 (8) MSB, you will have to associate this to Word #6
    pub i0_msb: u8,
}

impl Word5 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cis = ((dword & WORD5_CIS_MASK) >> WORD5_CIS_SHIFT) as u32;
        let cis = twos_complement(cis, 0xffff, 0x8000);
        let i0_msb = ((dword & WORD5_I0_MASK) >> WORD5_I0_SHIFT) as u8;
        Self { cis, i0_msb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.cis as u32) & 0xffff) << WORD5_CIS_SHIFT;
        value |= (self.i0_msb as u32) << WORD5_I0_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word6 {
    /// I0 (24) LSB, you will have to associate this to Word #5
    pub i0_lsb: u32,
}

impl Word6 {
    pub(crate) fn decode(dword: u32) -> Self {
        let i0_lsb = ((dword & WORD6_I0_MASK) >> WORD6_I0_SHIFT) as u32;
        Self { i0_lsb }
    }

    pub(crate) fn encode(&self) -> u32 {
        (self.i0_lsb as u32) << WORD6_I0_SHIFT
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word7 {
    pub crc: i32,

    /// Omega (8) MSB, you will have to associate this to Word #8
    pub omega_msb: u8,
}

impl Word7 {
    pub(crate) fn decode(dword: u32) -> Self {
        let crc = ((dword & WORD7_CRC_MASK) >> WORD7_CRC_SHIFT) as u32;
        let crc = twos_complement(crc, 0xffff, 0x8000);
        let omega_msb = ((dword & WORD7_OMEGA_MASK) >> WORD7_OMEGA_SHIFT) as u8;
        Self { crc, omega_msb }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.crc as u32) & 0xffff) << WORD7_CRC_SHIFT;
        value |= (self.omega_msb as u32) << WORD7_OMEGA_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word8 {
    /// Omega (24) LSB, you will have to associate this to Word #7
    pub omega_lsb: u32,
}

impl Word8 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega_lsb = ((dword & WORD8_OMEGA_MASK) >> WORD8_OMEGA_SHIFT) as u32;
        Self { omega_lsb }
    }

    pub(crate) fn encode(&self) -> u32 {
        (self.omega_lsb as u32) << WORD8_OMEGA_SHIFT
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word9 {
    // 24-bit Omega_dot
    pub omega_dot: i32,
}

impl Word9 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega_dot = ((dword & WORD9_OMEGADOT_MASK) >> WORD9_OMEGADOT_SHIFT) as u32;
        let omega_dot = twos_complement(omega_dot, 0xffffff, 0x800000);
        Self { omega_dot }
    }

    pub(crate) fn encode(&self) -> u32 {
        ((self.omega_dot & 0xffffff) as u32) << WORD9_OMEGADOT_SHIFT
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word10 {
    /// 8-bit IODE
    pub iode: u8,

    /// 14-bit IDOT
    pub idot: i32,
}

impl Word10 {
    pub(crate) fn decode(dword: u32) -> Self {
        let iode = ((dword & WORD10_IODE_MASK) >> WORD10_IODE_SHIFT) as u8;

        // 14-bit signed 2's
        let idot = ((dword & WORD10_IDOT_MASK) >> WORD10_IDOT_SHIFT) as u32;
        let idot = twos_complement(idot, 0x3fff, 0x2000);

        Self { iode, idot }
    }

    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.iode as u32) << WORD10_IODE_SHIFT;
        value |= ((self.idot as u32) & 0x3fff) << WORD10_IDOT_SHIFT;
        value
    }
}

#[cfg(test)]
mod frame3 {
    use super::*;

    #[test]
    fn word3_encoding() {
        for dword3 in [
            Word3 {
                omega0_msb: 0,
                cic: 1,
            },
            Word3 {
                omega0_msb: 1,
                cic: 0,
            },
            Word3 {
                omega0_msb: 10,
                cic: 12,
            },
            Word3 {
                omega0_msb: 255,
                cic: 12,
            },
        ] {
            let encoded = dword3.encode();
            let decoded = Word3::decode(encoded);
            assert_eq!(decoded, dword3);
        }
    }

    #[test]
    fn word4_encoding() {
        for dword4 in [
            Word4 { omega0_lsb: 0 },
            Word4 { omega0_lsb: 10 },
            Word4 { omega0_lsb: 250 },
            Word4 { omega0_lsb: 255 },
        ] {
            let encoded = dword4.encode();
            let decoded = Word4::decode(encoded);
            assert_eq!(decoded, dword4);
        }
    }

    #[test]
    fn word5_encoding() {
        for dword5 in [
            Word5 { cis: 0, i0_msb: 1 },
            Word5 { cis: 1, i0_msb: 0 },
            Word5 { cis: 4, i0_msb: 3 },
            Word5 {
                cis: 10,
                i0_msb: 255,
            },
            Word5 {
                cis: -100,
                i0_msb: 255,
            },
            Word5 {
                cis: -9999,
                i0_msb: 255,
            },
        ] {
            let encoded = dword5.encode();
            let decoded = Word5::decode(encoded);
            assert_eq!(decoded, dword5);
        }
    }

    #[test]
    fn word6_encoding() {
        for dword6 in [
            Word6 { i0_lsb: 0 },
            Word6 { i0_lsb: 1 },
            Word6 { i0_lsb: 255 },
        ] {
            let encoded = dword6.encode();
            let decoded = Word6::decode(encoded);
            assert_eq!(decoded, dword6);
        }
    }

    #[test]
    fn word7_encoding() {
        for dword7 in [
            Word7 {
                crc: 0,
                omega_msb: 1,
            },
            Word7 {
                crc: 1,
                omega_msb: 255,
            },
            Word7 {
                crc: -1,
                omega_msb: 0,
            },
            Word7 {
                crc: -1000,
                omega_msb: 250,
            },
            Word7 {
                crc: 1000,
                omega_msb: 254,
            },
        ] {
            let encoded = dword7.encode();
            let decoded = Word7::decode(encoded);
            assert_eq!(decoded, dword7);
        }
    }

    #[test]
    fn word8_encoding() {
        for dword8 in [
            Word8 { omega_lsb: 0 },
            Word8 { omega_lsb: 1 },
            Word8 { omega_lsb: 255 },
        ] {
            let encoded = dword8.encode();
            let decoded = Word8::decode(encoded);
            assert_eq!(decoded, dword8);
        }
    }

    #[test]
    fn word9_encoding() {
        for dword9 in [
            Word9 { omega_dot: 0 },
            Word9 { omega_dot: 10 },
            Word9 { omega_dot: -10 },
            Word9 { omega_dot: -1000 },
            Word9 { omega_dot: 1000 },
            Word9 { omega_dot: 250 },
        ] {
            let encoded = dword9.encode();
            let decoded = Word9::decode(encoded);
            assert_eq!(decoded, dword9);
        }
    }

    #[test]
    fn word10_encoding() {
        for dword10 in [
            Word10 { iode: 0, idot: 10 },
            Word10 {
                iode: 10,
                idot: -10,
            },
            Word10 {
                iode: 255,
                idot: -1000,
            },
            Word10 {
                iode: 254,
                idot: 1000,
            },
        ] {
            let encoded = dword10.encode();
            let decoded = Word10::decode(encoded);
            assert_eq!(decoded, dword10);
        }
    }

    #[test]
    fn frame3_encoding() {
        for (cic, cis, crc, i0, iode, idot, omega0, omega, omega_dot) in [
            (10.0, 11.0, 12.0, 13.0, 20, 30.0, 40.0, 50.0, 60.0),
            (11.0, 12.0, 13.0, 14.0, 25, 36.0, 47.0, 58.0, 69.0),
        ] {
            let frame3 = GpsQzssFrame3 {
                cic,
                cis,
                crc,
                i0,
                iode,
                idot,
                omega0,
                omega,
                omega_dot,
            };

            let encoded = frame3.encode();

            let mut extra = 0;
            let mut decoded = GpsQzssFrame3::default();

            for (i, dword) in encoded.iter().enumerate() {
                decoded
                    .decode_word(i + 3, *dword, &mut extra)
                    .unwrap_or_else(|_| {
                        panic!("Failed to decode dword {:3}=0x{:08X}", i, dword);
                    });
            }

            assert_eq!(frame3.cic, cic);
            assert_eq!(frame3.cis, cis);
            assert_eq!(frame3.crc, crc);
            assert_eq!(frame3.i0, i0);
            assert_eq!(frame3.iode, iode);
            assert_eq!(frame3.idot, idot);
            assert_eq!(frame3.omega0, omega0);
            assert_eq!(frame3.omega, omega);
            assert_eq!(frame3.omega_dot, omega_dot);
        }
    }
}
