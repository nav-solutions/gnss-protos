#[cfg(feature = "log")]
use log::trace;

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

/// GPS / QZSS Frame #2 interpretation
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame2 {
    /// Time of issue of ephemeris (s)
    pub toe_s: u32,

    /// IODE: Issue of Data (Ephemeris)
    pub iode: u8,

    /// Mean anomaly at reference time (in semi circles)
    pub m0: f64,

    /// Mean motion difference from computed value (in semi circles)
    pub dn: f64,

    /// Latitude cosine harmonic correction term
    pub cuc: f64,

    /// Latitude sine harmonic correction term
    pub cus: f64,

    /// Orbit radius sine harmonic correction term
    pub crs: f64,

    /// Eccentricity
    pub e: f64,

    /// Sqrt(a)
    pub sqrt_a: f64,

    /// Fit interval flag
    pub fit_int_flag: bool,

    /// 5-bit AODO
    pub aodo: u8,
}

#[derive(Debug, Default, Clone)]
pub struct Word3 {
    pub iode: u8,
    pub crs: i16,
}

impl Word3 {
    pub(crate) fn decode(dword: u32) -> Self {
        let iode = ((dword & WORD3_IODE_MASK) >> WORD3_IODE_SHIFT) as u8;
        let crs = ((dword & WORD3_CRS_MASK) >> WORD3_CRS_SHIFT) as i16;

        #[cfg(feature = "log")]
        trace!(
            "GPS Word3 dword=0x{:08x} iode=0x{:02} crs=0x{:04x}",
            dword,
            iode,
            crs
        );

        Self { iode, crs }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word4 {
    /// Delta n
    pub dn: i16,

    /// M0 (8) msb, you need to associate this to Subframe #2 Word #5
    pub m0_msb: u8,
}

impl Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        let dn = ((dword & WORD4_DELTA_N_MASK) >> WORD4_DELTA_N_SHIFT) as i16;
        let m0_msb = ((dword & WORD4_M0_MSB_MASK) >> WORD4_M0_MSB_SHIFT) as u8;

        #[cfg(feature = "log")]
        trace!("GPS Word3 dword=0x{:08x} dn=0x{:04x}", dword, dn);

        Self { dn, m0_msb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word5 {
    /// M0 (24) lsb, you need to associate this to Subframe #2 Word #4
    pub m0_lsb: u32,
}

impl Word5 {
    pub(crate) fn decode(dword: u32) -> Self {
        let m0_lsb = ((dword & WORD5_M0_LSB_MASK) >> WORD5_M0_LSB_SHIFT) as u32;
        Self { m0_lsb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word6 {
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
}

#[derive(Debug, Default, Clone)]
pub struct Word7 {
    /// LSB(24) eccentricity, you need to associate this to Subframe #2 Word #6
    pub e_lsb: u32,
}

impl Word7 {
    pub(crate) fn decode(dword: u32) -> Self {
        let e_lsb = ((dword & WORD7_E_LSB_MASK) >> WORD7_E_LSB_SHIFT) as u32;
        Self { e_lsb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word8 {
    pub cus: i16,

    /// MSB(8) A⁻¹: you need to associate this to Subframe #2 Word #9
    pub sqrt_a_msb: u8,
}

impl Word8 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cus = ((dword & WORD8_CUS_MASK) >> WORD8_CUS_SHIFT) as i16;
        let sqrt_a_msb = ((dword & WORD8_SQRTA_MSB_MASK) >> WORD8_SQRTA_MSB_SHIFT) as u8;
        Self { cus, sqrt_a_msb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word9 {
    /// LSB(24) A⁻¹: you need to associate this to Subframe #2 Word #8
    pub sqrt_a_lsb: u32,
}

impl Word9 {
    pub(crate) fn decode(dword: u32) -> Self {
        let sqrt_a_lsb = ((dword & WORD9_SQRTA_LSB_MASK) >> WORD9_SQRTA_LSB_SHIFT) as u32;
        Self { sqrt_a_lsb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word10 {
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
        #[cfg(feature = "log")]
        trace!("GPS Word10 dword=0x{:08x} toe=0x{:04x}", dword, toe);
        Self { toe, fitint, aodo }
    }
}

#[derive(Debug, Default, Clone)]
pub struct UnscaledFrame {
    pub word3: Word3,
    pub word4: Word4,
    pub word5: Word5,
    pub word6: Word6,
    pub word7: Word7,
    pub word8: Word8,
    pub word9: Word9,
    pub word10: Word10,
}

impl UnscaledFrame {
    pub fn scale(&self) -> GpsQzssFrame2 {
        GpsQzssFrame2 {
            iode: self.word3.iode,
            toe_s: (self.word10.toe as u32) * 16,
            crs: (self.word3.crs as f64) / 2.0_f64.powi(5),
            cus: (self.word8.cus as f64) / 2.0_f64.powi(29),
            cuc: (self.word6.cuc as f64) / 2.0_f64.powi(29),

            dn: {
                let dn = self.word4.dn as f64;
                dn / 2.0_f64.powi(43)
            },

            m0: {
                let mut m0 = self.word4.m0_msb as u32;
                m0 <<= 24;
                m0 |= self.word5.m0_lsb as u32;

                let m0 = (m0 as i32) as f64;
                m0 / 2.0_f64.powi(31)
            },

            e: {
                let mut e = self.word6.e_msb as u32;
                e <<= 24;
                e |= self.word7.e_lsb;

                (e as f64) / 2.0_f64.powi(33)
            },

            sqrt_a: {
                let mut sqrt_a = self.word8.sqrt_a_msb as u32;
                sqrt_a <<= 24;
                sqrt_a |= self.word9.sqrt_a_lsb;

                (sqrt_a as f64) / 2.0_f64.powi(19)
            },

            aodo: self.word10.aodo,
            fit_int_flag: self.word10.fitint,
        }
    }
}
