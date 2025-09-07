use crate::twos_complement;

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

impl GpsQzssFrame2 {
    fn word3(&self) -> Word3 {
        Word3 {
            iode: self.iode,
            crs: (self.crs * 2.0_f64.powi(5)) as i32,
        }
    }

    pub(crate) fn set_word3(&mut self, word: &Word3) {
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

    pub(crate) fn set_dn(&mut self, word: &Word4) {
        self.dn = (word.dn as f64) / 2.0_f64.powi(43);
    }

    fn word5(&self) -> Word5 {
        let m0 = (self.m0 * 2.0_f64.powi(31)).round() as i32;

        Word5 {
            m0_lsb: (m0 & 0x00ffffff) as u32,
        }
    }

    pub(crate) fn set_word5(&mut self, word: &Word5, m0_msb: u32) {
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

    pub(crate) fn set_cuc(&mut self, word: &Word6) {
        self.cuc = (word.cuc as f64) / 2.0_f64.powi(29);
    }

    fn word7(&self) -> Word7 {
        let e = (self.e * 2.0_f64.powi(33)).round() as i32;
        Word7 {
            e_lsb: (e & 0x00ffffff) as u32,
        }
    }

    pub(crate) fn set_word7(&mut self, word: &Word7, e_msb: u32) {
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

    pub(crate) fn set_word8(&mut self, word: &Word8) {
        self.cus = (word.cus as f64) / 2.0_f64.powi(29);
    }

    fn word9(&self) -> Word9 {
        let sqrt_a = (self.sqrt_a * 2.0_f64.powi(19)).round() as i32;
        Word9 {
            sqrt_a_lsb: (sqrt_a & 0x00ffffff) as u32,
        }
    }

    pub(crate) fn set_word9(&mut self, word: &Word9, sqrt_a_msb: u32) {
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

    pub(crate) fn set_word10(&mut self, word: &Word10) {
        self.aodo = word.aodo;
        self.fit_int_flag = word.fitint;
        self.toe = (word.toe as u32) * 16;
    }
}

#[derive(Debug, Default, Clone)]
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
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}

#[derive(Debug, Default, Clone)]
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
        0
    }
}
