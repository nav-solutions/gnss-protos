const GPS_PARITY_MASK: u32 = 0x000000ff;

const WORD3_IODE_MASK: u32 = 0xff000000;
const WORD3_IODE_SHIFT: u32 = 24;
const WORD3_CRS_MASK: u32 = 0x00ffff00;
const WORD3_CRS_SHIFT: u32 = 8;

const WORD4_DELTA_N_MASK: u32 = 0xffff0000;
const WORD4_DELTA_N_SHIFT: u32 = 16;
const WORD4_M0_MSB_MASK: u32 = 0x0000ff00;
const WORD4_M0_MSB_SHIFT: u32 = 8;

const WORD5_M0_LSB_MASK: u32 = 0xffffff00;
const WORD5_M0_LSB_SHIFT: u32 = 8;

const WORD6_CUC_MASK: u32 = 0xffff0000;
const WORD6_CUC_SHIFT: u32 = 16;
const WORD6_E_MSB_MASK: u32 = 0x0000ff00;
const WORD6_E_MSB_SHIFT: u32 = 8;

const WORD7_E_LSB_MASK: u32 = 0xffffff00;
const WORD7_E_LSB_SHIFT: u32 = 8;

const WORD8_CUS_MASK: u32 = 0xffff0000;
const WORD8_CUS_SHIFT: u32 = 16;
const WORD8_SQRTA_MSB_MASK: u32 = 0x0000ff00;
const WORD8_SQRTA_MSB_SHIFT: u32 = 8;

const WORD9_SQRTA_LSB_MASK: u32 = 0xffffff00;
const WORD9_SQRTA_LSB_SHIFT: u32 = 8;

const WORD10_TOE_MASK: u32 = 0xffff0000;
const WORD10_TOE_SHIFT: u32 = 16;


/// GPS / QZSS Frame #2 interpretation
#[derive(Debug, Default, Clone)]
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


#[derive(Debug, Clone)]
pub struct GpsSubframe2Word3 {
    pub iode: u8,
    pub crs: u16,
    pub parity: u8,
}

impl GpsSubframe2Word3 {
    pub(crate) fn decode(dword: u32) -> Self {
        let iode = ((dword & WORD3_IODE_MASK) >> WORD3_IODE_SHIFT) as u8;
        let crs = ((dword & WORD3_CRS_MASK) >> WORD3_CRS_SHIFT) as u16;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { iode, crs, parity }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe2Word4 {
    pub delta_n: u16,

    /// M0 (8) msb, you need to associate this to Subframe #2 Word #5
    pub m0_msb: u8,

    pub parity: u8,
}

impl GpsSubframe2Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        let delta_n = ((dword & WORD4_DELTA_N_MASK) >> WORD4_DELTA_N_SHIFT) as u16;
        let m0_msb = ((dword & WORD4_M0_MSB_MASK) >> WORD4_M0_MSB_SHIFT) as u8;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self {
            delta_n,
            m0_msb,
            parity,
        }
    }
}
#[derive(Debug, Clone)]
pub struct GpsSubframe2Word5 {
    /// M0 (24) lsb, you need to associate this to Subframe #2 Word #4
    pub m0_lsb: u32,

    pub parity: u8,
}

impl GpsSubframe2Word5 {
    pub(crate) fn decode(dword: u32) -> Self {
        let m0_lsb = ((dword & WORD5_M0_LSB_MASK) >> WORD5_M0_LSB_SHIFT) as u32;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { m0_lsb, parity }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe2Word6 {
    pub cuc: u16,

    /// MSB(8) eccentricity, you need to associate this to Subframe #2 Word #7
    pub e_msb: u8,

    pub parity: u8,
}

impl GpsSubframe2Word6 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cuc = ((dword & WORD6_CUC_MASK) >> WORD6_CUC_SHIFT) as u16;
        let e_msb = ((dword & WORD6_E_MSB_MASK) >> WORD6_E_MSB_SHIFT) as u8;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { cuc, e_msb, parity }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe2Word7 {
    /// LSB(24) eccentricity, you need to associate this to Subframe #2 Word #6
    pub e_lsb: u32,

    pub parity: u8,
}

impl GpsSubframe2Word7 {
    pub(crate) fn decode(dword: u32) -> Self {
        let e_lsb = ((dword & WORD7_E_LSB_MASK) >> WORD7_E_LSB_SHIFT) as u32;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { e_lsb, parity }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe2Word8 {
    pub cus: u16,

    /// MSB(8) A⁻¹: you need to associate this to Subframe #2 Word #9
    pub sqrt_a_msb: u8,

    pub parity: u8,
}

impl GpsSubframe2Word8 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cus = ((dword & WORD8_CUS_MASK) >> WORD8_CUS_SHIFT) as u16;
        let sqrt_a_msb = ((dword & WORD8_SQRTA_MSB_MASK) >> WORD8_SQRTA_MSB_SHIFT) as u8;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self {
            cus,
            sqrt_a_msb,
            parity,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe2Word9 {
    /// LSB(24) A⁻¹: you need to associate this to Subframe #2 Word #8
    pub sqrt_a_lsb: u32,

    pub parity: u8,
}

impl GpsSubframe2Word9 {
    pub(crate) fn decode(dword: u32) -> Self {
        let sqrt_a_lsb = ((dword & WORD9_SQRTA_LSB_MASK) >> WORD9_SQRTA_LSB_SHIFT) as u32;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { sqrt_a_lsb, parity }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe2Word10 {
    /// Time of issue of Ephemeris (u16)
    pub toe: u16,

    pub parity: u8,
}

impl GpsSubframe2Word10 {
    pub(crate) fn decode(dword: u32) -> Self {
        let toe = ((dword & WORD10_TOE_MASK) >> WORD10_TOE_SHIFT) as u16;
        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { toe, parity }
    }
}
