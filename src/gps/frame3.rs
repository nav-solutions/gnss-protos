const GPS_PARITY_MASK: u32 = 0x000000ff;

const WORD3_CIC_MASK: u32 = 0xffff0000;
const WORD3_CIC_SHIFT: u32 = 16;
const WORD3_OMEGA0_MSB_MASK: u32 = 0x0000ff00;
const WORD3_OMEGA0_MSB_SHIFT: u32 = 8;

const WORD4_OMEGA0_LSB_MASK: u32 = 0xffffff00;
const WORD4_OMEGA0_LSB_SHIFT: u32 = 8;

const WORD5_CIS_MASK: u32 = 0xffff0000;
const WORD5_CIS_SHIFT: u32 = 16;
const WORD5_I0_MSB_MASK: u32 = 0x0000ff00;
const WORD5_I0_MSB_SHIFT: u32 = 8;

const WORD6_I0_LSB_MASK: u32 = 0xffffff00;
const WORD6_I0_LSB_SHIFT: u32 = 8;

/// GPS / QZSS Frame #3 interpretation
#[derive(Debug, Default, Clone)]
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

#[derive(Debug, Clone)]
pub struct GpsSubframe3Word3 {
    pub cic: u16,

    /// MSB(8) bits of Omega0. You need to associate this to Frame #3 word #4
    pub omega0_msb: u8,

    pub parity: u8,
}

impl GpsSubframe3Word3 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cic = ((dword & WORD3_CIC_MASK) >> WORD3_CIC_SHIFT) as u16;

        let omega0_msb = ((dword & WORD3_OMEGA0_MSB_MASK) >> WORD3_OMEGA0_MSB_SHIFT) as u8;

        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self {
            cic,
            parity,
            omega0_msb,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe3Word4 {
    /// LSB(24) bits of Omega0. You need to associate this to Frame #3 word #3
    pub omega0_lsb: u8,

    pub parity: u8,
}

impl GpsSubframe3Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega0_lsb = ((dword & WORD4_OMEGA0_LSB_MASK) >> WORD4_OMEGA0_LSB_SHIFT) as u8;

        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { parity, omega0_lsb }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe3Word5 {
    pub cis: u16,

    /// MSB(8) bits of i0. You need to associate this to Frame #3 word #6
    pub i0_msb: u8,

    pub parity: u8,
}

impl GpsSubframe3Word5 {
    pub(crate) fn decode(dword: u32) -> Self {
        let cis = ((dword & WORD5_CIS_MASK) >> WORD5_CIS_SHIFT) as u16;

        let i0_msb = ((dword & WORD5_I0_MSB_MASK) >> WORD5_I0_MSB_SHIFT) as u8;

        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self {
            cis,
            parity,
            i0_msb,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSubframe3Word6 {
    /// LSB(24) bits of i0. You need to associate this to Frame #3 word #5
    pub i0_lsb: u8,

    pub parity: u8,
}

impl GpsSubframe3Word6 {
    pub(crate) fn decode(dword: u32) -> Self {
        let i0_lsb = ((dword & WORD6_I0_LSB_MASK) >> WORD6_I0_LSB_SHIFT) as u8;

        let parity = (dword & GPS_PARITY_MASK) as u8;

        Self { parity, i0_lsb }
    }
}
