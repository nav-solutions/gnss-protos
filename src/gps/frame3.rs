use crate::twos_complement;

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

/// GPS / QZSS Frame #3 interpretation
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

#[derive(Debug, Default, Clone)]
pub struct Word3 {
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
}

#[derive(Debug, Default, Clone)]
pub struct Word4 {
    /// Omega0 (24) LSB, you will have to associate this to Word #3
    pub omega0_lsb: u32,
}

impl Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega0_lsb = ((dword & WORD4_OMEGA0_MASK) >> WORD4_OMEGA0_SHIFT) as u32;
        Self { omega0_lsb }
    }
}
#[derive(Debug, Default, Clone)]
pub struct Word5 {
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
}

#[derive(Debug, Default, Clone)]
pub struct Word6 {
    /// I0 (24) LSB, you will have to associate this to Word #5
    pub i0_lsb: u32,
}

impl Word6 {
    pub(crate) fn decode(dword: u32) -> Self {
        let i0_lsb = ((dword & WORD6_I0_MASK) >> WORD6_I0_SHIFT) as u32;
        Self { i0_lsb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word7 {
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
}

#[derive(Debug, Default, Clone)]
pub struct Word8 {
    /// Omega (24) LSB, you will have to associate this to Word #7
    pub omega_lsb: u32,
}

impl Word8 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega_lsb = ((dword & WORD8_OMEGA_MASK) >> WORD8_OMEGA_SHIFT) as u32;
        Self { omega_lsb }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word9 {
    // 24-bit Omega_dot
    pub omega_dot: i32,
}

impl Word9 {
    pub(crate) fn decode(dword: u32) -> Self {
        let omega_dot = ((dword & WORD9_OMEGADOT_MASK) >> WORD9_OMEGADOT_SHIFT) as u32;
        let omega_dot = twos_complement(omega_dot, 0xffffff, 0x800000);
        Self { omega_dot }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word10 {
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
}
