use crate::twos_complement;

#[cfg(feature = "log")]
use log::trace;

const WORD3_WEEK_MASK: u32 = 0x3ff00000;
const WORD3_WEEK_SHIFT: u32 = 20;
const WORD3_CA_P_L2_MASK: u32 = 0x000C0000;
const WORD3_CA_P_L2_SHIFT: u32 = 18;
const WORD3_URA_MASK: u32 = 0x0003C000;
const WORD3_URA_SHIFT: u32 = 14;
const WORD3_HEALTH_MASK: u32 = 0x00003f00;
const WORD3_HEALTH_SHIFT: u32 = 8;
const WORD3_IODC_MASK: u32 = 0x000000c0;
const WORD3_IODC_SHIFT: u32 = 6;

const WORD4_L2P_DATA_MASK: u32 = 0x20000000;
const WORD4_RESERVED_MASK: u32 = 0x1fffffc0;
const WORD4_RESERVED_SHIFT: u32 = 6;

const WORD5_RESERVED_MASK: u32 = 0x3fffffc0;
const WORD5_RESERVED_SHIFT: u32 = 6;

const WORD6_RESERVED_MASK: u32 = 0x3fffffc0;
const WORD6_RESERVED_SHIFT: u32 = 6;

const WORD7_RESERVED_MASK: u32 = 0x3fffc000;
const WORD7_RESERVED_SHIFT: u32 = 14;
const WORD7_TGD_MASK: u32 = 0x00003fc0;
const WORD7_TGD_SHIFT: u32 = 6;

const WORD8_IODC_MASK: u32 = 0x3fc00000;
const WORD8_IODC_SHIFT: u32 = 22;
const WORD8_TOC_MASK: u32 = 0x003fffc0;
const WORD8_TOC_SHIFT: u32 = 6;

const WORD9_AF2_MASK: u32 = 0x3fc00000;
const WORD9_AF2_SHIFT: u32 = 22;
const WORD9_AF1_MASK: u32 = 0x003fffc0;
const WORD9_AF1_SHIFT: u32 = 6;

const WORD10_AF0_MASK: u32 = 0x3fffff00;
const WORD10_AF0_SHIFT: u32 = 8;

/// GPS / QZSS Frame #1 interpretation
#[derive(Debug, Default, Clone)]
pub struct GpsQzssFrame1 {
    /// 10-bit week counter (no rollover compensation).
    pub week: u16,

    /// 2-bit C/A or P ON L2.  
    /// When asserted, indicates the NAV data stream was commanded OFF on the L2 channel P-code.
    pub ca_or_p_l2: u8,

    /// 4-bit URA index. The lower the better, interpret as follow (error in meters)
    /// - 0:  0 < ura <= 2.4m
    /// - 1:  2.4 < ura <= 3.4m
    /// - 2:  3.4 < ura <= 4.85
    /// - 3:  4.85 < ura <= 6.85
    /// - 4:  6.85 < ura <= 9.65
    /// - 5:  9.65 < ura <= 13.65
    /// - 6:  13.65 < ura <= 24.00
    /// - 7:  24.00 < ura <= 48.00
    /// - 8:  48.00 < ura <= 96.00
    /// - 9:  96.00 < ura <= 192.00
    /// - 10: 192.00 < ura <=  384.00
    /// - 11: 384.00 < ura <=  768.00
    /// - 12: 768.00 < ura <= 1536.00
    /// - 13: 1536.00 < ura <= 3072.00
    /// - 14: 3072.00 < ura <= 6144.00
    /// - 15: 6144.00 < ura
    ///
    /// For each URA index, users may compute a nominal URA value (x)
    ///  - ura < 6: 2**(1+N/2)
    ///  - ura > 6: 2**(N-2)
    pub ura: u8,

    /// 6-bit SV Health. 0 means all good.
    pub health: u8,

    /// 10-bit IODC.  
    pub iodc: u16,

    /// Time of clock (s)
    pub toc_s: u32,

    /// 8-bit TGD (in seconds)
    pub tgd_s: f64,

    /// af2 (s.s⁻²)
    pub af2_s_s2: f64,

    /// af1 (s.s⁻1)
    pub af1_s_s: f64,

    /// af0 (s)
    pub af0_s: f64,

    /// 32-bit reserved word #4
    pub reserved_word4: u32,

    pub l2_p_data_flag: bool,

    /// 24-bit reserved word #5
    pub reserved_word5: u32,

    /// 24-bit reserved word #6
    pub reserved_word6: u32,

    ///16-bit reserved word #7
    pub reserved_word7: u16,
}

#[derive(Debug, Default, Clone)]
pub struct Word3 {
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
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word3 dword=0x{:08x}", dword);
        let week = ((dword & WORD3_WEEK_MASK) >> WORD3_WEEK_SHIFT) as u16;
        let ca_or_p_l2 = ((dword & WORD3_CA_P_L2_MASK) >> WORD3_CA_P_L2_SHIFT) as u8;
        let ura = ((dword & WORD3_URA_MASK) >> WORD3_URA_SHIFT) as u8;
        let health = ((dword & WORD3_HEALTH_MASK) >> WORD3_HEALTH_SHIFT) as u8;
        let iodc_msb = ((dword & WORD3_IODC_MASK) >> WORD3_IODC_SHIFT) as u8;

        Self {
            week,
            ca_or_p_l2,
            ura,
            health,
            iodc_msb,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word4 {
    pub l2_p_data_flag: bool,
    pub reserved: u32,
}

impl Word4 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word4 dword=0x{:08x}", dword);
        let l2_p_data_flag = (dword & WORD4_L2P_DATA_MASK) > 0;
        let reserved = ((dword & WORD4_RESERVED_MASK) >> WORD4_RESERVED_SHIFT) as u32;
        Self {
            l2_p_data_flag,
            reserved,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word5 {
    /// 24-bit reserved
    pub reserved: u32,
}

impl Word5 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word5 dword=0x{:08x}", dword);
        let reserved = (dword & WORD5_RESERVED_MASK) >> WORD5_RESERVED_SHIFT;
        Self { reserved }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word6 {
    /// 24-bit reserved
    pub reserved: u32,
}

impl Word6 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word6 dword=0x{:08x}", dword);
        let reserved = (dword & WORD6_RESERVED_MASK) >> WORD6_RESERVED_SHIFT;
        Self { reserved }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word7 {
    /// 16-bit reserved
    pub reserved: u16,

    /// TGD
    pub tgd: i8,
}

impl Word7 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word7 dword=0x{:08x}", dword);

        let reserved = ((dword & WORD7_RESERVED_MASK) >> WORD7_RESERVED_SHIFT) as u16;
        let tgd = ((dword & WORD7_TGD_MASK) >> WORD7_TGD_SHIFT) as i8;
        Self { reserved, tgd }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word8 {
    /// 8-bit IODC LSB to associate with Word # 3
    pub iodc_lsb: u8,

    /// 16 bit ToC
    pub toc: u16,
}

impl Word8 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word8 dword=0x{:08x}", dword);

        let iodc_lsb = ((dword & WORD8_IODC_MASK) >> WORD8_IODC_SHIFT) as u8;
        let toc = ((dword & WORD8_TOC_MASK) >> WORD8_TOC_SHIFT) as u16;
        Self { iodc_lsb, toc }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word9 {
    /// 8 bit af2
    pub af2: i8,

    /// 16 bit af1
    pub af1: i16,
}

impl Word9 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word9 dword=0x{:08x}", dword);

        let af2 = ((dword & WORD9_AF2_MASK) >> WORD9_AF2_SHIFT) as i8;
        let af1 = ((dword & WORD9_AF1_MASK) >> WORD9_AF1_SHIFT) as i16;
        Self { af2, af1 }
    }
}

#[derive(Debug, Default, Clone)]
pub struct Word10 {
    /// 22-bit af0
    pub af0: i32,
}

impl Word10 {
    pub(crate) fn decode(dword: u32) -> Self {
        #[cfg(feature = "log")]
        trace!("GPS Word10 dword=0x{:08x}", dword);
        let af0 = ((dword & WORD10_AF0_MASK) >> WORD10_AF0_SHIFT) as u32;
        let af0 = twos_complement(af0, 0x3fffff, 0x200000);
        Self { af0 }
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
    pub fn scale(&self) -> GpsQzssFrame1 {
        GpsQzssFrame1 {
            week: self.word3.week,
            ca_or_p_l2: self.word3.ca_or_p_l2,
            ura: self.word3.ura,
            health: self.word3.health,
            reserved_word4: self.word4.reserved,
            reserved_word5: self.word5.reserved,
            reserved_word6: self.word6.reserved,
            reserved_word7: self.word7.reserved,

            iodc: {
                let mut iodc = self.word3.iodc_msb as u16;
                iodc <<= 8;
                iodc |= self.word8.iodc_lsb as u16;
                iodc
            },

            l2_p_data_flag: self.word4.l2_p_data_flag,

            tgd_s: (self.word7.tgd as f64) / 2.0_f64.powi(31),
            toc_s: (self.word8.toc as u32) * 16,
            af2_s_s2: (self.word9.af2 as f64) / 2.0_f64.powi(55),
            af1_s_s: (self.word9.af1 as f64) / 2.0_f64.powi(43),
            af0_s: (self.word10.af0 as f64) / 2.0_f64.powi(31),
        }
    }
}
