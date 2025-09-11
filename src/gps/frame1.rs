use crate::{gps::GpsError, twos_complement};

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

/// [GpsQzssFrame1] Ephemeris #1 frame interpretation.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
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
    pub ura: u8,

    /// 6-bit SV Health. 0 means all good.
    pub health: u8,

    /// 10-bit IODC.  
    pub iodc: u16,

    /// Time of clock (in seconds)
    pub toc: u32,

    /// 8-bit TGD (in seconds)
    pub tgd: f64,

    /// af2 (in seconds per s^-^&)
    pub af2: f64,

    /// af1 (in seconds per seconds)
    pub af1: f64,

    /// 22-bit af0 (in seconds)
    pub af0: f64,

    /// 23-bit reserved word #4
    pub reserved_word4: u32,

    /// 1-bit flag
    pub l2_p_data_flag: bool,

    /// 24-bit reserved word #5
    pub reserved_word5: u32,

    /// 24-bit reserved word #6
    pub reserved_word6: u32,

    /// 16-bit reserved word #7
    pub reserved_word7: u16,
}

impl GpsQzssFrame1 {
    /// Computes binary URA from value in meters
    fn compute_ura(value_m: f64) -> u8 {
        if value_m <= 2.4 {
            0
        } else if value_m <= 3.4 {
            1
        } else if value_m <= 4.85 {
            2
        } else if value_m <= 6.85 {
            3
        } else if value_m <= 9.65 {
            4
        } else if value_m <= 13.65 {
            5
        } else if value_m <= 24.0 {
            6
        } else if value_m <= 48.0 {
            7
        } else if value_m <= 96.0 {
            8
        } else if value_m <= 192.0 {
            9
        } else if value_m <= 384.0 {
            10
        } else if value_m <= 768.0 {
            11
        } else if value_m <= 1536.0 {
            12
        } else if value_m <= 3072.0 {
            13
        } else if value_m <= 6144.0 {
            14
        } else {
            15
        }
    }

    /// Calculates nominal User Range Accuracy in meters
    pub fn nominal_user_range_accuracy(&self) -> f64 {
        // For each URA index, users may compute a nominal URA value (x)
        //  - ura < 6: 2**(1+N/2)
        //  - ura > 6: 2**(N-2)
        if self.ura <= 6 {
            2.0_f64.powi((1 + self.ura / 2) as i32)
        } else {
            2.0_f64.powi((self.ura / 2) as i32)
        }
    }

    /// Copies and returns [GpsQzssFrame1] with updated Week number
    pub fn with_week(mut self, week: u16) -> Self {
        self.week = week;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated health mask.
    pub fn with_health(mut self, health: u8) -> Self {
        self.health = health;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated time of clock in seconds.
    pub fn with_time_of_clock_seconds(mut self, toc_s: u32) -> Self {
        self.toc = toc_s;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated Total Group Delay (TGD) in seconds
    pub fn with_total_group_delay_seconds(mut self, tgd_s: f64) -> Self {
        self.tgd = tgd_s;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated User Range Accuracy
    /// in meters.
    pub fn with_user_range_accuracy_m(mut self, ura_m: f64) -> Self {
        self.ura = Self::compute_ura(ura_m);
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated User Range Accuracy
    /// from a nominal User Range Accuracy in meters.
    pub fn with_nominal_user_range_accuracy_m(mut self, ura_m: f64) -> Self {
        // For each URA index, users may compute a nominal URA value (x)
        //  - ura < 6: 2**(1+N/2)
        //  - ura > 6: 2**(N-2)
        let ura = if ura_m <= 24.0 {
            2.0 * (ura_m.log2() - 1.0)
        } else {
            2.0 + ura_m.log2()
        };

        self.ura = ura.round() as u8;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (0) term
    pub fn with_af0(mut self, af0: f64) -> Self {
        self.af0 = af0;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (1) term
    pub fn with_af1(mut self, af1: f64) -> Self {
        self.af1 = af1;
        self
    }

    /// Copies and returns [GpsQzssFrame1] with updated clock correction (2) term
    pub fn with_af2(mut self, af2: f64) -> Self {
        self.af2 = af2;
        self
    }

    pub(crate) fn decode_word(&mut self, ptr: usize, dword: u32) -> Result<(), GpsError> {
        match ptr {
            3 => {
                let word = Word3::decode(dword);
                self.set_word3(word);
            },
            4 => {
                let word = Word4::decode(dword);
                self.set_word4(word);
            },
            5 => {
                let word = Word5::decode(dword);
                self.set_word5(word);
            },
            6 => {
                let word = Word6::decode(dword);
                self.set_word6(word);
            },
            7 => {
                let word = Word7::decode(dword);
                self.set_word7(word);
            },
            8 => {
                let word = Word8::decode(dword);
                self.set_word8(word);
            },
            9 => {
                let word = Word9::decode(dword);
                self.set_word9(word);
            },
            10 => {
                let word = Word10::decode(dword);
                self.set_word10(word);
            },
            _ => return Err(GpsError::InternalFSM),
        }
        Ok(())
    }

    /// Updates scaled content from [Word3]
    fn set_word3(&mut self, word: Word3) {
        self.week = word.week;
        self.ura = word.ura;
        self.ca_or_p_l2 = word.ca_or_p_l2;
        self.health = word.health;
        self.iodc = (word.iodc_msb as u16) << 8;
    }

    /// Encodes a [Word3] from [GpsQzssFrame1]
    fn word3(&self) -> Word3 {
        Word3 {
            week: self.week,
            ura: self.ura,
            health: self.health,
            ca_or_p_l2: self.ca_or_p_l2,
            iodc_msb: ((self.iodc & 0x300) >> 8) as u8,
        }
    }

    /// Updates scaled content from [Word4]
    fn set_word4(&mut self, word: Word4) {
        self.l2_p_data_flag = word.l2_p_data_flag;
        self.reserved_word4 = word.reserved;
    }

    /// Encodes a [Word4] from [GpsQzssFrame1]
    fn word4(&self) -> Word4 {
        Word4 {
            reserved: self.reserved_word4,
            l2_p_data_flag: self.l2_p_data_flag,
        }
    }

    /// Updates scaled content from [Word5]
    fn set_word5(&mut self, word: Word5) {
        self.reserved_word5 = word.reserved;
    }

    /// Encodes a [Word5] from [GpsQzssFrame1]
    fn word5(&self) -> Word5 {
        Word5 {
            reserved: self.reserved_word5,
        }
    }

    /// Updates scaled content from [Word6]
    fn set_word6(&mut self, word: Word6) {
        self.reserved_word6 = word.reserved;
    }

    /// Encodes a [Word6] from [GpsQzssFrame1]
    fn word6(&self) -> Word6 {
        Word6 {
            reserved: self.reserved_word6,
        }
    }

    /// Updates scaled content from [Word7]
    fn set_word7(&mut self, word: Word7) {
        self.reserved_word7 = word.reserved;
        self.tgd = (word.tgd as f64) / 2.0_f64.powi(31);
    }

    /// Encodes a [Word7] from [GpsQzssFrame1]
    fn word7(&self) -> Word7 {
        Word7 {
            reserved: self.reserved_word7,
            tgd: (self.tgd * 2.0_f64.powi(31)).round() as i8,
        }
    }

    /// Updates scaled content from [Word8]
    fn set_word8(&mut self, word: Word8) {
        self.toc = (word.toc as u32) * 16;
        self.iodc |= word.iodc_lsb as u16;
    }

    /// Encodes a [Word8] from [GpsQzssFrame1]
    fn word8(&self) -> Word8 {
        Word8 {
            toc: (self.toc / 16) as u16,
            iodc_lsb: (self.iodc & 0xff) as u8,
        }
    }

    /// Updates scaled content from [Word9]
    fn set_word9(&mut self, word: Word9) {
        self.af2 = (word.af2 as f64) * 2.0_f64.powi(-55);
        self.af1 = (word.af1 as f64) * 2.0_f64.powi(-43);
    }

    /// Encodes a [Word9] from [GpsQzssFrame1]
    fn word9(&self) -> Word9 {
        Word9 {
            af2: (self.af2 * 2.0_f64.powi(43)).round() as i8,
            af1: (self.af1 * 2.0_f64.powi(55)).round() as i16,
        }
    }

    /// Updates scaled content from [Word10]
    fn set_word10(&mut self, word: Word10) {
        self.af0 = (word.af0 as f64) * 2.0_f64.powi(-31);
    }

    /// Encodes a [Word10] from [GpsQzssFrame1]
    fn word10(&self) -> Word10 {
        Word10 {
            af0: (self.af0 * 2.0_f64.powi(31)).round() as i32,
        }
    }

    /// Encodes this [GpsQzssFrame1] as a burst of 8 [u32] data words
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

#[derive(Debug, Copy, Default, Clone, PartialEq)]
pub(crate) struct Word3 {
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
    pub fn decode(dword: u32) -> Self {
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

    pub fn encode(&self) -> u32 {
        let mut value = 0;

        value |= ((self.week & 0x3ff) as u32) << WORD3_WEEK_SHIFT;
        value |= ((self.ca_or_p_l2 & 0x3) as u32) << WORD3_CA_P_L2_SHIFT;
        value |= ((self.ura & 0x07) as u32) << WORD3_URA_SHIFT;
        value |= ((self.health & 0x3f) as u32) << WORD3_HEALTH_SHIFT;
        value |= ((self.iodc_msb & 0x03) as u32) << WORD3_IODC_SHIFT;

        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word4 {
    pub l2_p_data_flag: bool,
    pub reserved: u32,
}

impl Word4 {
    pub fn decode(dword: u32) -> Self {
        let l2_p_data_flag = (dword & WORD4_L2P_DATA_MASK) > 0;
        let reserved = ((dword & WORD4_RESERVED_MASK) >> WORD4_RESERVED_SHIFT) as u32;
        Self {
            l2_p_data_flag,
            reserved,
        }
    }

    pub fn encode(&self) -> u32 {
        let mut value = 0;

        if self.l2_p_data_flag {
            value |= WORD4_L2P_DATA_MASK;
        }

        value |= (self.reserved & 0x7fffff) << WORD4_RESERVED_SHIFT;

        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word5 {
    /// 24-bit reserved
    pub reserved: u32,
}

impl Word5 {
    pub fn decode(dword: u32) -> Self {
        let reserved = (dword & WORD5_RESERVED_MASK) >> WORD5_RESERVED_SHIFT;
        Self { reserved }
    }

    pub fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.reserved & 0x0ffffff) << WORD5_RESERVED_SHIFT;
        value
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub(crate) struct Word6 {
    /// 24-bit reserved
    pub reserved: u32,
}

impl Word6 {
    pub fn decode(dword: u32) -> Self {
        let reserved = (dword & WORD6_RESERVED_MASK) >> WORD6_RESERVED_SHIFT;
        Self { reserved }
    }

    pub fn encode(&self) -> u32 {
        let mut value = 0;
        value |= (self.reserved & 0x0ffffff) << WORD6_RESERVED_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word7 {
    /// 16-bit reserved
    pub reserved: u16,

    /// TGD
    pub tgd: i8,
}

impl Word7 {
    pub fn decode(dword: u32) -> Self {
        let reserved = ((dword & WORD7_RESERVED_MASK) >> WORD7_RESERVED_SHIFT) as u16;
        let tgd = ((dword & WORD7_TGD_MASK) >> WORD7_TGD_SHIFT) as i8;
        Self { reserved, tgd }
    }

    pub fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.reserved as u32) & 0x0ffff) << WORD7_RESERVED_SHIFT;
        value |= ((self.tgd as u32) & 0xff) << WORD7_TGD_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word8 {
    /// 8-bit IODC LSB to associate with Word # 3
    pub iodc_lsb: u8,

    /// 16 bit ToC
    pub toc: u16,
}

impl Word8 {
    pub fn decode(dword: u32) -> Self {
        let iodc_lsb = ((dword & WORD8_IODC_MASK) >> WORD8_IODC_SHIFT) as u8;
        let toc = ((dword & WORD8_TOC_MASK) >> WORD8_TOC_SHIFT) as u16;
        Self { iodc_lsb, toc }
    }

    pub fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.iodc_lsb as u32) & 0xff) << WORD8_IODC_SHIFT;
        value |= ((self.toc as u32) & 0x0ffff) << WORD8_TOC_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word9 {
    /// 8 bit af2
    pub af2: i8,

    /// 16 bit af1
    pub af1: i16,
}

impl Word9 {
    pub fn decode(dword: u32) -> Self {
        let af2 = ((dword & WORD9_AF2_MASK) >> WORD9_AF2_SHIFT) as i8;
        let af1 = ((dword & WORD9_AF1_MASK) >> WORD9_AF1_SHIFT) as i16;
        Self { af2, af1 }
    }

    pub fn encode(&self) -> u32 {
        let mut value = 0;
        value |= ((self.af2 as u32) & 0x0ff) << WORD9_AF2_SHIFT;
        value |= ((self.af1 as u32) & 0x0ffff) << WORD9_AF1_SHIFT;
        value
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub(crate) struct Word10 {
    /// 22-bit af0
    pub af0: i32,
}

impl Word10 {
    pub fn decode(dword: u32) -> Self {
        let af0 = ((dword & WORD10_AF0_MASK) >> WORD10_AF0_SHIFT) as u32;
        let af0 = twos_complement(af0, 0x3fffff, 0x200000);
        Self { af0 }
    }

    pub fn encode(&self) -> u32 {
        ((self.af0 & 0x3fffff) as u32) << WORD10_AF0_SHIFT
    }
}

#[cfg(test)]
mod frame1 {
    use super::*;

    #[test]
    fn dword3_encoding() {
        for dword3 in [
            Word3 {
                week: 1,
                ca_or_p_l2: 0,
                ura: 0,
                health: 0,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 1,
                ura: 0,
                health: 0,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 0,
                ura: 1,
                health: 0,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 0,
                ura: 0,
                health: 1,
                iodc_msb: 0,
            },
            Word3 {
                week: 0,
                ca_or_p_l2: 0,
                ura: 0,
                health: 0,
                iodc_msb: 1,
            },
            Word3 {
                week: 1,
                ca_or_p_l2: 2,
                ura: 5,
                health: 1,
                iodc_msb: 0,
            },
        ] {
            let encoded = dword3.encode();
            let decoded = Word3::decode(encoded);
            assert_eq!(decoded, dword3);
        }
    }

    #[test]
    fn dword4_encoding() {
        for dword4 in [
            Word4 {
                l2_p_data_flag: true,
                reserved: 0,
            },
            Word4 {
                l2_p_data_flag: false,
                reserved: 1,
            },
            Word4 {
                l2_p_data_flag: true,
                reserved: 123,
            },
        ] {
            let encoded = dword4.encode();
            let decoded = Word4::decode(encoded);
            assert_eq!(decoded, dword4);
        }
    }

    #[test]
    fn dword5_encoding() {
        for dword5 in [Word5 { reserved: 0 }, Word5 { reserved: 120 }] {
            let encoded = dword5.encode();
            let decoded = Word5::decode(encoded);
            assert_eq!(decoded, dword5);
        }
    }

    #[test]
    fn dword6_encoding() {
        for dword6 in [Word6 { reserved: 0 }, Word6 { reserved: 120 }] {
            let encoded = dword6.encode();
            let decoded = Word6::decode(encoded);
            assert_eq!(decoded, dword6);
        }
    }

    #[test]
    fn dword7_encoding() {
        for dword7 in [
            Word7 {
                reserved: 0,
                tgd: 1,
            },
            Word7 {
                reserved: 120,
                tgd: 0,
            },
            Word7 {
                reserved: 120,
                tgd: 23,
            },
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
                iodc_lsb: 10,
                toc: 30,
            },
            Word8 {
                iodc_lsb: 30,
                toc: 10,
            },
        ] {
            let encoded = dword8.encode();
            let decoded = Word8::decode(encoded);
            assert_eq!(decoded, dword8);
        }
    }

    #[test]
    fn dword9_encoding() {
        for dword9 in [Word9 { af2: 10, af1: 9 }, Word9 { af2: 9, af1: 100 }] {
            let encoded = dword9.encode();
            let decoded = Word9::decode(encoded);
            assert_eq!(decoded, dword9);
        }
    }

    #[test]
    fn dword10_encoding() {
        for dword10 in [
            Word10 { af0: 0 },
            Word10 { af0: 100 },
            Word10 { af0: -1230 },
            Word10 { af0: -3140 },
        ] {
            let encoded = dword10.encode();
            let decoded = Word10::decode(encoded);
            assert_eq!(decoded, dword10);
        }
    }

    #[test]
    fn frame1_encoding() {
        for (
            week,
            ca_or_p_l2,
            ura,
            health,
            iodc,
            toc,
            tgd,
            af0,
            af1,
            af2,
            l2_p_data_flag,
            reserved_word4,
            reserved_word5,
            reserved_word6,
            reserved_word7,
        ) in [
            (1, 2, 3, 4, 5, 6, 7.0, 8.0, 9.0, 10.0, false, 11, 12, 13, 14),
            (0, 1, 2, 3, 4, 5, 6.0, 7.0, 8.0, 9.0, true, 10, 11, 12, 13),
        ] {
            let frame1 = GpsQzssFrame1 {
                week,
                ca_or_p_l2,
                ura,
                health,
                iodc,
                toc: toc * 16,
                tgd: tgd * 1.0E-9,
                af0: af0 * 1.0E-10,
                af1: af1 * 1.0E-11,
                af2: af2 * 1.0E-11,
                l2_p_data_flag,
                reserved_word4,
                reserved_word5,
                reserved_word6,
                reserved_word7,
            };

            let encoded = frame1.encode();

            let mut decoded = GpsQzssFrame1::default();

            for (i, dword) in encoded.iter().enumerate() {
                decoded.decode_word(i + 3, *dword).unwrap_or_else(|_| {
                    panic!("Failed to decode dword {:3}=0x{:08X}", i, dword);
                });
            }

            assert_eq!(decoded.ura, frame1.ura);
            assert_eq!(decoded.week, frame1.week);
            assert_eq!(decoded.toc, frame1.toc);
            assert_eq!(decoded.ca_or_p_l2, frame1.ca_or_p_l2);
            assert_eq!(decoded.l2_p_data_flag, frame1.l2_p_data_flag);
            assert_eq!(decoded.reserved_word4, frame1.reserved_word4);
            assert_eq!(decoded.reserved_word5, frame1.reserved_word5);
            assert_eq!(decoded.reserved_word6, frame1.reserved_word6);
            assert_eq!(decoded.reserved_word7, frame1.reserved_word7);

            assert!((decoded.af0 - frame1.af0).abs() < 1E-9);
            // assert!((decoded.af1 - frame1.af1).abs() < 1E-9);
            assert!((decoded.af2 - frame1.af2).abs() < 1E-9);
        }
    }

    #[test]
    fn user_range_accuracy() {
        for (value_m, encoded_ura) in [
            (0.1, 0),
            (1.0, 0),
            (2.4, 0),
            (2.5, 1),
            (2.6, 1),
            (3.4, 1),
            (3.5, 2),
            (95.0, 8),
            (96.0, 8),
            (96.1, 9),
            (3071.0, 13),
            (3072.0, 13),
            (3072.1, 14),
            (4000.1, 14),
        ] {
            let ura = GpsQzssFrame1::compute_ura(value_m);
            assert_eq!(ura, encoded_ura, "encoded incorrect URA from {}m", value_m);

            let mut frame1 = GpsQzssFrame1::default().with_user_range_accuracy_m(value_m);

            assert_eq!(frame1.ura, encoded_ura);

            let mut expected = GpsQzssFrame1::default();
            expected.ura = encoded_ura;

            // assert_eq!(
            //     frame1.with_nominal_user_range_accuracy_m(value_m).ura,
            //     encoded_ura,
            //     "failed for value={}m, encoded={}", value_m, encoded_ura,
            // );
        }
    }
}
