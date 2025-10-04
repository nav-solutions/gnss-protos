use crate::{
    gps::{GpsDataWord, GPS_WORDS_PER_FRAME},
    twos_complement,
};

const WORD3_DID_MASK: u32 = 0x0000_0003;
const WORD3_DID_SHIFT: usize = 0;
const WORD3_SVID_MASK: u32 = 0x0000_00fc;
const WORD3_SVID_SHIFT: usize = 2;
const WORD3_TOA_MASK: u32 = 0x0000_ff00;
const WORD3_TOA_SHIFT: usize = 8;
const WORD3_WEEK_MASK: u32 = 0x00ff_0000;
const WORD3_WEEK_SHIFT: usize = 16;

const WORD10_SPARE_MASK: u32 = 0x0007_ffff;
const WORD10_SPARE_SHIFT: usize = 0;
const WORD10_RES_MASK: u32 = 0x0038_0000;
const WORD10_RES_SHIFT: usize = 19;

/// [GpsQzssSatelliteHealth] reported for a specific satellite.
/// This information was upload by ground-stations to this emitter.
/// It allows receivers to obtain a global status report, by tracking only
/// one satellite.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssSatelliteHealth {
    /// Satellite ID
    pub sat_id: u8,

    /// 6-bit health
    pub health: u8,
}

/// [GpsQzssAlmanachStatus] is obtained page 25 of Frame-5.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssAlmanachStatus {
    /// 2-bit DATA ID
    pub data_id: u8,

    /// 6-bit SV ID
    pub sv_id: u8,

    /// 8-bit ToA (in seconds).
    pub toa_seconds: u8,

    /// 8-bit Week number to which the almanach reference time (toa)
    /// is referenced. This value is the 8-LSB of the full week number.
    pub week: u8,

    /// Array of [GpsQzssSatelliteHealth], uploaded by ground-stations,
    /// this will report the status of 24 satellites at once.
    /// Which gives a high level status report for the entire constellation,
    /// by tracking only one satellite.
    pub sat_health: [GpsQzssSatelliteHealth; 24],

    /// 3 reserved bits
    pub reserved: u8,

    /// 19 spare bits
    pub spare: u32,
}

#[derive(Debug, Copy, Default, Clone, PartialEq)]
struct Word3 {
    /// 2-bit DATA ID
    pub data_id: u8,

    /// 6-bit SV ID
    pub sv_id: u8,

    /// 8-bit ToA (in seconds).
    pub toa_seconds: u8,

    /// 8-bit Week number to which the almanach reference time (toa)
    /// is referenced. This value is the 8-LSB of the full week number.
    pub week: u8,
}

#[derive(Debug, Copy, Default, Clone, PartialEq)]
struct HealthWord {
    /// Sat #1
    pub health1: u8,

    /// Sat #2
    pub health2: u8,

    /// Sat #3
    pub health3: u8,

    /// Sat #4
    pub health4: u8,
}

#[derive(Debug, Copy, Default, Clone, PartialEq)]
struct Word10 {
    /// 3 reserved bits
    pub reserved: u8,

    /// 19 spare bits
    pub spare: u32,
}

impl GpsQzssAlmanachStatus {
    /// Decodes [Self] from 8 [GpsDataWord]s.
    /// This method does not care for frames parity.
    pub(crate) fn from_words(words: &[GpsDataWord]) -> Self {
        let mut s = Self::default();

        for i in 0..GPS_WORDS_PER_FRAME - 2 {
            match i {
                0 => s.set_word3(Word3::from_word(words[i])),
                1 => s.set_word4(HealthWord::from_word(words[i])),
                2 => s.set_word5(HealthWord::from_word(words[i])),
                3 => s.set_word6(HealthWord::from_word(words[i])),
                4 => s.set_word7(HealthWord::from_word(words[i])),
                5 => s.set_word8(HealthWord::from_word(words[i])),
                6 => s.set_word9(HealthWord::from_word(words[i])),
                7 => s.set_word10(Word10::from_word(words[i])),
                _ => unreachable!("expecting 8 data words"),
            }
        }

        s
    }

    fn word3(&self) -> Word3 {
        Word3 {
            week: self.week,
            sv_id: self.sv_id,
            data_id: self.data_id,
            toa_seconds: self.toa_seconds,
        }
    }

    fn set_word3(&mut self, word: Word3) {
        self.week = word.week;
        self.sv_id = word.sv_id;
        self.data_id = word.data_id;
        self.toa_seconds = word.toa_seconds;
    }

    fn word4(&self) -> HealthWord {
        HealthWord {
            health1: self.sat_health[0].health,
            health2: self.sat_health[1].health,
            health3: self.sat_health[2].health,
            health4: self.sat_health[3].health,
        }
    }

    fn set_word4(&mut self, word: HealthWord) {
        self.sat_health[0].health = word.health1;
        self.sat_health[1].health = word.health2;
        self.sat_health[2].health = word.health3;
        self.sat_health[3].health = word.health4;
    }

    fn word5(&self) -> HealthWord {
        HealthWord {
            health1: self.sat_health[4].health,
            health2: self.sat_health[5].health,
            health3: self.sat_health[6].health,
            health4: self.sat_health[7].health,
        }
    }

    fn set_word5(&mut self, word: HealthWord) {
        self.sat_health[4].health = word.health1;
        self.sat_health[5].health = word.health2;
        self.sat_health[6].health = word.health3;
        self.sat_health[7].health = word.health4;
    }

    fn word6(&self) -> HealthWord {
        HealthWord {
            health1: self.sat_health[8].health,
            health2: self.sat_health[9].health,
            health3: self.sat_health[10].health,
            health4: self.sat_health[11].health,
        }
    }

    fn set_word6(&mut self, word: HealthWord) {
        self.sat_health[8].health = word.health1;
        self.sat_health[9].health = word.health2;
        self.sat_health[10].health = word.health3;
        self.sat_health[11].health = word.health4;
    }

    fn word7(&self) -> HealthWord {
        HealthWord {
            health1: self.sat_health[12].health,
            health2: self.sat_health[13].health,
            health3: self.sat_health[14].health,
            health4: self.sat_health[15].health,
        }
    }

    fn set_word7(&mut self, word: HealthWord) {
        self.sat_health[12].health = word.health1;
        self.sat_health[13].health = word.health2;
        self.sat_health[14].health = word.health3;
        self.sat_health[15].health = word.health4;
    }

    fn word8(&self) -> HealthWord {
        HealthWord {
            health1: self.sat_health[16].health,
            health2: self.sat_health[17].health,
            health3: self.sat_health[18].health,
            health4: self.sat_health[19].health,
        }
    }

    fn set_word8(&mut self, word: HealthWord) {
        self.sat_health[16].health = word.health1;
        self.sat_health[17].health = word.health2;
        self.sat_health[18].health = word.health3;
        self.sat_health[19].health = word.health4;
    }

    fn word9(&self) -> HealthWord {
        HealthWord {
            health1: self.sat_health[20].health,
            health2: self.sat_health[21].health,
            health3: self.sat_health[22].health,
            health4: self.sat_health[23].health,
        }
    }

    fn set_word9(&mut self, word: HealthWord) {
        self.sat_health[20].health = word.health1;
        self.sat_health[21].health = word.health2;
        self.sat_health[22].health = word.health3;
        self.sat_health[23].health = word.health4;
    }

    fn word10(&self) -> Word10 {
        Word10 {
            reserved: self.reserved,
            spare: self.spare,
        }
    }

    fn set_word10(&mut self, word: Word10) {
        self.reserved = word.reserved;
        self.spare = word.spare;
    }

    /// Encodes this [GpsQzssAlmanachStatus] as a burst of 8 [GpsDataWord]s.
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

impl Word3 {
    /// Interprets this [GpsDataWord] as [Word3].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();

        let data_id = ((value & WORD3_DID_MASK) >> WORD3_DID_SHIFT) as u8;
        let sv_id = ((value & WORD3_SVID_MASK) >> WORD3_SVID_SHIFT) as u8;
        let toa_seconds = ((value & WORD3_TOA_MASK) >> WORD3_TOA_SHIFT) as u8;
        let week = ((value & WORD3_WEEK_MASK) >> WORD3_WEEK_SHIFT) as u8;

        Self {
            data_id,
            sv_id,
            toa_seconds,
            week,
        }
    }

    /// Encodes this [Word3] as [GpsDataWord].
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;
        value |= (self.week as u32) << WORD3_WEEK_SHIFT;
        value |= ((self.toa_seconds & 0x3) as u32) << WORD3_TOA_SHIFT;
        value |= ((self.sv_id & 0x07) as u32) << WORD3_SVID_SHIFT;
        value |= ((self.data_id & 0x03) as u32) << WORD3_DID_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

impl HealthWord {
    /// Interprets this [GpsDataWord] as [HealthWord]
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();

        Self {
            health1: (value & 0x0000_003f) as u8,
            health2: ((value & 0x0000_0fc0) >> 6) as u8,
            health3: ((value & 0x0003_f000) >> 12) as u8,
            health4: ((value & 0x00fc_0000) >> 18) as u8,
        }
    }

    /// Encodes this [HealthWord] to [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = (self.health1 as u32);
        value |= (self.health2 as u32) << 6;
        value |= (self.health3 as u32) << 12;
        value |= (self.health4 as u32) << 18;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

impl Word10 {
    /// Interprets this [GpsDataWord] as [Word10].
    pub fn from_word(word: GpsDataWord) -> Self {
        let value = word.value();
        let spare = (value & WORD10_SPARE_MASK) >> WORD10_SPARE_SHIFT;
        let reserved = ((value & WORD10_RES_MASK) >> WORD10_RES_SHIFT) as u8;
        Self { reserved, spare }
    }

    /// Encodes this [Word10] as [GpsDataWord]
    pub fn to_word(&self) -> GpsDataWord {
        let mut value = 0;
        value |= (self.spare & 0x0007_ffff) << WORD10_SPARE_SHIFT;
        value |= ((self.reserved & 0x7) as u32) << WORD10_RES_SHIFT;
        value <<= 2;
        GpsDataWord::from(value)
    }
}

#[cfg(test)]
mod frame1 {
    use super::*;

    #[test]
    fn dword3() {
        for dword3 in [
            Word3 {
                data_id: 0,
                sv_id: 0,
                toa_seconds: 10,
                week: 0,
            },
            Word3 {
                data_id: 1,
                sv_id: 10,
                toa_seconds: 10,
                week: 28,
            },
        ] {
            let gps_word = dword3.to_word();
            let decoded = Word3::from_word(gps_word);
            assert_eq!(decoded, dword3);
            assert_eq!(
                decoded.to_word(),
                gps_word,
                "Reciprocal failed for {:?}",
                dword3
            );
        }
    }

    #[test]
    fn dword10() {
        for dword10 in [
            Word10 {
                reserved: 0,
                spare: 0,
            },
            Word10 {
                reserved: 1,
                spare: 10,
            },
            Word10 {
                reserved: 2,
                spare: 10,
            },
        ] {
            let gps_word = dword10.to_word();
            let decoded = Word10::from_word(gps_word);
            assert_eq!(decoded, dword10);
            assert_eq!(decoded.to_word(), gps_word);
        }
    }

    #[test]
    fn encoding() {
        for (data_id, sv_id, toa_seconds, week, health_even, health_odd, reserved, spare) in [
            (0, 1, 10, 20, 4, 5, 1, 2),
            (1, 2, 20, 10, 5, 4, 2, 3),
            (1, 2, 20, 10, 5, 4, 3, 5),
        ] {
            let sat_health: [GpsQzssSatelliteHealth; 24] = Default::default();

            for i in 0..24 {
                if i % 2 == 0 {
                    sat_health[i] = health_even;
                } else {
                    sat_health[i] = health_odd;
                }
            }

            let frame = GpsQzssAlmanachStatus {
                data_id,
                sv_id,
                toa_seconds,
                week,
                sat_health,
                reserved,
                spare,
            };

            let words = frame1.to_words();

            let decoded = GpsQzssAlmanachStatus::from_words(&words);
            assert_eq!(decoded, frame);
        }
    }
}
