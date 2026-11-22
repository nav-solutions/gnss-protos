//! Almanac reference and health of satellites 1 to 24 (subframe 5, page 25, SV ID 51).

use crate::{
    bits::write_unsigned,
    gps::lnav::{GpsQzssLnavSatelliteHealth, GPS_LNAV_CONFIG_DID, GPS_LNAV_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// Number of satellites whose health is described by subframe 5 page 25 (PRN 1 to 24).
pub const GPS_LNAV_ALMANACH_HEALTH_SATELLITES: usize = 24; // TODO: improve defs

/// [GpsQzssLnavAlmanachHealth] (IS-GPS-200 §20.3.3.5.1.3), transmitted in subframe 5 page 25
/// (SV ID 51): almanac reference time and week, health of satellites 1 to 24.
///
/// The health of satellites 25 to 32 is transmitted in subframe 4 page 25
/// ([crate::gps::GpsQzssConfigHealth]).
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssLnavAlmanachHealth {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// t_oa: almanac reference time (in seconds of week, multiple of 4096)
    pub toa_seconds: u32,

    /// WN_a: almanac reference week (8 LSB of the week number)
    pub wna: u8,

    /// [GpsQzssLnavSatelliteHealth] of satellites 1 to 24
    pub health: [GpsQzssLnavSatelliteHealth; GPS_LNAV_ALMANACH_HEALTH_SATELLITES],

    /// Reserved and spare bits (22 bits) of word 10
    pub reserved: u32,
}

crate::bits::impl_eq_by_encoding!(GpsQzssLnavAlmanachHealth, 24); // TODO: improve defs

impl Default for GpsQzssLnavAlmanachHealth {
    fn default() -> Self {
        Self {
            data_id: GPS_LNAV_CONFIG_DID,
            toa_seconds: 0,
            wna: 0,
            health: Default::default(),
            reserved: 0,
        }
    }
}

impl Message<BigEndian> for GpsQzssLnavAlmanachHealth {
    fn encoding_size(&self) -> usize {
        24 // TODO: improve defs
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_LNAV_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavAlmanachHealth {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        // word 3
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(Self::SV_ID, 6)?;
        write_unsigned(stream, (self.toa_seconds >> 12) as i64, 8)?;
        stream.write_int(self.wna, 8)?;

        // words 4 to 9: 4 satellites per word
        for health in self.health.iter() {
            stream.write(health)?;
        }

        // word 10
        stream.write_int(self.reserved & 0x3f_ffff, 22)?;
        stream.write_int(0u8, 2) // t-bits: solved when the frame is encoded
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavAlmanachHealth {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;

        if sv_id != Self::SV_ID {
            return Err(BitError::UnmatchedDiscriminant {
                discriminant: sv_id as usize,
                enum_name: "GpsQzssLnavAlmanachHealth".to_string(),
            });
        }

        let toa_seconds = stream.read_int::<u32>(8)? << 12;
        let wna = stream.read_int::<u8>(8)?;

        let mut health =
            [GpsQzssLnavSatelliteHealth::default(); GPS_LNAV_ALMANACH_HEALTH_SATELLITES];

        for value in health.iter_mut() {
            *value = stream.read()?;
        }

        let reserved = stream.read_int::<u32>(22)?;
        let _t = stream.read_int::<u8>(2)?;

        Ok(Self {
            data_id,
            toa_seconds,
            wna,
            health,
            reserved,
        })
    }
}

impl GpsQzssLnavAlmanachHealth {
    /// SV (page) ID of subframe 5 page 25.
    pub const SV_ID: u8 = 51;

    /// Returns the [GpsQzssLnavSatelliteHealth] of satellite `prn` (1..=24).
    pub fn satellite_health(&self, prn: u8) -> Option<GpsQzssLnavSatelliteHealth> {
        if (1..=GPS_LNAV_ALMANACH_HEALTH_SATELLITES as u8).contains(&prn) {
            Some(self.health[prn as usize - 1])
        } else {
            None
        }
    }

    /// Copies and returns [GpsQzssLnavAlmanachHealth] with updated health of satellite `prn` (1..=24).
    pub fn with_satellite_health(mut self, prn: u8, health: GpsQzssLnavSatelliteHealth) -> Self {
        if (1..=GPS_LNAV_ALMANACH_HEALTH_SATELLITES as u8).contains(&prn) {
            self.health[prn as usize - 1] = health;
        }
        self
    }

    /// Copies and returns [GpsQzssLnavAlmanachHealth] with updated reference time
    /// (in seconds of week, multiple of 4096) and week (8 LSB are kept).
    pub fn with_reference(mut self, toa_seconds: u32, week: u32) -> Self {
        self.toa_seconds = toa_seconds;
        self.wna = (week & 0xff) as u8;
        self
    }

    /// Returns the complete almanac reference week, given a complete week number `week`
    /// close to the reference (±127 weeks), since WN_a is only transmitted on 8 bits.
    pub fn almanach_week(&self, week: u32) -> u32 {
        let diff = (self.wna as i64 - week as i64).rem_euclid(256);
        let diff = if diff >= 128 { diff - 256 } else { diff };
        (week as i64 + diff).max(0) as u32
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_reference(589_824, 2091)
            .with_satellite_health(21, GpsQzssLnavSatelliteHealth::from(0x3f))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bits::read_bits;

    #[test]
    fn reciprocal() {
        for page in [
            GpsQzssLnavAlmanachHealth::default(),
            GpsQzssLnavAlmanachHealth::model(),
            GpsQzssLnavAlmanachHealth {
                reserved: 0x2a_aaaa,
                ..GpsQzssLnavAlmanachHealth::model()
            },
        ] {
            let mut bytes = [0u8; 24];
            assert_eq!(page.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssLnavAlmanachHealth::decode(&bytes).unwrap();
            assert_eq!(decoded, page);
            assert_eq!(decoded.health, page.health);
            assert_eq!(decoded.toa_seconds, page.toa_seconds);
            assert_eq!(decoded.wna, page.wna);

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes);
        }
    }

    #[test]
    fn layout() {
        let mut page = GpsQzssLnavAlmanachHealth::default().with_reference(0xAB << 12, 0x1CD);
        page.reserved = 0x15_5555;

        for prn in 1..=24u8 {
            page = page.with_satellite_health(prn, GpsQzssLnavSatelliteHealth::from(prn + 30));
        }

        let mut bytes = [0u8; 24];
        page.encode(&mut bytes).unwrap();

        // word 3: data ID, SV ID, toa, WNa
        assert_eq!(read_bits(&bytes, 0, 2), Some(1));
        assert_eq!(read_bits(&bytes, 2, 6), Some(51));
        assert_eq!(read_bits(&bytes, 8, 8), Some(0xAB));
        assert_eq!(read_bits(&bytes, 16, 8), Some(0xCD));

        // words 4 to 9: 4 x 6-bit health per word
        for prn in 1..=24usize {
            assert_eq!(
                read_bits(&bytes, 24 + 6 * (prn - 1), 6),
                Some(prn as u32 + 30)
            );
        }

        // word 10
        assert_eq!(read_bits(&bytes, 168, 22), Some(0x15_5555));
        assert_eq!(read_bits(&bytes, 190, 2), Some(0));
    }

    #[test]
    fn accessors() {
        let page = GpsQzssLnavAlmanachHealth::model();
        assert!(page.satellite_health(1).unwrap().is_healthy());
        assert!(!page.satellite_health(21).unwrap().is_healthy());
        assert_eq!(page.satellite_health(0), None);
        assert_eq!(page.satellite_health(25), None);

        assert_eq!(page.wna, (2091 & 0xff) as u8);
        assert_eq!(page.almanach_week(2091), 2091);
        assert_eq!(page.almanach_week(2095), 2091);
        assert_eq!(page.almanach_week(2085), 2091);
        assert_eq!(page.almanach_week(2091 + 127), 2091);
    }
}
