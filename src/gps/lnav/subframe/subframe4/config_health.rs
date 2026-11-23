//! Satellites configuration and health (subframe 4, page 25, SV ID 63).

use crate::{
    gps::lnav::{GpsQzssLnavSatelliteHealth, GPS_LNAV_CONFIG_DID, GPS_LNAV_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// Number of satellites described by the configuration page (PRN 1 to 32).
pub const GPS_LNAV_CONFIG_SATELLITES: usize = 32;

/// Number of satellites whose health is described by the configuration page (PRN 25 to 32).
pub const GPS_LNAV_CONFIG_HEALTH_SATELLITES: usize = 8;

/// First PRN whose health is described by the configuration page.
const FIRST_HEALTH_PRN: u8 = 25;

/// [GpsQzssLnavConfigHealth] (IS-GPS-200 §20.3.3.5.1.4), transmitted in subframe 4 page 25
/// (SV ID 63): anti-spoofing flag and configuration of satellites 1 to 32,
/// health of satellites 25 to 32.
///
/// The health of satellites 1 to 24 is transmitted in subframe 5 page 25
/// ([crate::gps::GpsQzssAlmanachHealth]).
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssLnavConfigHealth {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// 4-bit configuration of satellites 1 to 32: the MSB is the anti-spoofing flag,
    /// the 3 LSB the satellite configuration (`000`: reserved or no satellite,
    /// `001`: Block II/IIA/IIR, `010`: Block IIR-M, `011`: Block IIF, `100`: GPS III...).
    pub configurations: [u8; GPS_LNAV_CONFIG_SATELLITES],

    /// [GpsQzssLnavSatelliteHealth] of satellites 25 to 32
    pub health: [GpsQzssLnavSatelliteHealth; GPS_LNAV_CONFIG_HEALTH_SATELLITES],

    /// Reserved bits (2 bits) of word 8
    pub reserved_word8: u8,

    /// Reserved bits (4 bits) of word 10
    pub reserved_word10: u8,
}

crate::bits::impl_eq_by_encoding!(GpsQzssLnavConfigHealth, 24); // TODO improve defs

impl Default for GpsQzssLnavConfigHealth {
    fn default() -> Self {
        Self {
            data_id: GPS_LNAV_CONFIG_DID,
            configurations: [0; GPS_LNAV_CONFIG_SATELLITES],
            health: Default::default(),
            reserved_word8: 0,
            reserved_word10: 0,
        }
    }
}

impl Message<BigEndian> for GpsQzssLnavConfigHealth {
    fn encoding_size(&self) -> usize {
        24 // TODO improve defs
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_LNAV_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavConfigHealth {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        // word 3: SV 1-4, words 4 to 7: SV 5-28, word 8: SV 29-32
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(Self::SV_ID, 6)?;

        for configuration in self.configurations.iter() {
            stream.write_int(configuration & 0xf, 4)?;
        }

        // word 8: reserved, SV 25 health
        stream.write_int(self.reserved_word8 & 0x3, 2)?;

        // words 8 to 10: SV 25 to 32 health
        for health in self.health.iter() {
            stream.write(health)?;
        }

        // word 10
        stream.write_int(self.reserved_word10 & 0xf, 4)?;
        stream.write_int(0u8, 2) // t-bits: solved when the frame is encoded
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavConfigHealth {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;

        if sv_id != Self::SV_ID {
            return Err(BitError::UnmatchedDiscriminant {
                discriminant: sv_id as usize,
                enum_name: "GpsQzssLnavConfigHealth".to_string(),
            });
        }

        let mut configurations = [0u8; GPS_LNAV_CONFIG_SATELLITES];

        for configuration in configurations.iter_mut() {
            *configuration = stream.read_int::<u8>(4)?;
        }

        let reserved_word8 = stream.read_int::<u8>(2)?;

        let mut health = [GpsQzssLnavSatelliteHealth::default(); GPS_LNAV_CONFIG_HEALTH_SATELLITES];

        for value in health.iter_mut() {
            *value = stream.read()?;
        }

        let reserved_word10 = stream.read_int::<u8>(4)?;
        let _t = stream.read_int::<u8>(2)?;

        Ok(Self {
            data_id,
            configurations,
            health,
            reserved_word8,
            reserved_word10,
        })
    }
}

impl GpsQzssLnavConfigHealth {
    /// SV (page) ID of the configuration and health page.
    pub const SV_ID: u8 = 63;

    /// Returns the 4-bit configuration of satellite `prn` (1..=32).
    pub fn configuration(&self, prn: u8) -> Option<u8> {
        if (1..=GPS_LNAV_CONFIG_SATELLITES as u8).contains(&prn) {
            Some(self.configurations[prn as usize - 1])
        } else {
            None
        }
    }

    /// Returns true if anti-spoofing is activated on satellite `prn` (1..=32).
    pub fn anti_spoofing(&self, prn: u8) -> Option<bool> {
        self.configuration(prn).map(|c| c & 0x8 > 0)
    }

    /// Returns the 3-bit configuration code (signal capabilities, satellite block)
    /// of satellite `prn` (1..=32).
    pub fn satellite_configuration(&self, prn: u8) -> Option<u8> {
        self.configuration(prn).map(|c| c & 0x7)
    }

    /// Returns the [GpsQzssLnavSatelliteHealth] of satellite `prn` (25..=32).
    pub fn satellite_health(&self, prn: u8) -> Option<GpsQzssLnavSatelliteHealth> {
        if (FIRST_HEALTH_PRN..FIRST_HEALTH_PRN + GPS_LNAV_CONFIG_HEALTH_SATELLITES as u8)
            .contains(&prn)
        {
            Some(self.health[(prn - FIRST_HEALTH_PRN) as usize])
        } else {
            None
        }
    }

    /// Copies and returns [GpsQzssLnavConfigHealth] with updated configuration of satellite `prn` (1..=32):
    /// anti-spoofing flag and 3-bit configuration code.
    pub fn with_configuration(mut self, prn: u8, anti_spoofing: bool, configuration: u8) -> Self {
        if (1..=GPS_LNAV_CONFIG_SATELLITES as u8).contains(&prn) {
            let mut value = configuration & 0x7;
            if anti_spoofing {
                value |= 0x8;
            }
            self.configurations[prn as usize - 1] = value;
        }
        self
    }

    /// Copies and returns [GpsQzssLnavConfigHealth] with updated health of satellite `prn` (25..=32).
    pub fn with_satellite_health(mut self, prn: u8, health: GpsQzssLnavSatelliteHealth) -> Self {
        if (FIRST_HEALTH_PRN..FIRST_HEALTH_PRN + GPS_LNAV_CONFIG_HEALTH_SATELLITES as u8)
            .contains(&prn)
        {
            self.health[(prn - FIRST_HEALTH_PRN) as usize] = health;
        }
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        let mut page = Self::default();
        for prn in 1..=32u8 {
            page = page.with_configuration(prn, prn % 7 != 0, (prn % 4) + 1);
        }
        page.with_satellite_health(27, GpsQzssLnavSatelliteHealth::from(0x3f))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bits::read_bits;

    #[test]
    fn reciprocal() {
        for page in [
            GpsQzssLnavConfigHealth::default(),
            GpsQzssLnavConfigHealth::model(),
            GpsQzssLnavConfigHealth {
                reserved_word8: 2,
                reserved_word10: 0xa,
                ..GpsQzssLnavConfigHealth::model()
            },
        ] {
            let mut bytes = [0u8; 24];
            assert_eq!(page.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssLnavConfigHealth::decode(&bytes).unwrap();
            assert_eq!(decoded, page);
            assert_eq!(decoded.configurations, page.configurations);
            assert_eq!(decoded.health, page.health);

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes);
        }
    }

    #[test]
    fn layout() {
        let mut page = GpsQzssLnavConfigHealth {
            reserved_word8: 0x1,
            reserved_word10: 0x9,
            ..Default::default()
        };

        for prn in 1..=32u8 {
            page = page.with_configuration(prn, prn % 2 == 0, prn % 8);
        }

        for prn in 25..=32u8 {
            page = page.with_satellite_health(prn, GpsQzssLnavSatelliteHealth::from(prn));
        }

        let mut bytes = [0u8; 24];
        page.encode(&mut bytes).unwrap();

        // word 3
        assert_eq!(read_bits(&bytes, 0, 2), Some(1));
        assert_eq!(read_bits(&bytes, 2, 6), Some(63));

        // 4 bits per satellite, from bit 8 (word 3) to bit 136 (word 8)
        for prn in 1..=32usize {
            let expected = ((prn % 8) | if prn % 2 == 0 { 8 } else { 0 }) as u32;
            assert_eq!(read_bits(&bytes, 8 + 4 * (prn - 1), 4), Some(expected));
        }

        // word 8: 2 reserved bits, then SV 25 health
        assert_eq!(read_bits(&bytes, 136, 2), Some(1));
        assert_eq!(read_bits(&bytes, 138, 6), Some(25));

        // word 9: SV 26 to 29, word 10: SV 30 to 32
        for prn in 26..=32usize {
            assert_eq!(read_bits(&bytes, 144 + 6 * (prn - 26), 6), Some(prn as u32));
        }

        // word 10: 4 reserved bits, t-bits
        assert_eq!(read_bits(&bytes, 186, 4), Some(9));
        assert_eq!(read_bits(&bytes, 190, 2), Some(0));
    }

    #[test]
    fn accessors() {
        let page = GpsQzssLnavConfigHealth::model();

        assert_eq!(page.anti_spoofing(1), Some(true));
        assert_eq!(page.anti_spoofing(7), Some(false));
        assert_eq!(page.satellite_configuration(3), Some(4));
        assert_eq!(page.satellite_configuration(4), Some(1));
        assert_eq!(page.configuration(0), None);
        assert_eq!(page.configuration(33), None);

        assert!(page.satellite_health(25).unwrap().is_healthy());
        assert!(!page.satellite_health(27).unwrap().is_healthy());
        assert_eq!(page.satellite_health(24), None);
        assert_eq!(page.satellite_health(33), None);
    }
}
