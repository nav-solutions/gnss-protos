//! Navigation Message Correction Table (subframe 4, page 13, SV ID 52).

use crate::{
    gps::{GPS_LNAV_CONFIG_DID, GPS_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// Number of Estimated Range Deviations in the NMCT.
pub const GPS_NMCT_ERD_COUNT: usize = 30;

/// ERD value (`100000` binary) indicating that no correction is available.
pub const GPS_NMCT_ERD_NOT_AVAILABLE: i8 = -32;

/// ERD scale factor (in meters).
const ERD_SCALE_M: f64 = 0.3;

/// [GpsQzssNmct] Navigation Message Correction Table (IS-GPS-200 §20.3.3.5.1.9),
/// transmitted in subframe 4 page 13 (SV ID 52).
///
/// The NMCT contains 30 Estimated Range Deviations (ERD), one for each of the other
/// satellites, in ascending PRN order, the transmitting satellite being skipped.
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssNmct {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// Availability Indicator (2 bits): `00` the NMCT is unencrypted and available,
    /// `01` encrypted, `10` / `11` not available to authorized users either.
    pub availability: u8,

    /// Estimated Range Deviations: 6-bit two's complement values, 0.3 m LSB.
    /// [GPS_NMCT_ERD_NOT_AVAILABLE] means that no correction is available.
    pub erd: [i8; GPS_NMCT_ERD_COUNT],
}

crate::bits::impl_eq_by_encoding!(GpsQzssNmct, 24); // TODO improve defs

impl Default for GpsQzssNmct {
    fn default() -> Self {
        Self {
            data_id: GPS_LNAV_CONFIG_DID,
            availability: 0,
            erd: [GPS_NMCT_ERD_NOT_AVAILABLE; GPS_NMCT_ERD_COUNT],
        }
    }
}

impl Message<BigEndian> for GpsQzssNmct {
    fn encoding_size(&self) -> usize {
        24
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssNmct {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(Self::SV_ID, 6)?;
        stream.write_int(self.availability & 0x3, 2)?;

        for erd in self.erd.iter() {
            let erd = (*erd).clamp(-32, 31);
            stream.write_int((erd as u8) & 0x3f, 6)?;
        }

        stream.write_int(0u8, 2) // t-bits: solved when the frame is encoded
    }
}

impl BitRead<'_, BigEndian> for GpsQzssNmct {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;

        if sv_id != Self::SV_ID {
            return Err(BitError::UnmatchedDiscriminant {
                discriminant: sv_id as usize,
                enum_name: "GpsQzssNmct".to_string(),
            });
        }

        let availability = stream.read_int::<u8>(2)?;

        let mut erd = [0i8; GPS_NMCT_ERD_COUNT];

        for value in erd.iter_mut() {
            // 6-bit two's complement
            *value = ((stream.read_int::<u8>(6)? << 2) as i8) >> 2;
        }

        let _t = stream.read_int::<u8>(2)?;

        Ok(Self {
            data_id,
            availability,
            erd,
        })
    }
}

impl GpsQzssNmct {
    /// SV (page) ID of the NMCT page.
    pub const SV_ID: u8 = 52;

    /// Returns true if the NMCT is available to all users (unencrypted).
    pub fn is_available(&self) -> bool {
        self.availability == 0
    }

    /// Returns the Estimated Range Deviation (in meters) of the i-th entry (0..30),
    /// if a correction is available.
    pub fn erd_meters(&self, index: usize) -> Option<f64> {
        let erd = *self.erd.get(index)?;

        if erd == GPS_NMCT_ERD_NOT_AVAILABLE {
            None
        } else {
            Some(erd as f64 * ERD_SCALE_M)
        }
    }

    /// Returns the index of the ERD that applies to satellite `prn` (1..=32),
    /// for an NMCT transmitted by satellite `transmitter` (1..=32).
    /// The transmitting satellite does not correct itself.
    pub fn erd_index(transmitter: u8, prn: u8) -> Option<usize> {
        if !(1..=32).contains(&transmitter) || !(1..=32).contains(&prn) || prn == transmitter {
            return None;
        }

        let index = if prn < transmitter { prn - 1 } else { prn - 2 } as usize;

        if index < GPS_NMCT_ERD_COUNT {
            Some(index)
        } else {
            None
        }
    }

    /// Copies and returns [GpsQzssNmct] with updated availability indicator (2 bits).
    pub fn with_availability(mut self, availability: u8) -> Self {
        self.availability = availability & 0x3;
        self
    }

    /// Copies and returns [GpsQzssNmct] with the i-th ERD updated (in meters).
    /// `None` marks the correction as not available.
    pub fn with_erd_meters(mut self, index: usize, erd_m: Option<f64>) -> Self {
        if let Some(value) = self.erd.get_mut(index) {
            *value = match erd_m {
                Some(erd_m) => (erd_m / ERD_SCALE_M).round().clamp(-31.0, 31.0) as i8,
                None => GPS_NMCT_ERD_NOT_AVAILABLE,
            };
        }
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        let mut nmct = Self::default();
        for i in 0..GPS_NMCT_ERD_COUNT {
            nmct = nmct.with_erd_meters(i, Some((i as f64 - 15.0) * 0.6));
        }
        nmct.with_erd_meters(7, None)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bits::read_bits;

    #[test]
    fn reciprocal() {
        for nmct in [
            GpsQzssNmct::default(),
            GpsQzssNmct::model(),
            GpsQzssNmct::model().with_availability(2),
        ] {
            let mut bytes = [0u8; 24];
            assert_eq!(nmct.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssNmct::decode(&bytes).unwrap();
            assert_eq!(decoded, nmct);
            assert_eq!(decoded.erd, nmct.erd);
            assert_eq!(decoded.availability, nmct.availability);

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes);
        }
    }

    #[test]
    fn layout() {
        // word 3: data ID, SV ID, AI, ERD1, ERD2, 2 MSB of ERD3
        let nmct = GpsQzssNmct::default()
            .with_availability(1)
            .with_erd_meters(0, Some(0.3))
            .with_erd_meters(1, Some(-0.3))
            .with_erd_meters(29, Some(9.3));

        let mut bytes = [0u8; 24];
        nmct.encode(&mut bytes).unwrap();

        assert_eq!(read_bits(&bytes, 0, 2), Some(1));
        assert_eq!(read_bits(&bytes, 2, 6), Some(52));
        assert_eq!(read_bits(&bytes, 8, 2), Some(1));
        assert_eq!(read_bits(&bytes, 10, 6), Some(1));
        assert_eq!(read_bits(&bytes, 16, 6), Some(0x3f));
        assert_eq!(read_bits(&bytes, 22, 6), Some(0x20));
        // ERD30: last 6 bits before the t-bits
        assert_eq!(read_bits(&bytes, 184, 6), Some(31));
        assert_eq!(read_bits(&bytes, 190, 2), Some(0));
    }

    #[test]
    fn erd_values() {
        let nmct = GpsQzssNmct::model();
        assert_eq!(nmct.erd_meters(7), None);
        assert!((nmct.erd_meters(0).unwrap() - -9.0).abs() < 1e-9);
        assert!((nmct.erd_meters(15).unwrap()).abs() < 1e-9);
        assert_eq!(nmct.erd_meters(30), None);

        // saturation
        let nmct = GpsQzssNmct::default().with_erd_meters(0, Some(100.0));
        assert_eq!(nmct.erd[0], 31);
        let nmct = GpsQzssNmct::default().with_erd_meters(0, Some(-100.0));
        assert_eq!(nmct.erd[0], -31);
    }

    #[test]
    fn erd_index() {
        assert_eq!(GpsQzssNmct::erd_index(5, 1), Some(0));
        assert_eq!(GpsQzssNmct::erd_index(5, 4), Some(3));
        assert_eq!(GpsQzssNmct::erd_index(5, 5), None);
        assert_eq!(GpsQzssNmct::erd_index(5, 6), Some(4));
        assert_eq!(GpsQzssNmct::erd_index(1, 31), Some(29));
        assert_eq!(GpsQzssNmct::erd_index(32, 30), Some(29));
        assert_eq!(GpsQzssNmct::erd_index(32, 31), None);
        // 31 other satellites but 30 ERDs
        assert_eq!(GpsQzssNmct::erd_index(1, 32), None);
        assert_eq!(GpsQzssNmct::erd_index(0, 3), None);
        assert_eq!(GpsQzssNmct::erd_index(3, 33), None);
    }

    #[test]
    fn wrong_sv_id() {
        let mut bytes = [0u8; 24];
        GpsQzssNmct::default().encode(&mut bytes).unwrap();
        bytes[0] = 0x40 | 53;
        assert!(GpsQzssNmct::decode(&bytes).is_err());
    }
}
