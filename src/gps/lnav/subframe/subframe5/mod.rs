//! Subframe 5 pages interpretation.

use crate::{
    gps::lnav::{
        subframe::{peek_sv_id, GpsQzssLnavReservedSubframe},
        GpsQzssLnavAlmanach, GPS_LNAV_SUBFRAME_DATA_BYTES, GPS_LNAV_SUPERFRAME_ITEMS,
        GPS_LNAV_WORD_DATA_BITS,
    },
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

mod health;

pub use health::{GpsQzssLnavAlmanachHealth, GPS_LNAV_ALMANACH_HEALTH_SATELLITES};

/// [GpsQzssLnavSubframe5] interprets a subframe 5 page, according to its SV (page) ID.
///
/// | Pages  | SV ID | Content                    |
/// |--------|-------|----------------------------|
/// | 1-24   | 1-24  | [GpsQzssLnavAlmanach]          |
/// | 25     | 51    | [GpsQzssLnavAlmanachHealth]    |
///
/// Any other SV ID (including dummy pages, SV ID 0) is preserved
/// as a [GpsQzssLnavReservedPage]: all data bits are preserved, so
/// [GpsQzssLnavSubframe5] is encoded back bit for bit.
#[derive(Debug, Copy, Clone)]
pub enum GpsQzssLnavSubframe5 {
    /// [GpsQzssLnavAlmanach] (pages 1 to 24)
    Almanach(GpsQzssLnavAlmanach),

    /// [GpsQzssLnavAlmanachHealth] (page 25)
    Health(GpsQzssLnavAlmanachHealth),

    /// [GpsQzssLnavReservedPage] (dummy and unexpected pages)
    Reserved(GpsQzssLnavReservedSubframe),
}

crate::bits::impl_eq_by_encoding!(GpsQzssLnavSubframe5, GPS_LNAV_SUBFRAME_DATA_BYTES);

impl Default for GpsQzssLnavSubframe5 {
    /// Builds a dummy page
    fn default() -> Self {
        Self::Reserved(Default::default())
    }
}

impl Message<BigEndian> for GpsQzssLnavSubframe5 {
    fn encoding_size(&self) -> usize {
        24 // TODO improve defs
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_LNAV_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavSubframe5 {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Almanach(page) => stream.write(page),
            Self::Health(page) => stream.write(page),
            Self::Reserved(page) => stream.write(page),
        }
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavSubframe5 {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]),
    /// and interprets them according to the SV (page) ID.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        match peek_sv_id(stream)? {
            1..=32 => Ok(Self::Almanach(stream.read()?)),
            GpsQzssLnavAlmanachHealth::SV_ID => Ok(Self::Health(stream.read()?)),
            _ => Ok(Self::Reserved(stream.read()?)),
        }
    }
}

impl GpsQzssLnavSubframe5 {
    /// Returns the SV (page) ID of this page.
    pub fn sv_id(&self) -> u8 {
        match self {
            Self::Almanach(page) => page.sv_id,
            Self::Health(_) => GpsQzssLnavAlmanachHealth::SV_ID,
            Self::Reserved(page) => page.sv_id,
        }
    }

    /// Returns the data ID (2 bits) of this page.
    pub fn data_id(&self) -> u8 {
        match self {
            Self::Almanach(page) => page.data_id,
            Self::Health(page) => page.data_id,
            Self::Reserved(page) => page.data_id,
        }
    }

    /// Returns the SV (page) ID transmitted on given page (1..=25), as defined by IS-GPS-200.
    pub fn page_sv_id(page: u8) -> Option<u8> {
        match page {
            1..=24 => Some(page),
            25 => Some(GpsQzssLnavAlmanachHealth::SV_ID),
            _ => None,
        }
    }

    /// Returns the pages (1..=25) on which this content is nominally transmitted.
    pub fn pages(&self) -> impl Iterator<Item = u8> {
        let sv_id = self.sv_id();
        (1..=GPS_LNAV_SUPERFRAME_ITEMS as u8)
            .filter(move |page| Self::page_sv_id(*page) == Some(sv_id))
    }

    /// Returns true if this content may be transmitted on given page (1..=25):
    /// the nominal SV ID of this page, or a dummy page (SV ID 0) on almanac pages.
    pub fn matches_page(&self, page: u8) -> bool {
        match Self::page_sv_id(page) {
            Some(sv_id) => sv_id == self.sv_id() || (page <= 24 && self.sv_id() == 0),
            None => false,
        }
    }

    /// Unwraps self as [GpsQzssLnavAlmanach] (if feasible)
    pub fn as_almanach(&self) -> Option<&GpsQzssLnavAlmanach> {
        match self {
            Self::Almanach(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavAlmanachHealth] (if feasible)
    pub fn as_health(&self) -> Option<&GpsQzssLnavAlmanachHealth> {
        match self {
            Self::Health(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavReservedPage] (if feasible)
    pub fn as_reserved(&self) -> Option<&GpsQzssLnavReservedSubframe> {
        match self {
            Self::Reserved(page) => Some(page),
            _ => None,
        }
    }

    /// Builds the nominal content of given page (1..=25), for testing purposes.
    #[cfg(test)]
    pub(crate) fn model(page: u8) -> Self {
        match page {
            1..=24 => Self::Almanach(
                GpsQzssLnavAlmanach::model()
                    .with_sv_id(page)
                    .with_eccentricity(0.0005 * page as f64),
            ),
            _ => Self::Health(GpsQzssLnavAlmanachHealth::model()),
        }
    }
}

#[cfg(test)]
mod frame5 {
    use super::*;

    #[test]
    fn pagination_id() {
        for (page, sv_id) in [
            (1, 1),
            (2, 2),
            (4, 4),
            (5, 5),
            (8, 8),
            (13, 13),
            (24, 24),
            (25, 51),
        ] {
            assert_eq!(
                GpsQzssLnavSubframe5::page_sv_id(page),
                Some(sv_id),
                "page {}",
                page
            );
            let model = GpsQzssLnavSubframe5::model(page);
            assert_eq!(model.sv_id(), sv_id);
            assert!(model.matches_page(page));
            assert_eq!(model.pages().collect::<Vec<_>>(), vec![page]);
        }

        assert_eq!(GpsQzssLnavSubframe5::page_sv_id(0), None);
        assert_eq!(GpsQzssLnavSubframe5::page_sv_id(26), None);

        // dummy pages replace almanacs
        assert!(GpsQzssLnavSubframe5::default().matches_page(3));
        assert!(!GpsQzssLnavSubframe5::default().matches_page(25));
        assert!(!GpsQzssLnavSubframe5::model(3).matches_page(4));
    }

    #[test]
    fn reciprocal() {
        for page in 1..=25u8 {
            let model = GpsQzssLnavSubframe5::model(page);

            let mut bytes = [0u8; 24];
            assert_eq!(model.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssLnavSubframe5::decode(&bytes).unwrap();
            assert_eq!(decoded, model, "page {}", page);
            assert_eq!(
                core::mem::discriminant(&decoded),
                core::mem::discriminant(&model)
            );

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes, "page {}", page);
        }
    }

    #[test]
    fn interpretation() {
        let mut bytes = [0u8; 24];

        for (sv_id, almanach, health) in [
            (0, false, false),
            (1, true, false),
            (24, true, false),
            (32, true, false),
            (51, false, true),
            (52, false, false),
            (63, false, false),
        ] {
            GpsQzssLnavReservedSubframe::model(sv_id)
                .encode(&mut bytes)
                .unwrap();
            let page = GpsQzssLnavSubframe5::decode(&bytes).unwrap();

            assert_eq!(page.sv_id(), sv_id);
            assert_eq!(page.as_almanach().is_some(), almanach, "sv {}", sv_id);
            assert_eq!(page.as_health().is_some(), health, "sv {}", sv_id);
            assert_eq!(page.as_reserved().is_some(), !(almanach || health));

            let mut reencoded = [0u8; 24];
            page.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes, "sv {}", sv_id);
        }
    }
}
