//! Subframe 5 pages interpretation.

use crate::{
    gps::{
        GpsQzssAlmanach, GpsQzssReservedPage, GPS_QZSS_ALMANACH_PAGES, GPS_SUBFRAME_DATA_BYTES,
        GPS_WORD_DATA_BITS,
    },
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

mod health;

pub use health::{GpsQzssAlmanachHealth, GPS_ALMANACH_HEALTH_SATELLITES};

/// [GpsQzssFrame5] interprets a subframe 5 page, according to its SV (page) ID.
///
/// | Pages  | SV ID | Content                    |
/// |--------|-------|----------------------------|
/// | 1-24   | 1-24  | [GpsQzssAlmanach]          |
/// | 25     | 51    | [GpsQzssAlmanachHealth]    |
///
/// Any other SV ID (including dummy pages, SV ID 0) is preserved
/// as a [GpsQzssReservedPage]: all data bits are preserved, so
/// [GpsQzssFrame5] is encoded back bit for bit.
#[derive(Debug, Copy, Clone)]
pub enum GpsQzssFrame5 {
    /// [GpsQzssAlmanach] (pages 1 to 24)
    Almanach(GpsQzssAlmanach),

    /// [GpsQzssAlmanachHealth] (page 25)
    Health(GpsQzssAlmanachHealth),

    /// [GpsQzssReservedPage] (dummy and unexpected pages)
    Reserved(GpsQzssReservedPage),
}

crate::bits::impl_eq_by_encoding!(GpsQzssFrame5, GPS_SUBFRAME_DATA_BYTES);

impl Default for GpsQzssFrame5 {
    /// Builds a dummy page
    fn default() -> Self {
        Self::Reserved(Default::default())
    }
}

impl Message<BigEndian> for GpsQzssFrame5 {
    fn encoding_size(&self) -> usize {
        GPS_SUBFRAME_DATA_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame5 {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Almanach(page) => stream.write(page),
            Self::Health(page) => stream.write(page),
            Self::Reserved(page) => stream.write(page),
        }
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame5 {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]),
    /// and interprets them according to the SV (page) ID.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        match crate::gps::peek_sv_id(stream)? {
            1..=32 => Ok(Self::Almanach(stream.read()?)),
            GpsQzssAlmanachHealth::SV_ID => Ok(Self::Health(stream.read()?)),
            _ => Ok(Self::Reserved(stream.read()?)),
        }
    }
}

impl GpsQzssFrame5 {
    /// Returns the SV (page) ID of this page.
    pub fn sv_id(&self) -> u8 {
        match self {
            Self::Almanach(page) => page.sv_id,
            Self::Health(_) => GpsQzssAlmanachHealth::SV_ID,
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
            25 => Some(GpsQzssAlmanachHealth::SV_ID),
            _ => None,
        }
    }

    /// Returns the pages (1..=25) on which this content is nominally transmitted.
    pub fn pages(&self) -> impl Iterator<Item = u8> {
        let sv_id = self.sv_id();
        (1..=GPS_QZSS_ALMANACH_PAGES as u8)
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

    /// Unwraps self as [GpsQzssAlmanach] (if feasible)
    pub fn as_almanach(&self) -> Option<&GpsQzssAlmanach> {
        match self {
            Self::Almanach(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssAlmanachHealth] (if feasible)
    pub fn as_health(&self) -> Option<&GpsQzssAlmanachHealth> {
        match self {
            Self::Health(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssReservedPage] (if feasible)
    pub fn as_reserved(&self) -> Option<&GpsQzssReservedPage> {
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
                GpsQzssAlmanach::model()
                    .with_sv_id(page)
                    .with_eccentricity(0.0005 * page as f64),
            ),
            _ => Self::Health(GpsQzssAlmanachHealth::model()),
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
                GpsQzssFrame5::page_sv_id(page),
                Some(sv_id),
                "page {}",
                page
            );
            let model = GpsQzssFrame5::model(page);
            assert_eq!(model.sv_id(), sv_id);
            assert!(model.matches_page(page));
            assert_eq!(model.pages().collect::<Vec<_>>(), vec![page]);
        }

        assert_eq!(GpsQzssFrame5::page_sv_id(0), None);
        assert_eq!(GpsQzssFrame5::page_sv_id(26), None);

        // dummy pages replace almanacs
        assert!(GpsQzssFrame5::default().matches_page(3));
        assert!(!GpsQzssFrame5::default().matches_page(25));
        assert!(!GpsQzssFrame5::model(3).matches_page(4));
    }

    #[test]
    fn reciprocal() {
        for page in 1..=25u8 {
            let model = GpsQzssFrame5::model(page);

            let mut bytes = [0u8; 24];
            assert_eq!(model.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssFrame5::decode(&bytes).unwrap();
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
            GpsQzssReservedPage::model(sv_id)
                .encode(&mut bytes)
                .unwrap();
            let page = GpsQzssFrame5::decode(&bytes).unwrap();

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
