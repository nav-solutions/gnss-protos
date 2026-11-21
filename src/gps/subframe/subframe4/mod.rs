//! Subframe 4 pages interpretation.

use crate::{
    gps::{
        subframe::{peek_sv_id, GpsQzssReservedSubframe},
        GpsQzssAlmanach, GPS_SUBFRAME_DATA_BYTES, GPS_SUPERFRAME_ITEMS, GPS_WORD_DATA_BITS,
    },
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

mod config_health;
mod iono_utc;
mod nmct;
mod special;

pub use config_health::{GpsQzssConfigHealth, GPS_CONFIG_HEALTH_SATELLITES, GPS_CONFIG_SATELLITES};
pub use iono_utc::GpsQzssIonosphereUtc;
pub use nmct::{GpsQzssNmct, GPS_NMCT_ERD_COUNT, GPS_NMCT_ERD_NOT_AVAILABLE};
pub use special::{GpsQzssSpecialMessage, GPS_SPECIAL_MESSAGE_CHARS};

/// SV (page) ID transmitted on each page (1 to 25) of subframe 4
/// (IS-GPS-200 Table 20-V). Almanac pages (2 to 5, 7 to 10) nominally
/// carry satellites 25 to 32, but may carry any satellite, or a dummy (0).
pub const GPS_FRAME4_PAGE_SV_IDS: [u8; GPS_SUPERFRAME_ITEMS] = [
    57, 25, 26, 27, 28, 57, 29, 30, 31, 32, 57, 62, 52, 53, 54, 57, 55, 56, 58, 59, 57, 60, 61, 62,
    63,
];

/// [GpsQzssSubframe4] interprets a subframe 4 page, according to its SV (page) ID.
///
/// | Pages                 | SV ID       | Content                       |
/// |-----------------------|-------------|-------------------------------|
/// | 2-5, 7-10             | 25-32       | [GpsQzssAlmanach]             |
/// | 13                    | 52          | [GpsQzssNmct]                 |
/// | 17                    | 55          | [GpsQzssSpecialMessage]       |
/// | 18                    | 56          | [GpsQzssIonosphereUtc]        |
/// | 25                    | 63          | [GpsQzssConfigHealth]         |
/// | 1, 6, 11, 12, 14-16, 19-24 | 53, 54, 57-62 | [GpsQzssReservedSubframe] |
///
/// The page number is not transmitted: it is defined by the time of transmission
/// (see [crate::gps::GpsQzssFrame::page_number]), several pages sharing the same SV ID.
/// The content type is defined by the SV ID, so a page is interpreted on its own.
///
/// Any SV ID that is not expected in subframe 4 (including dummy pages, SV ID 0)
/// is preserved as a [GpsQzssReservedSubframe]: all data bits are preserved, so
/// [GpsQzssSubframe4] is encoded back bit for bit.
#[derive(Debug, Copy, Clone)]
pub enum GpsQzssSubframe4 {
    /// [GpsQzssAlmanach] (pages 2 to 5 and 7 to 10)
    Almanach(GpsQzssAlmanach),

    /// [GpsQzssNmct] (page 13)
    Nmct(GpsQzssNmct),

    /// [GpsQzssSpecialMessage] (page 17)
    SpecialMessage(GpsQzssSpecialMessage),

    /// [GpsQzssIonosphereUtc] (page 18)
    IonosphereUtc(GpsQzssIonosphereUtc),

    /// [GpsQzssConfigHealth] (page 25)
    ConfigHealth(GpsQzssConfigHealth),

    /// [GpsQzssReservedSubframe] (reserved, spare and dummy pages)
    Reserved(GpsQzssReservedSubframe),
}

crate::bits::impl_eq_by_encoding!(GpsQzssSubframe4, GPS_SUBFRAME_DATA_BYTES);

impl Default for GpsQzssSubframe4 {
    /// Builds a dummy page
    fn default() -> Self {
        Self::Reserved(Default::default())
    }
}

impl Message<BigEndian> for GpsQzssSubframe4 {
    fn encoding_size(&self) -> usize {
        24 // TODO improve defs
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssSubframe4 {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Almanach(page) => stream.write(page),
            Self::Nmct(page) => stream.write(page),
            Self::SpecialMessage(page) => stream.write(page),
            Self::IonosphereUtc(page) => stream.write(page),
            Self::ConfigHealth(page) => stream.write(page),
            Self::Reserved(page) => stream.write(page),
        }
    }
}

impl BitRead<'_, BigEndian> for GpsQzssSubframe4 {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]),
    /// and interprets them according to the SV (page) ID.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        match peek_sv_id(stream)? {
            1..=32 => Ok(Self::Almanach(stream.read()?)),
            GpsQzssNmct::SV_ID => Ok(Self::Nmct(stream.read()?)),
            GpsQzssSpecialMessage::SV_ID => Ok(Self::SpecialMessage(stream.read()?)),
            GpsQzssIonosphereUtc::SV_ID => Ok(Self::IonosphereUtc(stream.read()?)),
            GpsQzssConfigHealth::SV_ID => Ok(Self::ConfigHealth(stream.read()?)),
            _ => Ok(Self::Reserved(stream.read()?)),
        }
    }
}

impl GpsQzssSubframe4 {
    /// Returns the SV (page) ID of this page.
    pub fn sv_id(&self) -> u8 {
        match self {
            Self::Almanach(page) => page.sv_id,
            Self::Nmct(_) => GpsQzssNmct::SV_ID,
            Self::SpecialMessage(_) => GpsQzssSpecialMessage::SV_ID,
            Self::IonosphereUtc(_) => GpsQzssIonosphereUtc::SV_ID,
            Self::ConfigHealth(_) => GpsQzssConfigHealth::SV_ID,
            Self::Reserved(page) => page.sv_id,
        }
    }

    /// Returns the data ID (2 bits) of this page.
    pub fn data_id(&self) -> u8 {
        match self {
            Self::Almanach(page) => page.data_id,
            Self::Nmct(page) => page.data_id,
            Self::SpecialMessage(page) => page.data_id,
            Self::IonosphereUtc(page) => page.data_id,
            Self::ConfigHealth(page) => page.data_id,
            Self::Reserved(page) => page.data_id,
        }
    }

    /// Returns the SV (page) ID transmitted on given page (1..=25), as defined by IS-GPS-200.
    pub fn page_sv_id(page: u8) -> Option<u8> {
        if (1..=GPS_SUPERFRAME_ITEMS as u8).contains(&page) {
            Some(GPS_FRAME4_PAGE_SV_IDS[page as usize - 1])
        } else {
            None
        }
    }

    /// Returns true if this page is an almanac page (2 to 5, 7 to 10).
    pub fn is_almanach_page(page: u8) -> bool {
        matches!(page, 2..=5 | 7..=10)
    }

    /// Returns the pages (1..=25) on which this content is nominally transmitted.
    pub fn pages(&self) -> impl Iterator<Item = u8> {
        let sv_id = self.sv_id();
        (1..=GPS_SUPERFRAME_ITEMS as u8).filter(move |page| Self::page_sv_id(*page) == Some(sv_id))
    }

    /// Returns true if this content may be transmitted on given page (1..=25):
    /// either the nominal SV ID of this page, or any almanac (or dummy)
    /// on almanac pages.
    pub fn matches_page(&self, page: u8) -> bool {
        match Self::page_sv_id(page) {
            Some(sv_id) => {
                sv_id == self.sv_id()
                    || (Self::is_almanach_page(page)
                        && (self.sv_id() == 0 || matches!(self, Self::Almanach(_))))
            },
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

    /// Unwraps self as [GpsQzssNmct] (if feasible)
    pub fn as_nmct(&self) -> Option<&GpsQzssNmct> {
        match self {
            Self::Nmct(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssSpecialMessage] (if feasible)
    pub fn as_special_message(&self) -> Option<&GpsQzssSpecialMessage> {
        match self {
            Self::SpecialMessage(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssIonosphereUtc] (if feasible)
    pub fn as_ionosphere_utc(&self) -> Option<&GpsQzssIonosphereUtc> {
        match self {
            Self::IonosphereUtc(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssConfigHealth] (if feasible)
    pub fn as_config_health(&self) -> Option<&GpsQzssConfigHealth> {
        match self {
            Self::ConfigHealth(page) => Some(page),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssReservedSubframe] (if feasible)
    pub fn as_reserved(&self) -> Option<&GpsQzssReservedSubframe> {
        match self {
            Self::Reserved(page) => Some(page),
            _ => None,
        }
    }

    /// Builds the nominal content of given page (1..=25), for testing purposes.
    #[cfg(test)]
    pub(crate) fn model(page: u8) -> Self {
        match Self::page_sv_id(page).unwrap_or(0) {
            sv_id @ 1..=32 => Self::Almanach(
                GpsQzssAlmanach::model()
                    .with_sv_id(sv_id)
                    .with_eccentricity(0.001 * sv_id as f64),
            ),
            GpsQzssNmct::SV_ID => Self::Nmct(GpsQzssNmct::model()),
            GpsQzssSpecialMessage::SV_ID => Self::SpecialMessage(GpsQzssSpecialMessage::model()),
            GpsQzssIonosphereUtc::SV_ID => Self::IonosphereUtc(GpsQzssIonosphereUtc::model()),
            GpsQzssConfigHealth::SV_ID => Self::ConfigHealth(GpsQzssConfigHealth::model()),
            sv_id => Self::Reserved(GpsQzssReservedSubframe::model(sv_id)),
        }
    }
}

#[cfg(test)]
mod frame4 {
    use super::*;

    #[test]
    fn pagination_id() {
        for (page, sv_id) in [
            (1, 57),
            (2, 25),
            (4, 27),
            (5, 28),
            (8, 30),
            (13, 52),
            (17, 55),
            (18, 56),
            (24, 62),
            (25, 63),
        ] {
            assert_eq!(
                GpsQzssSubframe4::page_sv_id(page),
                Some(sv_id),
                "page {}",
                page
            );
        }

        assert_eq!(GpsQzssSubframe4::page_sv_id(0), None);
        assert_eq!(GpsQzssSubframe4::page_sv_id(26), None);

        // every page model matches its own page
        for page in 1..=25u8 {
            let model = GpsQzssSubframe4::model(page);
            assert!(model.matches_page(page), "page {}", page);
            assert!(model.pages().any(|p| p == page), "page {}", page);
            assert_eq!(model.sv_id(), GpsQzssSubframe4::page_sv_id(page).unwrap());
        }

        // SV ID 57 is transmitted on 5 pages, 62 on 2 pages
        let reserved = GpsQzssSubframe4::model(1);
        assert_eq!(reserved.pages().collect::<Vec<_>>(), vec![1, 6, 11, 16, 21]);
        assert_eq!(
            GpsQzssSubframe4::model(12).pages().collect::<Vec<_>>(),
            vec![12, 24]
        );

        // almanac pages accept any almanac, and dummy pages
        let almanach = GpsQzssSubframe4::Almanach(GpsQzssAlmanach::model().with_sv_id(3));
        assert!(almanach.matches_page(2));
        assert!(almanach.matches_page(10));
        assert!(!almanach.matches_page(13));
        assert!(GpsQzssSubframe4::default().matches_page(7));
        assert!(!GpsQzssSubframe4::default().matches_page(18));
    }

    #[test]
    fn reciprocal() {
        for page in 1..=25u8 {
            let model = GpsQzssSubframe4::model(page);

            let mut bytes = [0u8; 24];
            assert_eq!(model.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssSubframe4::decode(&bytes).unwrap();
            assert_eq!(decoded, model, "page {}", page);
            assert_eq!(
                core::mem::discriminant(&decoded),
                core::mem::discriminant(&model),
                "page {}",
                page
            );

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes, "page {}", page);
        }
    }

    #[test]
    fn interpretation() {
        let mut bytes = [0u8; 24];

        for (sv_id, almanach, nmct, special, iono, config) in [
            (0, false, false, false, false, false),
            (1, true, false, false, false, false),
            (25, true, false, false, false, false),
            (32, true, false, false, false, false),
            (33, false, false, false, false, false),
            (51, false, false, false, false, false),
            (52, false, true, false, false, false),
            (55, false, false, true, false, false),
            (56, false, false, false, true, false),
            (57, false, false, false, false, false),
            (63, false, false, false, false, true),
        ] {
            GpsQzssReservedSubframe::model(sv_id)
                .encode(&mut bytes)
                .unwrap();
            let page = GpsQzssSubframe4::decode(&bytes).unwrap();

            assert_eq!(page.sv_id(), sv_id);
            assert_eq!(page.data_id(), 1);
            assert_eq!(page.as_almanach().is_some(), almanach, "sv {}", sv_id);
            assert_eq!(page.as_nmct().is_some(), nmct, "sv {}", sv_id);
            assert_eq!(page.as_special_message().is_some(), special, "sv {}", sv_id);
            assert_eq!(page.as_ionosphere_utc().is_some(), iono, "sv {}", sv_id);
            assert_eq!(page.as_config_health().is_some(), config, "sv {}", sv_id);
            assert_eq!(
                page.as_reserved().is_some(),
                !(almanach || nmct || special || iono || config)
            );

            // whatever the interpretation, the bits are preserved
            let mut reencoded = [0u8; 24];
            page.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes, "sv {}", sv_id);
        }
    }

    #[test]
    fn random_pages_bit_exact() {
        // any 192 bits, whatever the SV ID, in subframe 4 or 5:
        // interpreted and re-encoded bit for bit (t-bits aside)
        let mut seed = 0x2468_ACE1u32;

        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as u8
        };

        for sv_id in 0..64u8 {
            for _ in 0..32 {
                let mut bytes = [0u8; 24];

                for byte in bytes.iter_mut() {
                    *byte = rand();
                }

                bytes[0] = (bytes[0] & 0xc0) | sv_id;
                bytes[23] &= 0xfc;

                let page4 = GpsQzssSubframe4::decode(&bytes).unwrap();
                assert_eq!(page4.sv_id(), sv_id);

                let mut reencoded = [0u8; 24];
                page4.encode(&mut reencoded).unwrap();
                assert_eq!(reencoded, bytes, "SF4 SV ID {}: {:?}", sv_id, page4);

                let page5 = crate::gps::GpsQzssSubframe5::decode(&bytes).unwrap();
                assert_eq!(page5.sv_id(), sv_id);

                page5.encode(&mut reencoded).unwrap();
                assert_eq!(reencoded, bytes, "SF5 SV ID {}: {:?}", sv_id, page5);
            }
        }
    }

    #[test]
    fn bit_true_equality() {
        // a reserved page that carries an almanac equals the almanac
        let almanach = GpsQzssSubframe4::model(2);
        let mut bytes = [0u8; 24];
        almanach.encode(&mut bytes).unwrap();

        let mut data = [0u8; 23];
        data.copy_from_slice(&bytes[1..]);

        let reserved = GpsQzssSubframe4::Reserved(
            GpsQzssReservedSubframe::default()
                .with_sv_id(25)
                .with_data(data),
        );

        assert_eq!(reserved, almanach);
        assert_ne!(GpsQzssSubframe4::model(3), almanach);
    }
}
