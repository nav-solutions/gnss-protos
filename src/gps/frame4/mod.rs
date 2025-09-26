use crate::gps::{
    GpsDataWord,
    GpsError,
};

mod almanach;
mod klobuchar_utc;
mod raw;
mod health;

pub use almanach::*;
pub use klobuchar_utc::*;
pub use raw::*;
pub use health::*;

/// [GpsQzssFrame4] interpretations.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub enum GpsQzssFrame4 {
    /// Reserved [Page1] message
    Page1(GpsQzssFrame4Raw),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page2(GpsQzssFrame4Alm1),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page3(GpsQzssFrame4Alm2),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page4(GpsQzssFrame4Alm3),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page5(GpsQzssFrame4Alm4),
    
    /// Reserved [Page6] message
    Page6(GpsQzssFrame4Raw),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page7(GpsQzssFrame4Alm5),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page8(GpsQzssFrame4Alm6),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page9(GpsQzssFrame4Alm7),
    
    /// Almanach data for satellite 25 (included) through 32 (included).
    Page10(GpsQzssFrame4Alm8),

    /// Reserved [Page11] message
    Page11(GpsQzssFrame4Raw),
    
    /// Reserved [Page12] message
    Page12(GpsQzssFrame4Raw),
    
    /// Spare [Page13] message
    Page13(GpsQzssFrame4Raw),
    
    /// Spare [Page14] message
    Page14(GpsQzssFrame4Raw),
    
    /// Spare [Page15] message
    Page15(GpsQzssFrame4Raw),

    /// Reserved [Page16] message
    Page16(GpsQzssFrame4Raw),

    /// Special [Page17] message
    Page17(GpsQzssFrame4Raw),

    /// [GpsQzssFrame4KlobucharUtc] correction message
    Page18(GpsQzssFrame4KlobucharUtc),

    /// Reserved [Page19] message
    Page19(GpsQzssFrame4Raw),

    /// Reserved [Page20] message
    Page20(GpsQzssFrame4Raw),

    /// Reserved [Page21] message
    Page21(GpsQzssFrame4Raw),

    /// Reserved [Page22] message
    Page22(GpsQzssFrame4Raw),

    /// Reserved [Page23] message
    Page23(GpsQzssFrame4Raw),

    /// Reserved [Page24] message
    Page24(GpsQzssFrame4Raw),

    /// Satellite configuration for 32 satellites
    Page25(GpsQzssSatHealth),
}

impl GpsQzssFrame4 {
    /// Returns the page identification number of this [GpsQzssFrame4] interpretation
    pub fn page_id(&self) -> u8 {
        match self {
            Self::Page1(_) => 1,
            Self::Page2(_) => 2,
            Self::Page3(_) => 3,
            Self::Page4(_) => 4,
            Self::Page5(_) => 5,
            Self::Page6(_) => 6,
            Self::Page7(_) => 7,
            Self::Page8(_) => 8,
            Self::Page9(_) => 9,
            Self::Page10(_) => 10,
            Self::Page11(_) => 11,
            Self::Page12(_) => 12,
            Self::Page13(_) => 13,
            Self::Page14(_) => 14,
            Self::Page15(_) => 15,
            Self::Page16(_) => 16,
            Self::Page17(_) => 17,
            Self::Page18(_) => 18,
            Self::Page19(_) => 19,
            Self::Page20(_) => 20,
            Self::Page21(_) => 21,
            Self::Page22(_) => 22,
            Self::Page23(_) => 23,
            Self::Page24(_) => 24,
            Self::Page25(_) => 25,
        }
    }

    pub(crate) fn to_word(&self) -> GpsDataWord {
        let mut word = match self {
            Self::Page1(frame) => frame.to_word(),
            Self::Page2(frame) => frame.to_word(),
            Self::Page3(frame) => frame.to_word(),
            Self::Page4(frame) => frame.to_word(),
            Self::Page5(frame) => frame.to_word(),
            Self::Page6(frame) => frame.to_word(),
            Self::Page7(frame) => frame.to_word(),
            Self::Page8(frame) => frame.to_word(),
            Self::Page9(frame) => frame.to_word(),
            Self::Page10(frame) => frame.to_word(),
            Self::Page11(frame) => frame.to_word(),
            Self::Page12(frame) => frame.to_word(),
            Self::Page13(frame) => frame.to_word(),
            Self::Page14(frame) => frame.to_word(),
            Self::Page15(frame) => frame.to_word(),
            Self::Page16(frame) => frame.to_word(),
            Self::Page17(frame) => frame.to_word(),
            Self::Page18(frame) => frame.to_word(),
            Self::Page19(frame) => frame.to_word(),
            Self::Page20(frame) => frame.to_word(),
            Self::Page21(frame) => frame.to_word(),
            Self::Page22(frame) => frame.to_word(),
            Self::Page23(frame) => frame.to_word(),
            Self::Page24(frame) => frame.to_word(),
            Self::Page25(frame) => frame.to_word(),
        };

        word.with_page_id(self.page_id())
    }

    /// Decodes this [GpsDataWord] as [GpsQzssFrame4] page,
    /// if page is correct and supported.
    pub(crate) fn from_word(word: GpsDataWord) -> Result<Self, GpsError> {
        match word.page_id() {
            1 => Ok(Self::Page1(GpsQzssFrame4Raw::from_word(word))),
            2 => Ok(Self::Page2(GpsQzssFrame4Alm1::from_word(word))),
            3 => Ok(Self::Page3(GpsQzssFrame4Alm2::from_word(word))),
            4 => Ok(Self::Page4(GpsQzssFrame4Alm3::from_word(word))),
            5 => Ok(Self::Page5(GpsQzssFrame4Alm4::from_word(word))),
            6 => Ok(Self::Page6(GpsQzssFrame4Raw::from_word(word))),
            7 => Ok(Self::Page7(GpsQzssFrame4Raw::from_word(word))),
            8 => Ok(Self::Page8(GpsQzssFrame4Raw::from_word(word))),
            9 => Ok(Self::Page9(GpsQzssFrame4Raw::from_word(word))),
            10 => Ok(Self::Page10(GpsQzssFrame4Raw::from_word(word))),
            11 => Ok(Self::Page11(GpsQzssFrame4Raw::from_word(word))),
            12 => Ok(Self::Page12(GpsQzssFrame4Raw::from_word(word))),
            13 => Ok(Self::Page13(GpsQzssFrame4Raw::from_word(word))),
            14 => Ok(Self::Page14(GpsQzssFrame4Raw::from_word(word))),
            15 => Ok(Self::Page15(GpsQzssFrame4Raw::from_word(word))),
            16 => Ok(Self::Page16(GpsQzssFrame4Raw::from_word(word))),
            17 => Ok(Self::Page17(GpsQzssFrame4Raw::from_word(word))),
            18 => Ok(Self::Page18(GpsQzssFrame4KlobucharUtc::from_word(word))),
            19 => Ok(Self::Page19(GpsQzssFrame4Raw::from_word(word))),
            20 => Ok(Self::Page20(GpsQzssFrame4Raw::from_word(word))),
            21 => Ok(Self::Page21(GpsQzssFrame4Raw::from_word(word))),
            22 => Ok(Self::Page22(GpsQzssFrame4Raw::from_word(word))),
            23 => Ok(Self::Page23(GpsQzssFrame4Raw::from_word(word))),
            24 => Ok(Self::Page24(GpsQzssFrame4Raw::from_word(word))),
            25 => Ok(Self::Page25(GpsQzssSatHealth::from_word(word))),
            _ => Err(GpsError::InvalidPage),
        }
    }
}

#[cfg(test)]
mod frame4 {

    #[test]
    fn pagination_id() {
        for (page, id) in [
            (GpsQzssFrame4::Page1(Default::default()), 1),
            (GpsQzssFrame4::Page2(Default::default()), 2),
            (GpsQzssFrame4::Page4(Default::default()), 4),
            (GpsQzssFrame4::Page5(Default::default()), 5),
            (GpsQzssFrame4::Page8(Default::default()), 8),
            (GpsQzssFrame4::Page13(Default::default()), 13),
            (GpsQzssFrame4::Page24(Default::default()), 24),
            (GpsQzssFrame4::Page25(Default::default()), 25),
        ] {
            assert_eq!(page.page_id(), id, "returned invalid page-id for {}", page);
        }
    }
}
