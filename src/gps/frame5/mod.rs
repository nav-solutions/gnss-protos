use crate::gps::{
    GpsDataWord,
    GpsError,
    GpsQzssAlmanach,
};

mod status;
pub use status::*;

/// [GpsQzssFrame5] message interpretation
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub enum GpsQzssFrame5 {
    /// [GpsQzssAlmanach] for satellite #1
    Page1(GpsQzssAlmanach),
    
    /// [GpsQzssAlmanach] for satellite #2
    Page2(GpsQzssAlmanach),
    
    /// [GpsQzssAlmanach] for satellite #3
    Page3(GpsQzssAlmanach),
    
    /// [GpsQzssAlmanach] for satellite #4
    Page4(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #5
    Page5(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #6
    Page6(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #7
    Page7(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #8
    Page8(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #9
    Page9(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #10
    Page10(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #11
    Page11(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #12
    Page12(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #13
    Page13(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #14
    Page14(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #15
    Page15(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #16
    Page16(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #17
    Page17(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #18
    Page18(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #19
    Page19(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #20
    Page20(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #21
    Page21(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #22
    Page22(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #23
    Page23(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #24
    Page24(GpsQzssAlmanach),

    /// [GpsQzssAlmanachStatus] gives satellite #1 (included) through #24 (included)
    /// health status and other general Almanach infos.
    Page25(GpsQzssAlmanachStatus),
}

impl GpsQzssFrame5 {
    /// Returns the page identification number of this [GpsQzssFrame5] interpretation
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

    /// Decodes this [GpsDataWord] as [GpsQzssFrame5],
    /// if page is correct and supported.
    pub(crate) fn from_word(word: GpsDataWord) -> Result<Self, GpsError> {
        match word.page_id() {
            1 => Ok(Self::Page1(GpsQzssAlmanach::from_word(word))),
            2 => Ok(Self::Page2(GpsQzssAlmanach::from_word(word))),
            3 => Ok(Self::Page3(GpsQzssAlmanach::from_word(word))),
            4 => Ok(Self::Page4(GpsQzssAlmanach::from_word(word))),
            5 => Ok(Self::Page5(GpsQzssAlmanach::from_word(word))),
            6 => Ok(Self::Page6(GpsQzssAlmanach::from_word(word))),
            7 => Ok(Self::Page7(GpsQzssAlmanach::from_word(word))),
            8 => Ok(Self::Page8(GpsQzssAlmanach::from_word(word))),
            9 => Ok(Self::Page9(GpsQzssAlmanach::from_word(word))),
            10 => Ok(Self::Page10(GpsQzssAlmanach::from_word(word))),
            11 => Ok(Self::Page11(GpsQzssAlmanach::from_word(word))),
            12 => Ok(Self::Page12(GpsQzssAlmanach::from_word(word))),
            13 => Ok(Self::Page13(GpsQzssAlmanach::from_word(word))),
            14 => Ok(Self::Page14(GpsQzssAlmanach::from_word(word))),
            15 => Ok(Self::Page15(GpsQzssAlmanach::from_word(word))),
            16 => Ok(Self::Page16(GpsQzssAlmanach::from_word(word))),
            17 => Ok(Self::Page17(GpsQzssAlmanach::from_word(word))),
            18 => Ok(Self::Page18(GpsQzssAlmanach::from_word(word))),
            19 => Ok(Self::Page19(GpsQzssAlmanach::from_word(word))),
            20 => Ok(Self::Page20(GpsQzssAlmanach::from_word(word))),
            21 => Ok(Self::Page21(GpsQzssAlmanach::from_word(word))),
            22 => Ok(Self::Page22(GpsQzssAlmanach::from_word(word))),
            23 => Ok(Self::Page23(GpsQzssAlmanach::from_word(word))),
            24 => Ok(Self::Page24(GpsQzssAlmanach::from_word(word))),
            25 => Ok(Self::Page25(GpsQzssAlmanachStatus::from_word(word))),
            _ => Err(GpsError::InvalidPage),
        }
    }
}

#[cfg(test)]
mod frame5 {

    #[test]
    fn pagination_id() {
        for (page, id) in [
            (GpsQzssFrame5::Page1(Default::default()), 1),
            (GpsQzssFrame5::Page2(Default::default()), 2),
            (GpsQzssFrame5::Page4(Default::default()), 4),
            (GpsQzssFrame5::Page5(Default::default()), 5),
            (GpsQzssFrame5::Page8(Default::default()), 8),
            (GpsQzssFrame5::Page13(Default::default()), 13),
            (GpsQzssFrame5::Page24(Default::default()), 24),
            (GpsQzssFrame5::Page25(Default::default()), 25),
        ] {
            assert_eq!(page.page_id(), id, "returned invalid page-id for {}", page);
        }
    }
}
