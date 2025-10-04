use crate::gps::{GpsDataWord, GpsError, GpsQzssAlmanach, GPS_WORDS_PER_FRAME};

mod status;
pub use status::*;

/// [GpsQzssFrame5] message interpretation
#[derive(Debug, Copy, Clone, PartialEq)]
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

impl Default for GpsQzssFrame5 {
    /// Builds a default null [GpsQzssFrame5::Page1].
    fn default() -> Self {
        Self::Page1(Default::default())
    }
}

impl GpsQzssFrame5 {
    #[cfg(test)]
    /// Creates a [GpsQzssFrame5] for testing purposes
    pub fn model() -> Self {
        Self::Page1(GpsQzssAlmanach::model())
    }

    /// Decodes a burst of 8 [GpsDataWord]s as [GpsQzssFrame5].
    /// Page must be correctly supported.
    pub fn from_words(words: &[GpsDataWord]) -> Result<Self, GpsError> {
        // grab page number
        let value = words[0].value();
        let page_id = ((value & 0x00c0_0000) >> 22) as u8;

        match page_id {
            1 => Ok(Self::Page1(GpsQzssAlmanach::from_words(words))),
            2 => Ok(Self::Page2(GpsQzssAlmanach::from_words(words))),
            3 => Ok(Self::Page3(GpsQzssAlmanach::from_words(words))),
            4 => Ok(Self::Page4(GpsQzssAlmanach::from_words(words))),
            5 => Ok(Self::Page5(GpsQzssAlmanach::from_words(words))),
            6 => Ok(Self::Page6(GpsQzssAlmanach::from_words(words))),
            7 => Ok(Self::Page7(GpsQzssAlmanach::from_words(words))),
            8 => Ok(Self::Page8(GpsQzssAlmanach::from_words(words))),
            9 => Ok(Self::Page9(GpsQzssAlmanach::from_words(words))),
            10 => Ok(Self::Page10(GpsQzssAlmanach::from_words(words))),
            11 => Ok(Self::Page11(GpsQzssAlmanach::from_words(words))),
            12 => Ok(Self::Page12(GpsQzssAlmanach::from_words(words))),
            13 => Ok(Self::Page13(GpsQzssAlmanach::from_words(words))),
            14 => Ok(Self::Page14(GpsQzssAlmanach::from_words(words))),
            15 => Ok(Self::Page15(GpsQzssAlmanach::from_words(words))),
            16 => Ok(Self::Page16(GpsQzssAlmanach::from_words(words))),
            17 => Ok(Self::Page17(GpsQzssAlmanach::from_words(words))),
            18 => Ok(Self::Page18(GpsQzssAlmanach::from_words(words))),
            19 => Ok(Self::Page19(GpsQzssAlmanach::from_words(words))),
            20 => Ok(Self::Page20(GpsQzssAlmanach::from_words(words))),
            21 => Ok(Self::Page21(GpsQzssAlmanach::from_words(words))),
            22 => Ok(Self::Page22(GpsQzssAlmanach::from_words(words))),
            23 => Ok(Self::Page23(GpsQzssAlmanach::from_words(words))),
            24 => Ok(Self::Page24(GpsQzssAlmanach::from_words(words))),
            25 => Ok(Self::Page25(GpsQzssAlmanachStatus::from_words(words))),
            _ => Err(GpsError::UnknownFrame5Page),
        }
    }

    /// Encodes this [GpsQzssFrame5] as a burst of 8 [GpsDataWord]s
    pub fn to_words(&self) -> [GpsDataWord; GPS_WORDS_PER_FRAME - 2] {
        Default::default()
    }

    /// Returns the page identification number of this [GpsQzssFrame5] interpretation
    pub fn page_number(&self) -> u8 {
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

    /// Returns the next page number, according to the
    /// standardized message rotation.
    pub fn next_page(&self) -> u8 {
        match self {
            Self::Page1(_) => 2,
            Self::Page2(_) => 3,
            Self::Page3(_) => 4,
            Self::Page4(_) => 5,
            Self::Page5(_) => 6,
            Self::Page6(_) => 7,
            Self::Page7(_) => 8,
            Self::Page8(_) => 9,
            Self::Page9(_) => 10,
            Self::Page10(_) => 11,
            Self::Page11(_) => 12,
            Self::Page12(_) => 13,
            Self::Page13(_) => 14,
            Self::Page14(_) => 15,
            Self::Page15(_) => 16,
            Self::Page16(_) => 17,
            Self::Page17(_) => 18,
            Self::Page18(_) => 19,
            Self::Page19(_) => 20,
            Self::Page20(_) => 21,
            Self::Page21(_) => 22,
            Self::Page22(_) => 23,
            Self::Page23(_) => 24,
            Self::Page24(_) => 25,
            Self::Page25(_) => 1,
        }
    }
}

#[cfg(test)]
mod frame5 {
    use super::GpsQzssFrame5;

    #[test]
    fn pagination() {
        for (page, id, next_id) in [
            (GpsQzssFrame5::Page1(Default::default()), 1, 2),
            (GpsQzssFrame5::Page2(Default::default()), 2, 3),
            (GpsQzssFrame5::Page4(Default::default()), 4, 5),
            (GpsQzssFrame5::Page5(Default::default()), 5, 6),
            (GpsQzssFrame5::Page8(Default::default()), 8, 9),
            (GpsQzssFrame5::Page13(Default::default()), 13, 14),
            (GpsQzssFrame5::Page24(Default::default()), 24, 25),
            (GpsQzssFrame5::Page25(Default::default()), 25, 1),
        ] {
            assert_eq!(
                page.page_number(),
                id,
                "returned invalid page-id for {:?}",
                page
            );
            assert_eq!(page.next_page(), next_id, "invalid frame rotation");
        }
    }
}
