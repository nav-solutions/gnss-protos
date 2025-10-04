use crate::gps::{
    GpsDataWord, GpsError, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrame5,
    GpsQzssFrameId, GPS_WORDS_PER_FRAME,
};

/// GPS / QZSS Interpreted subframes
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsQzssSubframe {
    /// GPS Ephemeris Frame #1
    Ephemeris1(GpsQzssFrame1),

    /// GPS Ephemeris Frame #2
    Ephemeris2(GpsQzssFrame2),

    /// GPS Ephemeris Frame #3
    Ephemeris3(GpsQzssFrame3),

    /// GPS Almanach Frame #5
    Almanach5(GpsQzssFrame5),
}

impl Default for GpsQzssSubframe {
    /// Builds a default [GpsQzssSubFrame::Ephemeris1]
    fn default() -> Self {
        Self::Ephemeris1(Default::default())
    }
}

impl GpsQzssSubframe {
    /// Generates a realistic frame model for testing purposes
    #[cfg(test)]
    pub fn model(frame_id: GpsQzssFrameId) -> Self {
        match frame_id {
            GpsQzssFrameId::Ephemeris1 => Self::Ephemeris1(GpsQzssFrame1::model()),
            GpsQzssFrameId::Ephemeris2 => Self::Ephemeris2(GpsQzssFrame2::model()),
            GpsQzssFrameId::Ephemeris3 => Self::Ephemeris3(GpsQzssFrame3::model()),
            GpsQzssFrameId::Almanach5 => Self::Almanach5(GpsQzssFrame5::model()),
        }
    }

    /// Unwraps self as [GpsQzssFrame1] reference (if feasible)
    pub fn as_eph1(&self) -> Option<GpsQzssFrame1> {
        match self {
            Self::Ephemeris1(frame) => Some(*frame),
            _ => None,
        }
    }

    /// Unwraps self as mutable [GpsQzssFrame1] reference (if feasible)
    pub fn as_mut_eph1(&mut self) -> Option<&mut GpsQzssFrame1> {
        match self {
            Self::Ephemeris1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    pub fn as_eph2(&self) -> Option<GpsQzssFrame2> {
        match self {
            Self::Ephemeris2(frame) => Some(*frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    pub fn as_mut_eph2(&mut self) -> Option<&mut GpsQzssFrame2> {
        match self {
            Self::Ephemeris2(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame3] reference (if feasible)
    pub fn as_eph3(&self) -> Option<GpsQzssFrame3> {
        match self {
            Self::Ephemeris3(frame) => Some(*frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame3] reference (if feasible)
    pub fn as_mut_eph3(&mut self) -> Option<&mut GpsQzssFrame3> {
        match self {
            Self::Ephemeris3(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame5] reference (if feasible)
    pub fn as_alm5(&self) -> Option<GpsQzssFrame5> {
        match self {
            Self::Almanach5(frame) => Some(*frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame5] reference (if feasible)
    pub fn as_mut_alm5(&mut self) -> Option<&mut GpsQzssFrame5> {
        match self {
            Self::Almanach5(frame) => Some(frame),
            _ => None,
        }
    }

    /// Decodes [Self] from 8 [GpsDataWord]s.
    /// This method does not care for frames parity.
    /// This method is infaillible for the ephemeris subframes.
    /// It may fail if the paginated almanach subframes come with an invalid page number.
    pub(crate) fn decode(
        frame_id: GpsQzssFrameId,
        words: &[GpsDataWord],
    ) -> Result<Self, GpsError> {
        match frame_id {
            GpsQzssFrameId::Ephemeris1 => Ok(Self::Ephemeris1(GpsQzssFrame1::from_words(words))),
            GpsQzssFrameId::Ephemeris2 => Ok(Self::Ephemeris2(GpsQzssFrame2::from_words(words))),
            GpsQzssFrameId::Ephemeris3 => Ok(Self::Ephemeris3(GpsQzssFrame3::from_words(words))),
            GpsQzssFrameId::Almanach5 => Ok(Self::Almanach5(GpsQzssFrame5::from_words(words)?)),
        }
    }

    /// Encodes this [GpsQzssSubframe] as a burst of 8 [GpsDataWord]s.
    pub(crate) fn to_words(&self) -> [GpsDataWord; GPS_WORDS_PER_FRAME - 2] {
        match self {
            Self::Ephemeris1(subframe) => subframe.to_words(),
            Self::Ephemeris2(subframe) => subframe.to_words(),
            Self::Ephemeris3(subframe) => subframe.to_words(),
            Self::Almanach5(subframe) => subframe.to_words(),
        }
    }
}
