use crate::gps::{
    // GpsQzssFrame2,
    // GpsQzssFrame3,
    GpsQzssFrame1,
};

#[cfg(test)]
use crate::gps::GpsQzssFrameId;

use bitbuffer::{BigEndian, BitError, BitWrite, BitWriteStream};

/// GPS / QZSS Interpreted subframes
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsQzssSubframe {
    /// GPS Ephemeris Frame #1
    Ephemeris1(GpsQzssFrame1),
    // /// GPS Ephemeris Frame #2
    // Ephemeris2(GpsQzssFrame2),

    // /// GPS Ephemeris Frame #3
    // Ephemeris3(GpsQzssFrame3),
}

impl BitWrite<BigEndian> for GpsQzssSubframe {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Ephemeris1(eph1) => stream.write(eph1),
        }
    }
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
            _ => panic!("not yet"),
            // GpsQzssFrameId::Ephemeris2 => Self::Ephemeris2(GpsQzssFrame2::model()),
            // GpsQzssFrameId::Ephemeris3 => Self::Ephemeris3(GpsQzssFrame3::model()),
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

    // /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    // pub fn as_eph2(&self) -> Option<GpsQzssFrame2> {
    //     match self {
    //         Self::Ephemeris2(frame) => Some(*frame),
    //         _ => None,
    //     }
    // }

    // /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    // pub fn as_mut_eph2(&mut self) -> Option<&mut GpsQzssFrame2> {
    //     match self {
    //         Self::Ephemeris2(frame) => Some(frame),
    //         _ => None,
    //     }
    // }

    // /// Unwraps self as [GpsQzssFrame3] reference (if feasible)
    // pub fn as_eph3(&self) -> Option<GpsQzssFrame3> {
    //     match self {
    //         Self::Ephemeris3(frame) => Some(*frame),
    //         _ => None,
    //     }
    // }

    // /// Unwraps self as [GpsQzssFrame3] reference (if feasible)
    // pub fn as_mut_eph3(&mut self) -> Option<&mut GpsQzssFrame3> {
    //     match self {
    //         Self::Ephemeris3(frame) => Some(frame),
    //         _ => None,
    //     }
    // }
}

#[cfg(test)]
mod test {
    use crate::gps::{GpsBuffer, GpsQzssSubframe};

    #[test]
    fn default_reciprocal() {
        for subframe in [GpsQzssSubframe::Ephemeris1(Default::default())] {
            let mut buf = GpsBuffer::default();
        }
    }
}
