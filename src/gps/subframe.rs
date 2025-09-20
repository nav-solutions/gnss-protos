use crate::gps::{
    GpsQzssFrame1,
    GpsQzssFrame2,
    GpsQzssFrame3,
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
}

impl Default for GpsQzssSubframe {
    /// Builds a default [GpsQzssSubFrame::Ephemeris1]
    fn default() -> Self {
        Self::Ephemeris1(Default::default())
    }
}

impl GpsQzssSubframe {
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

    /// Decodes a [GpsQzssSubframe] from a 8 [GpsDataByte] slice
    /// supports padding on each word termination (not intra words)
    pub(crate) fn decode(frame_id: GpsQzssFrameId, bytes: &[GpsDataByte]) -> Self {
        // for i in 0..8 {
        //     let dword = ByteArray::new(&bytes[8+i*4..8+i*4+4])
        //         .value_u32();

        //     let _ = subframe.decode_word(i +3, dword);  // IMPROVE
        //

        match frame_id {
            GpsQzssFrameId::Ephemeris1 => Self::Ephemeris1(GpsQzssFrame1::decode(bytes)),
            GpsQzssFrameId::Ephemeris2 => Self::Ephemeris2(GpsQzssFrame2::decode(bytes)),
            GpsQzssFrameId::Ephemeris3 => Self::Ephemeris3(GpsQzssFrame3::decode(bytes)),
        }
    }

    /// Encode this [GpsQzssSubframe] into 8 [u32] data burst.
    pub(crate) fn encode(&self) -> [u32; 8] {
        match self {
            Self::Ephemeris1(subframe) => subframe.encode(),
            Self::Ephemeris2(subframe) => subframe.encode(),
            Self::Ephemeris3(subframe) => subframe.encode(),
        }
    }
}
