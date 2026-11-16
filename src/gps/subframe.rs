use crate::gps::{GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrameId};

use bitbuffer::{BigEndian, BitError, BitReadStream, BitWrite, BitWriteStream};

/// Number of data bytes (parity free) of a subframe payload (words 3 to 10).
pub const GPS_SUBFRAME_DATA_BYTES: usize = 24;

/// GPS / QZSS Interpreted subframes
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsQzssSubframe {
    /// GPS Ephemeris Frame #1
    Ephemeris1(GpsQzssFrame1),

    /// GPS Ephemeris Frame #2
    Ephemeris2(GpsQzssFrame2),

    /// GPS Ephemeris Frame #3
    Ephemeris3(GpsQzssFrame3),

    /// GPS Almanach / Status Frame #4, not interpreted yet:
    /// raw (parity free) data bits of words 3 to 10.
    Almanach4([u8; GPS_SUBFRAME_DATA_BYTES]),

    /// GPS Almanach / Status Frame #5, not interpreted yet:
    /// raw (parity free) data bits of words 3 to 10.
    Almanach5([u8; GPS_SUBFRAME_DATA_BYTES]),
}

impl BitWrite<BigEndian> for GpsQzssSubframe {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Ephemeris1(eph) => stream.write(eph),
            Self::Ephemeris2(eph) => stream.write(eph),
            Self::Ephemeris3(eph) => stream.write(eph),
            Self::Almanach4(raw) | Self::Almanach5(raw) => stream.write_bytes(raw),
        }
    }
}

impl GpsQzssSubframe {
    /// Reads the subframe payload (192 parity free data bits),
    /// whose content is defined by the [GpsQzssFrameId] found in the HOW.
    pub(crate) fn read(
        stream: &mut BitReadStream<'_, BigEndian>,
        frame_id: GpsQzssFrameId,
    ) -> Result<Self, BitError> {
        match frame_id {
            GpsQzssFrameId::Ephemeris1 => Ok(Self::Ephemeris1(stream.read()?)),
            GpsQzssFrameId::Ephemeris2 => Ok(Self::Ephemeris2(stream.read()?)),
            GpsQzssFrameId::Ephemeris3 => Ok(Self::Ephemeris3(stream.read()?)),
            GpsQzssFrameId::Almanach4 | GpsQzssFrameId::Almanach5 => {
                let mut raw = [0u8; GPS_SUBFRAME_DATA_BYTES];
                raw.copy_from_slice(&stream.read_bytes(GPS_SUBFRAME_DATA_BYTES)?);

                if frame_id == GpsQzssFrameId::Almanach4 {
                    Ok(Self::Almanach4(raw))
                } else {
                    Ok(Self::Almanach5(raw))
                }
            },
        }
    }

    /// Returns the [GpsQzssFrameId] matching this subframe.
    pub fn frame_id(&self) -> GpsQzssFrameId {
        match self {
            Self::Ephemeris1(_) => GpsQzssFrameId::Ephemeris1,
            Self::Ephemeris2(_) => GpsQzssFrameId::Ephemeris2,
            Self::Ephemeris3(_) => GpsQzssFrameId::Ephemeris3,
            Self::Almanach4(_) => GpsQzssFrameId::Almanach4,
            Self::Almanach5(_) => GpsQzssFrameId::Almanach5,
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
            GpsQzssFrameId::Ephemeris2 => Self::Ephemeris2(GpsQzssFrame2::model()),
            GpsQzssFrameId::Ephemeris3 => Self::Ephemeris3(GpsQzssFrame3::model()),
            GpsQzssFrameId::Almanach4 => Self::Almanach4([0x5A; GPS_SUBFRAME_DATA_BYTES]),
            GpsQzssFrameId::Almanach5 => Self::Almanach5([0xA5; GPS_SUBFRAME_DATA_BYTES]),
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

    // /// Unwraps self as [GpsQzssFrame5] reference (if feasible)
    // pub fn as_alm5(&self) -> Option<GpsQzssFrame5> {
    //     match self {
    //         Self::Almanach5(frame) => Some(*frame),
    //         _ => None,
    //     }
    // }

    // /// Unwraps self as [GpsQzssFrame5] reference (if feasible)
    // pub fn as_mut_alm5(&mut self) -> Option<&mut GpsQzssFrame5> {
    //     match self {
    //         Self::Almanach5(frame) => Some(frame),
    //         _ => None,
    //     }
    // }
}
