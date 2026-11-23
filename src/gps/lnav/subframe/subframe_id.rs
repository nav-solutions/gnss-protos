use crate::gps::lnav::GpsError;

use bitbuffer::{BigEndian, BitError, BitReadSized, BitWrite, BitWriteSized, BitWriteStream};

#[derive(Debug, Default, PartialEq, Copy, Clone, BitReadSized, BitWriteSized)]
#[discriminant_bits = 3]
pub enum GpsQzssLnavSubframeId {
    #[default]
    /// GPS / QZSS Ephemeris subframe #1
    Ephemeris1,

    /// GPS / QZSS Ephemeris subframe #2
    Ephemeris2,

    /// GPS / QZSS Ephemeris subframe #3
    Ephemeris3,

    /// GPS / QZSS Almanach / Status subframe #4
    Almanach4,

    /// GPS / QZSS Almanach / Status subframe #5
    Almanach5,
}

impl BitWrite<BigEndian> for GpsQzssLnavSubframeId {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.encode(), 3)
    }
}

impl core::fmt::Display for GpsQzssLnavSubframeId {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        match self {
            Self::Ephemeris1 => write!(f, "EPH-1"),
            Self::Ephemeris2 => write!(f, "EPH-2"),
            Self::Ephemeris3 => write!(f, "EPH-3"),
            Self::Almanach4 => write!(f, "ALM-4"),
            Self::Almanach5 => write!(f, "ALM-5"),
        }
    }
}

impl GpsQzssLnavSubframeId {
    /// [GpsQzssLnavSubframeId] decoding attempt
    pub(crate) fn decode(mask: u8) -> Result<Self, GpsError> {
        match mask {
            1 => Ok(Self::Ephemeris1),
            2 => Ok(Self::Ephemeris2),
            3 => Ok(Self::Ephemeris3),
            4 => Ok(Self::Almanach4),
            5 => Ok(Self::Almanach5),
            _ => Err(GpsError::UnknownFrameType),
        }
    }

    /// Encodes this [GpsQzssLnavSubframeId] as [u8]
    pub fn encode(&self) -> u8 {
        match self {
            Self::Ephemeris1 => 1,
            Self::Ephemeris2 => 2,
            Self::Ephemeris3 => 3,
            Self::Almanach4 => 4,
            Self::Almanach5 => 5,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn reciprocal() {
        for (value, expected) in [
            (1, GpsQzssLnavSubframeId::Ephemeris1),
            (2, GpsQzssLnavSubframeId::Ephemeris2),
            (3, GpsQzssLnavSubframeId::Ephemeris3),
            (4, GpsQzssLnavSubframeId::Almanach4),
            (5, GpsQzssLnavSubframeId::Almanach5),
        ] {
            let frame_id = GpsQzssLnavSubframeId::decode(value).unwrap();
            assert_eq!(frame_id, expected);
            assert_eq!(frame_id.encode(), value);
        }
    }
}
