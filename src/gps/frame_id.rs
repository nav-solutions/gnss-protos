use crate::gps::GpsError;

use bitbuffer::{BigEndian, BitError, BitReadSized, BitWrite, BitWriteSized, BitWriteStream};

#[derive(Debug, Default, PartialEq, Copy, Clone, BitReadSized, BitWriteSized)]
#[discriminant_bits = 3]
pub enum GpsQzssFrameId {
    #[default]
    /// GPS / QZSS Ephemeris subframe #1
    Ephemeris1,

    /// GPS / QZSS Ephemeris subframe #2
    Ephemeris2,

    /// GPS / QZSS Ephemeris subframe #3
    Ephemeris3,

    /// GPS / QZSS Almanach / Status subframe #4
    Almanach4,
    // /// GPS / QZSS Almanach / Status subframe #5
    // Almanach5,
}

impl BitWrite<BigEndian> for GpsQzssFrameId {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.encode(), 3)
    }
}

#[cfg(feature = "std")]
impl std::fmt::Display for GpsQzssFrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::Ephemeris1 => write!(f, "EPH-1"),
            Self::Ephemeris2 => write!(f, "EPH-2"),
            Self::Ephemeris3 => write!(f, "EPH-3"),
            Self::Almanach4 => write!(f, "ALM-4"),
            // Self::Almanach5 => write!(f, "ALM-5"),
        }
    }
}

impl GpsQzssFrameId {
    /// [GpsQzssFrameId] decoding attempt
    pub(crate) fn decode(mask: u8) -> Result<Self, GpsError> {
        match mask {
            1 => Ok(Self::Ephemeris1),
            2 => Ok(Self::Ephemeris2),
            3 => Ok(Self::Ephemeris3),
            4 => Ok(Self::Almanach4),
            // 5 => Ok(Self::Almanach5),
            _ => Err(GpsError::UnknownFrameType),
        }
    }

    /// Encodes this [GpsQzssFrameId] as [u8]
    pub fn encode(&self) -> u8 {
        match self {
            Self::Ephemeris1 => 1,
            Self::Ephemeris2 => 2,
            Self::Ephemeris3 => 3,
            Self::Almanach4 => 4,
            // Self::Almanach5 => 5,
        }
    }
}

#[cfg(test)]
mod test {
    use crate::gps::GpsQzssFrameId;

    #[test]
    fn reciprocal() {
        for (value, expected) in [
            (1, GpsQzssFrameId::Ephemeris1),
            (2, GpsQzssFrameId::Ephemeris2),
            (3, GpsQzssFrameId::Ephemeris3),
            (4, GpsQzssFrameId::Almanach4),
            // (5, GpsQzssFrameId::Almanach5),
        ] {
            let frame_id = GpsQzssFrameId::decode(value).unwrap();
            assert_eq!(frame_id, expected);
            assert_eq!(frame_id.encode(), value);
        }
    }
}
