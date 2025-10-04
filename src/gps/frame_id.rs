use crate::gps::GpsError;

#[derive(Debug, Default, PartialEq, Copy, Clone)]
pub enum GpsQzssFrameId {
    #[default]
    /// GPS / QZSS Ephemeris subframe #1
    Ephemeris1,

    /// GPS / QZSS Ephemeris subframe #2
    Ephemeris2,

    /// GPS / QZSS Ephemeris subframe #3
    Ephemeris3,

    /// GPS / QZSS Almanach subframe #5
    Almanach5,
}

#[cfg(feature = "std")]
impl std::fmt::Display for GpsQzssFrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::Ephemeris1 => write!(f, "EPH-1"),
            Self::Ephemeris2 => write!(f, "EPH-2"),
            Self::Ephemeris3 => write!(f, "EPH-3"),
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
            _ => Err(GpsError::UnknownFrameType),
        }
    }

    /// Encodes this [GpsQzssFrameId] as [u8]
    pub fn encode(&self) -> u8 {
        match self {
            Self::Ephemeris1 => 1,
            Self::Ephemeris2 => 2,
            Self::Ephemeris3 => 3,
        }
    }

    /// Returns the message that should follow, according
    /// to the [GpsQzssMessage] rotation definition.
    pub fn next(&self) -> GpsQzssFrameId {
        match self {
            Self::Ephemeris1 => Self::Ephemeris2,
            Self::Ephemeris2 => Self::Ephemeris3,
            Self::Ephemeris3 => Self::Almanach5,
            Self::Almanach5 => Self::Ephemeris1,
        }
    }
}

#[cfg(test)]
mod test {
    use super::GpsQzssFrameId;

    #[test]
    fn frame_id_decoding() {
        for (value, expected) in [
            (1, GpsQzssFrameId::Ephemeris1),
            (2, GpsQzssFrameId::Ephemeris2),
            (3, GpsQzssFrameId::Ephemeris3),
        ] {
            let frame_id = GpsQzssFrameId::decode(value).unwrap();
            assert_eq!(frame_id, expected);
        }
    }
}
