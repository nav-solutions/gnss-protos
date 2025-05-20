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
