use crate::gps::GpsError;

use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadSized, BitReadStream, BitWrite, BitWriteSized,
    BitWriteStream, Endianness, LittleEndian,
};

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

    /// GPS / QZSS Almanach / Status subframe #5
    Almanach5,
}

impl BitWrite<BigEndian> for GpsQzssFrameId {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Ephemeris1 => stream.write_int::<u8>(1, 3),
            Self::Ephemeris2 => stream.write_int::<u8>(2, 3),
            Self::Ephemeris3 => stream.write_int::<u8>(3, 3),
            Self::Almanach4 => stream.write_int::<u8>(4, 3),
            Self::Almanach5 => stream.write_int::<u8>(5, 3),
        }
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
            Self::Almanach5 => write!(f, "ALM-5"),
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
            5 => Ok(Self::Almanach5),
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
            Self::Almanach5 => 5,
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{GpsBuffer, GpsQzssFrameId},
        Buffering,
    };

    #[test]
    fn from_int() {
        for (value, expected) in [
            (1, GpsQzssFrameId::Ephemeris1),
            (2, GpsQzssFrameId::Ephemeris2),
            (3, GpsQzssFrameId::Ephemeris3),
        ] {
            let frame_id = GpsQzssFrameId::decode(value).unwrap();
            assert_eq!(frame_id, expected);
        }
    }

    #[test]
    fn from_stream() {
        for (fid, encoded_value) in [
            (GpsQzssFrameId::Ephemeris1, 1 << 5),
            (GpsQzssFrameId::Ephemeris2, 2 << 5),
            (GpsQzssFrameId::Ephemeris3, 3 << 5),
            (GpsQzssFrameId::Almanach4, 4 << 5),
            (GpsQzssFrameId::Almanach5, 5 << 5),
        ] {
            let mut buffer = GpsBuffer::default();
            let mut stream = buffer.bit_write_stream();

            stream.write(&fid).unwrap_or_else(|e| {
                panic!("Failed to encode FID {}: {}", fid, e);
            });

            assert_eq!(buffer.to_slice()[0], encoded_value);
        }
    }
}
