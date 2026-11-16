use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadBuffer, BitReadStream, BitWrite, BitWriteStream,
};

use crate::{
    galileo::{
        GalileoError, GalileoInavWord0, GalileoInavWord1, GalileoInavWord2, GalileoInavWord3,
        GalileoInavWord4, GalileoInavWord5, GalileoInavWord6, GALILEO_INAV_WORD_BITS,
        GALILEO_INAV_WORD_BYTES,
    },
    Message,
};

/// Reads the 6-bit word type and verifies it matches `expected`.
pub(crate) fn read_word_type(
    stream: &mut BitReadStream<'_, BigEndian>,
    expected: u8,
) -> Result<(), BitError> {
    let word_type = stream.read_int::<u8>(6)?;

    if word_type == expected {
        Ok(())
    } else {
        Err(BitError::UnmatchedDiscriminant {
            discriminant: word_type as usize,
            enum_name: "galileo-word-type".to_string(),
        })
    }
}

/// Encodes any message as a 128-bit word.
pub(crate) fn encode_word_bytes<W: BitWrite<BigEndian>>(
    word: &W,
) -> Result<[u8; GALILEO_INAV_WORD_BYTES], BitError> {
    let mut bytes = [0u8; GALILEO_INAV_WORD_BYTES];

    {
        let mut stream = BitWriteStream::from_slice(&mut bytes, BigEndian);
        stream.write(word)?;
    }

    Ok(bytes)
}

/// Implements [PartialEq] by comparing the binary representations,
/// so floating point values are compared at the ICD resolution.
macro_rules! impl_eq_by_encoding {
    ($t:ty) => {
        impl PartialEq for $t {
            fn eq(&self, rhs: &Self) -> bool {
                match (
                    crate::galileo::word::encode_word_bytes(self),
                    crate::galileo::word::encode_word_bytes(rhs),
                ) {
                    (Ok(lhs), Ok(rhs)) => lhs == rhs,
                    _ => false,
                }
            }
        }
    };
}

pub(crate) use impl_eq_by_encoding;

/// Galileo I/NAV word (128 bits), carried by one nominal [crate::galileo::GalileoInavPage].
/// The 6 first bits define the word type, hence its content.
///
/// Word types 0 to 6 are interpreted, which is enough for real time navigation
/// (ephemeris, clock, ionosphere, health and time). Other word types
/// (almanacs 7 to 10, reduced CED 16, FEC2 Reed-Solomon 17 to 20, ARAIM 22, dummy 63..)
/// are decoded and encoded as [GalileoInavWord::Raw] data bits.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GalileoInavWord {
    /// Word type 0: spare word, may carry the GST.
    Spare(GalileoInavWord0),

    /// Word type 1: Ephemeris (1/4).
    Ephemeris1(GalileoInavWord1),

    /// Word type 2: Ephemeris (2/4).
    Ephemeris2(GalileoInavWord2),

    /// Word type 3: Ephemeris (3/4) and SISA.
    Ephemeris3(GalileoInavWord3),

    /// Word type 4: Ephemeris (4/4) and clock correction.
    Ephemeris4(GalileoInavWord4),

    /// Word type 5: Ionospheric correction, BGD, health and GST.
    Ionosphere(GalileoInavWord5),

    /// Word type 6: GST-UTC conversion.
    GstUtc(GalileoInavWord6),

    /// Any other word type, not interpreted (yet): its 128 bits, word type included, MSB first.
    /// It should not carry one of the interpreted word types, otherwise
    /// it would be interpreted when decoded.
    Raw([u8; GALILEO_INAV_WORD_BYTES]),
}

impl Default for GalileoInavWord {
    /// Builds a default [GalileoInavWord::Spare] word.
    fn default() -> Self {
        Self::Spare(Default::default())
    }
}

impl Message<BigEndian> for GalileoInavWord {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord {
    /// Reads the 128 bits of the word, and interprets it according to its word type.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let mut bytes = [0u8; GALILEO_INAV_WORD_BYTES];
        bytes.copy_from_slice(&stream.read_bytes(GALILEO_INAV_WORD_BYTES)?);
        Self::decode_bytes(&bytes).map_err(|e| match e {
            GalileoError::BitStream(e) => e,
            e => BitError::UnmatchedDiscriminant {
                discriminant: 0,
                enum_name: e.to_string(),
            },
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Spare(word) => stream.write(word),
            Self::Ephemeris1(word) => stream.write(word),
            Self::Ephemeris2(word) => stream.write(word),
            Self::Ephemeris3(word) => stream.write(word),
            Self::Ephemeris4(word) => stream.write(word),
            Self::Ionosphere(word) => stream.write(word),
            Self::GstUtc(word) => stream.write(word),
            Self::Raw(bytes) => stream.write_bytes(bytes),
        }
    }
}

impl GalileoInavWord {
    /// Interprets 128 bits (16 bytes, MSB first) according to their word type.
    pub fn decode_bytes(bytes: &[u8; GALILEO_INAV_WORD_BYTES]) -> Result<Self, GalileoError> {
        let mut stream = BitReadStream::new(BitReadBuffer::new(bytes, BigEndian));

        match bytes[0] >> 2 {
            GalileoInavWord0::WORD_TYPE => Ok(Self::Spare(stream.read()?)),
            GalileoInavWord1::WORD_TYPE => Ok(Self::Ephemeris1(stream.read()?)),
            GalileoInavWord2::WORD_TYPE => Ok(Self::Ephemeris2(stream.read()?)),
            GalileoInavWord3::WORD_TYPE => Ok(Self::Ephemeris3(stream.read()?)),
            GalileoInavWord4::WORD_TYPE => Ok(Self::Ephemeris4(stream.read()?)),
            GalileoInavWord5::WORD_TYPE => Ok(Self::Ionosphere(stream.read()?)),
            GalileoInavWord6::WORD_TYPE => Ok(Self::GstUtc(stream.read()?)),
            _ => Ok(Self::Raw(*bytes)),
        }
    }

    /// Encodes this word as 128 bits (16 bytes, MSB first).
    pub fn encode_bytes(&self) -> Result<[u8; GALILEO_INAV_WORD_BYTES], GalileoError> {
        Ok(encode_word_bytes(self)?)
    }

    /// Returns the word type (6 bits) of this [GalileoInavWord].
    pub fn word_type(&self) -> u8 {
        match self {
            Self::Spare(_) => GalileoInavWord0::WORD_TYPE,
            Self::Ephemeris1(_) => GalileoInavWord1::WORD_TYPE,
            Self::Ephemeris2(_) => GalileoInavWord2::WORD_TYPE,
            Self::Ephemeris3(_) => GalileoInavWord3::WORD_TYPE,
            Self::Ephemeris4(_) => GalileoInavWord4::WORD_TYPE,
            Self::Ionosphere(_) => GalileoInavWord5::WORD_TYPE,
            Self::GstUtc(_) => GalileoInavWord6::WORD_TYPE,
            Self::Raw(bytes) => bytes[0] >> 2,
        }
    }

    /// Returns the IODnav of an ephemeris word (types 1 to 4).
    pub fn iodnav(&self) -> Option<u16> {
        match self {
            Self::Ephemeris1(word) => Some(word.iodnav),
            Self::Ephemeris2(word) => Some(word.iodnav),
            Self::Ephemeris3(word) => Some(word.iodnav),
            Self::Ephemeris4(word) => Some(word.iodnav),
            _ => None,
        }
    }

    /// Returns the GST (week number, time of week in seconds) if this word carries it
    /// (valid type 0, or type 5).
    pub fn gst(&self) -> Option<(u16, u32)> {
        match self {
            Self::Spare(word) if word.has_valid_time() => Some((word.wn, word.tow)),
            Self::Ionosphere(word) => Some((word.wn, word.tow)),
            _ => None,
        }
    }

    #[cfg(test)]
    pub(crate) fn model(word_type: u8) -> Self {
        match word_type {
            0 => Self::Spare(GalileoInavWord0::model()),
            1 => Self::Ephemeris1(GalileoInavWord1::model()),
            2 => Self::Ephemeris2(GalileoInavWord2::model()),
            3 => Self::Ephemeris3(GalileoInavWord3::model()),
            4 => Self::Ephemeris4(GalileoInavWord4::model()),
            5 => Self::Ionosphere(GalileoInavWord5::model()),
            6 => Self::GstUtc(GalileoInavWord6::model()),
            word_type => {
                let mut bytes = [0xA5u8; GALILEO_INAV_WORD_BYTES];
                bytes[0] = (word_type << 2) | 0x1;
                Self::Raw(bytes)
            },
        }
    }

    /// Unwraps self as [GalileoInavWord0] (if feasible)
    pub fn as_word0(&self) -> Option<GalileoInavWord0> {
        match self {
            Self::Spare(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as [GalileoInavWord1] (if feasible)
    pub fn as_word1(&self) -> Option<GalileoInavWord1> {
        match self {
            Self::Ephemeris1(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as [GalileoInavWord2] (if feasible)
    pub fn as_word2(&self) -> Option<GalileoInavWord2> {
        match self {
            Self::Ephemeris2(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as [GalileoInavWord3] (if feasible)
    pub fn as_word3(&self) -> Option<GalileoInavWord3> {
        match self {
            Self::Ephemeris3(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as [GalileoInavWord4] (if feasible)
    pub fn as_word4(&self) -> Option<GalileoInavWord4> {
        match self {
            Self::Ephemeris4(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as [GalileoInavWord5] (if feasible)
    pub fn as_word5(&self) -> Option<GalileoInavWord5> {
        match self {
            Self::Ionosphere(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as [GalileoInavWord6] (if feasible)
    pub fn as_word6(&self) -> Option<GalileoInavWord6> {
        match self {
            Self::GstUtc(word) => Some(*word),
            _ => None,
        }
    }

    /// Unwraps self as raw (not interpreted) data bits (if feasible)
    pub fn as_raw(&self) -> Option<[u8; GALILEO_INAV_WORD_BYTES]> {
        match self {
            Self::Raw(bytes) => Some(*bytes),
            _ => None,
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{galileo::GalileoInavWord, Message};

    const ALL_TYPES: [u8; 14] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 16, 17, 63];

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord::default();
        let mut buffer = [0u8; 16];
        assert_eq!(default.encode(&mut buffer).unwrap(), 16);
        assert_eq!(GalileoInavWord::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        for word_type in ALL_TYPES {
            let model = GalileoInavWord::model(word_type);
            assert_eq!(model.word_type(), word_type);

            let bytes = model.encode_bytes().unwrap();
            assert_eq!(bytes[0] >> 2, word_type);

            let decoded = GalileoInavWord::decode_bytes(&bytes).unwrap();
            assert_eq!(decoded, model, "word type {}", word_type);
            assert_eq!(decoded.encode_bytes().unwrap(), bytes);

            // through the BitRead / BitWrite traits
            let mut buffer = [0u8; 32];
            model.encode(&mut buffer).unwrap();
            assert_eq!(GalileoInavWord::decode(&buffer).unwrap(), model);
        }
    }

    #[test]
    fn accessors() {
        let word = GalileoInavWord::model(1);
        assert!(word.as_word1().is_some());
        assert!(word.as_word2().is_none());
        assert_eq!(word.iodnav(), Some(123));
        assert_eq!(word.gst(), None);

        assert_eq!(GalileoInavWord::model(0).gst(), Some((1339, 455_901)));
        assert_eq!(GalileoInavWord::model(5).gst(), Some((1339, 455_905)));
        assert_eq!(GalileoInavWord::default().gst(), None);
        assert!(GalileoInavWord::model(16).as_raw().is_some());
    }
}
