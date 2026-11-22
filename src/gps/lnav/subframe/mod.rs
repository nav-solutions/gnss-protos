use crate::{
    constants::SECONDS_PER_WEEK,
    gps::lnav::{
        GpsError, GpsQzssLnavHow, GpsQzssLnavTelemetry, GPS_LNAV_FRAME_DURATION_S,
        GPS_LNAV_INVERTED_PREAMBLE_BYTE, GPS_LNAV_PREAMBLE_BYTE, GPS_LNAV_SUBFRAME_BITS,
        GPS_LNAV_SUBFRAME_BYTES, GPS_LNAV_SUBFRAME_DATA_BYTES, GPS_LNAV_SUBFRAME_DURATION_S,
        GPS_LNAV_SUBFRAME_WORDS, GPS_LNAV_SUPERFRAME_ITEMS, GPS_LNAV_WORD_BITS,
    },
    Message,
};

use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadBuffer, BitReadStream, BitWrite, BitWriteStream,
};

mod decoder;
mod reserved;
mod subframe1;
mod subframe2;
mod subframe3;
mod subframe4;
mod subframe5;
mod subframe_id;

pub use decoder::*;
pub use reserved::*;
pub use subframe1::*;
pub use subframe2::*;
pub use subframe3::*;
pub use subframe4::*;
pub use subframe5::*;
pub use subframe_id::*;

/// Returns the SV (page) ID of the subframe 4 or 5 page that starts
/// at the current position, without consuming the stream.
fn peek_sv_id(
    stream: &mut crate::BitReadStream<'_, crate::BigEndian>,
) -> Result<u8, crate::BitError> {
    let start = stream.pos();
    let _data_id = stream.read_int::<u8>(2)?;
    let sv_id = stream.read_int::<u8>(6)?;
    stream.set_pos(start)?;
    Ok(sv_id)
}

/// GPS / QZSS Interpreted subframes
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsQzssLnavSubframeEnum {
    /// GPS Ephemeris Subframe class #1
    Ephemeris1(GpsQzssLnavSubframe1),

    /// GPS Ephemeris Subframe class #2
    Ephemeris2(GpsQzssLnavSubframe2),

    /// GPS Ephemeris Subframe class #3
    Ephemeris3(GpsQzssLnavSubframe3),

    /// GPS Almanach / Status Frame #4: one of the 25 pages of subframe 4,
    /// interpreted according to its SV (page) ID.
    Almanach4(GpsQzssLnavSubframe4),

    /// GPS Almanach / Status Frame #5: one of the 25 pages of subframe 5,
    /// interpreted according to its SV (page) ID.
    Almanach5(GpsQzssLnavSubframe5),
}

impl BitWrite<BigEndian> for GpsQzssLnavSubframeEnum {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        match self {
            Self::Ephemeris1(eph) => stream.write(eph),
            Self::Ephemeris2(eph) => stream.write(eph),
            Self::Ephemeris3(eph) => stream.write(eph),
            Self::Almanach4(page) => stream.write(page),
            Self::Almanach5(page) => stream.write(page),
        }
    }
}

impl GpsQzssLnavSubframeEnum {
    /// Reads the subframe payload (192 parity free data bits),
    /// whose content is defined by the [GpsQzssLnavSubframeId] found in the HOW.
    pub(crate) fn read(
        stream: &mut BitReadStream<'_, BigEndian>,
        frame_id: GpsQzssLnavSubframeId,
    ) -> Result<Self, BitError> {
        match frame_id {
            GpsQzssLnavSubframeId::Ephemeris1 => Ok(Self::Ephemeris1(stream.read()?)),
            GpsQzssLnavSubframeId::Ephemeris2 => Ok(Self::Ephemeris2(stream.read()?)),
            GpsQzssLnavSubframeId::Ephemeris3 => Ok(Self::Ephemeris3(stream.read()?)),
            GpsQzssLnavSubframeId::Almanach5 => Ok(Self::Almanach5(stream.read()?)),
            GpsQzssLnavSubframeId::Almanach4 => Ok(Self::Almanach4(stream.read()?)),
        }
    }

    /// Returns the [GpsQzssLnavSubframeId] matching this subframe.
    pub fn frame_id(&self) -> GpsQzssLnavSubframeId {
        match self {
            Self::Ephemeris1(_) => GpsQzssLnavSubframeId::Ephemeris1,
            Self::Ephemeris2(_) => GpsQzssLnavSubframeId::Ephemeris2,
            Self::Ephemeris3(_) => GpsQzssLnavSubframeId::Ephemeris3,
            Self::Almanach4(_) => GpsQzssLnavSubframeId::Almanach4,
            Self::Almanach5(_) => GpsQzssLnavSubframeId::Almanach5,
        }
    }
}

impl Default for GpsQzssLnavSubframeEnum {
    /// Builds a default [GpsQzssLnavSubframeEnum::Ephemeris1]
    fn default() -> Self {
        Self::Ephemeris1(Default::default())
    }
}

impl GpsQzssLnavSubframeEnum {
    /// Generates a realistic frame model for testing purposes
    #[cfg(test)]
    pub fn model(frame_id: GpsQzssLnavSubframeId) -> Self {
        match frame_id {
            GpsQzssLnavSubframeId::Ephemeris1 => Self::Ephemeris1(GpsQzssLnavSubframe1::model()),
            GpsQzssLnavSubframeId::Ephemeris2 => Self::Ephemeris2(GpsQzssLnavSubframe2::model()),
            GpsQzssLnavSubframeId::Ephemeris3 => Self::Ephemeris3(GpsQzssLnavSubframe3::model()),
            GpsQzssLnavSubframeId::Almanach4 => Self::Almanach4(GpsQzssLnavSubframe4::model(18)),
            GpsQzssLnavSubframeId::Almanach5 => Self::Almanach5(GpsQzssLnavSubframe5::model(7)),
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe1] reference (if feasible)
    pub fn as_eph1(&self) -> Option<&GpsQzssLnavSubframe1> {
        match self {
            Self::Ephemeris1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as mutable [GpsQzssLnavSubframe1] reference (if feasible)
    pub fn as_mut_eph1(&mut self) -> Option<&mut GpsQzssLnavSubframe1> {
        match self {
            Self::Ephemeris1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe2] reference (if feasible)
    pub fn as_eph2(&self) -> Option<&GpsQzssLnavSubframe2> {
        match self {
            Self::Ephemeris2(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe2] reference (if feasible)
    pub fn as_mut_eph2(&mut self) -> Option<&mut GpsQzssLnavSubframe2> {
        match self {
            Self::Ephemeris2(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe3] reference (if feasible)
    pub fn as_eph3(&self) -> Option<&GpsQzssLnavSubframe3> {
        match self {
            Self::Ephemeris3(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe3] reference (if feasible)
    pub fn as_mut_eph3(&mut self) -> Option<&mut GpsQzssLnavSubframe3> {
        match self {
            Self::Ephemeris3(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe4] reference (if feasible)
    pub fn as_alm4(&self) -> Option<&GpsQzssLnavSubframe4> {
        match self {
            Self::Almanach4(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as mutable [GpsQzssLnavSubframe4] reference (if feasible)
    pub fn as_mut_alm4(&mut self) -> Option<&mut GpsQzssLnavSubframe4> {
        match self {
            Self::Almanach4(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssLnavSubframe5] reference (if feasible)
    pub fn as_alm5(&self) -> Option<&GpsQzssLnavSubframe5> {
        match self {
            Self::Almanach5(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as mutable [GpsQzssLnavSubframe5] reference (if feasible)
    pub fn as_mut_alm5(&mut self) -> Option<&mut GpsQzssLnavSubframe5> {
        match self {
            Self::Almanach5(frame) => Some(frame),
            _ => None,
        }
    }
}

pub(crate) const WORD_MASK: u32 = 0x3fff_ffff;

/// 24-bit data mask.
const DATA_MASK: u32 = 0x00ff_ffff;

/// Parity equations (IS-GPS-200 table 20-XIV) expressed as bit masks,
/// applied to a 32-bit value laid out as `[D29*, D30*, d1..d24, 0 x 6]`.
const PARITY_MASKS: [u32; 6] = [
    0xBB1F_3480, // D25
    0x5D8F_9A40, // D26
    0xAEC7_CD00, // D27
    0x5763_E680, // D28
    0x6BB1_F340, // D29
    0x8B7A_89C0, // D30
];

/// Computes the 6 parity bits (D25..D30, D25 being the MSB) of 24 *source*
/// data bits (right aligned), given the last two parity bits of the previous word.
fn parity(data: u32, prev_d29: bool, prev_d30: bool) -> u8 {
    let mut value = (data & DATA_MASK) << 6;

    if prev_d29 {
        value |= 0x8000_0000;
    }

    if prev_d30 {
        value |= 0x4000_0000;
    }

    let mut parity = 0u8;

    for mask in PARITY_MASKS.iter() {
        parity <<= 1;
        parity |= ((value & mask).count_ones() & 0x1) as u8;
    }

    parity
}

/// Returns (D29, D30) of a 30-bit (right aligned) word.
#[inline]
fn d29_d30(word: u32) -> (bool, bool) {
    (word & 0x2 > 0, word & 0x1 > 0)
}

/// Encodes 24 source data bits into a 30-bit transmitted word (right aligned),
/// following the previous transmitted word `prev` (only its 2 LSB matter).
/// When D30* is set, the transmitted data bits are inverted.
fn encode_word(data: u32, prev: u32) -> u32 {
    let (d29, d30) = d29_d30(prev);
    let data = data & DATA_MASK;
    let parity = parity(data, d29, d30) as u32;

    let transmitted = if d30 { !data & DATA_MASK } else { data };

    (transmitted << 6) | parity
}

/// Encodes 22 source data bits (right aligned, bits 23 and 24 being left free),
/// solving bits 23-24 so the resulting word ends with D29 = D30 = 0.
/// This is required for the HOW (word 2) and word 10 (IS-GPS-200 §20.3.5.2),
/// so that every subframe starts with D29* = D30* = 0.
fn encode_word_with_t_bits(data: u32, prev: u32) -> u32 {
    let data = data & !0x3 & DATA_MASK;

    for t in 0..4 {
        let word = encode_word(data | t, prev);

        if word & 0x3 == 0 {
            return word;
        }
    }

    // There is always one combination of the two free bits
    // that nulls D29 and D30 (the system is invertible).
    unreachable!("GPS t-bits: no solution");
}

/// Decodes a 30-bit received word (right aligned) into its 24 source data bits,
/// following the previous received word `prev` (only its 2 LSB matter).
/// When `check_parity` is set, the parity is verified.
fn decode_word(word: u32, prev: u32, check_parity: bool) -> Result<u32, GpsError> {
    let (d29, d30) = d29_d30(prev);

    let mut data = (word >> 6) & DATA_MASK;

    if d30 {
        data = !data & DATA_MASK;
    }

    if check_parity && parity(data, d29, d30) != (word & 0x3f) as u8 {
        return Err(GpsError::Parity);
    }

    Ok(data)
}

/// GPS / QZSS interpreted frame (one 300-bit subframe).
///
/// [GpsQzssFrame] implements [Message]: [Message::encode] and [Message::decode]
/// work on the 300 transmitted bits (10 words of 30 bits, parity included, 38 bytes
/// with 4 padding bits). To work at any bit position, or to stream consecutive frames,
/// use the [BitRead] / [BitWrite] implementations with a [bitbuffer] stream:
///
/// ```
/// use gnss_protos::{
///     BigEndian, BitReadBuffer, BitReadStream, BitWriteStream,
///     gps::lnav::{
///         GpsQzssLnavSubframe, GpsQzssLnavHow,
///         GPS_LNAV_SUBFRAME_BITS,
///     },
/// };
///
/// let frame = GpsQzssLnavSubframe::default()
///     .with_hand_over_word(GpsQzssLnavHow::default().with_tow_seconds(259_956));
///
/// // 3 consecutive frames, 5 bits after the start of the stream
/// let mut bytes = Vec::new();
/// let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
/// writer.write_int(0u8, 5).unwrap();
///
/// for _ in 0..3 {
///     writer.write(&frame).unwrap();
/// }
///
/// let mut reader = BitReadStream::new(BitReadBuffer::new(&bytes, BigEndian));
/// reader.set_pos(5).unwrap();
///
/// for _ in 0..3 {
///     let decoded: GpsQzssLnavSubframe = reader.read().unwrap();
///     assert_eq!(decoded, frame);
/// }
///
/// assert_eq!(reader.pos(), 5 + 3 * GPS_LNAV_SUBFRAME_BITS);
/// ```
///
/// Parity is always computed when encoding, and always verified when decoding.
/// Inverted frames (inverted preamble, 180° carrier phase ambiguity) are decoded transparently.
/// Words 2 and 10 end with D29 = D30 = 0 as required by the ICD, so frames can be concatenated as is.
///
/// Parity and polarity are only handled here, the
/// messages ([GpsQzssLnavTelemetry], [GpsQzssLnavHow], [GpsQzssLnavSubframe]) only
/// see the 240 parity free data bits.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssLnavSubframe {
    /// [GpsQzssLnavTelemetry] describes the following frame and contains
    /// the sync-byte, therefore initiates a [GpsQzssFrame].
    pub telemetry: GpsQzssLnavTelemetry,

    /// [GpsQzssLnavHow] describes following frame.
    pub how: GpsQzssLnavHow,

    /// [GpsQzssLnavSubframeEnum] depends on associated [GpsQzssLnavHow].
    pub subframe: GpsQzssLnavSubframeEnum,
}

impl Message<BigEndian> for GpsQzssLnavSubframe {
    /// [GpsQzssLnavSubframe] encoding size (in bytes) including parity bits.
    fn encoding_size(&self) -> usize {
        GPS_LNAV_SUBFRAME_BYTES
    }

    /// [GpsQzssLnavSubframe] encoding size (in bits) including parity bits
    fn encoding_bits(&self) -> usize {
        GPS_LNAV_SUBFRAME_BITS
    }
}

/// [BitRead] only allows [BitError]s: express GPS errors as such.
fn bit_error(e: GpsError) -> BitError {
    match e {
        GpsError::BufferRead(e) => e,
        e => BitError::UnmatchedDiscriminant {
            discriminant: 0,
            enum_name: e.to_string(),
        },
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavSubframe {
    /// Reads a complete 300-bit frame (with parity) from the stream, at its current position.
    /// The parity is always verified. Inverted frames are supported.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let mut words = [0u32; GPS_LNAV_SUBFRAME_WORDS];

        for word in words.iter_mut() {
            *word = stream.read_int::<u32>(GPS_LNAV_WORD_BITS)?;
        }

        Self::from_words(&words, true).map_err(bit_error)
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavSubframe {
    /// Writes a complete 300-bit frame (with parity) to the stream.
    /// Frames written one after the other form a valid GPS bit stream.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        let words = self.to_words().map_err(bit_error)?;

        for word in words.iter() {
            stream.write_int(*word, GPS_LNAV_WORD_BITS)?;
        }

        Ok(())
    }
}

impl GpsQzssLnavSubframe {
    #[cfg(test)]
    pub(crate) fn model(frame_id: GpsQzssLnavSubframeId) -> Self {
        Self::default()
            .with_telemetry(GpsQzssLnavTelemetry::model())
            .with_hand_over_word(GpsQzssLnavHow::model(frame_id))
            .with_subframe_enum(GpsQzssLnavSubframeEnum::model(frame_id))
    }

    /// Copies and returns with updated [GpsQzssLnavHow].
    pub fn with_hand_over_word(mut self, how: GpsQzssLnavHow) -> Self {
        self.how = how;
        self.how.sbf_id = self.subframe.frame_id();
        self
    }

    /// Copies and returns with updated [GpsQzssLnavTelemetry] data word
    pub fn with_telemetry(mut self, telemetry: GpsQzssLnavTelemetry) -> Self {
        self.telemetry = telemetry;
        self
    }

    /// Copies and returns an updated [GpsQzssLnavSubframe].
    /// The frame ID of the [GpsQzssLnavHow] is updated accordingly.
    pub fn with_subframe_enum(mut self, subframe: GpsQzssLnavSubframeEnum) -> Self {
        self.subframe = subframe;
        self.how.sbf_id = subframe.frame_id();
        self
    }

    /// Returns transmission start time in seconds of current week.
    pub fn week_transmission_time_s(&self) -> u32 {
        // SBF(n+1): HOW carries the time of the next subframe.
        let mut tow = self.how.tow as u32;
        tow += SECONDS_PER_WEEK;
        tow -= GPS_LNAV_SUBFRAME_DURATION_S;
        tow % SECONDS_PER_WEEK
    }

    /// Returns the page number (1..=25) of the frame this subframe belongs to.
    pub fn page_number(&self) -> u8 {
        let subf = self.week_transmission_time_s() / GPS_LNAV_FRAME_DURATION_S;
        (subf % GPS_LNAV_SUPERFRAME_ITEMS as u32) as u8 + 1
    }

    /// Returns true if this subframe is the very first subframe (EPH-1 of page 1)
    /// of a 12'30" superframe.
    pub fn starts_superframe_period(&self) -> bool {
        // let start = self.week_transmission_time_s();
        // self.subframe.frame_id() == GpsQzssLnavSubframeId::Ephemeris1
        //     && start % GPS_LNAV_SUPERFRAME_DURATION_S == 0
        //     && start + GPS_LNAV_SUPERFRAME_DURATION_S <= SECONDS_PER_WEEK
        self.subframe.frame_id() == GpsQzssLnavSubframeId::Ephemeris1 && self.page_number() == 1
    }

    /// Returns true if this subframe terminates a 12'30s superframe period.
    pub fn ends_superframe_period(&self) -> bool {
        self.subframe.frame_id() == GpsQzssLnavSubframeId::Almanach5
            && self.page_number() == GPS_LNAV_SUPERFRAME_ITEMS as u8
    }

    /// Interprets the 240 parity free data bits (30 bytes) of a frame.
    fn from_data(data: &[u8; GPS_LNAV_SUBFRAME_DATA_BYTES]) -> Result<Self, GpsError> {
        let mut stream = BitReadStream::new(BitReadBuffer::new(data, BigEndian));

        let telemetry = stream
            .read::<GpsQzssLnavTelemetry>()
            .map_err(|_| GpsError::InvalidPreamble)?;

        let how = stream
            .read::<GpsQzssLnavHow>()
            .map_err(|_| GpsError::UnknownFrameType)?;

        let subframe = GpsQzssLnavSubframeEnum::read(&mut stream, how.sbf_id)?;

        Ok(Self {
            telemetry,
            how,
            subframe,
        })
    }

    /// Encodes this frame as 240 parity free data bits (30 bytes).
    /// The HOW frame ID always matches the [GpsQzssLnavSubframe] actually encoded.
    fn to_data(self) -> Result<[u8; GPS_LNAV_SUBFRAME_DATA_BYTES], GpsError> {
        let mut data = [0u8; GPS_LNAV_SUBFRAME_DATA_BYTES];

        {
            let mut stream = BitWriteStream::from_slice(&mut data, BigEndian);
            stream.write(&self.telemetry)?;
            stream.write(&self.how.with_subframe_id(self.subframe.frame_id()))?;
            stream.write(&self.subframe)?;
        }

        Ok(data)
    }

    /// Interprets 10 transmitted words (30-bit, right aligned).
    /// The polarity is resolved from the preamble (normal 0x8B or inverted 0x74):
    /// an inverted frame is decoded transparently.
    /// Parity verification is optional for the [crate::gps::GpsQzssDecoder] only:
    /// the [BitRead] implementation always verifies it.
    pub(crate) fn from_words(
        words: &[u32; GPS_LNAV_SUBFRAME_WORDS],
        check_parity: bool,
    ) -> Result<Self, GpsError> {
        let preamble = ((words[0] & WORD_MASK) >> 22) as u8;

        // Inverted stream <=> previous word ended with D30* = 1
        let mut prev = if preamble == GPS_LNAV_PREAMBLE_BYTE {
            0
        } else if preamble == GPS_LNAV_INVERTED_PREAMBLE_BYTE {
            WORD_MASK
        } else {
            return Err(GpsError::InvalidPreamble);
        };

        let mut data = [0u8; GPS_LNAV_SUBFRAME_DATA_BYTES];

        for (i, word) in words.iter().enumerate() {
            let word = word & WORD_MASK;
            let d = decode_word(word, prev, check_parity)?;

            data[3 * i] = (d >> 16) as u8;
            data[3 * i + 1] = (d >> 8) as u8;
            data[3 * i + 2] = d as u8;

            // Inverting the whole stream inverts D29*/D30* as well, and
            // parity(d, !D29*, !D30*) == !parity(d, D29*, D30*): an inverted
            // stream is self consistent, nothing special to do from here on.
            prev = word;
        }

        Self::from_data(&data)
    }

    /// Encodes this frame as 10 transmitted words (30-bit, right aligned), parity included.
    /// Words 2 (HOW) and 10 have their bits 23-24 solved, as required by the ICD.
    fn to_words(self) -> Result<[u32; GPS_LNAV_SUBFRAME_WORDS], GpsError> {
        let data = self.to_data()?;

        let mut words = [0u32; GPS_LNAV_SUBFRAME_WORDS];

        // Every subframe is preceded by a word 10 ending with 00
        let mut prev = 0u32;

        for (i, word) in words.iter_mut().enumerate() {
            let d = ((data[3 * i] as u32) << 16)
                | ((data[3 * i + 1] as u32) << 8)
                | (data[3 * i + 2] as u32);

            *word = if i == 1 || i == GPS_LNAV_SUBFRAME_WORDS - 1 {
                encode_word_with_t_bits(d, prev)
            } else {
                encode_word(d, prev)
            };

            prev = *word;
        }

        Ok(words)
    }
}

#[cfg(test)]
mod test {
    use super::{decode_word, encode_word, encode_word_with_t_bits, parity, WORD_MASK};

    use crate::{
        bits::{read_bits, write_bits},
        gps::lnav::{
            subframe::*,
            test_vectors::{
                bytes_to_words, on_air, to_polarity_corrected, words_to_bytes, UBLOX_EPH1,
                UBLOX_EPH2, UBLOX_EPH3,
            },
            GpsError, GpsQzssLnavHow, GpsQzssLnavSubframeEnum, GpsQzssLnavSubframeId,
            GPS_LNAV_PREAMBLE_BYTE, GPS_LNAV_SUBFRAME_BYTES,
        },
        BigEndian, Message,
    };

    use bitbuffer::{BitReadBuffer, BitReadStream, BitWriteStream};

    const ALL_IDS: [GpsQzssLnavSubframeId; 5] = [
        GpsQzssLnavSubframeId::Ephemeris1,
        GpsQzssLnavSubframeId::Ephemeris2,
        GpsQzssLnavSubframeId::Ephemeris3,
        GpsQzssLnavSubframeId::Almanach4,
        GpsQzssLnavSubframeId::Almanach5,
    ];

    /// Encodes with the [Message] trait, verifies the encoded size,
    /// and returns the 38 encoded bytes.
    fn encode(frame: &GpsQzssLnavSubframe) -> [u8; GPS_LNAV_SUBFRAME_BYTES] {
        let mut buffer = [0u8; GPS_LNAV_SUBFRAME_BYTES];
        assert_eq!(frame.encode(&mut buffer).unwrap(), GPS_LNAV_SUBFRAME_BYTES);
        buffer
    }

    #[test]
    fn default_reciprocal() {
        let default = GpsQzssLnavSubframe::default();

        let mut buffer = [0; 1024];
        assert!(
            default.encode(&mut buffer).is_ok(),
            "failed to encode frame"
        );

        assert_eq!(buffer[0], 0x8B, "does not start with preamble");

        let decoded = GpsQzssLnavSubframe::decode(&buffer).unwrap_or_else(|e| {
            panic!("failed to decode frame: {}", e);
        });

        assert_eq!(decoded, default);
    }

    #[test]
    fn model_bit_exact_reciprocal() {
        for frame_id in ALL_IDS {
            let frame = GpsQzssLnavSubframe::model(frame_id);
            let encoded = encode(&frame);

            // 4 padding bits are left null
            assert_eq!(encoded[GPS_LNAV_SUBFRAME_BYTES - 1] & 0x0f, 0);

            let decoded = GpsQzssLnavSubframe::decode(&encoded).unwrap();
            assert_eq!(decoded, frame, "{:?}", frame_id);

            // encode(decode(x)) == x, bit for bit
            assert_eq!(encode(&decoded), encoded, "{:?}", frame_id);
        }
    }

    #[test]
    fn model_reciprocal_any_bit_offset() {
        for frame_id in ALL_IDS {
            let frame = GpsQzssLnavSubframe::model(frame_id);
            let reference = encode(&frame);

            for offset in 0..24 {
                let mut bytes = Vec::new();

                {
                    let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
                    writer
                        .write_int(0xAAAAAAu32 >> (24 - offset), offset)
                        .unwrap();
                    writer.write(&frame).unwrap();
                    assert_eq!(writer.bit_len(), offset + GPS_LNAV_SUBFRAME_BITS);
                    writer.write_int(0x5u8, 3).unwrap();
                }

                // leading bits preserved
                assert_eq!(
                    read_bits(&bytes, 0, offset),
                    Some(0xAAAAAAu32 >> (24 - offset))
                );

                // the 300 bits are identical at any offset
                for bit in 0..GPS_LNAV_SUBFRAME_BITS {
                    assert_eq!(
                        read_bits(&bytes, offset + bit, 1),
                        read_bits(&reference, bit, 1),
                        "{:?} @{} bit #{}",
                        frame_id,
                        offset,
                        bit
                    );
                }

                // trailing bits
                assert_eq!(
                    read_bits(&bytes, offset + GPS_LNAV_SUBFRAME_BITS, 3),
                    Some(0x5)
                );

                let mut reader = BitReadStream::new(BitReadBuffer::new(&bytes, BigEndian));
                reader.set_pos(offset).unwrap();

                let decoded: GpsQzssLnavSubframe = reader.read().unwrap_or_else(|e| {
                    panic!("{:?} @{}: {}", frame_id, offset, e);
                });

                assert_eq!(decoded, frame, "{:?} @{}", frame_id, offset);
                assert_eq!(reader.pos(), offset + GPS_LNAV_SUBFRAME_BITS);
            }
        }
    }

    #[test]
    fn consecutive_subframes_form_a_valid_stream() {
        // frames written back to back: each word's parity depends on
        // the previous word, the HOW and word 10 must end with 00.
        let frames: Vec<_> = ALL_IDS
            .iter()
            .map(|id| GpsQzssLnavSubframe::model(*id))
            .collect();

        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            for frame in frames.iter() {
                writer.write(frame).unwrap();
            }
        }

        let mut prev = 0u32;

        for i in 0..frames.len() * 10 {
            let word = read_bits(&bytes, i * 30, 30).unwrap();
            assert!(
                decode_word(word, prev, true).is_ok(),
                "parity error on word #{}",
                i
            );

            if i % 10 == 1 || i % 10 == 9 {
                assert_eq!(word & 0x3, 0, "word #{} must end with D29=D30=0", i);
            }

            prev = word;
        }

        let mut reader = BitReadStream::new(BitReadBuffer::new(&bytes, BigEndian));

        for frame in frames.iter() {
            assert_eq!(reader.read::<GpsQzssLnavSubframe>().unwrap(), *frame);
        }
    }

    #[test]
    fn inverted_frame() {
        for frame_id in ALL_IDS {
            let frame = GpsQzssLnavSubframe::model(frame_id);
            let mut inverted = encode(&frame);

            for byte in inverted.iter_mut() {
                *byte = !*byte;
            }

            assert_eq!(inverted[0], !GPS_LNAV_PREAMBLE_BYTE);
            assert_eq!(GpsQzssLnavSubframe::decode(&inverted).unwrap(), frame);
        }
    }

    #[test]
    fn buffer_too_small() {
        let frame = GpsQzssLnavSubframe::model(GpsQzssLnavSubframeId::Ephemeris1);

        // does not panic
        let mut buffer = [0u8; GPS_LNAV_SUBFRAME_BYTES - 1];
        assert!(frame.encode(&mut buffer).is_err());

        let encoded = encode(&frame);
        assert!(GpsQzssLnavSubframe::decode(&encoded[..GPS_LNAV_SUBFRAME_BYTES - 1]).is_err());
        assert!(GpsQzssLnavSubframe::decode(&[]).is_err());
    }

    #[test]
    fn invalid_preamble() {
        let mut encoded = encode(&GpsQzssLnavSubframe::model(
            GpsQzssLnavSubframeId::Ephemeris3,
        ));
        encoded[0] = 0x00;
        assert!(GpsQzssLnavSubframe::decode(&encoded).is_err());
    }

    #[test]
    fn parity_error_is_reported() {
        let frame = GpsQzssLnavSubframe::model(GpsQzssLnavSubframeId::Ephemeris2);
        let mut words = bytes_to_words(&encode(&frame));
        words[5] ^= 0x100;

        assert!(GpsQzssLnavSubframe::decode(&words_to_bytes(&words)).is_err());

        assert!(matches!(
            GpsQzssLnavSubframe::from_words(&words, true),
            Err(GpsError::Parity)
        ));

        // without verification, only the corrupted bit is wrong
        assert!(GpsQzssLnavSubframe::from_words(&words, false).is_ok());
    }

    #[test]
    fn single_bit_errors_are_detected() {
        for frame_id in ALL_IDS {
            let encoded = encode(&GpsQzssLnavSubframe::model(frame_id));

            // every single bit of the 300 is protected
            // (preamble bits turn into an invalid preamble)
            for bit in 0..GPS_LNAV_SUBFRAME_BITS {
                let mut corrupted = encoded;
                corrupted[bit / 8] ^= 0x80 >> (bit % 8);
                assert!(
                    GpsQzssLnavSubframe::decode(&corrupted).is_err(),
                    "{:?}: bit #{} flip not detected",
                    frame_id,
                    bit
                );
            }

            // padding bits are not
            let mut padded = encoded;
            padded[GPS_LNAV_SUBFRAME_BYTES - 1] |= 0x0f;
            assert!(GpsQzssLnavSubframe::decode(&padded).is_ok());
        }
    }

    #[test]
    fn parity_real_ublox_frames() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            let frame = on_air(frame);
            let mut prev = 0;
            for (i, word) in frame.iter().enumerate() {
                assert!(
                    decode_word(*word, prev, true).is_ok(),
                    "parity error on word #{} (0x{:08X})",
                    i + 1,
                    word
                );
                prev = *word;
            }

            // HOW and word 10 have D29 = D30 = 0
            assert_eq!(frame[1] & 0x3, 0);
            assert_eq!(frame[9] & 0x3, 0);
        }
    }

    #[test]
    fn word_reciprocal() {
        // encode_word / decode_word are mirror operations, for any D29* / D30*
        for data in [0x000000, 0xFFFFFF, 0x8B0000, 0x123456, 0xABCDEF, 0x555555] {
            for prev in 0..4 {
                let word = encode_word(data, prev);
                assert_eq!(word & !WORD_MASK, 0);
                assert_eq!(decode_word(word, prev, true).unwrap(), data);

                // D30* inverts the transmitted data bits
                if prev & 0x1 > 0 {
                    assert_eq!(word >> 6, !data & 0xff_ffff);
                } else {
                    assert_eq!(word >> 6, data);
                }

                assert_eq!(
                    (word & 0x3f) as u8,
                    parity(data, prev & 0x2 > 0, prev & 0x1 > 0)
                );
            }
        }
    }

    #[test]
    fn t_bits() {
        for (data, prev) in [(0x123400, 0), (0xABCDEF, 1), (0x000000, 2), (0xFFFFFF, 3)] {
            let word = encode_word_with_t_bits(data, prev);
            assert_eq!(word & 0x3, 0);
            let decoded = decode_word(word, prev, true).unwrap();
            assert_eq!(decoded & !0x3, data & !0x3);
        }
    }

    #[test]
    fn real_ublox_frames_bit_exact() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            // transmitted bit stream
            let bytes = words_to_bytes(&on_air(frame));
            assert_eq!(bytes[0], GPS_LNAV_PREAMBLE_BYTE);

            let decoded = GpsQzssLnavSubframe::decode(&bytes).unwrap();

            // re-encoding the frame must give back the exact same bit stream
            assert_eq!(encode(&decoded), bytes);
            assert_eq!(to_polarity_corrected(&bytes_to_words(&bytes)), frame);
        }
    }

    #[test]
    fn real_ublox_frames() {
        let decode = |words| GpsQzssLnavSubframe::decode(&words_to_bytes(&on_air(words))).unwrap();

        // EPH-1
        let frame = decode(UBLOX_EPH1);

        assert_eq!(frame.telemetry.message, 0x13E);
        assert!(!frame.telemetry.integrity);
        assert!(!frame.telemetry.reserved_bit);
        assert!(!frame.how.alert);
        assert!(frame.how.anti_spoofing);
        assert_eq!(frame.how.sbf_id, GpsQzssLnavSubframeId::Ephemeris1);
        assert_eq!(frame.how.tow, 259_956);

        let eph1 = frame.subframe.as_eph1().unwrap();
        assert_eq!(eph1.week, 318);
        assert_eq!(eph1.toc, 266_400);
        assert!(eph1.health.is_healthy());
        assert!((eph1.af1 - 1.023181539495E-11).abs() < 1e-14);
        assert!((eph1.af0 - -4.524961113930E-04).abs() < 1.0e-11);
        assert_eq!(eph1.af2, 0.0);

        // EPH-2
        let frame = decode(UBLOX_EPH2);
        assert_eq!(frame.how.sbf_id, GpsQzssLnavSubframeId::Ephemeris2);

        let eph2 = frame.subframe.as_eph2().unwrap();
        assert_eq!(eph2.toe, 266_400);
        assert_eq!(eph2.crs, -1.843750000000e+000);
        assert!((eph2.sqrt_a - 5.153602432251e+003).abs() < 1e-9);
        assert!((eph2.m0 - 9.768415465951e-001).abs() < 1e-9);
        assert!((eph2.cuc - -5.587935447693e-008).abs() < 1e-9);
        assert!((eph2.e - 8.578718174249e-003).abs() < 1e-9);
        assert!((eph2.cus - 8.093193173409e-006).abs() < 1e-9);
        assert!((eph2.dn - 1.444277586415e-009).abs() < 1e-9);
        assert!(!eph2.fit_int_flag);

        // EPH-3
        let frame = decode(UBLOX_EPH3);
        assert_eq!(frame.how.sbf_id, GpsQzssLnavSubframeId::Ephemeris3);

        let eph3 = frame.subframe.as_eph3().unwrap();
        assert!((eph3.cic - 8.009374141693e-008).abs() < 1e-9);
        assert!((eph3.cis - -1.955777406693e-007).abs() < 1E-9);
        assert!((eph3.crc - 2.225625000000e+002).abs() < 1E-9);
        assert!((eph3.i0 - 3.070601043291e-001).abs() < 1e-9);
        assert!((eph3.idot - 1.548414729768e-010).abs() < 1E-9);
        assert!((eph3.omega0 - -6.871047024615e-001).abs() < 1e-9);
        assert!((eph3.omega_dot - -2.449269231874e-009).abs() < 1e-9);
        assert!((eph3.omega - -6.554632573389e-001).abs() < 1e-9);
    }

    #[test]
    fn hand_over_word_frame_id_follows_subframe() {
        // a HOW announcing another subframe is corrected when encoding
        let mut frame = GpsQzssLnavSubframe::model(GpsQzssLnavSubframeId::Ephemeris2);
        frame.how.sbf_id = GpsQzssLnavSubframeId::Almanach5;

        let decoded = GpsQzssLnavSubframe::decode(&encode(&frame)).unwrap();
        assert_eq!(decoded.how.sbf_id, GpsQzssLnavSubframeId::Ephemeris2);
        assert_eq!(decoded.subframe, frame.subframe);
    }

    #[test]
    fn raw_almanach_subframes() {
        // raw data bits, interpreted (as reserved pages: SV ID 0 and 63)
        // through the Message trait of the subframe 4 / 5 pages
        let raw4: [u8; GPS_LNAV_SUBFRAME_DATA_BYTES] = core::array::from_fn(|i| i as u8 * 7);
        let raw5: [u8; GPS_LNAV_SUBFRAME_DATA_BYTES] = core::array::from_fn(|i| 0xff - i as u8);

        for (frame_id, subframe, expected) in [
            (
                GpsQzssLnavSubframeId::Almanach4,
                GpsQzssLnavSubframeEnum::Almanach4(GpsQzssLnavSubframe4::decode(&raw4).unwrap()),
                raw4,
            ),
            (
                GpsQzssLnavSubframeId::Almanach5,
                GpsQzssLnavSubframeEnum::Almanach5(GpsQzssLnavSubframe5::decode(&raw5).unwrap()),
                raw5,
            ),
        ] {
            let frame = GpsQzssLnavSubframe::model(frame_id).with_subframe_enum(subframe);
            let encoded = encode(&frame);
            let decoded = GpsQzssLnavSubframe::decode(&encoded).unwrap();

            // the 2 last bits of word 10 (t-bits) are solved for the parity:
            // they are decoded as zeros
            let mut raw = [0u8; GPS_LNAV_SUBFRAME_DATA_BYTES];

            match (decoded.subframe, subframe) {
                (GpsQzssLnavSubframeEnum::Almanach4(a), GpsQzssLnavSubframeEnum::Almanach4(b)) => {
                    assert!(a.as_reserved().is_some());
                    assert_eq!(a, b);
                    a.encode(&mut raw).unwrap();
                },
                (GpsQzssLnavSubframeEnum::Almanach5(a), GpsQzssLnavSubframeEnum::Almanach5(b)) => {
                    assert!(a.as_reserved().is_some());
                    assert_eq!(a, b);
                    a.encode(&mut raw).unwrap();
                },
                _ => panic!("wrong subframe type"),
            };

            assert_eq!(raw[..23], expected[..23]);
            assert_eq!(raw[23], expected[23] & 0xfc);

            // the bit stream is preserved
            assert_eq!(encode(&decoded), encoded);

            // and decoding is now stable
            assert_eq!(
                GpsQzssLnavSubframe::decode(&encode(&decoded)).unwrap(),
                decoded
            );
        }
    }

    #[test]
    fn random_data_bits_reciprocal() {
        // any 240 data bits (valid preamble and frame ID) are interpreted, and
        // re-encoded bit for bit: no field is lost or altered by the interpretation.
        // Only the t-bits (bits 23-24 of words 2 and 10) are solved for the parity.
        let mut seed = 0x1234_5678u32;

        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as u8
        };

        for frame_id in 1..=5u8 {
            for _ in 0..256 {
                let mut data = [0u8; GPS_LNAV_SUBFRAME_DATA_BYTES];

                for byte in data.iter_mut() {
                    *byte = rand();
                }

                data[0] = GPS_LNAV_PREAMBLE_BYTE;

                // HOW: frame ID (bits 20-22 of word 2), t-bits null
                data[5] = (data[5] & 0xe0) | (frame_id << 2);
                data[29] &= 0xfc;

                let frame = GpsQzssLnavSubframe::from_data(&data).unwrap();
                assert_eq!(frame.to_data().unwrap(), data, "frame #{}", frame_id);

                // and through the Message trait (parity)
                let encoded = encode(&frame);
                assert_eq!(GpsQzssLnavSubframe::decode(&encoded).unwrap(), frame);
                assert_eq!(
                    encode(&GpsQzssLnavSubframe::decode(&encoded).unwrap()),
                    encoded
                );
            }
        }
    }

    #[test]
    fn bit_io_30_bits() {
        for offset in 0..16 {
            let mut buf = [0u8; 16];
            write_bits(&mut buf, offset, 0x2AB_CDEF5 & WORD_MASK, 30).unwrap();
            assert_eq!(read_bits(&buf, offset, 30), Some(0x2AB_CDEF5 & WORD_MASK));
        }
    }

    #[test]
    fn page_number() {
        for (eph1_week_t, page_num) in [
            (0, 1),
            (30, 2),
            (60, 3),
            (90, 4),
            (120, 5),
            (720, 25),
            (750, 1),
            (780, 2),
            (810, 3),
            (3600, 21),
            (86_400, 6),
            (302_400, 6),
            (603_750, 1),
            (604_500, 1),
            (604_770, 10),
        ] {
            for id in 1..=5 {
                let how = GpsQzssLnavHow::default().with_tow_seconds(
                    (eph1_week_t + id * GPS_LNAV_SUBFRAME_DURATION_S) % SECONDS_PER_WEEK,
                );

                let sbf_id = GpsQzssLnavSubframeId::decode(id as u8).unwrap();

                let sbf = match id {
                    1 => GpsQzssLnavSubframeEnum::Ephemeris1(Default::default()),
                    2 => GpsQzssLnavSubframeEnum::Ephemeris2(Default::default()),
                    3 => GpsQzssLnavSubframeEnum::Ephemeris3(Default::default()),
                    4 => GpsQzssLnavSubframeEnum::Almanach4(Default::default()),
                    _ => GpsQzssLnavSubframeEnum::Almanach5(Default::default()),
                };

                let sbf = GpsQzssLnavSubframe::default()
                    .with_hand_over_word(how.with_subframe_id(sbf_id))
                    .with_subframe_enum(sbf);

                println!("{:?}", sbf);

                assert_eq!(
                    sbf.week_transmission_time_s(),
                    (eph1_week_t + (id - 1) * GPS_LNAV_SUBFRAME_DURATION_S) % SECONDS_PER_WEEK
                );

                assert_eq!(sbf.page_number(), page_num);
            }
        }
    }
}
