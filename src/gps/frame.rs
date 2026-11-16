use crate::{
    gps::{
        link::{self, GPS_FRAME_DATA_BYTES},
        GpsError, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
        GPS_WORDS_PER_FRAME, GPS_WORD_BITS,
    },
    Message,
};

#[cfg(test)]
use crate::gps::GpsQzssFrameId;

use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadBuffer, BitReadStream, BitWrite, BitWriteStream,
};

/// GPS / QZSS interpreted frame (one 300-bit subframe).
///
/// A frame exists under three representations:
/// - the transmitted bit stream: 10 words of 30 bits, with parity
///   ([Self::decode_words], [Self::encode_words], [Self::decode_at], [Self::encode_at],
///   and the [BitRead] / [BitWrite] implementations).
/// - the 240 parity free data bits ([Self::decode_data], [Self::encode_data]).
/// - this interpreted structure.
///
/// Parity and polarity are only handled by the [link] layer, the
/// messages ([GpsQzssTelemetry], [GpsQzssHow], [GpsQzssSubframe]) only
/// see the parity free data bits.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame {
    /// [GpsQzssTelemetry] describes the following frame and contains
    /// the sync-byte, therefore initiates a [GpsQzssFrame].
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,

    /// [GpsQzssSubframe] depends on associated [GpsQzssHow].
    pub subframe: GpsQzssSubframe,
}

impl Message<BigEndian> for GpsQzssFrame {
    fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GPS_FRAME_BITS
    }
}

/// [BitRead] only allows [BitError]s: express link errors as such.
fn link_error(e: GpsError) -> BitError {
    match e {
        GpsError::BufferRead(e) => e,
        e => BitError::UnmatchedDiscriminant {
            discriminant: 0,
            enum_name: e.to_string(),
        },
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame {
    /// Reads a complete 300-bit frame (with parity) from the stream, at its current position.
    /// The parity is always verified. Inverted frames are supported.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let mut words = [0u32; GPS_WORDS_PER_FRAME];

        for word in words.iter_mut() {
            *word = stream.read_int::<u32>(GPS_WORD_BITS)?;
        }

        Self::decode_words(&words, true).map_err(link_error)
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame {
    /// Writes a complete 300-bit frame (with parity) to the stream.
    /// Frames written one after the other form a valid GPS bit stream.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        let words = self.encode_words().map_err(link_error)?;

        for word in words.iter() {
            stream.write_int(*word, GPS_WORD_BITS)?;
        }

        Ok(())
    }
}

impl GpsQzssFrame {
    #[cfg(test)]
    pub(crate) fn model(frame_id: GpsQzssFrameId) -> Self {
        Self::default()
            .with_telemetry(GpsQzssTelemetry::model())
            .with_hand_over_word(GpsQzssHow::model(frame_id))
            .with_subframe(GpsQzssSubframe::model(frame_id))
    }

    /// Copies and returns with updated [GpsQzssHow].
    pub fn with_hand_over_word(mut self, how: GpsQzssHow) -> Self {
        self.how = how;
        self.how.frame_id = self.subframe.frame_id();
        self
    }

    /// Copies and returns with updated [GpsQzssTelemetry] data word
    pub fn with_telemetry(mut self, telemetry: GpsQzssTelemetry) -> Self {
        self.telemetry = telemetry;
        self
    }

    /// Copies and returns an updated [GpsQzssSubframe].
    /// The frame ID of the [GpsQzssHow] is updated accordingly.
    pub fn with_subframe(mut self, subframe: GpsQzssSubframe) -> Self {
        self.subframe = subframe;
        self.how.frame_id = subframe.frame_id();
        self
    }

    /// Interprets the 240 parity free data bits (30 bytes) of a frame.
    pub fn decode_data(data: &[u8; GPS_FRAME_DATA_BYTES]) -> Result<Self, GpsError> {
        let mut stream = BitReadStream::new(BitReadBuffer::new(data, BigEndian));

        let telemetry = stream
            .read::<GpsQzssTelemetry>()
            .map_err(|_| GpsError::InvalidPreamble)?;

        let how = stream
            .read::<GpsQzssHow>()
            .map_err(|_| GpsError::UnknownFrameType)?;

        let subframe = GpsQzssSubframe::read(&mut stream, how.frame_id)?;

        Ok(Self {
            telemetry,
            how,
            subframe,
        })
    }

    /// Encodes this frame as 240 parity free data bits (30 bytes).
    /// The HOW frame ID always matches the [GpsQzssSubframe] actually encoded.
    pub fn encode_data(&self) -> Result<[u8; GPS_FRAME_DATA_BYTES], GpsError> {
        let mut data = [0u8; GPS_FRAME_DATA_BYTES];

        {
            let mut stream = BitWriteStream::from_slice(&mut data, BigEndian);
            stream.write(&self.telemetry)?;
            stream.write(&self.how.with_frame_id(self.subframe.frame_id()))?;
            stream.write(&self.subframe)?;
        }

        Ok(data)
    }

    /// Interprets 10 transmitted words (30-bit, right aligned).
    /// Inverted frames (preamble 0x74) are supported.
    pub fn decode_words(
        words: &[u32; GPS_WORDS_PER_FRAME],
        check_parity: bool,
    ) -> Result<Self, GpsError> {
        let data = link::decode_frame(words, check_parity)?;
        Self::decode_data(&data)
    }

    /// Interprets 10 words as reported by receivers that already corrected
    /// the polarity of each word (u-blox UBX-RXM-SFRBX for example).
    /// See [link::from_polarity_corrected].
    pub fn decode_polarity_corrected_words(
        words: &[u32; GPS_WORDS_PER_FRAME],
        check_parity: bool,
    ) -> Result<Self, GpsError> {
        Self::decode_words(&link::from_polarity_corrected(words), check_parity)
    }

    /// Encodes this frame as 10 transmitted words (30-bit, right aligned),
    /// parity included. Words 2 and 10 end with D29 = D30 = 0 as required by the ICD,
    /// so frames can be concatenated as is.
    pub fn encode_words(&self) -> Result<[u32; GPS_WORDS_PER_FRAME], GpsError> {
        let data = self.encode_data()?;
        Ok(link::encode_frame(&data))
    }

    /// Decodes a frame starting at any bit position (`bit_offset`) of `slice`.
    /// The frame must start at that position exactly: use the [crate::gps::GpsQzssDecoder]
    /// to search for it.
    pub fn decode_at(
        slice: &[u8],
        bit_offset: usize,
        check_parity: bool,
    ) -> Result<Self, GpsError> {
        let words = link::read_words(slice, bit_offset).ok_or(GpsError::WouldNotFit)?;
        Self::decode_words(&words, check_parity)
    }

    /// Encodes this frame at any bit position (`bit_offset`) of `slice`.
    /// Bits surrounding the frame are preserved.
    /// Returns the number of bits written (always [GPS_FRAME_BITS]).
    pub fn encode_at(&self, slice: &mut [u8], bit_offset: usize) -> Result<usize, GpsError> {
        let words = self.encode_words()?;
        link::write_words(slice, bit_offset, &words).ok_or(GpsError::WouldNotFit)?;
        Ok(GPS_FRAME_BITS)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{link, GpsError, GpsQzssFrame, GpsQzssFrameId, GPS_FRAME_BITS},
        Message,
    };

    const ALL_IDS: [GpsQzssFrameId; 5] = [
        GpsQzssFrameId::Ephemeris1,
        GpsQzssFrameId::Ephemeris2,
        GpsQzssFrameId::Ephemeris3,
        GpsQzssFrameId::Almanach4,
        GpsQzssFrameId::Almanach5,
    ];

    #[test]
    fn default_reciprocal() {
        let default = GpsQzssFrame::default();

        let mut buffer = [0; 1024];
        assert!(
            default.encode(&mut buffer).is_ok(),
            "failed to encode frame"
        );

        assert_eq!(buffer[0], 0x8B, "does not start with preamble");

        let decoded = GpsQzssFrame::decode(&buffer).unwrap_or_else(|e| {
            panic!("failed to decode frame: {}", e);
        });

        assert_eq!(decoded, default);
    }

    #[test]
    fn model_reciprocal_any_bit_offset() {
        for frame_id in ALL_IDS {
            let frame = GpsQzssFrame::model(frame_id);

            for offset in 0..24 {
                let mut buffer = [0xAAu8; 64];
                let size = frame.encode_at(&mut buffer, offset).unwrap();
                assert_eq!(size, GPS_FRAME_BITS);

                // surrounding bits preserved
                assert_eq!(
                    link::read_bits(&buffer, 0, offset),
                    Some(0xAAAAAAu32 >> (24 - offset))
                );

                let decoded = GpsQzssFrame::decode_at(&buffer, offset, true).unwrap_or_else(|e| {
                    panic!("{:?} @{}: {}", frame_id, offset, e);
                });

                assert_eq!(decoded, frame, "{:?} @{}", frame_id, offset);
            }
        }
    }

    #[test]
    fn buffer_too_small() {
        let frame = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);
        let mut buffer = [0u8; 38];
        assert!(frame.encode_at(&mut buffer, 4).is_ok());
        assert!(matches!(
            frame.encode_at(&mut buffer, 5),
            Err(GpsError::WouldNotFit)
        ));
        assert!(matches!(
            GpsQzssFrame::decode_at(&buffer, 5, true),
            Err(GpsError::WouldNotFit)
        ));
    }

    #[test]
    fn parity_error_is_reported() {
        let frame = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris2);
        let mut words = frame.encode_words().unwrap();
        words[5] ^= 0x100;

        assert!(matches!(
            GpsQzssFrame::decode_words(&words, true),
            Err(GpsError::Parity)
        ));

        // without verification, only the corrupted bit is wrong
        assert!(GpsQzssFrame::decode_words(&words, false).is_ok());
    }

    #[test]
    fn real_ublox_frames() {
        use link::test::{UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3};

        // EPH-1
        let frame = GpsQzssFrame::decode_polarity_corrected_words(&UBLOX_EPH1, true).unwrap();

        assert_eq!(frame.telemetry.message, 0x13E);
        assert!(!frame.telemetry.integrity);
        assert!(!frame.telemetry.reserved_bit);
        assert!(!frame.how.alert);
        assert!(frame.how.anti_spoofing);
        assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris1);
        assert_eq!(frame.how.tow, 259_956);

        let eph1 = frame.subframe.as_eph1().unwrap();
        assert_eq!(eph1.week, 318);
        assert_eq!(eph1.toc, 266_400);
        assert!(eph1.health.is_healthy());
        assert!((eph1.af1 - 1.023181539495E-11).abs() < 1e-14);
        assert!((eph1.af0 - -4.524961113930E-04).abs() < 1.0e-11);
        assert_eq!(eph1.af2, 0.0);

        // bit exact re-encoding
        assert_eq!(
            link::to_polarity_corrected(&frame.encode_words().unwrap()),
            UBLOX_EPH1
        );

        // EPH-2
        let frame = GpsQzssFrame::decode_polarity_corrected_words(&UBLOX_EPH2, true).unwrap();
        assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris2);

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

        assert_eq!(
            link::to_polarity_corrected(&frame.encode_words().unwrap()),
            UBLOX_EPH2
        );

        // EPH-3
        let frame = GpsQzssFrame::decode_polarity_corrected_words(&UBLOX_EPH3, true).unwrap();
        assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris3);

        let eph3 = frame.subframe.as_eph3().unwrap();
        assert!((eph3.cic - 8.009374141693e-008).abs() < 1e-9);
        assert!((eph3.cis - -1.955777406693e-007).abs() < 1E-9);
        assert!((eph3.crc - 2.225625000000e+002).abs() < 1E-9);
        assert!((eph3.i0 - 3.070601043291e-001).abs() < 1e-9);
        assert!((eph3.idot - 1.548414729768e-010).abs() < 1E-9);
        assert!((eph3.omega0 - -6.871047024615e-001).abs() < 1e-9);
        assert!((eph3.omega_dot - -2.449269231874e-009).abs() < 1e-9);
        assert!((eph3.omega - -6.554632573389e-001).abs() < 1e-9);

        assert_eq!(
            link::to_polarity_corrected(&frame.encode_words().unwrap()),
            UBLOX_EPH3
        );
    }
}
