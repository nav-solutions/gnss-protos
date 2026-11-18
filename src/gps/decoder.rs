use crate::{
    bits::get_bit,
    gps::{
        frame::GPS_WORD_MASK, GpsQzssFrame, GPS_FRAME_BITS, GPS_INVERTED_PREAMBLE_BYTE,
        GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME,
    },
};

#[cfg(feature = "log")]
use log::trace;

/// [GpsQzssDecoder] synchronizes to a GPS / QZSS bit stream and decodes
/// [GpsQzssFrame]s, wherever they are located (any bit alignment).
///
/// It works as a 300-bit shift register, fed one bit at a time
/// ([Self::push_bit], typical of a tracking loop delivering one navigation bit
/// every 20 ms) or from a byte slice at any bit offset ([Self::decode]).
/// Each time the register starts with a preamble (normal or inverted, the carrier
/// phase ambiguity is resolved), the 10 words are verified and interpreted.
///
/// The decoder has a small constant memory footprint and does not allocate.
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssDecoder {
    /// 300-bit shift register: 10 words of 30 bits, oldest word first.
    words: [u32; GPS_WORDS_PER_FRAME],

    /// Number of bits accumulated since last synchronization (saturates at 300).
    fill: usize,

    /// True when parity should be verified (default).
    parity_verification: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a new [GpsQzssDecoder] with parity verification.
    fn default() -> Self {
        Self {
            words: [0; GPS_WORDS_PER_FRAME],
            fill: 0,
            parity_verification: true,
        }
    }
}

impl GpsQzssDecoder {
    /// Copies and returns [GpsQzssDecoder] with parity verification (default).
    pub fn with_parity_verification(mut self) -> Self {
        self.parity_verification = true;
        self
    }

    /// Copies and returns [GpsQzssDecoder] without parity verification.
    /// Not recommended: the preamble alone (8 bits) is a weak synchronization
    /// pattern and data bits will regularly look like a preamble.
    pub fn without_parity_verification(mut self) -> Self {
        self.parity_verification = false;
        self
    }

    /// Drops the current synchronization state.
    pub fn reset(&mut self) {
        self.words = [0; GPS_WORDS_PER_FRAME];
        self.fill = 0;
    }

    /// Pushes a new bit into the decoder.
    /// Returns a [GpsQzssFrame] when this bit completes a valid frame.
    pub fn push_bit(&mut self, bit: bool) -> Option<GpsQzssFrame> {
        let mut carry = bit as u32;

        for word in self.words.iter_mut().rev() {
            let msb = (*word >> 29) & 0x1;
            *word = ((*word << 1) | carry) & GPS_WORD_MASK;
            carry = msb;
        }

        if self.fill < GPS_FRAME_BITS {
            self.fill += 1;
        }

        if self.fill < GPS_FRAME_BITS {
            return None;
        }

        let preamble = (self.words[0] >> 22) as u8;

        if preamble != GPS_PREAMBLE_BYTE && preamble != GPS_INVERTED_PREAMBLE_BYTE {
            return None;
        }

        match GpsQzssFrame::from_words(&self.words, self.parity_verification) {
            Ok(frame) => {
                // a frame was consumed: the next one needs 300 new bits
                self.fill = 0;
                Some(frame)
            },
            #[cfg(feature = "log")]
            Err(e) => {
                trace!("(GPS/QZSS) preamble candidate rejected: {}", e);
                None
            },
            #[cfg(not(feature = "log"))]
            Err(_) => None,
        }
    }

    /// Feeds bits from `slice`, starting at absolute bit position `bit_offset`,
    /// until a frame is decoded or the slice is exhausted.
    ///
    /// Returns the number of bits consumed, and the decoded frame if any.
    /// The decoder state is preserved between calls: a frame may span
    /// several slices. To decode a complete buffer:
    ///
    /// ```
    /// use gnss_protos::{GpsQzssDecoder, GpsQzssFrame};
    ///
    /// let bytes = [0u8; 128]; // any input
    /// let mut decoder = GpsQzssDecoder::default();
    /// let mut position = 0; // in bits!
    ///
    /// loop {
    ///     let (consumed, frame) = decoder.decode(&bytes, position);
    ///     position += consumed;
    ///
    ///     if let Some(frame) = frame {
    ///         // a frame ended at bit `position`
    ///         // (it started at `position - GPS_FRAME_BITS`)
    ///     } else {
    ///         break; // slice exhausted
    ///     }
    /// }
    /// ```
    pub fn decode(&mut self, slice: &[u8], bit_offset: usize) -> (usize, Option<GpsQzssFrame>) {
        let total = slice.len() * 8;
        let mut pos = bit_offset;

        while pos < total {
            let bit = get_bit(slice, pos);
            pos += 1;

            if let Some(frame) = self.push_bit(bit) {
                return (pos - bit_offset, Some(frame));
            }
        }

        (pos.saturating_sub(bit_offset), None)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        bits::get_bit,
        gps::{
            test_vectors::{on_air, UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3},
            GpsQzssDecoder, GpsQzssFrame, GpsQzssFrameId, GpsQzssSubframe, GPS_FRAME_BITS,
            GPS_WORD_BITS,
        },
        BigEndian,
    };

    use bitbuffer::BitWriteStream;

    /// Builds a realistic stream: 5 consecutive subframes (1 to 5),
    /// with evolving TLM/HOW contents.
    fn stream_model(num_frames: usize) -> Vec<GpsQzssFrame> {
        let ids = [
            GpsQzssFrameId::Ephemeris1,
            GpsQzssFrameId::Ephemeris2,
            GpsQzssFrameId::Ephemeris3,
            GpsQzssFrameId::Almanach4,
            GpsQzssFrameId::Almanach5,
        ];

        (0..num_frames)
            .map(|i| {
                let mut frame = GpsQzssFrame::model(ids[i % 5]);
                frame.telemetry.message = (i as u16) & 0x3fff;
                frame.telemetry.integrity = i % 2 == 0;
                frame.how.tow = 6 * (100 + i as u32);
                frame.how.alert = i % 3 == 0;

                if let GpsQzssSubframe::Ephemeris1(eph1) = &mut frame.subframe {
                    eph1.week = (i as u16) & 0x3ff;
                }

                frame
            })
            .collect()
    }

    /// Pseudo random bit generator
    struct Noise(u16);

    impl Noise {
        fn bit(&mut self) -> bool {
            self.0 = (self.0 >> 1) ^ (-((self.0 & 1) as i16) as u16 & 0xB400);
            self.0 & 0x1 > 0
        }
    }

    /// Streams the frames contiguously, after `offset` bits of noise,
    /// followed by a few bits of noise.
    fn encode_stream(frames: &[GpsQzssFrame], offset: usize) -> Vec<u8> {
        let mut noise = Noise(0xACE1);
        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);

            for _ in 0..offset {
                writer.write_bool(noise.bit()).unwrap();
            }

            for frame in frames.iter() {
                writer.write(frame).unwrap();
            }

            for _ in 0..13 {
                writer.write_bool(noise.bit()).unwrap();
            }
        }

        bytes
    }

    fn decode_all(decoder: &mut GpsQzssDecoder, buffer: &[u8]) -> Vec<(usize, GpsQzssFrame)> {
        let mut ret = Vec::new();
        let mut position = 0;

        loop {
            let (consumed, frame) = decoder.decode(buffer, position);
            position += consumed;

            match frame {
                Some(frame) => ret.push((position - GPS_FRAME_BITS, frame)),
                None => return ret,
            }
        }
    }

    #[test]
    fn synchronization_any_bit_offset() {
        let frames = stream_model(10);

        for offset in 0..64 {
            let buffer = encode_stream(&frames, offset);
            let mut decoder = GpsQzssDecoder::default();
            let decoded = decode_all(&mut decoder, &buffer);

            assert_eq!(decoded.len(), frames.len(), "offset={}", offset);

            for (i, (position, frame)) in decoded.iter().enumerate() {
                assert_eq!(*position, offset + i * GPS_FRAME_BITS, "offset={}", offset);
                assert_eq!(*frame, frames[i], "offset={} frame #{}", offset, i);
            }
        }
    }

    #[test]
    fn inverted_stream() {
        let frames = stream_model(6);

        for offset in [0, 3, 11] {
            let buffer: Vec<u8> = encode_stream(&frames, offset).iter().map(|b| !b).collect();

            let mut decoder = GpsQzssDecoder::default();
            let decoded = decode_all(&mut decoder, &buffer);

            assert_eq!(decoded.len(), frames.len());

            for (i, (_, frame)) in decoded.iter().enumerate() {
                assert_eq!(*frame, frames[i]);
            }
        }
    }

    #[test]
    fn bit_by_bit_and_split_slices() {
        let frames = stream_model(5);
        let buffer = encode_stream(&frames, 5);

        // bit by bit
        let mut decoder = GpsQzssDecoder::default();
        let mut decoded = Vec::new();

        for pos in 0..buffer.len() * 8 {
            if let Some(frame) = decoder.push_bit(get_bit(&buffer, pos)) {
                decoded.push(frame);
            }
        }

        assert_eq!(decoded, frames);

        // random sized chunks: frames span several slices
        let mut decoder = GpsQzssDecoder::default();
        let mut decoded = Vec::new();

        for chunk in buffer.chunks(7) {
            let mut position = 0;
            loop {
                let (consumed, frame) = decoder.decode(chunk, position);
                position += consumed;
                match frame {
                    Some(frame) => decoded.push(frame),
                    None => break,
                }
            }
        }

        assert_eq!(decoded, frames);
    }

    #[test]
    fn corrupted_frame_is_dropped() {
        let frames = stream_model(5);
        let mut buffer = encode_stream(&frames, 9);

        // flip one bit in the middle of frame #2 (data bit of word 4)
        let bit = 9 + 2 * GPS_FRAME_BITS + 3 * GPS_WORD_BITS + 10;
        buffer[bit / 8] ^= 0x80 >> (bit % 8);

        let mut decoder = GpsQzssDecoder::default();
        let decoded: Vec<_> = decode_all(&mut decoder, &buffer)
            .into_iter()
            .map(|(_, f)| f)
            .collect();

        assert_eq!(decoded.len(), 4);
        assert_eq!(decoded[0], frames[0]);
        assert_eq!(decoded[1], frames[1]);
        assert_eq!(decoded[2], frames[3]);
        assert_eq!(decoded[3], frames[4]);

        // without parity verification, the corrupted frame is kept
        let mut decoder = GpsQzssDecoder::default().without_parity_verification();
        let decoded = decode_all(&mut decoder, &buffer);
        assert!(decoded.len() >= 5);
        assert!(decoded.iter().any(|(_, f)| *f == frames[3]));
    }

    #[test]
    fn reset() {
        let frames = stream_model(2);
        let buffer = encode_stream(&frames, 0);

        // interrupted in the middle of the first frame
        let mut decoder = GpsQzssDecoder::default();
        for pos in 0..GPS_FRAME_BITS / 2 {
            assert!(decoder.push_bit(get_bit(&buffer, pos)).is_none());
        }

        decoder.reset();

        // the end of the first frame is lost, the second one is decoded
        let decoded: Vec<_> = decode_all(&mut decoder, &buffer[GPS_FRAME_BITS / 2 / 8..])
            .into_iter()
            .map(|(_, f)| f)
            .collect();

        assert_eq!(decoded, vec![frames[1]]);
    }

    #[test]
    fn real_ublox_stream() {
        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            writer.write_int(0u16, 13).unwrap();

            for words in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
                for word in on_air(words).iter() {
                    writer.write_int(*word, GPS_WORD_BITS).unwrap();
                }
            }

            writer.write_int(0u8, 8).unwrap();
        }

        let mut decoder = GpsQzssDecoder::default();
        let decoded = decode_all(&mut decoder, &bytes);

        assert_eq!(decoded.len(), 3);
        assert_eq!(decoded[0].0, 13);
        assert!(decoded[0].1.subframe.as_eph1().is_some());
        assert!(decoded[1].1.subframe.as_eph2().is_some());
        assert!(decoded[2].1.subframe.as_eph3().is_some());

        // bit exact: the decoded frames re-encode into the very same stream
        let mut reencoded = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut reencoded, BigEndian);
            writer.write_int(0u16, 13).unwrap();

            for (_, frame) in decoded.iter() {
                writer.write(frame).unwrap();
            }

            writer.write_int(0u8, 8).unwrap();
        }

        assert_eq!(reencoded, bytes);
    }
}
