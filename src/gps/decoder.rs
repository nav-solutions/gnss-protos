use crate::gps::{
    GpsDataWord, GpsQzssFrame, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS,
    GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME,
};

#[cfg(feature = "log")]
use log::{debug, error, trace};

/// [GpsQzssDecoder] can decode GPS (or QZSS) messages.
/// By [Default], our [GpsQzssDecoder] does not verify parity,
/// so does not invalid any message.
///
/// ```
/// use std::fs::File;
/// use std::io::Read;
///
/// use gnss_protos::gps::{GpsDecoder, GPS_FRAME_BITS};
///
/// // Feeds some of our GPS messages example,
/// // which is equivalent to real-time acquisition
///
/// let mut buffer = [0u8; 1024];
///
/// let mut fd = File::open("data/GPS/eph1.bin")
///     .unwrap();
///
/// read(&mut buffer).unwrap();
///
/// // The decoder does not verify parity at the moment
/// let mut decoder = GpsDecoder::default();
///
/// // parse first message
/// let (size, decoded) = decoder.decode(&buffer);
///
/// // this is not exactly std/io compatible, because we consider bits
/// // here, not bytes.
/// assert_eq!(size, GPS_FRAME_BITS);
///
/// let decoded = decoded.unwrap(); // message found
///
/// assert_eq!(decoded.telemetry.message, 0x1234);
/// assert_eq!(decoded.telemetry.integrity, true);
/// assert_eq!(decoded.telemetry.reserved_bit, true);
///
/// assert_eq!(decoded.how.tow, 0x5_6789);
/// assert_eq!(decoded.how.alert, true);
/// assert_eq!(decoded.how.anti_spoofing, true);
///
/// // we have 128 frames in this file
/// for i in 1..128 {
///     let (size, decoded) = decoder.decode(&buffer);
///     assert_eq!(size, GPS_FRAME_BITS);
///
///     // test pattern
///     assert_eq!(decoded.telemetry.message, 0x1234 +i);
///     assert_eq!(decoded.how.tow, 0x5_6789 +i);
///
///     if i % 2 == 0 {
///         assert_eq!(decoded.telemetry.integrity, false);
///         assert_eq!(decoded.telemetry.reserved_bit, false);
///         assert_eq!(decoded.how.alert, false);
///         assert_eq!(decoded.how.anti_spoofing, false);
///     } else {
///         assert_eq!(decoded.telemetry.integrity, true);
///         assert_eq!(decoded.telemetry.reserved_bit, true);
///         assert_eq!(decoded.how.alert, true);
///         assert_eq!(decoded.how.anti_spoofing, true);
///     }
/// }
/// ```
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssDecoder {
    /// Aligned bits
    buffer: [u8; GPS_FRAME_BYTES],

    /// [GpsDataWord]s
    words: [GpsDataWord; GPS_WORDS_PER_FRAME - 2],

    /// True when parity verification is requested
    parity_verification: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] that does not verify parity.
    fn default() -> Self {
        Self {
            words: Default::default(),
            parity_verification: false,
            buffer: [0; GPS_FRAME_BYTES],
        }
    }
}

impl GpsQzssDecoder {
    /// Creates a new [GpsQzssDecoder] with parity verification.
    /// Our [Default] [GpsQzssDecoder] does not verify the parity bits at the moment,
    /// you have to specifically turn it on.
    /// When this is switched on, any frame or subframe that comes with invalid parity is rejected by the parser.
    pub fn with_parity_verification(mut self) -> Self {
        self.parity_verification = true;
        self
    }

    /// Packs 38 bytes (10x 30-bit + 4bit padding) correcty aligned to [u8], ready to process.
    ///
    /// ## Input
    /// - slice: &[u8], will panic if not [GPS_FRAME_BYTES] byte long!
    /// - preamble_offset in bits!
    fn resync_align(&mut self, slice: &[u8], preamble_offset_bit: usize) {
        // byte index
        let byte_index = preamble_offset_bit / 8;

        // bit index within byte
        let bit_index = preamble_offset_bit % 8;

        if bit_index == 0 {
            self.buffer
                .copy_from_slice(&slice[byte_index..byte_index + GPS_FRAME_BYTES]);
        } else {
            panic!("not supported yet");
            // slice[byte_index..byte_index + GPS_FRAME_BYTES]
            //     .into_iter()
            //     .map(|b| {
            //         *b
            //     })
            //     .collect()
        }
    }

    /// Locates the preamble bit marker (sync byte) within a buffer
    ///
    /// ## Input
    /// - slice: slice of bytes, must be [GPS_FRAME_BYTES] byte long
    /// - size: total number of bytes
    ///
    /// ## Returns
    /// - offset in bits !
    pub(crate) fn find_preamble(slice: &[u8], size: usize) -> Option<usize> {
        for i in 0..size - GPS_FRAME_BYTES + 1 {
            if slice[i] == GPS_PREAMBLE_BYTE {
                return Some(i * 8);
            }

            // intra byte test
            let mut byte1_mask = 0x7F;
            let mut byte2_mask = 0x80;

            for j in 1..8 {
                let mut value = slice[i + 1];
                value >>= 8 - j;
                value |= (slice[i] & byte1_mask) << j;

                byte1_mask >>= 1;
                byte2_mask |= 0x1 << 8 - j;

                if value == GPS_PREAMBLE_BYTE {
                    return Some(i * 8 + j);
                }
            }
        }

        None
    }

    /// Decodes the first valid [GpsQzssFrame] found in this read-only [u8] buffer.
    /// [GpsQzssDecoder] will align itself to the Sync byte, which is not aligned to [u8],
    /// because GPS/QZSS is made of 30 bit data words.
    ///
    /// ## Input
    /// - buffer: read-only [u8] buffer
    /// - size: buffer size (in bytes)
    ///
    /// ## Ouput
    /// - Total number of _bits_ that were consumed (not bytes!).
    /// You are expected to discard all processed _bits_ not to decode the same frame twice.
    /// - Optional [GpsQzssFrame] correctly decoded. First in order of appearance in the buffer.
    pub fn decode(&mut self, buffer: &[u8], size: usize) -> (usize, Option<GpsQzssFrame>) {
        let mut dword = 0u32;

        // locate preamble
        let preamble_offset_bit = Self::find_preamble(buffer, size);

        if preamble_offset_bit.is_none() {
            // marks all bits as consumed
            return (size * 8 - GPS_FRAME_BITS, None);
        }

        // align to sync byte
        let preamble_offset_bit = preamble_offset_bit.unwrap();

        #[cfg(feature = "log")]
        trace!("(GPS/QZSS)  [preamble]: pos={}", preamble_offset_bit);

        self.resync_align(buffer, preamble_offset_bit);

        dword = u32::from_be_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);

        let gps_word = GpsDataWord::from(dword);

        let telemetry = match GpsQzssTelemetry::from_word(gps_word) {
            Ok(telemetry) => {
                #[cfg(feature = "log")]
                debug!("(GPS/QZSS) [telemetry]: {}", telemetry);
                telemetry
            },
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
            #[cfg(feature = "log")]
            Err(e) => {
                error!("(GPS/QZSS) [telemetry]: {}", e);
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
        };

        dword = (buffer[7] as u32) << 4;
        dword |= (buffer[6] as u32) << (8 - 2);
        dword |= (buffer[5] as u32) << (16 - 2);
        dword |= (buffer[4] as u32) << (24 - 2);
        dword |= (buffer[3] as u32) << (32 - 2);

        let gps_word = GpsDataWord::from(dword);

        #[cfg(feature = "log")]
        trace!("(GPS/QZSS)       [how]: {:?}", gps_word);

        let how = match GpsQzssHow::from_word(gps_word) {
            Ok(how) => {
                #[cfg(feature = "log")]
                debug!("(GPS/QZSS)       [how]: {}", how);
                how
            },
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
            #[cfg(feature = "log")]
            Err(e) => {
                error!("(GPS/QZSS) [how]: {}", e);
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
        };

        // collect 8 data words
        dword = ((buffer[11] & 0xC0) as u32) >> 4;
        dword |= ((buffer[10] & 0x0f) as u32) << 4;
        dword |= (((buffer[10] & 0xf0) as u32) >> 4) << 8;
        dword |= ((buffer[9] & 0x0f) as u32) << 12;
        dword |= (((buffer[9] & 0xf0) as u32) >> 4) << 16;
        dword |= ((buffer[8] & 0x0f) as u32) << 20;
        dword |= (((buffer[8] & 0xf0) as u32) >> 4) << 24;
        dword |= ((buffer[7] & 0x0f) as u32) << 28;
        self.words[0] = GpsDataWord::from(dword);

        dword = (buffer[14] as u32) << (8 - 6);
        dword |= (buffer[13] as u32) << (16 - 6);
        dword |= (buffer[12] as u32) << (24 - 6);
        dword |= (buffer[11] as u32) << (32 - 6);
        self.words[1] = GpsDataWord::from(dword);

        dword = (buffer[15] as u32) << 24;
        dword |= (buffer[16] as u32) << 16;
        dword |= (buffer[17] as u32) << 8;
        dword |= buffer[18] as u32;
        self.words[2] = GpsDataWord::from(dword);

        dword = (buffer[22] as u32) << 4;
        dword |= (buffer[21] as u32) << (8 - 2);
        dword |= (buffer[20] as u32) << (16 - 2);
        dword |= (buffer[19] as u32) << (24 - 2);
        dword |= (buffer[18] as u32) << (32 - 2);
        self.words[3] = GpsDataWord::from(dword);

        dword = (buffer[26] as u32) << 6;
        dword |= (buffer[25] as u32) << (8 - 4);
        dword |= (buffer[24] as u32) << (16 - 4);
        dword |= (buffer[23] as u32) << (24 - 4);
        dword |= (buffer[22] as u32) << (32 - 4);
        self.words[4] = GpsDataWord::from(dword);

        dword = (buffer[29] as u32);
        dword |= (buffer[28] as u32) << 8;
        dword |= (buffer[27] as u32) << 16;
        dword |= (buffer[26] as u32) << 24;
        dword <<= 2;
        self.words[5] = GpsDataWord::from(dword);

        dword = (buffer[30] as u32) << 24;
        dword |= (buffer[31] as u32) << 16;
        dword |= (buffer[32] as u32) << 8;
        dword |= buffer[33] as u32;
        self.words[6] = GpsDataWord::from(dword);

        dword = (buffer[37] as u32) << 4;
        dword |= (buffer[36] as u32) << (8 - 2);
        dword |= (buffer[35] as u32) << (16 - 2);
        dword |= (buffer[34] as u32) << (24 - 2);
        dword |= (buffer[33] as u32) << (32 - 2);
        self.words[7] = GpsDataWord::from(dword);

        // interprets
        let frame = GpsQzssFrame {
            how,
            telemetry,
            subframe: GpsQzssSubframe::decode(how.frame_id, &self.words),
        };

        return (preamble_offset_bit + GPS_FRAME_BITS, Some(frame));
    }
}

#[cfg(test)]
mod decoder {
    use std::{fs::File, io::Read};

    use crate::{
        gps::{GpsQzssDecoder, GPS_FRAME_BITS, GPS_FRAME_BYTES},
        tests::insert_zeros,
    };

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    #[test]
    fn preamble_bin_files() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut buffer = [0; 1024];

        for file in ["eph1.bin", "eph2.bin"] {
            let filename = format!("data/GPS/{}", file);

            let mut file = File::open(&filename).unwrap_or_else(|e| {
                panic!("failed to open file {}: {}", filename, e);
            });

            file.read(&mut buffer).unwrap();

            assert_eq!(GpsQzssDecoder::find_preamble(&buffer, 1024), Some(0));

            // test delay < 1 byte
            for i in 1..7 {
                let delayed = insert_zeros(&buffer, i);
                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 1024),
                    Some(i),
                    "failed for bit position {}",
                    i
                );
            }

            // test 1 byte offset
            let delayed = insert_zeros(&buffer, 8);

            assert_eq!(
                GpsQzssDecoder::find_preamble(&delayed, 1024),
                Some(8),
                "failed for bit position 8"
            );

            // test 1 byte + bits
            for i in 1..7 {
                let delayed = insert_zeros(&buffer, i + 8);

                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 1024),
                    Some(8 + i),
                    "failed for bit position {}",
                    8 + i
                );
            }

            // test other values
            for i in 2..17 {
                let delayed = insert_zeros(&buffer, i * 8);

                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 1024),
                    Some(i * 8),
                    "failed for bit position {}",
                    i * 8
                );

                for j in 1..7 {
                    let delayed = insert_zeros(&buffer, i * 8 + j);
                    assert_eq!(
                        GpsQzssDecoder::find_preamble(&delayed, 1024),
                        Some(i * 8 + j),
                        "failed for bit position {}",
                        i * 8 + j
                    );
                }
            }
        }
    }

    #[test]
    fn eph1_bin() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        assert_eq!(GPS_FRAME_BYTES, 38);

        let mut buffer = [0; 1024];
        let mut file = File::open("data/GPS/eph1.bin").unwrap();
        file.read(&mut buffer).unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&buffer, 1024);

        // TODO
        assert_eq!(size, GPS_FRAME_BITS, "returned invalid size");

        let decoded = decoded.unwrap_or_else(|| {
            panic!("did not decoded GPS frame!");
        });

        assert_eq!(decoded.telemetry.message, 0x1234);
        assert_eq!(decoded.telemetry.integrity, true);
        assert_eq!(decoded.telemetry.reserved_bit, true);
    }
}
