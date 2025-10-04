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
/// use gnss_protos::{GpsQzssDecoder, GPS_FRAME_BITS};
///
/// // Feeds some of our GPS messages example,
/// // which is equivalent to real-time acquisition
///
/// let mut buffer = [0u8; 1024];
///
/// let mut fd = File::open("data/GPS/eph1.bin")
///     .unwrap();
///
/// let size = fd.read(&mut buffer).unwrap();
///
/// // The decoder does not verify parity at the moment
/// let mut decoder = GpsQzssDecoder::default();
///
/// // TODO example
/// ```
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssDecoder {
    /// Enough bytes to store everything +1
    /// so we can manipulate and realign everything.
    buffer: [u8; GPS_FRAME_BYTES + 1],

    /// [GpsDataWord]s to avoid allocation
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
            buffer: [0; GPS_FRAME_BYTES + 1],
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

        #[cfg(feature = "log")]
        trace!("(GPS/QZSS)  [preamble]: off={}", bit_index);

        // copies to first position
        self.buffer[0..GPS_FRAME_BYTES]
            .copy_from_slice(&slice[byte_index..byte_index + GPS_FRAME_BYTES]);

        if bit_index > 0 {
            let (byte1_mask, byte2_mask) = match bit_index {
                1 => (0x7f, 0xfe),
                2 => (0x3f, 0xfc),
                3 => (0x1f, 0xf8),
                4 => (0x0f, 0xf0),
                5 => (0x08, 0xf0),
                6 => (0x0f, 0xf0),
                7 => (0x0f, 0xf0),
                _ => unreachable!("compiler issue"),
            };

            for i in 0..GPS_FRAME_BYTES {
                let mut mask1 = byte1_mask;
                let mut mask2 = byte2_mask;

                self.buffer[i] &= mask1;
                self.buffer[i + 1] &= mask2;

                self.buffer[i] >>= bit_index;
                self.buffer[i + 1] >>= bit_index;
            }
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
    fn find_preamble(slice: &[u8], size: usize) -> Option<usize> {
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
                byte2_mask |= 0x1 << (8 - j);

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
        let mut dword;

        // locate preamble
        let preamble_offset_bit = Self::find_preamble(buffer, size);

        if preamble_offset_bit.is_none() {
            // marks all bits as consumed
            return (size * 8 - GPS_FRAME_BITS, None);
        }

        // align to sync byte
        let preamble_offset_bit = preamble_offset_bit.unwrap();

        #[cfg(feature = "log")]
        trace!(
            "(GPS/QZSS)  [preamble]: pos={} [0x{:02X} 0x{:02X} 0x{:02X} 0x{:02X}]",
            preamble_offset_bit,
            buffer[preamble_offset_bit / 8],
            buffer[preamble_offset_bit / 8],
            buffer[preamble_offset_bit / 8],
            buffer[preamble_offset_bit / 8],
        );

        self.resync_align(buffer, preamble_offset_bit);

        dword = u32::from_be_bytes([
            self.buffer[0],
            self.buffer[1],
            self.buffer[2],
            self.buffer[3],
        ]);

        let gps_word = GpsDataWord::from(dword);
        let parity = gps_word.parity(&Default::default(), false);

        // panic!("LSB=0x{:02}X PAR=0x{:02X}", (gps_word.value() & 0x3f) as u8, parity);

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
                error!("(GPS/QZSS) [telemetry]: {} ({:?})", e, gps_word);
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
        };

        dword = self.buffer[7] as u32;
        dword |= (self.buffer[6] as u32) << 8;
        dword |= (self.buffer[5] as u32) << 16;
        dword |= (self.buffer[4] as u32) << 24;
        dword >>= 2;
        dword |= ((self.buffer[3] as u32) & 0x03) << 28;

        let gps_word = GpsDataWord::from(dword);
        let parity = gps_word.parity(&Default::default(), false);

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
        dword = ((self.buffer[11] & 0xC0) as u32) >> 4;
        dword |= ((self.buffer[10] & 0x0f) as u32) << 4;
        dword |= (((self.buffer[10] & 0xf0) as u32) >> 4) << 8;
        dword |= ((self.buffer[9] & 0x0f) as u32) << 12;
        dword |= (((self.buffer[9] & 0xf0) as u32) >> 4) << 16;
        dword |= ((self.buffer[8] & 0x0f) as u32) << 20;
        dword |= (((self.buffer[8] & 0xf0) as u32) >> 4) << 24;
        dword |= ((self.buffer[7] & 0x0f) as u32) << 28;

        self.words[0] = GpsDataWord::from(dword);

        dword = (self.buffer[14] as u32) << (8 - 6);
        dword |= (self.buffer[13] as u32) << (16 - 6);
        dword |= (self.buffer[12] as u32) << (24 - 6);
        dword |= (self.buffer[11] as u32) << (32 - 6);

        self.words[1] = GpsDataWord::from(dword);

        dword = (self.buffer[15] as u32) << 24;
        dword |= (self.buffer[16] as u32) << 16;
        dword |= (self.buffer[17] as u32) << 8;
        dword |= self.buffer[18] as u32;

        self.words[2] = GpsDataWord::from(dword);

        dword = (self.buffer[22] as u32) << 4;
        dword |= (self.buffer[21] as u32) << (8 - 2);
        dword |= (self.buffer[20] as u32) << (16 - 2);
        dword |= (self.buffer[19] as u32) << (24 - 2);
        dword |= (self.buffer[18] as u32) << (32 - 2);

        self.words[3] = GpsDataWord::from(dword);

        dword = self.buffer[25] as u32;
        dword |= (self.buffer[24] as u32) << 8;
        dword |= (self.buffer[23] as u32) << 16;
        dword |= (self.buffer[22] as u32) << 24;
        dword <<= 4;
        dword |= ((self.buffer[26] & 0xC0) as u32) >> 6;

        self.words[4] = GpsDataWord::from(dword);

        dword = self.buffer[29] as u32;
        dword |= (self.buffer[28] as u32) << 8;
        dword |= (self.buffer[27] as u32) << 16;
        dword |= (self.buffer[26] as u32) << 24;
        dword <<= 2;

        self.words[5] = GpsDataWord::from(dword);

        dword = (self.buffer[30] as u32) << 24;
        dword |= (self.buffer[31] as u32) << 16;
        dword |= (self.buffer[32] as u32) << 8;
        dword |= self.buffer[33] as u32;
        self.words[6] = GpsDataWord::from(dword);

        dword = (self.buffer[37] as u32) << 4;
        dword |= (self.buffer[36] as u32) << (8 - 2);
        dword |= (self.buffer[35] as u32) << (16 - 2);
        dword |= (self.buffer[34] as u32) << (24 - 2);
        dword |= (self.buffer[33] as u32) << (32 - 2);

        self.words[7] = GpsDataWord::from(dword);

        // subframe decoding may fail on invalid page 4 or 5 subpages.
        let frame = match GpsQzssSubframe::decode(how.frame_id, &self.words) {
            Ok(subframe) => Some(GpsQzssFrame {
                telemetry,
                how,
                subframe,
            }),
            #[cfg(not(feature = "log"))]
            Err(_) => None,
            #[cfg(feature = "log")]
            Err(e) => {
                error!("frame #4 or #5 pagination issue: {}", e);
                None
            },
        };

        (preamble_offset_bit + GPS_FRAME_BITS, frame)
    }
}

#[cfg(test)]
mod decoder {
    use std::{fs::File, io::Read};

    use crate::{
        gps::{
            GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3,
            GpsQzssFrameId, GPS_FRAME_BITS, GPS_FRAME_BYTES,
        },
        tests::insert_zeros,
    };

    use crate::tests::init_logger;

    use log::info;

    #[test]
    fn preamble_search() {
        init_logger();

        let mut buffer = [0; 8192];

        for file in ["eph1.bin", "eph2.bin"] {
            let filename = format!("data/GPS/{}", file);

            let mut file = File::open(&filename).unwrap_or_else(|e| {
                panic!("failed to open file {}: {}", filename, e);
            });

            file.read(&mut buffer).unwrap();

            assert_eq!(GpsQzssDecoder::find_preamble(&buffer, 8192), Some(0));

            // test delay < 1 byte
            for i in 1..7 {
                let delayed = insert_zeros(&buffer, i);
                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 8192),
                    Some(i),
                    "failed for bit position {}",
                    i
                );
            }

            // test 1 byte offset
            let delayed = insert_zeros(&buffer, 8);

            assert_eq!(
                GpsQzssDecoder::find_preamble(&delayed, 8192),
                Some(8),
                "failed for bit position 8"
            );

            // test 1 byte + bits
            for i in 1..7 {
                let delayed = insert_zeros(&buffer, i + 8);

                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 8192),
                    Some(8 + i),
                    "failed for bit position {}",
                    8 + i
                );
            }

            // test other values
            for i in 2..17 {
                let delayed = insert_zeros(&buffer, i * 8);

                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 8192),
                    Some(i * 8),
                    "failed for bit position {}",
                    i * 8
                );

                for j in 1..7 {
                    let delayed = insert_zeros(&buffer, i * 8 + j);
                    assert_eq!(
                        GpsQzssDecoder::find_preamble(&delayed, 8192),
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
        init_logger();

        let mut ptr = 0;
        let mut message = 0;
        let mut buffer = [0; 8192]; // single read

        let mut file = File::open("data/GPS/eph1.bin").unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);

        let mut size = file.read(&mut buffer).unwrap();

        assert!(size > 0, "file is empty");

        // consume everything
        loop {
            if message == 128 {
                // we're done
                break;
            }

            // grab a frame
            let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);

            message += 1;

            if message == 1 {
                // first RX
                assert_eq!(processed_size, GPS_FRAME_BITS); // bits!
            } else {
                // following RX
                // TODO +8 expected here, not 16
                assert_eq!(processed_size, GPS_FRAME_BITS + 16); // bits!
            }

            let decoded = decoded.unwrap(); // success (we have 128 frames)

            let subf = decoded.subframe.as_eph1().unwrap_or_else(|| {
                panic!("wrong frame type decoded");
            });

            if message == 1 {
                // verify initial values
                assert_eq!(decoded, model, "invalid initial value");
            } else {
                // test pattern
                assert_eq!(
                    decoded.telemetry.message,
                    model.telemetry.message + message - 1,
                    "error at message {}",
                    message
                );

                if message % 2 == 0 {
                    assert_eq!(decoded.telemetry.integrity, false);
                    assert_eq!(decoded.telemetry.reserved_bit, false);
                    assert_eq!(decoded.how.alert, false);
                    assert_eq!(decoded.how.anti_spoofing, false);
                } else {
                    assert_eq!(decoded.telemetry.integrity, true);
                    assert_eq!(decoded.telemetry.reserved_bit, true);
                    assert_eq!(decoded.how.alert, true);
                    assert_eq!(decoded.how.anti_spoofing, true);
                }
            }

            info!("EPH-1.bin MESSAGE {}", message + 1);

            ptr += processed_size / 8 - 1;
            size -= processed_size / 8 - 1;

            if size <= GPS_FRAME_BYTES - 2 {
                assert_eq!(message, 128, "did not parse enough messages");
            }
        }
        assert_eq!(message, 128, "did not parse enough messages");
    }

    #[test]
    fn eph1_bin_delayed() {
        init_logger();

        let mut buffer = [0; 8192];

        // TODO add more cases
        for zeros in 1..7 {
            let mut ptr = 0;
            let mut message = 0;
            let mut buffer = [0; 8192]; // single read

            let mut file = File::open("data/GPS/eph1.bin").unwrap();

            let mut decoder = GpsQzssDecoder::default();

            let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);

            let mut size = file.read(&mut buffer).unwrap();
            assert!(size > 0, "file is empty");

            let delayed = insert_zeros(&buffer, zeros);

            // consume everything
            loop {
                if message == 128 {
                    // we're done
                    break;
                }

                // grab a frame
                let (processed_size, decoded) = decoder.decode(&delayed[ptr..], size);

                message += 1;

                if message == 1 {
                    // first RX
                    assert_eq!(processed_size, GPS_FRAME_BITS + zeros); // bits!
                } else {
                    // following RX
                    // TODO +8 expected here, not 16
                    assert_eq!(processed_size, GPS_FRAME_BITS + 16 + zeros); // bits!
                }

                let decoded = decoded.unwrap(); // success (we have 128 frames)

                let subf = decoded.subframe.as_eph1().unwrap_or_else(|| {
                    panic!("wrong frame type decoded");
                });

                if message == 1 {
                    // verify initial values
                    assert_eq!(decoded, model, "invalid initial value");
                } else {
                    // test pattern
                    assert_eq!(
                        decoded.telemetry.message,
                        model.telemetry.message + message - 1,
                        "error at message {}",
                        message
                    );

                    if message % 2 == 0 {
                        assert_eq!(decoded.telemetry.integrity, false);
                        assert_eq!(decoded.telemetry.reserved_bit, false);
                        assert_eq!(decoded.how.alert, false);
                        assert_eq!(decoded.how.anti_spoofing, false);
                    } else {
                        assert_eq!(decoded.telemetry.integrity, true);
                        assert_eq!(decoded.telemetry.reserved_bit, true);
                        assert_eq!(decoded.how.alert, true);
                        assert_eq!(decoded.how.anti_spoofing, true);
                    }
                }

                info!("EPH-1.bin MESSAGE {}", message + 1);

                ptr += processed_size / 8 - 1;
                size -= processed_size / 8 - 1;

                if size <= GPS_FRAME_BYTES - 2 {
                    assert_eq!(message, 128, "did not parse enough messages");
                }
            }
            assert_eq!(message, 128, "did not parse enough messages");
        }
    }

    #[test]
    fn eph2_bin() {
        init_logger();

        let mut ptr = 0;
        let mut message = 0;
        let mut buffer = [0; 8192]; // single read

        let mut file = File::open("data/GPS/eph2.bin").unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris2);

        let mut size = file.read(&mut buffer).unwrap();

        assert!(size > 0, "file is empty");

        // consume everything
        loop {
            if message == 128 {
                // we're done
                break;
            }

            // grab a frame
            let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);

            message += 1;

            if message == 1 {
                // first RX
                assert_eq!(processed_size, GPS_FRAME_BITS); // bits!
            } else {
                // following RX
                // TODO +8 expected here, not 16
                assert_eq!(processed_size, GPS_FRAME_BITS + 16); // bits!
            }

            let decoded = decoded.unwrap(); // success (we have 128 frames)

            let subf = decoded.subframe.as_eph2().unwrap_or_else(|| {
                panic!("wrong frame type decoded");
            });

            if message == 1 {
                // verify initial values
                assert_eq!(decoded, model, "invalid initial value");
            } else {
                // test pattern
                assert_eq!(
                    decoded.telemetry.message,
                    model.telemetry.message + message - 1,
                    "error at message {}",
                    message
                );

                if message % 2 == 0 {
                    assert_eq!(decoded.telemetry.integrity, false);
                    assert_eq!(decoded.telemetry.reserved_bit, false);
                    assert_eq!(decoded.how.alert, false);
                    assert_eq!(decoded.how.anti_spoofing, false);
                } else {
                    assert_eq!(decoded.telemetry.integrity, true);
                    assert_eq!(decoded.telemetry.reserved_bit, true);
                    assert_eq!(decoded.how.alert, true);
                    assert_eq!(decoded.how.anti_spoofing, true);
                }
            }

            info!("EPH-2.bin MESSAGE {}", message + 1);

            ptr += processed_size / 8 - 1;
            size -= processed_size / 8 - 1;

            if size <= GPS_FRAME_BYTES - 2 {
                assert_eq!(message, 128, "did not parse enough messages");
            }
        }
        assert_eq!(message, 128, "did not parse enough messages");
    }

    #[test]
    fn eph3_bin() {
        init_logger();

        let mut ptr = 0;
        let mut message = 0;
        let mut buffer = [0; 8192]; // single read

        let mut file = File::open("data/GPS/eph3.bin").unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris3);

        let mut size = file.read(&mut buffer).unwrap();

        assert!(size > 0, "file is empty");

        // consume everything
        loop {
            if message == 128 {
                // we're done
                break;
            }

            // grab a frame
            let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);

            message += 1;

            if message == 1 {
                // first RX
                assert_eq!(processed_size, GPS_FRAME_BITS); // bits!
            } else {
                // following RX
                // TODO +8 expected here, not 16
                assert_eq!(processed_size, GPS_FRAME_BITS + 16); // bits!
            }

            let decoded = decoded.unwrap(); // success (we have 128 frames)

            let subf = decoded.subframe.as_eph3().unwrap_or_else(|| {
                panic!("wrong frame type decoded");
            });

            if message == 1 {
                // verify initial values
                assert_eq!(decoded, model, "invalid initial value");
            } else {
                // test pattern
                assert_eq!(
                    decoded.telemetry.message,
                    model.telemetry.message + message - 1,
                    "error at message {}",
                    message
                );

                if message % 2 == 0 {
                    assert_eq!(decoded.telemetry.integrity, false);
                    assert_eq!(decoded.telemetry.reserved_bit, false);
                    assert_eq!(decoded.how.alert, false);
                    assert_eq!(decoded.how.anti_spoofing, false);
                } else {
                    assert_eq!(decoded.telemetry.integrity, true);
                    assert_eq!(decoded.telemetry.reserved_bit, true);
                    assert_eq!(decoded.how.alert, true);
                    assert_eq!(decoded.how.anti_spoofing, true);
                }
            }

            info!("EPH-3.bin MESSAGE {}", message + 1);

            ptr += processed_size / 8 - 1;
            size -= processed_size / 8 - 1;

            if size <= GPS_FRAME_BYTES - 2 {
                assert_eq!(message, 128, "did not parse enough messages");
            }
        }
        assert_eq!(message, 128, "did not parse enough messages");
    }

    #[test]
    fn burst_bin() {
        init_logger();

        let mut ptr = 0;
        let mut message = 0;
        let mut buffer = [0; 8192]; // single read

        let mut file = File::open("data/GPS/burst.bin").unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris3);

        let mut size = file.read(&mut buffer).unwrap();

        assert!(size > 0, "file is empty");

        // consume everything
        loop {
            if message == 128 {
                // we're done
                break;
            }

            // grab a frame
            let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);

            message += 1;

            if message == 1 {
                // first RX
                assert_eq!(processed_size, GPS_FRAME_BITS); // bits!
            } else {
                // following RX
                // TODO +8 expected here, not 16
                assert_eq!(processed_size, GPS_FRAME_BITS + 16); // bits!
            }

            let decoded = decoded.unwrap(); // success (we have 128 frames)

            // TODO
            info!("BURST.bin MESSAGE {}", message + 1);

            ptr += processed_size / 8 - 1;
            size -= processed_size / 8 - 1;

            if size <= GPS_FRAME_BYTES - 2 {
                assert_eq!(message, 128, "did not parse enough messages");
            }
        }

        assert_eq!(message, 128, "did not parse enough messages");
    }
}
