use crate::{
    buffer::{Buffer, BufferingError},
    gps::{
        GpsDataWord, GpsQzssFrame, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS,
        GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME,
    },
    Decoder,
};

#[cfg(feature = "std")]
mod std;

#[cfg(feature = "log")]
use log::{debug, error, trace};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
enum State {
    /// [State::Preamble] first/initial state,
    /// decoding the preamble bits
    #[default]
    Preamble,

    /// Decoding [GpsQzssTelemetry] word
    TLM,

    /// Decoding [GpsQzssHow] word
    HOW,
    // /// Decoding [GpsQzssSubframe] words (x8)
    // Subframe,
}

impl State {
    /// Returns the size to read for this [State]
    pub fn read_size(&self) -> usize {
        match self {
            Self::Preamble => 8,
            Self::TLM => GPS_FRAME_BYTES,
            Self::HOW => GPS_FRAME_BYTES,
            // Self::Subframe => GPS_FRAME_BYTES * 8,
        }
    }

    /// Returns the consumed number of bits for this [State]
    pub fn consumed_bits(&self) -> usize {
        match self {
            Self::Preamble => 1,
            Self::TLM => 30,
            Self::HOW => 30,
            // Self::Subframe => 30 * 8,
        }
    }
}

/// [GpsQzssDecoder] can decode GPS (or QZSS) messages.
///
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
/// let mut fd = File::open("data/GPS/eph1.bin")
///     .unwrap();
///
/// // feed this data to the decoder using .write()
/// let size = fd.read(&mut buffer).unwrap();
///
/// // The decoder does not verify parity at the moment
/// let mut decoder = GpsQzssDecoder::default();
///
/// // TODO example
/// ```
#[derive(Clone)]
pub struct GpsQzssDecoder {
    /// Current state
    state: State,

    /// Latest [GpsQzssTelemetry] decoded
    telemetry: GpsQzssTelemetry,

    /// Latest [GpsQzssHow] decoded
    how: GpsQzssHow,

    /// Buffer for more than one entire message.
    /// All we truly need is more than one entire message.
    /// But aliging this value makes deployment more efficient.
    /// And providing bigger storage makes the Read/Write buffering interactions
    /// more efficient too.
    buffer: Buffer<1024>,

    /// [GpsDataWord]s to avoid allocation
    words: [GpsDataWord; GPS_WORDS_PER_FRAME - 2],

    /// True when parity verification is requested
    parity_verification: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a [Default] [GpsQzssDecoder] without parity verification.
    fn default() -> Self {
        Self {
            state: State::default(),
            how: Default::default(),
            telemetry: Default::default(),
            words: Default::default(),
            parity_verification: false,
            buffer: Buffer::<1024>::default(),
        }
    }
}

impl Decoder for GpsQzssDecoder {
    type M = GpsQzssFrame;

    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError> {
        self.buffer.fill(src)
    }

    fn decode(&mut self) -> Option<Self::M> {
        loop {
            let available = self.buffer.read_available();
            let needed = self.state.read_size();

            let mut consumed_bits = 0;

            if available < needed {
                // can't attempt decoding current state
                return None;
            }

            // decode state
            let next_state = match self.state {
                State::Preamble => {
                    // grabs one byte
                    let byte = self.buffer.view().next()?;

                    if byte == GPS_PREAMBLE_BYTE {
                        #[cfg(feature = "log")]
                        trace!("(GPS/QZSS)  [preamble]");
                        State::TLM
                    } else {
                        State::Preamble
                    }
                },
                State::TLM => {
                    // grabs one word
                    match self.buffer.view().gps_data_word() {
                        Some(word) => match GpsQzssTelemetry::from_word(word) {
                            Ok(telemetry) => {
                                #[cfg(feature = "log")]
                                debug!("(GPS/QZSS) [telemetry]: {}", telemetry);

                                self.telemetry = telemetry;
                                State::HOW
                            },
                            #[cfg(not(feature = "log"))]
                            Err(_) => State::Preamble, // reset
                            Err(e) => {
                                error!("(GPS/QZSS) [telemetry]: {} ({:?})", e, word);
                                State::Preamble // reset
                            },
                        },
                        None => State::Preamble, // reset
                    }
                },
                State::HOW => {
                    // grabs one word
                    match self.buffer.view().gps_data_word() {
                        Some(word) => match GpsQzssHow::from_word(word) {
                            Ok(how) => {
                                #[cfg(feature = "log")]
                                debug!("(GPS/QZSS)       [how]: {}", how);

                                self.how = how;
                                State::Preamble // TODO
                            },
                            #[cfg(not(feature = "log"))]
                            Err(_) => State::Preamble, // reset
                            Err(e) => {
                                error!("(GPS/QZSS) [how]: {}", e);
                                State::Preamble // reset
                            },
                        },
                        None => State::Preamble, // reset
                    }
                },
                // State::Subframe => {
                //     // only interpretation at this point or parity failures
                //     let frame = GpsQzssFrame {
                //         how,
                //         telemetry,
                //         subframe: GpsQzssSubframe::decode(how.frame_id, &self.words),
                //     };
                // },
            };

            // consume
            self.buffer.discard_bits_mut(self.state.consumed_bits());
            self.state = next_state;
        }
    }
}

impl GpsQzssDecoder {
    /// Creates a [GpsQzssDecoder] with parity verification (not supported yet).
    pub fn with_parity_verification(&self) -> Self {
        let mut s = self.clone();
        s.parity_verification = true;
        s
    }

    // /// Packs 38 bytes (10x 30-bit + 4bit padding) correcty aligned to [u8], ready to process.
    // ///
    // /// ## Input
    // /// - slice: &[u8], will panic if not [GPS_FRAME_BYTES] byte long!
    // /// - preamble_offset in bits!
    // fn resync_align(&mut self, slice: &[u8], preamble_offset_bit: usize) {
    //     // byte index
    //     let byte_index = preamble_offset_bit / 8;

    //     // bit index within byte
    //     let bit_index = preamble_offset_bit % 8;

    //     // copies to first position
    //     self.buffer[0..GPS_FRAME_BYTES]
    //         .copy_from_slice(&slice[byte_index..byte_index + GPS_FRAME_BYTES]);

    //     if bit_index > 0 {
    //         let (byte1_mask, byte2_mask) = match bit_index {
    //             1 => (0x7f, 0xfe),
    //             2 => (0x3f, 0xfc),
    //             3 => (0x1f, 0xf8),
    //             4 => (0x0f, 0xf0),
    //             5 => (0x08, 0xf0),
    //             6 => (0x0f, 0xf0),
    //             7 => (0x0f, 0xf0),
    //             _ => unreachable!("compiler issue"),
    //         };

    //         for i in 0..GPS_FRAME_BYTES {
    //             let mut mask1 = byte1_mask;
    //             let mut mask2 = byte2_mask;

    //             self.buffer[i] &= mask1;
    //             self.buffer[i + 1] &= mask2;

    //             self.buffer[i] >>= bit_index;
    //             self.buffer[i + 1] >>= bit_index;
    //         }
    //     }
    // }

    // /// Locates the preamble bit marker (sync byte) within a buffer
    // ///
    // /// ## Input
    // /// - slice: slice of bytes, must be [GPS_FRAME_BYTES] byte long
    // /// - size: total number of bytes
    // ///
    // /// ## Returns
    // /// - offset in bits !
    // fn find_preamble(slice: &[u8], size: usize) -> Option<usize> {
    //     for i in 0..size - GPS_FRAME_BYTES + 1 {
    //         if slice[i] == GPS_PREAMBLE_BYTE {
    //             return Some(i * 8);
    //         }

    //         // intra byte test
    //         let mut byte1_mask = 0x7F;
    //         let mut byte2_mask = 0x80;

    //         for j in 1..8 {
    //             let mut value = slice[i + 1];
    //             value >>= 8 - j;
    //             value |= (slice[i] & byte1_mask) << j;

    //             byte1_mask >>= 1;
    //             byte2_mask |= 0x1 << (8 - j);

    //             if value == GPS_PREAMBLE_BYTE {
    //                 return Some(i * 8 + j);
    //             }
    //         }
    //     }

    //     None
    // }
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
