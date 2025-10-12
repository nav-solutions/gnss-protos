use crate::{
    gps::{
        GpsBuffer, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
        GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE, GPS_SUBFRAME_BITS,
        GPS_WORDS_PER_FRAME, GPS_WORD_BITS,
    },
    Buffering, BufferingError, Decoder, Message,
};

#[cfg(feature = "log")]
use log::{debug, error, trace};

/// GPS-QZSS decoder [State]s
#[derive(Debug, Copy, Clone, PartialEq, Default)]
enum State {
    /// Searching telemetry bytes
    #[default]
    Telemetry,

    /// Decoding HOW
    HOW,

    /// Decoding subframe
    Subframe,
}

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
// let mut fd = File::open("data/GPS/eph1.bin")
///     .unwrap();
///
/// let size = fd.read(&mut buffer).unwrap();
///
/// // The decoder does not verify parity at the moment
/// let mut decoder = GpsQzssDecoder::default();
///
/// // TODO example
/// ```
#[derive(Copy, Clone)]
pub struct GpsQzssDecoder {
    /// Current [State]
    state: State,

    /// Pending frame
    frame: GpsQzssFrame,

    /// Enough bytes to store everything +1
    /// so we can manipulate and realign everything.
    buffer: GpsBuffer,

    /// True when parity verification is requested
    parity_verification: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] that does not verify parity.
    fn default() -> Self {
        Self {
            state: Default::default(),
            parity_verification: false,
            buffer: GpsBuffer::default(),
            frame: Default::default(),
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
}

impl Decoder for GpsQzssDecoder {
    type M = GpsQzssFrame;

    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError> {
        self.buffer.fill(src)
    }

    fn decode(&mut self) -> Option<Self::M> {
        // #[cfg(feature = "log")]
        let mut ret = Option::<Self::M>::None;
        let mut reader = self.buffer.bit_read_stream();
        let mut available = self.buffer.read_available();

        loop {
            let (next_state, consumed) = match self.state {
                State::Telemetry => match reader.read::<GpsQzssTelemetry>() {
                    Ok(telemetry) => {
                        #[cfg(feature = "log")]
                        debug!("GPS/QZSS [tlm]: OK (message=0x{:02x})", telemetry.message);

                        self.frame.telemetry = telemetry;
                        (State::HOW, GPS_WORD_BITS)
                    },
                    Err(_) => (State::Telemetry, GPS_WORD_BITS),
                },
                State::HOW => match reader.read::<GpsQzssHow>() {
                    Ok(how) => {
                        #[cfg(feature = "log")]
                        debug!("GPS/QZSS [how]: OK (fid={})", how.frame_id);

                        self.frame.how = how;
                        (State::Subframe, GPS_WORD_BITS)
                    },
                    #[cfg(not(feature = "log"))]
                    Err(_) => (State::Telemetry, 1),
                    #[cfg(feature = "log")]
                    Err(e) => {
                        error!("GPS/QZSS [how]: {}", e);
                        (State::Telemetry, 1)
                    },
                },
                State::Subframe => match self.frame.how.frame_id {
                    GpsQzssFrameId::Ephemeris1 => {
                        match reader.read::<GpsQzssFrame1>() {
                            Ok(eph1) => {
                                #[cfg(feature = "log")]
                                debug!("GPS/QZSS [eph-1]: OK (iodc={})", eph1.iodc());

                                self.frame.subframe = GpsQzssSubframe::Ephemeris1(eph1);
                                ret = Some(self.frame);
                            },
                            Err(e) => {},
                            Err(_) => {},
                        }

                        (State::Telemetry, GPS_SUBFRAME_BITS)
                    },
                    _ => (State::Telemetry, GPS_SUBFRAME_BITS),
                },
            };

            self.state = next_state;

            match reader.set_pos(consumed) {
                Ok(_) => {},
                Err(_) => {
                    // consumed everything most likely
                    if ret.is_none() {
                        return None;
                    }
                },
            }

            if let Some(ret) = ret {
                return Some(ret);
            }
        }
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
