use crate::gps::{
    GpsDataByte, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
    BitReader,
    GPS_WORD_BITS,
    GPS_FRAME_BYTES,
    GPS_FRAME_BITS,
    GPS_WORDS_PER_FRAME,
};

#[cfg(feature = "log")]
use log::{error, trace};

#[derive(Default, Debug, Copy, Clone, PartialEq)]
enum State {
    /// Telemetry dword parsing
    #[default]
    Telemetry,

    /// How dword parsing
    How,

    /// Data word parsing
    DataWord,
}

/// [GpsQzssDecoder] is a structure to work on byte per byte approach,
/// not compatible with truly raw / real-time GPS-QZSS data bits.
/// Prefer the [GpsQzssFrame::decode] functions to work from a buffer of raw bytes.
#[derive(Clone, Copy)]
pub struct GpsQzssDecoder {
    /// Frame counter
    ptr: usize,

    /// data word storage
    pub(crate) dword: u32,

    /// total number of bits collected
    pub(crate) collected: usize,

    /// Current [State]
    state: State,

    /// Latest [GpsQzssTelemetry]
    tlm: GpsQzssTelemetry,

    /// Latest [GpsQzssHow]
    how: GpsQzssHow,

    /// Pending [GpsQzssSubframe]
    subframe: GpsQzssSubframe,

    /// u32 storage for words that overlap
    storage: u32,

    /// True if parity verification is requested
    parity_check: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] with parity bit verification
    fn default() -> Self {
        Self {
            ptr: 0,
            dword: 0,
            storage: 0,
            collected: 0,
            parity_check: true,
            tlm: Default::default(),
            how: Default::default(),
            state: Default::default(),
            subframe: Default::default(),
        }
    }
}

impl GpsQzssDecoder {
    /// Resets this [GpsQzssDecoder]
    fn reset(mut self) {
        self.ptr = 0;
        self.dword = 0;
        self.collected = 0;
        self.state = Default::default();
        self.how = Default::default();
        self.subframe = Default::default();
    }

    /// Processes a [GpsDataByte] from a GPS-QZSS data stream,
    /// returns a [GpsQzssFrame] when a correct frame has entirely been identified and decoded.
    ///
    /// ## Input
    /// - byte: [GpsDataByte] wrapper to describe whether your stream of byte
    /// is padded or not
    ///
    /// ## Output
    /// - [Option::None] in most cases.
    /// - [GpsQzssFrame] once all 10 sucessive GPS data words were correctly identified
    /// and decoded.
    pub fn consume_byte(&mut self, byte: GpsDataByte) -> Option<GpsQzssFrame> {
        // collect bytes
        match byte {
            GpsDataByte::LsbPadded(byte) => {
                self.collected += 6;
                self.dword <<= 6;
                self.dword |= (byte as u32) >> 2;
            },
            GpsDataByte::MsbPadded(byte) => {
                self.collected += 6;
                self.dword <<= 6;
                self.dword |= (byte & 0x3f) as u32;
            },
            GpsDataByte::Byte(byte) => {
                self.collected += 8;
                self.dword <<= 8;
                self.dword |= byte as u32;
            },
        }

        #[cfg(feature = "log")]
        trace!(
            "GPS - data collection - ptr={} dword=0x{:08x}",
            self.ptr,
            self.dword
        );

        if self.collected < 30 {
            // Can't proceed
            return None;
        }

        // data word processing
        let dword = self.dword;

        let mut ret = Option::<GpsQzssFrame>::None;

        // buffer decoding attempt
        #[cfg(feature = "log")]
        trace!(
            "GPS - state={:?} - decoding dword=0x{:08x}",
            self.state,
            dword
        );

        match self.state {
            State::Telemetry => match GpsQzssTelemetry::decode(dword) {
                Ok(tlm) => {
                    self.tlm = tlm;
                    self.state = State::How;
                    #[cfg(feature = "log")]
                    trace!("GPS - TLM {:?}", tlm);
                },
                #[cfg(feature = "log")]
                Err(e) => {
                    error!("GPS - TLM decoding error: {:?}", e);
                },
                #[cfg(not(feature = "log"))]
                Err(_) => {},
            },

            State::How => match GpsQzssHow::decode(dword) {
                Ok(how) => {
                    self.how = how;
                    self.state = State::DataWord;
                    self.ptr = 3;

                    match how.frame_id {
                        GpsQzssFrameId::Ephemeris1 => {
                            self.subframe = GpsQzssSubframe::Ephemeris1(Default::default());
                        },
                        GpsQzssFrameId::Ephemeris2 => {
                            self.subframe = GpsQzssSubframe::Ephemeris2(Default::default());
                        },
                        GpsQzssFrameId::Ephemeris3 => {
                            self.subframe = GpsQzssSubframe::Ephemeris3(Default::default());
                        },
                    }
                    #[cfg(feature = "log")]
                    trace!("GPS - HOW {:?}", how);
                },
                #[cfg(feature = "log")]
                Err(e) => {
                    error!("GPS - HOW decoding error: {:?}", e);
                    self.ptr = 0;
                    self.dword = 0;
                    self.state = State::default();
                },
                #[cfg(not(feature = "log"))]
                Err(_) => {},
            },

            State::DataWord => {
                match self.how.frame_id {
                    GpsQzssFrameId::Ephemeris1 => {
                        if let Some(frame) = self.subframe.as_mut_eph1() {
                            match frame.decode_word(self.ptr, self.dword) {
                                Ok(_) => {
                                    #[cfg(feature = "log")]
                                    trace!("GPS - EPH #1 Word#{} {:?}", self.ptr, self.dword);
                                },
                                Err(e) => {
                                    #[cfg(feature = "log")]
                                    trace!("GPS error: {}", e);

                                    self.reset();
                                    return None;
                                },
                            }
                        } else {
                            self.reset();
                            return None;
                        }
                    },
                    GpsQzssFrameId::Ephemeris2 => {
                        if let Some(frame) = self.subframe.as_mut_eph2() {
                            match frame.decode_word(self.ptr, self.dword, &mut self.storage) {
                                Ok(_) => {
                                    #[cfg(feature = "log")]
                                    trace!("GPS - EPH #2 Word#{} {:?}", self.ptr, self.dword);
                                },
                                Err(e) => {
                                    #[cfg(feature = "log")]
                                    trace!("GPS error: {}", e);

                                    self.reset();
                                    return None;
                                },
                            }
                        } else {
                            self.reset();
                            return None;
                        }
                    },
                    GpsQzssFrameId::Ephemeris3 => {
                        if let Some(frame) = self.subframe.as_mut_eph3() {
                            match frame.decode_word(self.ptr, self.dword, &mut self.storage) {
                                Ok(_) => {
                                    #[cfg(feature = "log")]
                                    trace!("GPS - EPH #3 Word#{} {:?}", self.ptr, self.dword);
                                },
                                Err(e) => {
                                    #[cfg(feature = "log")]
                                    trace!("GPS error: {}", e);

                                    self.reset();
                                    return None;
                                },
                            }
                        } else {
                            self.reset();
                            return None;
                        }
                    },
                }

                self.ptr += 1;
            },
        }

        // reset
        self.collected = 0;
        self.dword = 0;

        if self.ptr == 11 {
            ret = Some(GpsQzssFrame {
                how: self.how,
                telemetry: self.tlm,
                subframe: self.subframe,
            });

            self.ptr = 0;
            self.state = State::Telemetry;
        }

        ret
    }

    /// Create a new [GpsQzssDecoder] that will not verify the parity bits
    pub fn without_parity_verification(&self) -> Self {
        Self {
            ptr: self.ptr,
            collected: 0,
            dword: 0,
            storage: 0,
            state: self.state,
            parity_check: false,
            tlm: self.tlm.clone(),
            how: self.how.clone(),
            subframe: self.subframe.clone(),
        }
    }
}

#[cfg(test)]
mod test {
    use std::io::Read;
    use std::fs::File;

    use crate::{
        gps::{GpsDataByte, GpsQzssDecoder, GpsQzssSubframe},
        GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssTelemetry,
    };

    use crate::tests::from_ublox_be_bytes;

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    #[test]
    fn test_tlm_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 3);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, true);
    }

    #[test]
    fn test_ublox_tlm_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::msb_padded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B), // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        let bytes = [0x22, 0xC1, 0x3E, 0x1B];
        let bytes = from_ublox_be_bytes(&bytes);

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);
    }

    #[test]
    fn test_tlmhow_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::msb_padded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B), // TLM
            GpsDataByte::MsbPadded(0x15),
            GpsDataByte::Byte(0x27),
            GpsDataByte::Byte(0xC9),
            GpsDataByte::Byte(0x73), // HOW
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        assert_eq!(decoder.how.alert, false);
        assert_eq!(decoder.how.anti_spoofing, true);
        assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);
    }

    #[test]
    fn test_ublox_tlmhow_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::MsbPadded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B), // TLM
            GpsDataByte::MsbPadded(0x15),
            GpsDataByte::Byte(0x27),
            GpsDataByte::Byte(0xC9),
            GpsDataByte::Byte(0x73), // HOW
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        assert_eq!(decoder.how.alert, false);
        assert_eq!(decoder.how.anti_spoofing, true);
        assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);

        let bytes = from_ublox_be_bytes(&[0x22, 0xC1, 0x3E, 0x1B, 0x15, 0x27, 0xC9, 0x73]);

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        assert_eq!(decoder.how.alert, false);
        assert_eq!(decoder.how.anti_spoofing, true);
        assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);
    }

    #[test]
    fn test_incomplete_frame() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = [
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.consume_byte(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded!");

        let bytes = [
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // TLM
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // HOW
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD3
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD4
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD5
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD6
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD7
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD8
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.consume_byte(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded");
    }
}
