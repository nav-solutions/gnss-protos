#[cfg(feature = "log")]
use log::{error, trace};

use crate::gps::{GpsQzssFrame, GpsQzssHow, GpsQzssTelemetry, GPS_BITMASK};

#[derive(Debug, Copy, Clone, Default)]
enum State {
    /// GPS TLM word
    #[default]
    TLM,

    /// GPS How word
    HOW,

    /// Word count
    Word(usize),
}

#[derive(Debug)]
pub struct GpsQzssDecoder {
    /// Current [State]
    state: State,

    /// Internal pointer
    ptr: usize,

    /// Internal [u32] dword
    dword: u32,

    /// Latest [GpsQzssTelemetry]
    tlm: GpsQzssTelemetry,

    /// Latest [GpsQzssHow]
    how: GpsQzssHow,

    /// True if parity verification is requested
    parity_check: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] with parity bit verification
    fn default() -> Self {
        Self {
            state: Default::default(),
            ptr: Default::default(),
            dword: Default::default(),
            tlm: Default::default(),
            how: Default::default(),
            parity_check: true,
        }
    }
}

impl GpsQzssDecoder {
    /// Parse a GPS/QZSS stream of bytes, which must be aligned to 32-bit with 2 MSB padding
    pub fn parse(&mut self, byte: u8) -> Option<GpsQzssFrame> {
        self.ptr += 1;

        self.dword <<= 8;
        self.dword |= byte as u32;

        let mut ret = Option::<GpsQzssFrame>::None;

        #[cfg(feature = "log")]
        trace!(
            "GPS (state={:?}) ptr={} dword=0x{:08x}",
            self.state,
            self.ptr,
            self.dword
        );

        if self.ptr == 4 {
            // 2 MSB padding
            self.dword &= GPS_BITMASK;

            match self.state {
                State::TLM => {
                    let tlm = GpsQzssTelemetry::decode(self.dword, self.parity_check);

                    match tlm {
                        Ok(tlm) => {
                            self.tlm = tlm;
                            self.state = State::HOW;
                        },
                        #[cfg(feature = "log")]
                        Err(e) => {
                            error!("GPS (state={:?}): {:?}", self.state, e);
                        },
                        #[cfg(not(feature = "log"))]
                        Err(_) => {},
                    }
                },

                State::HOW => {
                    let how = GpsQzssHow::decode(self.dword, self.parity_check);

                    match how {
                        Ok(how) => {
                            self.how = how;
                            self.state = State::Word(0);
                        },
                        #[cfg(feature = "log")]
                        Err(e) => {
                            error!("GPS (state={:?}): {:?}", self.state, e);
                        },
                        #[cfg(not(feature = "log"))]
                        Err(_) => {},
                    }
                },

                State::Word(word) => match word {
                    0 => {
                        self.state = State::Word(1);
                    },
                    1 => {
                        self.state = State::Word(2);
                    },
                    2 => {
                        self.state = State::Word(3);
                    },
                    3 => {
                        self.state = State::Word(4);
                    },
                    4 => {
                        self.state = State::Word(5);
                    },
                    5 => {
                        self.state = State::Word(6);
                    },
                    6 => {
                        ret = Some(GpsQzssFrame {
                            telemetry: self.tlm.clone(),
                            how: self.how.clone(),
                        });

                        self.reset();
                    },
                    #[cfg(feature = "log")]
                    word => {
                        error!(
                            "GPS (state={:?}): internal error dword=#{}",
                            self.state, word
                        );
                        self.reset();
                    },
                    #[cfg(not(feature = "log"))]
                    _ => {
                        self.reset();
                    },
                },
            }

            self.ptr = 0;
        }

        ret
    }

    /// Create a new [GpsQzssDecoder] that will not verify the parity bits
    pub fn without_parity_verification(&self) -> Self {
        Self {
            state: self.state,
            ptr: self.ptr,
            dword: self.dword,
            parity_check: false,
            tlm: self.tlm.clone(),
            how: self.how.clone(),
        }
    }

    /// Reset internal state
    fn reset(&mut self) {
        self.dword &= 0x00000000;
        self.ptr = 0;
        self.state = State::default();
    }
}

#[cfg(test)]
mod test {
    use crate::gps::GpsQzssDecoder;

    #[cfg(feature = "std")]
    use crate::tests::init_logger;
    #[test]
    fn test_incomplete_frame() {
        #[cfg(feature = "std")]
        init_logger();

        let mut found = false;

        let bytes = [
            0x8B, 0x00, 0x00, 0x00, // TLM
            0x00, 0x00, 0x01, 0x00, // HOW
            0x00, 0x00, 0x01, 0x00, // Word 3
            0x00, 0x00, 0x01, 0x00, // Word 4
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.parse(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded!");

        let bytes = [
            0x8B, 0x00, 0x00, 0x00, // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.parse(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded");

        let bytes = [
            0x8B, 0x00, 0x00, 0x00, // TLM
            0x00, 0x00, 0x01, 0x00, // HOW
            0x00, 0x00, 0x01, 0x00, // Word 1
            0x00, 0x00, 0x01, 0x00, // Word 2
            0x00, 0x00, 0x01, 0x00, // Word 3
            0x00, 0x00, 0x01, 0x00, // Word 4
            0x00, 0x00, 0x01, 0x00, // Word 5
            0x00, 0x00, 0x01, // Word 6
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.parse(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded!");
    }

    #[test]
    fn test_eph1_frame() {
        #[cfg(feature = "std")]
        init_logger();

        let mut found = false;

        let bytes = [
            0x8B, 0x00, 0x00, 0x00, // TLM
            0x00, 0x00, 0x01, 0x00, // HOW
            0x00, 0x00, 0x01, 0x00, // Word 3
            0x00, 0x00, 0x01, 0x00, // Word 4
            0x00, 0x00, 0x01, 0x00, // Word 5
            0x00, 0x00, 0x01, 0x00, // Word 6
            0x00, 0x00, 0x01, 0x00, // Word 7
            0x00, 0x00, 0x01, 0x00, // Word 8
            0x00, 0x00, 0x01, 0x00, // Word 9
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(frame) = decoder.parse(byte) {
                found = true;
            }
        }

        assert!(found, "GPS decoding failed!");
    }
}
