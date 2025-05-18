#[cfg(feature = "log")]
use log::{error, trace};

use crate::gps::{
    frame1::{
        Word10 as Frame1Word10, Word3 as Frame1Word3, Word4 as Frame1Word4, Word5 as Frame1Word5,
        Word6 as Frame1Word6, Word7 as Frame1Word7, Word8 as Frame1Word8, Word9 as Frame1Word9,
    },
    GpsQzssFrame, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, UnscaledSubframe, GPS_BITMASK,
};

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

    /// Pending [UnscaledSubframe]
    subframe: UnscaledSubframe,

    /// True if parity verification is requested
    parity_check: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] with parity bit verification
    fn default() -> Self {
        Self {
            parity_check: true,
            state: Default::default(),
            ptr: Default::default(),
            dword: Default::default(),
            tlm: Default::default(),
            how: Default::default(),
            subframe: Default::default(),
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
                            self.state = State::Word(3);
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
                    3 => {
                        let word3 = Frame1Word3::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word3 = word3,
                        }

                        self.state = State::Word(4);
                    },
                    4 => {
                        let word4 = Frame1Word4::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word4 = word4,
                        }

                        self.state = State::Word(5);
                    },
                    5 => {
                        let word5 = Frame1Word5::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word5 = word5,
                        }

                        self.state = State::Word(6);
                    },
                    6 => {
                        let word6 = Frame1Word6::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word6 = word6,
                        }

                        self.state = State::Word(7);
                    },
                    7 => {
                        let word7 = Frame1Word7::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word7 = word7,
                        }
                        self.state = State::Word(8);
                    },
                    8 => {
                        let word8 = Frame1Word8::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word8 = word8,
                        }

                        self.state = State::Word(9);
                    },
                    9 => {
                        let word9 = Frame1Word9::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word9 = word9,
                        }
                        self.state = State::Word(10);
                    },
                    10 => {
                        let word10 = Frame1Word10::decode(self.dword);
                        match &mut self.subframe {
                            UnscaledSubframe::Eph1(frame1) => frame1.word10 = word10,
                        }

                        ret = Some(GpsQzssFrame {
                            how: self.how.clone(),
                            telemetry: self.tlm.clone(),
                            subframe: match &self.subframe {
                                UnscaledSubframe::Eph1(frame1) => {
                                    GpsQzssSubframe::Eph1(frame1.scale())
                                },
                            },
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
            subframe: self.subframe.clone(),
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
    use crate::{gps::GpsQzssDecoder, GpsQzssSubframe};

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
            // TLM
            0x8B, 0x04, 0xF8, 0x00, 0x54, 0x9F, 0x25, 0x00, 0x13, 0xE4, 0x00, 0x04, // WORD3
            0x10, 0x4F, 0x5D, 0x31, // WORD4
            0x97, 0x44, 0xE6, 0xE7, // WORD5
            0x07, 0x75, 0x57, 0x83, // WORD6
            0x33, 0x0C, 0x80, 0xB5, // WORD7
            0x92, 0x50, 0x42, 0xA1, // WORD8
            0x80, 0x00, 0x16, 0x84, // WORD9
            0x31, 0x2C, 0x30, 0x33, // WORD10
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(frame) = decoder.parse(byte) {
                match frame.subframe {
                    GpsQzssSubframe::Eph1(frame1) => {
                        assert_eq!(frame1.af2_s_s2, 0.0);
                        assert!((frame1.af1_s_s - 1.023181539495e-011).abs() < 1e-14);
                        assert!((frame1.af0_s - -4.524961113930e-004).abs() < 1.0e-11);
                        assert_eq!(frame1.week, 318);
                        assert_eq!(frame1.toc_s, 266_400);
                        assert_eq!(frame1.health, 0);
                    },
                    _ => panic!("incorrect subframe decoded!"),
                }

                found = true;
            }
        }

        assert!(found, "GPS decoding failed!");
    }
}
