use crate::gps::{
    frame1::{
        Word10 as Ephemeris1Word10, Word3 as Ephemeris1Word3, Word4 as Ephemeris1Word4,
        Word5 as Ephemeris1Word5, Word6 as Ephemeris1Word6, Word7 as Ephemeris1Word7,
        Word8 as Ephemeris1Word8, Word9 as Ephemeris1Word9,
    },
    GpsDataByte, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
    GPS_PREAMBLE_MASK,
};

pub(crate) const GPS_PARITY_MASK: u32 = 0x3f;

#[cfg(feature = "log")]
use log::{debug, error, trace};

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

pub struct GpsQzssDecoder {
    /// Frame counter
    ptr: usize,

    /// data word storage
    pub(crate) dword: u32,

    /// total number of bits collected
    pub(crate) collected: usize,

    /// next shift to the right
    next_shift: usize,

    /// Current [State]
    state: State,

    /// Latest [GpsQzssTelemetry]
    tlm: GpsQzssTelemetry,

    /// Latest [GpsQzssHow]
    how: GpsQzssHow,

    /// Pending [GpsQzssSubframe]
    subframe: GpsQzssSubframe,

    /// True if parity verification is requested
    parity_check: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] with parity bit verification
    fn default() -> Self {
        let state = State::default();

        Self {
            state,
            ptr: 0,
            dword: 0,
            collected: 0,
            next_shift: 0,
            parity_check: true,
            tlm: Default::default(),
            how: Default::default(),
            subframe: Default::default(),
        }
    }
}

impl GpsQzssDecoder {
    /// Parses a GPS/QZSS stream of [GpsDataByte]s, returns a [GpsQzssFrame] when
    /// one is identified and the last data word has been correctly processed.
    pub fn parse(&mut self, byte: GpsDataByte) -> Option<GpsQzssFrame> {
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
                    trace!("GPS - TLM {:?}", tlm);
                },
                Err(e) => {
                    error!("GPS - TLM decoding error: {:?}", e);
                },
            },

            State::How => match GpsQzssHow::decode(dword) {
                Ok(how) => {
                    self.how = how;
                    self.state = State::DataWord;
                    self.ptr = 3;

                    match how.frame_id {
                        GpsQzssFrameId::Ephemeris1 => {
                            self.subframe = GpsQzssSubframe::Eph1(Default::default());
                        },
                        GpsQzssFrameId::Ephemeris2 => {
                            self.subframe = GpsQzssSubframe::Eph2(Default::default());
                        },
                        GpsQzssFrameId::Ephemeris3 => {
                            panic!("not yet");
                        },
                    }
                    trace!("GPS - HOW {:?}", how);
                },
                Err(e) => {
                    error!("GPS - HOW decoding error: {:?}", e);
                },
            },

            State::DataWord => {
                match self.how.frame_id {
                    GpsQzssFrameId::Ephemeris1 => {
                        match self.ptr {
                            3 => {
                                let word = Ephemeris1Word3::decode(self.dword);

                                let frame = self.subframe.as_mut_eph1().expect("internal error");

                                frame.week = word.week;
                                frame.ura = word.ura;
                                frame.ca_or_p_l2 = word.ca_or_p_l2;
                                frame.health = word.health;
                                frame.iodc |= (word.iodc_msb as u16) << 8;
                                trace!("GPS - EPH #1 Word#3 {:?}", word);
                            },
                            4 => {
                                let word = Ephemeris1Word4::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.l2_p_data_flag = word.l2_p_data_flag;
                                frame.reserved_word4 = word.reserved;
                                trace!("GPS - EPH #1 Word#4 {:?}", word);
                            },
                            5 => {
                                // TODO word3.iodc_msb
                                let word = Ephemeris1Word5::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.reserved_word5 = word.reserved;
                                trace!("GPS - EPH #1 Word#5 {:?}", word);
                            },
                            6 => {
                                let word = Ephemeris1Word6::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.reserved_word6 = word.reserved;
                                trace!("GPS - EPH #1 Word#6 {:?}", word);
                            },
                            7 => {
                                let word = Ephemeris1Word7::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.tgd = (word.tgd as f64) / 2_f64.powi(31);
                                frame.reserved_word7 = word.reserved;
                                trace!("GPS - EPH #1 Word#7 {:?}", word);
                            },
                            8 => {
                                let word = Ephemeris1Word8::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.toc = (word.toc as u32) * 16;
                                frame.iodc |= word.iodc_lsb as u16;
                                trace!("GPS - EPH #1 Word#8 {:?}", word);
                            },
                            9 => {
                                let word = Ephemeris1Word9::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.af2 = (word.af2 as f64) / 2.0_f64.powi(55);
                                frame.af1 = (word.af1 as f64) / 2.0_f64.powi(43);
                                trace!("GPS - EPH #1 Word#9 {:?}", word);
                            },
                            10 => {
                                let word = Ephemeris1Word10::decode(self.dword);
                                let frame = self.subframe.as_mut_eph1().expect("internal error");
                                frame.af0 = (word.af0 as f64) / 2.0_f64.powi(31);
                                trace!("GPS - EPH #1 Word#10 {:?}", word);
                            },
                            _ => {
                                unreachable!("invalid state");
                            },
                        }
                    },
                    GpsQzssFrameId::Ephemeris2 => {
                        panic!("not yet");
                    },
                    GpsQzssFrameId::Ephemeris3 => {
                        panic!("not yet");
                    },
                }

                self.ptr += 1;
            },
        }

        // reset
        self.collected = 0;
        self.dword = 0;
        self.next_shift = 0;

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
            next_shift: 0,
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
    use crate::{
        gps::{GpsDataByte, GpsQzssDecoder, GpsQzssSubframe},
        GpsQzssFrameId,
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
            let _ = decoder.parse(byte);
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
            let _ = decoder.parse(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        let bytes = [0x22, 0xC1, 0x3E, 0x1B];
        let bytes = from_ublox_be_bytes(&bytes);

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.parse(byte);
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
            GpsDataByte::msb_padded(0x73),
            GpsDataByte::Byte(0xC9),
            GpsDataByte::Byte(0x27),
            GpsDataByte::Byte(0x15), // HOW
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.parse(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);
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
            let _ = decoder.parse(byte);
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
            let _ = decoder.parse(byte);
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
            if let Some(_) = decoder.parse(byte) {
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
            if let Some(_) = decoder.parse(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded");
    }

    #[test]
    fn test_eph1_frame() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            // TLM
            0x22, 0xC1, 0x3E, 0x1B, // HOW
            0x73, 0xC9, 0x27, 0x15, // WORD3
            0x13, 0xE4, 0x00, 0x04, //WORD4
            0x10, 0x4F, 0x5D, 0x31, //WORD5
            0x97, 0x44, 0xE6, 0xD7, // WORD6
            0x07, 0x75, 0x57, 0x83, //WORD7
            0x33, 0x0C, 0x80, 0xB5, // WORD8
            0x92, 0x50, 0x42, 0xA1, // WORD9
            0x80, 0x00, 0x16, 0x84, //WORD10
            0x31, 0x2C, 0x30, 0x33,
        ]);

        let mut decoder = GpsQzssDecoder::default().without_parity_verification();

        for byte in bytes {
            if let Some(frame) = decoder.parse(byte) {
                assert_eq!(frame.telemetry.message, 3);
                assert_eq!(frame.telemetry.integrity, true);
                assert_eq!(frame.telemetry.reserved_bits, false);

                assert_eq!(frame.how.tow, 3 * 6);
                assert_eq!(frame.how.alert, false);
                assert_eq!(frame.how.anti_spoofing, true);
                assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris1);

                match frame.subframe {
                    GpsQzssSubframe::Eph1(frame1) => {
                        assert_eq!(frame1.af2, 0.0);
                        assert!((frame1.af1 - 1.023181539495e-011).abs() < 1e-14);
                        assert!((frame1.af0 - -4.524961113930e-004).abs() < 1.0e-11);
                        assert_eq!(frame1.week, 318);
                        assert_eq!(frame1.toc, 266_400);
                        assert_eq!(frame1.health, 0);
                        found = true;
                    },
                    _ => panic!("incorrect subframe decoded!"),
                }
            }
        }

        assert!(found, "GPS decoding failed!");
    }
}
