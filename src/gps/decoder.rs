use crate::gps::{
    GpsDataByte, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
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
    fn test_eph1_frame_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            // TLM
            0x22, 0xC1, 0x3E, 0x1B, // HOW
            0x15, 0x27, 0xC9, 0x73, // WORD3
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
                assert_eq!(decoder.tlm.message, 0x13E);
                assert_eq!(decoder.tlm.integrity, false);
                assert_eq!(decoder.tlm.reserved_bits, false);

                assert_eq!(decoder.how.alert, false);
                assert_eq!(decoder.how.anti_spoofing, true);
                assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);

                match frame.subframe {
                    GpsQzssSubframe::Ephemeris1(frame1) => {
                        assert_eq!(frame1.af2, 0.0);
                        assert!((frame1.af1 - 1.023181539495E-11).abs() < 1e-14);
                        assert!((frame1.af0 - -4.524961113930E-04).abs() < 1.0e-11);
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

    #[test]
    fn test_eph2_frame_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            // TLM
            0x22, 0xC1, 0x3E, 0x1B, // HOW
            0x15, 0x27, 0xEA, 0x1B, // WORD3
            0x12, 0x7F, 0xF1, 0x65, //WORD4
            0x8C, 0x68, 0x1F, 0x7C, //WORD5
            0x02, 0x49, 0x34, 0x15, // WORD6
            0xBF, 0xF8, 0x81, 0x1E, //WORD7
            0x99, 0x1B, 0x81, 0x14, // W0RD8
            0x04, 0x3E, 0x68, 0x6E, // WORD9
            0x83, 0x34, 0x72, 0x21, // WORD10
            0x90, 0x42, 0x9F, 0x7B,
        ]);

        let mut decoder = GpsQzssDecoder::default().without_parity_verification();

        for byte in bytes {
            if let Some(frame) = decoder.parse(byte) {
                assert_eq!(decoder.tlm.message, 0x13E);
                assert_eq!(decoder.tlm.integrity, false);
                assert_eq!(decoder.tlm.reserved_bits, false);

                assert_eq!(decoder.how.alert, false);
                assert_eq!(decoder.how.anti_spoofing, true);
                assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris2);

                match frame.subframe {
                    GpsQzssSubframe::Ephemeris2(frame2) => {
                        assert_eq!(frame2.toe, 266_400);
                        assert_eq!(frame2.crs, -1.843750000000e+000);
                        assert!((frame2.sqrt_a - 5.153602432251e+003).abs() < 1e-9);
                        assert!((frame2.m0 - 9.768415465951e-001).abs() < 1e-9);
                        assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-9);
                        assert!((frame2.e - 8.578718174249e-003).abs() < 1e-9);
                        assert!((frame2.cus - 8.093193173409e-006).abs() < 1e-9);
                        assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-6);
                        assert!((frame2.dn - 1.444277586415e-009).abs() < 1e-9);
                        assert_eq!(frame2.fit_int_flag, false);
                        found = true;
                    },
                    _ => panic!("incorrect subframe decoded!"),
                }
            }
        }

        assert!(found, "GPS decoding failed!");
    }

    #[test]
    fn test_eph3_frame_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            0x22, 0xC1, 0x3E, 0x1B, 0x15, 0x28, 0x0B, 0xDB, 0x00, 0x0A, 0xEA, 0x34, 0x03, 0x3C,
            0xFF, 0xEE, 0xBF, 0xE5, 0xC9, 0xEB, 0x13, 0x6F, 0xB6, 0x4E, 0x86, 0xF4, 0xAB, 0x2C,
            0x06, 0x71, 0xEB, 0x44, 0x3F, 0xEA, 0xF6, 0x02, 0x92, 0x45, 0x52, 0x13,
        ]);

        let mut decoder = GpsQzssDecoder::default().without_parity_verification();

        for byte in bytes {
            if let Some(frame) = decoder.parse(byte) {
                assert_eq!(decoder.tlm.message, 0x13E);
                assert_eq!(decoder.tlm.integrity, false);
                assert_eq!(decoder.tlm.reserved_bits, false);

                assert_eq!(decoder.how.alert, false);
                assert_eq!(decoder.how.anti_spoofing, true);
                assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris3);

                match frame.subframe {
                    GpsQzssSubframe::Ephemeris3(frame3) => {
                        assert!((frame3.cic - 8.009374141693e-008).abs() < 1e-9);
                        assert!((frame3.cis - -1.955777406693e-007).abs() < 1E-9);
                        assert!((frame3.crc - 2.225625000000e+002).abs() < 1E-9);
                        assert!((frame3.i0 - 3.070601043291e-001).abs() < 1e-9);
                        assert!((frame3.idot - 1.548414729768e-010).abs() < 1E-9);
                        assert!((frame3.omega0 - -6.871047024615e-001).abs() < 1e-9);
                        assert!((frame3.omega_dot - -2.449269231874e-009).abs() < 1e-9);
                        assert!((frame3.omega - -6.554632573389e-001).abs() < 1e-9);
                        found = true;
                    },
                    _ => panic!("incorrect subframe decoded!"),
                }
            }
        }

        assert!(found, "GPS decoding failed!");
    }

    #[test]
    fn test_eph1_frame_encoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        for (
            ith,
            (tow, alert, anti_spoofing, frame_id, message, integrity, tlm_reserved_bits, dwords),
        ) in [
            (
                259956,
                false,
                false,
                GpsQzssFrameId::Ephemeris1,
                0x13E,
                false,
                false,
                [
                    0x1527C100, 0x22C13E00, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                    0x00000000, 0x00000000, 0x00000000, 0x00000000,
                ],
            ),
            (
                259957,
                true,
                true,
                GpsQzssFrameId::Ephemeris1,
                0x13F,
                true,
                true,
                [
                    0x1527D900, 0x22C13FC0, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                    0x00000000, 0x00000000, 0x00000000, 0x00000000,
                ],
            ),
        ]
        .iter()
        .enumerate()
        {
            let frame = GpsQzssFrame {
                how: GpsQzssHow {
                    tow: *tow,
                    alert: *alert,
                    frame_id: *frame_id,
                    anti_spoofing: *anti_spoofing,
                },
                telemetry: GpsQzssTelemetry {
                    message: *message,
                    integrity: *integrity,
                    reserved_bits: *tlm_reserved_bits,
                },
                subframe: Default::default(),
            };

            let encoded = frame.encode();

            for (i, expected) in dwords.iter().enumerate() {
                assert_eq!(
                    encoded[i], *expected,
                    "TEST #{} DWORD({}) expected 0x{:08X}, got 0x{:08X}",
                    ith, i, dwords[i], encoded[i]
                );
            }
        }
    }
}
