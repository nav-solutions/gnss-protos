use crate::{
    bitstream::{BitStream, Byte},
    gps::{
        GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, State,
        GPS_PREAMBLE_MASK,
    },
};

#[cfg(feature = "log")]
use log::{debug, trace};

pub struct GpsQzssDecoder {
    /// Frame counter
    ptr: usize,

    /// Current [State]
    state: State,

    /// Previous [State]
    prev_state: State,

    /// [BitStream] collecter
    bitstream: BitStream,

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
            prev_state: state,
            parity_check: true,
            tlm: Default::default(),
            how: Default::default(),
            subframe: Default::default(),
            bitstream: BitStream::msbf().with_collection_size(state.bin_size()),
        }
    }
}

impl GpsQzssDecoder {
    /// Parses a GPS/QZSS stream of bytes, returns a [GpsQzssFrame] when
    /// one is identified and the last data word has been correctly processed.
    ///
    /// GPS binary streams are made 30 bit MSBF data words, so they are not aligned.   
    /// This API allows you to consider both real (unaligned) GPS data streams and
    /// streams that were padded to a convenient format.
    ///
    /// 1. Let's say you only have _actual_ GPS data from a binary file.
    /// Simply forward all bytes and wrap them as plain [Byte::Byte]s.   
    /// Even in that scenario, the stream is most likely terminated by some padding
    /// during the file encoding process. Emphasize that with one last [Byte::LsbPadded],
    /// and we are able to process all data frames.   
    ///
    /// 2. When working with padded data words (usually 32-bit padded),
    /// like when using U-Blox raw data streams for instance, that insert a 2-MSB bit padding.
    /// Since GPS expects MSB first, then streams would start with a single [Byte::MsbPadded], and follow
    /// with 3 plain data [Byte::Byte]s. Remember that GPS expects MSB first!
    pub fn parse(&mut self, byte: Byte) -> Option<GpsQzssFrame> {
        let dword = self.bitstream.collect(byte)?;

        let mut ret = Option::<GpsQzssFrame>::None;

        #[cfg(feature = "log")]
        trace!(
            "GPS - state={:?} - ptr={} dword=0x{:08x}",
            self.state,
            self.ptr,
            dword
        );

        let next_state = match self.state {
            State::Preamble => {
                if dword as u8 == GPS_PREAMBLE_MASK {
                    State::TlmIntegrity
                } else {
                    State::Preamble
                }
            },
            State::TlmIntegrity => {
                self.tlm = GpsQzssTelemetry {
                    message: (dword >> 2) as u16,
                    integrity: (dword & 0x02) > 0,
                    reserved_bits: (dword & 0x01) > 0,
                };

                #[cfg(feature = "log")]
                debug!("decoded tlm {:?}", self.tlm);

                State::Parity
            },
            State::Parity => State::How,
            State::How => State::HowParity,
            State::HowParity => {
                panic!("done");
            },
            State::DataWord => State::DataWordParity,
            State::DataWordParity => {
                panic!("done");
            },
        };

        if next_state != self.state {
            self.new_state(next_state);
        }

        ret
    }

    /// Create a new [GpsQzssDecoder] that will not verify the parity bits
    pub fn without_parity_verification(&self) -> Self {
        Self {
            ptr: self.ptr,
            state: self.state,
            parity_check: false,
            tlm: self.tlm.clone(),
            how: self.how.clone(),
            prev_state: self.prev_state,
            bitstream: self.bitstream,
            subframe: self.subframe.clone(),
        }
    }

    /// Reset internal state
    fn reset(&mut self) {
        self.ptr = 0;
        self.new_state(State::default());
    }

    /// Update [State]
    fn new_state(&mut self, state: State) {
        self.prev_state = self.state;
        self.state = state;
        self.bitstream.set_size_to_collect(state.bin_size());

        #[cfg(feature = "log")]
        trace!("GPS new state={:?}", self.state);
    }

    fn data_word_decoding(&mut self, dword: u32) -> State {
        match self.how.frame_id {
            GpsQzssFrameId::Ephemeris1 => self.eph1_word_decoding(dword),
            GpsQzssFrameId::Ephemeris2 => self.eph2_word_decoding(dword),
            GpsQzssFrameId::Ephemeris3 => self.eph3_word_decoding(dword),
        }
    }

    fn eph1_word_decoding(&mut self, dword: u32) -> State {
        let next_state = match self.ptr {
            0 => {
                if let Some(frame1) = self.subframe.as_mut_eph1() {
                    // let week = ((dword & WORD3_WEEK_MASK) >> WORD3_WEEK_SHIFT) as u16;
                    // let ca_or_p_l2 = ((dword & WORD3_CA_P_L2_MASK) >> WORD3_CA_P_L2_SHIFT) as u8;
                    // let ura = ((dword & WORD3_URA_MASK) >> WORD3_URA_SHIFT) as u8;
                    // let health = ((dword & WORD3_HEALTH_MASK) >> WORD3_HEALTH_SHIFT) as u8;
                    // let iodc_msb = ((dword & WORD3_IODC_MASK) >> WORD3_IODC_SHIFT) as u8;

                    // frame1.week = word3.week;
                    // frame1.ca_or_p_l2 = word3.ca_or_p_l2;
                    // frame1.ura = word3.ura;
                    // frame1.health = word3.health;

                    // self.iodc_msb = word3.iodc_msb;
                    State::default()
                } else {
                    State::default()
                }
            },
            1 => State::default(),
            2 => State::default(),
            3 => State::default(),
            4 => State::default(),
            5 => State::default(),
            _ => State::default(),
        };

        self.ptr += 1;
        next_state
    }

    fn eph2_word_decoding(&mut self, dword: u32) -> State {
        State::default()
    }

    fn eph3_word_decoding(&mut self, dword: u32) -> State {
        State::default()
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{GpsQzssDecoder, GpsQzssSubframe},
        prelude::Byte,
    };

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
            if let Some(_) = decoder.parse(Byte::byte(byte)) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded!");

        let bytes = [
            0x8Bu8, 0x00u8, 0x00u8, 0x00u8, // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.parse(Byte::byte(byte)) {
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
            if let Some(_) = decoder.parse(Byte::byte(byte)) {
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
            0x8B, 0x04, 0xF8, 0x00, // HOW
            0x54, 0x9F, 0x25, 0x00, 0x13, 0xE4, 0x00, 0x04, // WORD3
            0x10, 0x4F, 0x5D, 0x31, // WORD4
            0x97, 0x44, 0xE6, 0xE7, // WORD5
            0x07, 0x75, 0x57, 0x83, // WORD6
            0x33, 0x0C, 0x80, 0xB5, // WORD7
            0x92, 0x50, 0x42, 0xA1, // WORD8
            0x80, 0x00, 0x16, 0x84, // WORD9
            0x31, 0x2C, 0x30, 0x33, // WORD10
        ];

        let mut decoder = GpsQzssDecoder::default().without_parity_verification();

        for byte in bytes {
            if let Some(frame) = decoder.parse(Byte::byte(byte)) {
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
//     #[test]
//     fn test_eph2_frame() {
//         #[cfg(feature = "std")]
//         init_logger();

//         let mut found = false;

//         let bytes = [
//             0x8B, 0x04, 0xF8, 0x00, // 1,
//             0x54, 0x9F, 0xA8, 0x00, // 2,
//             0x49, 0xff, 0xc5, 0x00, // 3
//             0x31, 0xA0, 0x7d, 0x00, // 4,
//             0x09, 0x24, 0xD0, 0x00, // 5
//             0xFF, 0xE2, 0x04, 0x00, // 6
//             0x64, 0x6E, 0x04, 0x00, // 7
//             0x10, 0xF9, 0xA1, 0x00, // 8
//             0x0C, 0xD1, 0xC8, 0x00, // 9
//             0x41, 0x0A, 0x7d, 0x00, // 10
//         ];

//         let mut decoder = GpsQzssDecoder::default().without_parity_verification();

//         for byte in bytes {
//             if let Some(frame) = decoder.parse(byte) {
//                 match frame.subframe {
//                     GpsQzssSubframe::Eph2(frame2) => {
//                         // assert_eq!(frame2.dn, 1.444277586415e-009);
//                         // assert!((frame2.dn - 1.444277586415e-009).abs() < 1e-9);
//                         assert_eq!(frame2.fit_int_flag, true);

//                         assert_eq!(frame2.toe_s, 266_400);
//                         assert_eq!(frame2.iode, 0x27);
//                         // assert_eq!(frame2.crs, -1.843750000000e+000);

//                         // assert!((frame2.sqrt_a - 5.153602432251e+003).abs() < 1e-9);
//                         // assert!((frame2.m0 - 9.768415465951e-001).abs() < 1e-9);
//                         // assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-9);
//                         // assert!((frame2.e - 8.578718174249e-003).abs() < 1e-9);
//                         // assert!((frame2.cus - 8.093193173409e-006).abs() < 1e-9);
//                         // assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-6);
//                         found = true;
//                     },
//                     _ => panic!("incorrect subframe decoded!"),
//                 }
//             }
//         }

//         assert!(found, "GPS decoding failed!");
//     }
// }
