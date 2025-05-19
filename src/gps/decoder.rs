#[cfg(feature = "log")]
use log::{debug, error, trace};

use crate::{
    bitstream::{BitStream, Byte},
    gps::{
        frame1::{
            Word10 as Frame1Word10, Word3 as Frame1Word3, Word4 as Frame1Word4,
            Word5 as Frame1Word5, Word6 as Frame1Word6, Word7 as Frame1Word7, Word8 as Frame1Word8,
            Word9 as Frame1Word9,
        },
        frame2::{
            Word10 as Frame2Word10, Word3 as Frame2Word3, Word4 as Frame2Word4,
            Word5 as Frame2Word5, Word6 as Frame2Word6, Word7 as Frame2Word7, Word8 as Frame2Word8,
            Word9 as Frame2Word9,
        },
        GpsQzssFrame, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
    },
    GpsQzssFrameId,
};

const GPS_PREAMBLE_MASK: u8 = 0x8b;
const GPS_DATAWORD_SIZE: usize = 30;

#[derive(Debug, Copy, Clone, PartialEq, Default)]
pub enum State {
    #[default]
    Preamble,
    TlmIntegrity,
    Parity,
    Tow,
    FrameId,
    DataWord,
}

impl State {
    pub fn encoding_size(&self) -> usize {
        match self {
            Self::Preamble => 8,
            Self::TlmIntegrity => 16,
            Self::Parity => 6,
            Self::Tow => 17,
            Self::FrameId => 7,
            Self::DataWord => GPS_DATAWORD_SIZE - 6,
        }
    }
}

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
            bitstream: BitStream::msbf().with_collection_size(state.encoding_size()),
        }
    }
}

impl GpsQzssDecoder {
    /// Parse a GPS/QZSS stream of bytes, which must be aligned to 32-bit with 2 MSB padding
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
            State::TlmIntegrity => State::Parity,
            State::Parity => {
                match self.prev_state {
                    State::TlmIntegrity => State::Tow,
                    State::FrameId => {
                        self.ptr = 0;
                        State::DataWord
                    },
                    _ => {
                        // invalid case!
                        State::default()
                    },
                }
            },
            State::Tow => {
                let tow = (dword & 0x1ffff) * 6;
                self.how.tow = tow;

                #[cfg(feature = "log")]
                debug!("GPS TOW - value={} (s)", tow);

                State::FrameId
            },
            State::FrameId => {
                self.how.alert = (dword & 0x80) > 0;
                self.how.anti_spoofing = (dword & 0x40) > 0;

                if let Ok(frame_id) = GpsQzssFrameId::decode((dword & 0x7) as u8) {
                    #[cfg(feature = "log")]
                    debug!("GPS FRAMEID - value={:?}", frame_id);

                    self.how.frame_id = frame_id;

                    State::Parity
                } else {
                    #[cfg(feature = "log")]
                    error!("GPS invalid frame id=0x{:02X}", dword & 0x7);
                    State::default()
                }
            },
            State::DataWord => self.data_word_decoding(dword),
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
        self.bitstream.set_size_to_collect(state.encoding_size());

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
            0x54, 0x9F, 0x25, 0x00, //HOW
            0x13, 0xE4, 0x00, 0x04, // WORD3
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
                        assert_eq!(frame1.af2_s_s2, 0.0);
                        assert!((frame1.af1_s_s - 1.023181539495e-011).abs() < 1e-14);
                        assert!((frame1.af0_s - -4.524961113930e-004).abs() < 1.0e-11);
                        assert_eq!(frame1.week, 318);
                        assert_eq!(frame1.toc_s, 266_400);
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
