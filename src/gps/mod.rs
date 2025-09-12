/// GPS data word size (in bits!)
pub const GPS_WORD_BITS: usize = 30;

/// Number of words in a frame
pub const GPS_WORDS_PER_FRAME: usize = 10;

/// Total GPS/QZSS frame size (in bits!)
pub const GPS_FRAME_BITS: usize = GPS_WORDS_PER_FRAME * GPS_WORD_BITS;

/// Total GPS/QZSS frame size (in bytes!)
pub const GPS_FRAME_BYTES: usize = (GPS_FRAME_BITS / 8) + 1;

mod bytes;
mod decoding;
mod encoding;
mod errors;
mod frame1;
mod frame2;
mod frame3;
mod frame_id;
mod how;
mod reader;
mod tlm;

pub use bytes::GpsDataByte;
pub use errors::GpsError;
pub use frame_id::GpsQzssFrameId;
pub use how::GpsQzssHow;
pub use tlm::GpsQzssTelemetry;

pub use frame1::GpsQzssFrame1;
pub use frame2::GpsQzssFrame2;
pub use frame3::GpsQzssFrame3;

pub(crate) use reader::BitReader;

/// GPS / QZSS interpreted frame.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame {
    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,

    /// [GpsQzssTelemetry] describes following frame.
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssSubframe] depends on associated How.
    pub subframe: GpsQzssSubframe,
}

impl GpsQzssFrame {
    /// Copies and returns with updated [GpsQzssHow] data word
    pub fn with_how_word(mut self, how: GpsQzssHow) -> Self {
        self.how = how;
        self
    }

    /// Copies and returns with updated [GpsQzssTelemetry] data word
    pub fn with_telemetry(mut self, telemetry: GpsQzssTelemetry) -> Self {
        self.telemetry = telemetry;
        self
    }

    /// Copies and returns an updated [GpsQzssSubframe]
    pub fn with_subframe(mut self, subframe: GpsQzssSubframe) -> Self {
        self.subframe = subframe;
        self
    }
}

/// GPS / QZSS Interpreted subframes
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsQzssSubframe {
    /// GPS Ephemeris Frame #1
    Ephemeris1(GpsQzssFrame1),

    /// GPS Ephemeris Frame #2
    Ephemeris2(GpsQzssFrame2),

    /// GPS Ephemeris Frame #3
    Ephemeris3(GpsQzssFrame3),
}

impl Default for GpsQzssSubframe {
    /// Builds a default [GpsQzssSubFrame::Ephemeris1]
    fn default() -> Self {
        Self::Ephemeris1(Default::default())
    }
}

impl GpsQzssSubframe {
    /// Unwraps self as [GpsQzssFrame1] reference (if feasible)
    pub fn as_eph1(&self) -> Option<&GpsQzssFrame1> {
        match self {
            Self::Ephemeris1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as mutable [GpsQzssFrame1] reference (if feasible)
    pub fn as_mut_eph1(&mut self) -> Option<&mut GpsQzssFrame1> {
        match self {
            Self::Ephemeris1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    pub fn as_eph2(&self) -> Option<&GpsQzssFrame2> {
        match self {
            Self::Ephemeris2(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    pub fn as_mut_eph2(&mut self) -> Option<&mut GpsQzssFrame2> {
        match self {
            Self::Ephemeris2(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame3] reference (if feasible)
    pub fn as_eph3(&self) -> Option<&GpsQzssFrame3> {
        match self {
            Self::Ephemeris3(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame3] reference (if feasible)
    pub fn as_mut_eph3(&mut self) -> Option<&mut GpsQzssFrame3> {
        match self {
            Self::Ephemeris3(frame) => Some(frame),
            _ => None,
        }
    }

    /// Decodes a [GpsQzssSubframe] from a 8 [GpsDataByte] slice
    /// supports padding on each word termination (not intra words)
    pub(crate) fn decode(frame_id: GpsQzssFrameId, bytes: &[GpsDataByte]) -> Self {
        // for i in 0..8 {
        //     let dword = ByteArray::new(&bytes[8+i*4..8+i*4+4])
        //         .value_u32();

        //     let _ = subframe.decode_word(i +3, dword);  // IMPROVE
        //

        match frame_id {
            GpsQzssFrameId::Ephemeris1 => Self::Ephemeris1(GpsQzssFrame1::decode(bytes)),
            GpsQzssFrameId::Ephemeris2 => Self::Ephemeris2(GpsQzssFrame2::decode(bytes)),
            GpsQzssFrameId::Ephemeris3 => Self::Ephemeris3(GpsQzssFrame3::decode(bytes)),
        }
    }

    /// Encode this [GpsQzssSubframe] into 8 [u32] data burst.
    pub(crate) fn encode(&self) -> [u32; 8] {
        match self {
            Self::Ephemeris1(subframe) => subframe.encode(),
            Self::Ephemeris2(subframe) => subframe.encode(),
            Self::Ephemeris3(subframe) => subframe.encode(),
        }
    }
}

// /// Verifies 24-bit LSB (right aligned) parity
// pub(crate) fn check_parity(value: u32) -> bool {
//     let data = value >> 6;
//     let expected = parity_encoding(data);
//     let parity = (value & 0x3f) as u8;
//
//     if expected == parity {
//         true
//     } else {
//         #[cfg(feature = "log")]
//         error!(
//             "GPS: parity error - expected 0x{:02X} - got 0x{:02X}",
//             expected, parity
//         );
//
//         false
//     }
// }
//
// /// Encodes 24-bit LSB into 6 bit parity (right aligned!)
// pub(crate) fn parity_encoding(value: u32) -> u8 {
//     let generator = 0x61_u32;
//     let mut reg = value << 6;
//
//     for _ in 0..24 {
//         if reg & (1 << 29) != 0 {
//             reg ^= generator << 23;
//         }
//         reg <<= 1;
//     }
//
//     ((reg >> 30) & 0x3f) as u8
// }

#[cfg(test)]
mod test {
    use crate::gps::{GpsDataByte, GpsQzssFrameId, GpsQzssSubframe};

    #[test]
    fn eph1_reciprocal() {
        let subframe = GpsQzssSubframe::decode(
            GpsQzssFrameId::Ephemeris1,
            &[
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::MsbPadded(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
                GpsDataByte::Byte(0x00),
            ],
        );

        let encoded = subframe.encode();
    }
}
