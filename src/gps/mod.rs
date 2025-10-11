/// GPS preamble (SYNC) byte
pub const GPS_PREAMBLE_BYTE: u8 = 0x8B;

/// GPS data word size (in bits!)
pub const GPS_WORD_BITS: usize = 30;

/// Number of words in a frame
pub const GPS_WORDS_PER_FRAME: usize = 10;

/// Total GPS/QZSS frame size (in bits!)
pub const GPS_FRAME_BITS: usize = GPS_WORDS_PER_FRAME * GPS_WORD_BITS;

/// Total GPS/QZSS frame size (in bytes!)
pub const GPS_FRAME_BYTES: usize = (GPS_FRAME_BITS / 8) + 1;

/// Parity bit mask (for each [GpsDataWord])
pub(crate) const GPS_PARITY_MASK: u32 = 0x0000_003f;

/// Payload bit mask
pub(crate) const GPS_PAYLOAD_MASK: u32 = 0xffff_ffc0;

/// Number of parity bits for each [GpsDataWord]
pub(crate) const GPS_PARITY_SIZE: usize = 6;

// /// L1 C/A code length
// pub const GPS_L1_CA_CODE_LEN: usize = 1023;

pub(crate) fn rad_to_semicircles(rad: f64) -> f64 {
    rad.to_degrees() * (2.0_f64.powi(31) / 90.0)
}

mod bytes;
pub use bytes::GpsDataByte;

mod word;
pub use word::GpsDataWord;

// mod cdma;
// pub use cdma::GpsQzssModulator;

// mod almanach;
// pub use almanach::GpsQzssAlmanach;

mod decoder;
pub use decoder::GpsQzssDecoder;

mod decoding;

mod errors;
pub use errors::GpsError;

mod frame1;
pub use frame1::GpsQzssFrame1;

mod frame2;
pub use frame2::GpsQzssFrame2;

mod frame3;
pub use frame3::GpsQzssFrame3;

// mod frame4;
// pub use frame4::GpsQzssFrame4;

// mod frame5;
// pub use frame5::GpsQzssFrame5;

mod frame_id;
pub use frame_id::GpsQzssFrameId;

mod how;
pub use how::GpsQzssHow;

mod tlm;
pub use tlm::GpsQzssTelemetry;

mod frame;
pub use frame::GpsQzssFrame;

mod subframe;
pub use subframe::GpsQzssSubframe;

#[cfg(test)]
mod test {
    use crate::gps::{GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_WORDS_PER_FRAME};

    #[test]
    fn gps_properties() {
        assert_eq!(GPS_FRAME_BYTES, 38);
        assert_eq!(GPS_FRAME_BITS, 300);
        assert_eq!(GPS_WORDS_PER_FRAME, 10);
    }
}
