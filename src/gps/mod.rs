//! GPS / QZSS LNAV protocol (IS-GPS-200, IS-QZSS-PNT).
//!
//! - [link]: parity and polarity, converts 300 transmitted bits into 240 data bits, and back.
//! - [GpsQzssFrame]: telemetry, hand-over word and subframe, read and written on the data bits.
//! - [GpsQzssDecoder]: synchronizes itself on a bit stream.
//! - [Modulator]: spreads the encoded bit stream with the L1 C/A code of one satellite.

/// GPS preamble (SYNC) byte
pub const GPS_PREAMBLE_BYTE: u8 = 0x8B;

/// GPS data word size (in bits)
pub const GPS_WORD_BITS: usize = 30;

/// GPS data word size (in bytes)
pub const GPS_WORD_BYTES: usize = 4;

/// Number of words in a frame
pub const GPS_WORDS_PER_FRAME: usize = 10;

/// Total GPS/QZSS frame size (in bits!)
pub const GPS_FRAME_BITS: usize = GPS_WORDS_PER_FRAME * GPS_WORD_BITS;

// /// Total GPS/QZSS subframe size (in bits!)
// pub(crate) const GPS_SUBFRAME_BITS: usize = (GPS_WORDS_PER_FRAME - 2) * GPS_WORD_BITS;

/// Total GPS/QZSS frame size (in bytes!)
pub const GPS_FRAME_BYTES: usize = (GPS_FRAME_BITS / 8) + 1;

// /// Parity bit mask (for each [GpsDataWord])
// pub(crate) const GPS_PARITY_MASK: u32 = 0x0000_003f;

// /// Number of parity bits for each [GpsDataWord]
// pub(crate) const GPS_PARITY_SIZE: usize = 6;

pub(crate) use crate::bits::{
    from_scaled, rad_to_semicircles, to_scaled, write_signed, write_unsigned,
};

mod modulator;
pub use modulator::{
    Modulator, GPS_DATA_RATE, GPS_L1_CA_CHIPS_PER_BIT, GPS_L1_CA_CHIP_RATE,
    GPS_L1_CA_CODES_PER_BIT, GPS_L1_CA_CODE_LEN,
};

mod almanach;
pub use almanach::GpsQzssAlmanach;

mod decoder;
pub use decoder::GpsQzssDecoder;

mod errors;
pub use errors::GpsError;

pub mod link;

mod health;
pub use health::GpsQzssSatelliteHealth;

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
