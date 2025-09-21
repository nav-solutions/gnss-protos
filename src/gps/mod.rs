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

/// Total number of bits in a subframe
pub(crate) const GPS_SUBFRAME_BITS: usize = (GPS_WORDS_PER_FRAME - 2) * GPS_WORD_BITS;

/// Parity bit mask (for each [GpsDataWord])
pub(crate) const GPS_PARITY_MASK: u32 = 0x0000_003f;

/// Number of parity bits for each [GpsDataWord]
pub(crate) const GPS_PARITY_SIZE: usize = 6;

mod bytes;
pub use bytes::GpsDataByte;

mod word;
pub use word::GpsDataWord;

// mod cdma;

// mod decoder;
// pub use decoder::GpsQzssDecoder;

mod decoding;
mod encoding;

mod errors;
pub use errors::GpsError;

mod frame1;
pub use frame1::GpsQzssFrame1;

// mod frame2;
// pub use frame2::GpsQzssFrame2;

// mod frame3;
// pub use frame3::GpsQzssFrame3;

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
