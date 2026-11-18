//! GPS / QZSS LNAV protocol (L1 C/A), following IS-GPS-200.
//!
//! A GPS subframe ([GpsQzssFrame]) is 300 bits long (6 seconds at 50 bits per second):
//! 10 words of 30 bits, each word carrying 24 data bits followed by 6 parity bits.
//! The protocol is organized in two levels:
//!
//! - [GpsQzssFrame]: the only place that deals with the transmitted bits. Its
//!   [Message] implementation (and the [bitbuffer::BitRead] / [bitbuffer::BitWrite] traits)
//!   encodes and decodes the 300 transmitted bits: parity (IS-GPS-200 §20.3.5.2),
//!   data inversion (D30*), bits 23-24 of words 2 and 10, inverted streams
//!   (180° carrier phase ambiguity).
//! - the messages ([GpsQzssTelemetry], [GpsQzssHow], [GpsQzssFrame1] to [GpsQzssFrame3])
//!   are read and written on the 240 parity free data bits. Parameters that the ICD splits over
//!   two words (M0, e, sqrt(A)...) are contiguous there.
//!
//! We also provide [GpsQzssMessage] which describes the complete 25-page rotation,
//! which is most useful to people interested in the constellation Almanach. The 25-page
//! rotation is recorded in 12'30" and describes gives the entire almanach.
//!
//! The [GpsQzssDecoder] synchronizes itself on a raw bit stream.
//!
//! All messages implement the [Message] trait: they may be encoded / decoded
//! from / to bytes, or written / read at any bit position of a [bitbuffer] stream,
//! consecutive frames forming a valid GPS bit stream.

#[cfg(doc)]
use crate::Message;

/// GPS preamble (SYNC) byte
pub const GPS_PREAMBLE_BYTE: u8 = 0x8B;

/// Inverted GPS preamble, as observed when the carrier phase is locked
/// with a 180° ambiguity: the whole bit stream is then inverted.
pub const GPS_INVERTED_PREAMBLE_BYTE: u8 = !GPS_PREAMBLE_BYTE;

/// GPS data word size (in bits), parity included
pub const GPS_WORD_BITS: usize = 30;

/// GPS data word size (in bytes)
pub const GPS_WORD_BYTES: usize = 4;

/// Number of data bits (parity excluded) per GPS word.
pub const GPS_WORD_DATA_BITS: usize = 24;

/// Number of words in a frame
pub const GPS_WORDS_PER_FRAME: usize = 10;

/// Total GPS/QZSS frame size (in bits!)
pub const GPS_FRAME_BITS: usize = GPS_WORDS_PER_FRAME * GPS_WORD_BITS;

/// Total GPS/QZSS frame size (in bytes!)
pub const GPS_FRAME_BYTES: usize = (GPS_FRAME_BITS / 8) + 1;

/// GPS Week number wraps at 1024
pub const GPS_WEEK_ROLLOVER: u32 = 1024;

/// A GPS/QZSS frame (5 subframes) lasts 30s. Frames are
/// aligned to the week start.
pub const GPS_FRAME_DURATION_S: u32 = 30;

/// A GPS/QZSS subframe lasts 6. We have 5 subframes per frame.
pub const GPS_SUBFRAME_DURATION_S: u32 = 6;

/// SF1.toc and SF2.toe are expressed with a 16s LSB
pub const GPS_TOC_TOE_LSB_S: u32 = 16;

/// Number of data (parity free) bytes per GPS frame.
pub(crate) const GPS_FRAME_DATA_BYTES: usize = GPS_WORDS_PER_FRAME * GPS_WORD_DATA_BITS / 8;

/// Number of [GpsQzssFrame]s in a rotation: EPH-1,2,3 then ALM-4,5.
pub const GPS_QZSS_ROTATION_ITEMS: usize = 5;

/// Number of "pages" or complete EPH-1,2,3 ALM-4,5 rotation in a GPS/QZSS message.
/// Which contains the complete Almanach information for the entire constellation.
pub const GPS_QZSS_MESSAGE_ITEMS: usize = 25;

/// Total size (in bytes) that must be allocated to fit a complete 25-page GPS message,
/// including Almanach for the entire constellation. This is usually not enough for a true
/// application that may require double/tripple buffering, but at least this defines your basis.
pub const GPS_QZSS_MESSAGE_BYTES: usize = GPS_QZSS_MESSAGE_ITEMS * GPS_FRAME_BYTES;

/// Total number of bits in a 25-page GPS message.
pub const GPS_QZSS_MESSAGE_BITS: usize = GPS_QZSS_MESSAGE_ITEMS * GPS_FRAME_BITS;

mod almanach;
mod decoder;
mod errors;
mod frame;
mod frame1;
mod frame2;
mod frame3;
mod frame_id;
mod health;
mod how;
mod message;
mod subframe;
mod tlm;

pub use almanach::GpsQzssAlmanach;
pub use decoder::GpsQzssDecoder;
pub use errors::GpsError;
pub use frame::GpsQzssFrame;
pub use frame1::GpsQzssFrame1;
pub use frame2::GpsQzssFrame2;
pub use frame3::GpsQzssFrame3;
pub use frame_id::GpsQzssFrameId;
pub use message::GpsQzssMessage;
pub use health::GpsQzssSatelliteHealth;
pub use how::GpsQzssHow;
pub use subframe::{GpsQzssSubframe, GPS_SUBFRAME_DATA_BYTES};
pub use tlm::GpsQzssTelemetry;

#[cfg(test)]
pub(crate) mod test_vectors;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod test {
    use crate::gps::{
        GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_FRAME_DATA_BYTES, GPS_INVERTED_PREAMBLE_BYTE,
        GPS_SUBFRAME_DATA_BYTES, GPS_WORDS_PER_FRAME,
    };

    #[test]
    fn gps_properties() {
        assert_eq!(GPS_FRAME_BYTES, 38);
        assert_eq!(GPS_FRAME_BITS, 300);
        assert_eq!(GPS_WORDS_PER_FRAME, 10);
        assert_eq!(GPS_FRAME_DATA_BYTES, 30);
        assert_eq!(GPS_SUBFRAME_DATA_BYTES, 24);
        assert_eq!(GPS_INVERTED_PREAMBLE_BYTE, 0x74);
    }
}
