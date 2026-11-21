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

/// GPS preamble (synchronization) byte
pub const GPS_PREAMBLE_BYTE: u8 = 0x8B;

/// Inverted GPS preamble, as observed when the carrier phase is locked
/// with a 180° ambiguity: the whole bit stream is then inverted.
pub const GPS_INVERTED_PREAMBLE_BYTE: u8 = !GPS_PREAMBLE_BYTE;

/// GPS/QZSS week counters are encoded on 10bits, which rolls over
/// every 19.5 years.
pub const GPS_WEEK_ROLLOVER: u32 = 1024;

/// SF1.toc and SF2.toe are expressed with a 16s LSB
pub const GPS_TOC_TOE_LSB_S: u32 = 16;

/// GPS data word size (in bits), parity included
pub const GPS_WORD_BITS: usize = 30;

/// GPS/QZSS payload bits in a GPS/QZSS word: 6 parity bits are exlucded.
pub const GPS_WORD_DATA_BITS: usize = 24;

/// Total number of bytes needed to allocate a GPS/QZSS word: 30 bits rounded up to 4 bytes.
pub const GPS_WORD_BYTES: usize = 4;

/// 10 GPS/QZSS Words per subframe (300 bits)
pub const GPS_SUBFRAME_WORDS: usize = 10;

/// Total number of bits contained in a GPS/QZSS frame: 10 30-bit words
pub const GPS_SUBFRAME_BITS: usize = GPS_SUBFRAME_WORDS * GPS_WORD_BITS;

/// A GPS/QZSS subframe lasts 6 seconds.
pub const GPS_SUBFRAME_DURATION_S: u32 = 6;

/// Total number of bytes required to allocate a 300-bit GPS/QZSS frame: 38 bytes
/// with one byte half zero-padded.
pub const GPS_SUBFRAME_BYTES: usize = GPS_SUBFRAME_BITS / 8 + 1;

/// Total number of payload bytes (=parity free) in a subframe: Words3-10 included: 27 bytes
const GPS_SUBFRAME_DATA_BYTES: usize = GPS_WORD_BITS * 8 / 8;

/// Number of GPS/QZSS Subframes (EPH1, EPH2..) in a 30s frame.
pub const GPS_FRAME_ITEMS: usize = 5;

/// Total number of bits in a 30s frame
pub const GPS_FRAME_BITS: usize = GPS_FRAME_ITEMS * GPS_SUBFRAME_BITS;

/// Total number of bytes required to allocate a 30s page
pub const GPS_FRAME_BYTES: usize = GPS_FRAME_BITS / 8 + 1;

/// A GPS/QZSS frame lasts 30 seconds.
pub const GPS_FRAME_DURATION_S: u32 = GPS_FRAME_ITEMS as u32 * GPS_SUBFRAME_DURATION_S;

/// Number of GPS/QZSS 30s frames in a complete constellation Almanach, which we refer
/// to as "Super Frame".
pub const GPS_SUPERFRAME_ITEMS: usize = 25;

/// Total number of bits composing a complete Almanach: 12'30s broadcast: 37_500 bits.
pub const GPS_SUPERFRAME_BITS: usize = GPS_SUPERFRAME_ITEMS * GPS_FRAME_BITS;

/// Total number of bytes required to allocate a complete almanach, 12'30s of GPS/QZSS broadcast,
/// which means 25x 30s cycles: 4_688 bytes.
pub const GPS_SUPERFRAME_BYTES: usize = GPS_SUPERFRAME_BITS / 8 + 1;

/// Duration of a constellation Almanach in seconds: 25 * 30s = 750s = 12'30s.
pub const GPS_SUPERFRAME_DURATION_S: u32 = GPS_SUPERFRAME_ITEMS as u32 * GPS_FRAME_DURATION_S;

/// Data ID of the GPS / QZSS LNAV message (`01`), transmitted in subframes 4 and 5.
/// Used when decoding or composing subframes 4 and 5.
pub const GPS_LNAV_CONFIG_DID: u8 = 1;

mod almanach;
mod decoder;
mod errors;
mod frame;
mod health;
mod how;
mod subframe;
// mod superframe;
mod tlm;

// #[cfg(feature = "hifitime")]
// mod time;
//
pub use almanach::GpsQzssAlmanach;
pub use decoder::GpsQzssDecoder;
pub use errors::GpsError;
pub use frame::*;
pub use health::GpsQzssSatelliteHealth;
pub use how::GpsQzssHow;
pub use subframe::*;
// pub use superframe::*;
pub use tlm::GpsQzssTelemetry;

// #[cfg(feature = "hifitime")]
// use time::GpsQzssLnavTemporalPosition;

#[cfg(test)]
pub(crate) mod test_vectors;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod test {
    use crate::gps::{
        GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_FRAME_DURATION_S, GPS_FRAME_ITEMS,
        GPS_INVERTED_PREAMBLE_BYTE, GPS_SUBFRAME_BITS, GPS_SUBFRAME_BYTES, GPS_SUBFRAME_DATA_BYTES,
        GPS_SUBFRAME_DURATION_S, GPS_SUBFRAME_WORDS, GPS_SUPERFRAME_BITS, GPS_SUPERFRAME_BYTES,
        GPS_SUPERFRAME_DURATION_S, GPS_SUPERFRAME_ITEMS, GPS_WEEK_ROLLOVER, GPS_WORD_BITS,
        GPS_WORD_BYTES,
    };

    #[test]
    fn gps_binary_constants() {
        assert_eq!(GPS_WEEK_ROLLOVER, 1024);
        assert_eq!(GPS_INVERTED_PREAMBLE_BYTE, 0x74);
    }

    #[test]
    fn gps_protos_constant() {
        assert_eq!(GPS_WORD_BITS, 30);
        assert_eq!(GPS_WORD_BYTES, 4);

        assert_eq!(GPS_SUBFRAME_WORDS, 10);
        assert_eq!(GPS_SUBFRAME_BITS, 300);
        assert_eq!(GPS_SUBFRAME_BYTES, 38);
        assert_eq!(GPS_SUBFRAME_DATA_BYTES, 30);
        assert_eq!(GPS_SUBFRAME_DURATION_S, 6);

        assert_eq!(GPS_FRAME_ITEMS, 5);
        assert_eq!(GPS_FRAME_BITS, 1500);
        assert_eq!(GPS_FRAME_BYTES, 188);
        assert_eq!(GPS_FRAME_DURATION_S, 30);

        assert_eq!(GPS_SUPERFRAME_ITEMS, 25);
        assert_eq!(GPS_SUPERFRAME_BITS, 37_500);
        assert_eq!(GPS_SUPERFRAME_BYTES, 4_688);
        assert_eq!(GPS_SUPERFRAME_DURATION_S, 750);
    }
}
