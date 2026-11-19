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

/// Number of [GpsQzssFrame]s (subframes) in a frame: EPH-1,2,3 then ALM-4,5.
/// A frame lasts 30 s.
pub const GPS_QZSS_ROTATION_ITEMS: usize = 5;

/// Number of pages of subframes 4 and 5: the complete almanac is transmitted
/// over 25 frames (12'30").
pub const GPS_QZSS_ALMANACH_PAGES: usize = 25;

/// Number of pages (frames of 5 subframes) in a GPS/QZSS message: the 12'30" rotation,
/// which contains the complete almanac for the entire constellation.
pub const GPS_QZSS_MESSAGE_ITEMS: usize = GPS_QZSS_ALMANACH_PAGES;

/// Number of [GpsQzssFrame]s (subframes) in a GPS/QZSS message (12'30").
pub const GPS_QZSS_MESSAGE_FRAMES: usize = GPS_QZSS_MESSAGE_ITEMS * GPS_QZSS_ROTATION_ITEMS;

/// Total number of bits in a complete GPS/QZSS message (125 contiguous subframes).
pub const GPS_QZSS_MESSAGE_BITS: usize = GPS_QZSS_MESSAGE_FRAMES * GPS_FRAME_BITS;

/// Total size (in bytes) that must be allocated to fit a complete GPS/QZSS message
/// (125 contiguous subframes), including the almanac of the entire constellation.
/// This is usually not enough for a true application that may require double/triple
/// buffering, but at least this defines your basis.
pub const GPS_QZSS_MESSAGE_BYTES: usize = (GPS_QZSS_MESSAGE_BITS + 7) / 8;

/// Duration (in seconds) of a complete GPS/QZSS message (12'30").
pub const GPS_QZSS_MESSAGE_DURATION_S: u32 = GPS_QZSS_MESSAGE_ITEMS as u32 * GPS_FRAME_DURATION_S;

/// GPS week duration (in seconds)
pub const GPS_WEEK_SECONDS: u32 = 604_800;

/// Data ID of the GPS / QZSS LNAV message (`01`), transmitted in subframes 4 and 5.
pub const GPS_QZSS_DATA_ID: u8 = 1;

mod almanach;
mod decoder;
mod errors;
mod frame;
mod frame1;
mod frame2;
mod frame3;
mod frame4;
mod frame5;
mod frame_id;
mod health;
mod how;
mod message;
mod reserved;
mod subframe;
mod tlm;

pub use almanach::GpsQzssAlmanach;
pub use decoder::GpsQzssDecoder;
pub use errors::GpsError;
pub use frame::GpsQzssFrame;
pub use frame1::GpsQzssFrame1;
pub use frame2::GpsQzssFrame2;
pub use frame3::GpsQzssFrame3;
pub use frame4::{
    GpsQzssConfigHealth, GpsQzssFrame4, GpsQzssIonosphereUtc, GpsQzssNmct, GpsQzssSpecialMessage,
    GPS_CONFIG_HEALTH_SATELLITES, GPS_CONFIG_SATELLITES, GPS_FRAME4_PAGE_SV_IDS,
    GPS_NMCT_ERD_COUNT, GPS_NMCT_ERD_NOT_AVAILABLE, GPS_SPECIAL_MESSAGE_CHARS,
};
pub use frame5::{GpsQzssAlmanachHealth, GpsQzssFrame5, GPS_ALMANACH_HEALTH_SATELLITES};
pub use frame_id::GpsQzssFrameId;
pub use health::GpsQzssSatelliteHealth;
pub use how::GpsQzssHow;
pub use message::{GpsQzssMessage, GpsQzssMessageBuilder, GpsQzssMessagePage};
pub use reserved::{GpsQzssReservedPage, GPS_PAGE_DATA_BYTES};
pub use subframe::{GpsQzssSubframe, GPS_SUBFRAME_DATA_BYTES};
pub use tlm::GpsQzssTelemetry;

/// Returns the SV (page) ID of the subframe 4 or 5 page that starts
/// at the current position, without consuming the stream.
pub(crate) fn peek_sv_id(
    stream: &mut crate::BitReadStream<'_, crate::BigEndian>,
) -> Result<u8, crate::BitError> {
    let start = stream.pos();
    let _data_id = stream.read_int::<u8>(2)?;
    let sv_id = stream.read_int::<u8>(6)?;
    stream.set_pos(start)?;
    Ok(sv_id)
}

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

    #[test]
    fn message_properties() {
        use crate::gps::{
            GPS_QZSS_MESSAGE_BITS, GPS_QZSS_MESSAGE_BYTES, GPS_QZSS_MESSAGE_DURATION_S,
            GPS_QZSS_MESSAGE_FRAMES, GPS_QZSS_MESSAGE_ITEMS, GPS_QZSS_ROTATION_ITEMS,
            GPS_SUBFRAME_DURATION_S, GPS_WEEK_SECONDS,
        };

        assert_eq!(GPS_QZSS_ROTATION_ITEMS, 5);
        assert_eq!(GPS_QZSS_MESSAGE_ITEMS, 25);
        assert_eq!(GPS_QZSS_MESSAGE_FRAMES, 125);
        assert_eq!(GPS_QZSS_MESSAGE_BITS, 37_500);
        assert_eq!(GPS_QZSS_MESSAGE_BYTES, 4688);

        // 12'30"
        assert_eq!(GPS_QZSS_MESSAGE_DURATION_S, 750);
        assert_eq!(
            GPS_QZSS_MESSAGE_FRAMES as u32 * GPS_SUBFRAME_DURATION_S,
            GPS_QZSS_MESSAGE_DURATION_S
        );

        // a week is not a whole number of messages:
        // the last rotation of the week is interrupted
        assert_eq!(GPS_WEEK_SECONDS % GPS_QZSS_MESSAGE_DURATION_S, 300);
    }
}
