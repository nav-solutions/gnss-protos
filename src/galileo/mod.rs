//! Galileo I/NAV protocol (E1-B and E5b-I), following the
//! Galileo OS SIS ICD v2.1.
//!
//! The protocol is organized in two levels, from the antenna to the data:
//!
//! - [GalileoInavPage]: the only place that deals with transmitted symbols. Its
//!   [Message] implementation (and the [bitbuffer::BitRead] / [bitbuffer::BitWrite] traits)
//!   encodes and decodes the 500 transmitted symbols: synchronization pattern,
//!   convolutional encoding (FEC) and Viterbi decoding, block interleaving, CRC-24Q,
//!   inverted streams (180° carrier phase ambiguity). Two page parts (even, then odd)
//!   form a page, that carries one 128-bit [GalileoInavWord] and the OSNMA, SAR and SSP fields.
//! - [GalileoInavWord]: the navigation data, interpreted according to the word type
//!   ([GalileoInavWord1] to [GalileoInavWord4] for the ephemeris, [GalileoInavWord5]
//!   for the ionosphere, BGD and health, ...).
//!
//! The [GalileoInavDecoder] synchronizes itself on a raw symbol stream.
//!
//! All messages implement the [Message] trait: they may be encoded / decoded
//! from / to bytes, or written / read at any symbol position of a [bitbuffer] stream,
//! consecutive pages forming a valid I/NAV symbol stream.

#[cfg(doc)]
use crate::Message;

/// Number of bits of an I/NAV word.
pub const GALILEO_INAV_WORD_BITS: usize = 128;

/// Number of bytes of an I/NAV word.
pub const GALILEO_INAV_WORD_BYTES: usize = GALILEO_INAV_WORD_BITS / 8;

/// I/NAV symbol rate on E1-B and E5b-I (symbols per second).
pub const GALILEO_INAV_SYMBOL_RATE: usize = 250;

/// Number of symbols in the I/NAV synchronization pattern.
pub const GALILEO_INAV_SYNC_SYMBOLS: usize = 10;

/// I/NAV synchronization pattern (`0101100000`), right aligned.
pub const GALILEO_INAV_SYNC_PATTERN: u32 = 0b01_0110_0000;

/// Total number of symbols of a page part, as transmitted (sync pattern included):
/// 1 second at 250 symbols per second.
pub const GALILEO_INAV_PART_SYMBOLS: usize = GALILEO_INAV_SYNC_SYMBOLS + 240;

mod fec;

mod errors;
pub use errors::GalileoError;

mod health;
pub use health::{GalileoSignalHealth, GalileoSisa};

mod word;
pub use word::GalileoInavWord;

mod word0;
pub use word0::{GalileoInavWord0, GALILEO_INAV_WORD0_SPARE_BYTES};

mod word1;
pub use word1::GalileoInavWord1;

mod word2;
pub use word2::GalileoInavWord2;

mod word3;
pub use word3::GalileoInavWord3;

mod word4;
pub use word4::GalileoInavWord4;

mod word5;
pub use word5::{GalileoInavWord5, GALILEO_IONO_REGIONS};

mod word6;
pub use word6::GalileoInavWord6;

pub(crate) mod page;
pub use page::{
    GalileoInavPage, GalileoInavPageType, GALILEO_INAV_PAGE_BYTES, GALILEO_INAV_PAGE_SYMBOLS,
};

mod decoder;
pub use decoder::{GalileoInavDecoder, GALILEO_INAV_DEFAULT_MAX_SYMBOL_ERRORS};

#[cfg(test)]
pub(crate) mod test_vectors;

#[cfg(test)]
mod test {
    use crate::galileo::{
        fec::GALILEO_INAV_PART_ENCODED_SYMBOLS, GALILEO_INAV_PAGE_BYTES, GALILEO_INAV_PAGE_SYMBOLS,
        GALILEO_INAV_PART_SYMBOLS, GALILEO_INAV_SYMBOL_RATE, GALILEO_INAV_SYNC_SYMBOLS,
    };

    #[test]
    fn galileo_properties() {
        assert_eq!(GALILEO_INAV_PART_SYMBOLS, 250);
        assert_eq!(
            GALILEO_INAV_PART_SYMBOLS,
            GALILEO_INAV_SYNC_SYMBOLS + GALILEO_INAV_PART_ENCODED_SYMBOLS
        );
        assert_eq!(GALILEO_INAV_PAGE_SYMBOLS, 500);
        assert_eq!(GALILEO_INAV_PAGE_BYTES, 63);
        // a page lasts 2 seconds
        assert_eq!(GALILEO_INAV_PAGE_SYMBOLS / GALILEO_INAV_SYMBOL_RATE, 2);
    }
}
