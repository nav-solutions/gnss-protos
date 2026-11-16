//! Galileo I/NAV protocol (E1-B and E5b-I), following the
//! Galileo OS SIS ICD v2.1.
//!
//! The protocol is organized in three layers, from the antenna to the data:
//!
//! - [link]: the only place that deals with transmitted symbols: synchronization
//!   pattern, convolutional encoding (FEC) and Viterbi decoding, block interleaving,
//!   CRC-24Q. It converts 250 symbols into a 120-bit page part, and back.
//! - [GalileoInavPage]: two page parts (even, then odd) form a page, that carries one 128-bit
//!   [GalileoInavWord] and the OSNMA, SAR and SSP fields. The CRC is verified here.
//! - [GalileoInavWord]: the navigation data, interpreted according to the word type
//!   ([GalileoInavWord1] to [GalileoInavWord4] for the ephemeris, [GalileoInavWord5]
//!   for the ionosphere, BGD and health, ...).
//!
//! The [GalileoInavDecoder] synchronizes itself on a raw symbol stream.

/// Number of bits of an I/NAV word.
pub const GALILEO_INAV_WORD_BITS: usize = 128;

/// Number of bytes of an I/NAV word.
pub const GALILEO_INAV_WORD_BYTES: usize = GALILEO_INAV_WORD_BITS / 8;

/// I/NAV symbol rate on E1-B and E5b-I (symbols per second).
pub const GALILEO_INAV_SYMBOL_RATE: usize = 250;

pub mod link;

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

mod page;
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
        link::GALILEO_INAV_PART_SYMBOLS, GALILEO_INAV_PAGE_BYTES, GALILEO_INAV_PAGE_SYMBOLS,
        GALILEO_INAV_SYMBOL_RATE,
    };

    #[test]
    fn galileo_properties() {
        assert_eq!(GALILEO_INAV_PART_SYMBOLS, 250);
        assert_eq!(GALILEO_INAV_PAGE_SYMBOLS, 500);
        assert_eq!(GALILEO_INAV_PAGE_BYTES, 63);
        // a page lasts 2 seconds
        assert_eq!(GALILEO_INAV_PAGE_SYMBOLS / GALILEO_INAV_SYMBOL_RATE, 2);
    }
}
