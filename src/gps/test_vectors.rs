//! Real GPS subframes (EPH-1, 2, 3) as reported by a u-blox receiver
//! (UBX-RXM-SFRBX), and test-only conversion helpers.
//!
//! u-blox receivers do not report the words as transmitted: each 30-bit word
//! (data *and* parity) is XOR'ed with the D30* bit of the previous word, so that
//! the data bits come out non-inverted ("polarity corrected" words).

use crate::{
    bits::{read_bits, write_bits},
    gps::{GPS_FRAME_BYTES, GPS_WORDS_PER_FRAME, GPS_WORD_BITS},
};

/// 30-bit mask.
const WORD_MASK: u32 = 0x3fff_ffff;

/// EPH-1, 30-bit words, right aligned.
pub(crate) const UBLOX_EPH1: [u32; 10] = [
    0x22C13E1B, 0x1527C973, 0x13E40004, 0x104F5D31, 0x1744E6D7, 0x07755783, 0x330C80B5, 0x125042A1,
    0x00001684, 0x312C3033,
];

/// EPH-2, 30-bit words, right aligned.
pub(crate) const UBLOX_EPH2: [u32; 10] = [
    0x22C13E1B, 0x1527EA1B, 0x127FF165, 0x0C681F7C, 0x02493415, 0x3FF8811E, 0x191B8114, 0x043E686E,
    0x03347221, 0x10429F7B,
];

/// EPH-3, 30-bit words, right aligned.
pub(crate) const UBLOX_EPH3: [u32; 10] = [
    0x22C13E1B, 0x15280BDB, 0x000AEA34, 0x033CFFEE, 0x3FE5C9EB, 0x136FB64E, 0x06F4AB2C, 0x0671EB44,
    0x3FEAF602, 0x12455213,
];

/// Converts polarity corrected words (u-blox) to the transmitted words.
pub(crate) fn from_polarity_corrected(
    words: &[u32; GPS_WORDS_PER_FRAME],
) -> [u32; GPS_WORDS_PER_FRAME] {
    let mut ret = [0u32; GPS_WORDS_PER_FRAME];
    let mut prev = 0u32;

    for (i, word) in words.iter().enumerate() {
        ret[i] = if prev & 0x1 > 0 {
            !word & WORD_MASK
        } else {
            word & WORD_MASK
        };

        prev = ret[i];
    }

    ret
}

/// Mirror operation of [from_polarity_corrected].
pub(crate) fn to_polarity_corrected(
    words: &[u32; GPS_WORDS_PER_FRAME],
) -> [u32; GPS_WORDS_PER_FRAME] {
    let mut ret = [0u32; GPS_WORDS_PER_FRAME];
    let mut prev = 0u32;

    for (i, word) in words.iter().enumerate() {
        let word = word & WORD_MASK;

        ret[i] = if prev & 0x1 > 0 {
            !word & WORD_MASK
        } else {
            word
        };

        prev = word;
    }

    ret
}

/// u-blox words, converted back to the transmitted words.
pub(crate) fn on_air(words: [u32; GPS_WORDS_PER_FRAME]) -> [u32; GPS_WORDS_PER_FRAME] {
    from_polarity_corrected(&words)
}

/// Packs 10 transmitted words (30-bit, right aligned) into the 38 bytes
/// that [crate::Message::decode] expects (4 null padding bits).
pub(crate) fn words_to_bytes(words: &[u32; GPS_WORDS_PER_FRAME]) -> [u8; GPS_FRAME_BYTES] {
    let mut bytes = [0u8; GPS_FRAME_BYTES];

    for (i, word) in words.iter().enumerate() {
        write_bits(&mut bytes, i * GPS_WORD_BITS, *word, GPS_WORD_BITS).unwrap();
    }

    bytes
}

/// Mirror operation of [words_to_bytes].
pub(crate) fn bytes_to_words(bytes: &[u8]) -> [u32; GPS_WORDS_PER_FRAME] {
    let mut words = [0u32; GPS_WORDS_PER_FRAME];

    for (i, word) in words.iter_mut().enumerate() {
        *word = read_bits(bytes, i * GPS_WORD_BITS, GPS_WORD_BITS).unwrap();
    }

    words
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn polarity_corrected_reciprocal() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            assert_eq!(
                to_polarity_corrected(&from_polarity_corrected(&frame)),
                frame
            );
        }
    }

    #[test]
    fn words_bytes_reciprocal() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            let bytes = words_to_bytes(&frame);
            assert_eq!(bytes[GPS_FRAME_BYTES - 1] & 0x0f, 0);
            assert_eq!(bytes_to_words(&bytes), frame);
        }
    }
}
