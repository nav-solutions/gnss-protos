//! GPS / QZSS scenario tests: realistic frames, bit-exact reciprocity
//! through the [crate::Message] trait, bitbuffer streams and the [crate::gps::GpsQzssDecoder].

use crate::gps::{test_vectors::bytes_to_words, GpsQzssSubframe};

// mod almanach;
mod eph1;
mod eph2;
mod eph3;

/// Verifies that `encoded` starts with a valid 300-bit GPS subframe:
/// parity of each word, and words 2 & 10 ending with D29 = D30 = 0.
pub(crate) fn assert_valid_subframe(encoded: &[u8]) {
    let words = bytes_to_words(encoded);
    assert!(
        GpsQzssSubframe::from_words(&words, true).is_ok(),
        "invalid parity"
    );
    assert_eq!(words[1] & 0x3, 0, "HOW must end with D29=D30=0");
    assert_eq!(words[9] & 0x3, 0, "word 10 must end with D29=D30=0");
}
