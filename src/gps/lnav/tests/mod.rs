//! GPS / QZSS scenario tests: realistic frames, bit-exact reciprocity
//! through the [crate::Message] trait, bitbuffer streams and the [crate::gps::GpsQzssDecoder].

use crate::gps::lnav::{
    test_vectors::bytes_to_words, GpsQzssLnavSubfDecoder, GpsQzssLnavSubframe,
    GPS_LNAV_SUBFRAME_BYTES,
};
use std::{fs::File, io::Read};

mod almanach;
mod eph1;
mod eph2;
mod eph3;
mod frame;
mod superframe;

/// Verifies that `encoded` starts with a valid 300-bit GPS subframe:
/// parity of each word, and words 2 & 10 ending with D29 = D30 = 0.
pub(crate) fn assert_valid_subframe(encoded: &[u8]) {
    let words = bytes_to_words(encoded);
    assert!(
        GpsQzssLnavSubframe::from_words(&words, true).is_ok(),
        "invalid parity"
    );
    assert_eq!(words[1] & 0x3, 0, "HOW must end with D29=D30=0");
    assert_eq!(words[9] & 0x3, 0, "word 10 must end with D29=D30=0");
}

/// Structure to parse GPS/QZSS subframe from any binary file
pub struct GpsQzssLnavSbfBinDecoder {
    file: File,
    decoder: GpsQzssLnavSubfDecoder,
    buffer: [u8; GPS_LNAV_SUBFRAME_BYTES],
}

impl GpsQzssLnavSbfBinDecoder {
    pub fn new(path: &str) -> Self {
        Self {
            buffer: [0; GPS_LNAV_SUBFRAME_BYTES],
            decoder: GpsQzssLnavSubfDecoder::default(),
            file: File::open(path).unwrap_or_else(|e| {
                panic!("failed to open file '{}': {}", path, e);
            }),
        }
    }

    pub fn parse_next(&mut self) -> Option<GpsQzssLnavSubframe> {
        match self.file.read_exact(&mut self.buffer) {
            Ok(_) => {
                let (_, subf) = self.decoder.decode(&self.buffer, 0);
                subf
            },
            Err(_) => None,
        }
    }
}
