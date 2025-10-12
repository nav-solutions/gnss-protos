use std::{
    // fs::File,
    // io::Read,
    sync::Once,
};

use log::LevelFilter;

// use crate::gps::{
// GpsQzssFrameId, // GpsQzssHow,
// GpsQzssTelemetry,
// GPS_WORDS_PER_FRAME,
// };

mod gps;

static INIT: Once = Once::new();

pub fn init_logger() {
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Trace)
            .init();
    });
}

/// Simple method to insert the desired number of zero (bitwise)
/// in a stream, at the begginning of the stream, simply "delaying" the following values.
pub fn insert_zeros(slice: &[u8], num_zero_bits: usize) -> Vec<u8> {
    let size = slice.len();

    let extra_bit = num_zero_bits % 8;

    let mut extra_bytes = num_zero_bits / 8;

    if extra_bit > 0 {
        extra_bytes += 1;
    }

    println!("extra bytes: {}", extra_bytes);

    let mut ret = vec![0u8; size + extra_bytes];

    // first copy as is
    ret[..size].copy_from_slice(slice);

    if extra_bit > 0 {
        let shift = 8 - extra_bit;
        let mut mask = 0x00;

        for i in 0..extra_bit {
            mask |= 1 << (8 - i - 1);
        }

        println!("MASK=0x{:02X}", mask);

        for i in 0..size - 1 {
            ret[size - i - 1] >>= extra_bit;
            ret[size - i - 1] |= ret[size - i - 2] << (8 - extra_bit)
        }

        ret[0] >>= extra_bit;
    }

    ret
}

#[test]
fn test_zeros_padder() {
    let test_values = [0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];

    for zeros in 0..=1 {
        let delayed = insert_zeros(&test_values, zeros);

        let expected = match zeros {
            0 => vec![0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA],
            1 => vec![0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00],
            _ => panic!("untested value"),
        };

        assert_eq!(
            delayed, expected,
            "wrong results for {} inserted zeros",
            zeros
        );
    }

    let test_values = [0x55, 0x55, 0x55, 0x55];

    for zeros in 0..=1 {
        let delayed = insert_zeros(&test_values, zeros);

        let expected = match zeros {
            0 => vec![0x55, 0x55, 0x55, 0x55],
            1 => vec![0x2A, 0xAA, 0xAA, 0xAA, 0x80],
            _ => panic!("untested value"),
        };

        assert_eq!(
            delayed, expected,
            "wrong results for {} inserted zeros",
            zeros
        );
    }
}
