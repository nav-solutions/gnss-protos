use std::{fs::File, io::Read, sync::Once};

use log::LevelFilter;

use crate::gps::{GpsDataWord, GpsQzssFrameId, GpsQzssHow, GpsQzssTelemetry, GPS_WORDS_PER_FRAME};

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

#[cfg(feature = "gps")]
pub fn from_ublox_bytes<const N: usize>(bytes: &[u8; N]) -> [GpsDataWord; GPS_WORDS_PER_FRAME] {
    let mut ret: [GpsDataWord; GPS_WORDS_PER_FRAME] = Default::default();

    let mut count = 0;
    let mut value = 0u32;

    for i in 0..N {
        match i % 4 {
            0 => value |= (bytes[i % 4 + count * 4] as u32) << 26,
            1 => value |= (bytes[i % 4 + count * 4] as u32) << 18,
            2 => value |= (bytes[i % 4 + count * 4] as u32) << 10,
            3 => {
                value |= (bytes[i % 4 + count * 4] as u32) << 2;
                ret[count] = GpsDataWord::from(value);
                count += 1;
                value = 0;

                if count == GPS_WORDS_PER_FRAME {
                    return ret;
                }
            },
            _ => unreachable!("compiler issue"),
        }
    }

    ret
}

#[test]
fn test_from_ublox_bytes() {
    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x27, 0xC9, 0x73, // WORD3
        0x13, 0xE4, 0x00, 0x04, //WORD4
        0x10, 0x4F, 0x5D, 0x31, //WORD5
        0x97, 0x44, 0xE6, 0xD7, // WORD6
        0x07, 0x75, 0x57, 0x83, //WORD7
        0x33, 0x0C, 0x80, 0xB5, // WORD8
        0x92, 0x50, 0x42, 0xA1, // WORD9
        0x80, 0x00, 0x16, 0x84, //WORD10
        0x31, 0x2C, 0x30, 0x33,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris1);

    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x27, 0xEA, 0x1B, // WORD3
        0x12, 0x7F, 0xF1, 0x65, // WORD4
        0x8C, 0x68, 0x1F, 0x7C, // WORD5
        0x02, 0x49, 0x34, 0x15, // WORD6
        0xBF, 0xF8, 0x81, 0x1E, // WORD7
        0x99, 0x1B, 0x81, 0x14, // W0RD8
        0x04, 0x3E, 0x68, 0x6E, // WORD9
        0x83, 0x34, 0x72, 0x21, // WORD10
        0x90, 0x42, 0x9F, 0x7B,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris2);

    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x28, 0x0B, 0xDB, // WORD3
        0x00, 0x0A, 0xEA, 0x34, // WORD4
        0x03, 0x3C, 0xFF, 0xEE, // WORD5
        0xBF, 0xE5, 0xC9, 0xEB, // WORD6
        0x13, 0x6F, 0xB6, 0x4E, // WORD7
        0x86, 0xF4, 0xAB, 0x2C, // WORD8
        0x06, 0x71, 0xEB, 0x44, // WORD9
        0x3F, 0xEA, 0xF6, 0x02, // WORD10
        0x92, 0x45, 0x52, 0x13,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris3);

    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x27, 0xC9, 0x73, // WORD3
        0x00, 0x0A, 0xEA, 0x34, // WORD4
        0x03, 0x3C, 0xFF, 0xEE, // WORD5
        0xBF, 0xE5, 0xC9, 0xEB, // WORD6
        0x13, 0x6F, 0xB6, 0x4E, // WORD7
        0x86, 0xF4, 0xAB, 0x2C, // WORD8
        0x06, 0x71, 0xEB, 0x44, // WORD9
        0x3F, 0xEA, 0xF6, 0x02, // WORD10
        0x92, 0x45, 0x52, 0x13,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris1);
}
