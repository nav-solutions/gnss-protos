#[cfg(feature = "std")]
use std::sync::Once;

#[cfg(feature = "std")]
use log::LevelFilter;

use crate::GpsDataByte;

#[cfg(feature = "std")]
static INIT: Once = Once::new();

pub fn init_logger() {
    #[cfg(feature = "std")]
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Trace)
            .init();
    });
}

pub fn from_ublox_be_bytes<const N: usize>(bytes: &[u8; N]) -> Vec<GpsDataByte> {
    let mut ret = Vec::new();
    let mut word32 = 0u32;

    for (index, byte) in bytes.iter().enumerate() {
        if index % 4 == 0 {
            ret.push(GpsDataByte::msb_padded(*byte));
        } else {
            ret.push(GpsDataByte::Byte(*byte));
        }
    }

    ret
}

#[test]
fn test_from_ublox_be_bytes() {
    let bytes = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B,
    ];

    let bytes = from_ublox_be_bytes(&bytes);
    assert_eq!(bytes.len(), 4);

    let bytes = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x73, 0xC9, 0x27, 0x15, // WORD3
        0x13, 0xE4, 0x00, 0x04, //WORD4
        0x10, 0x4F, 0x5D, 0x31, //WORD5
        0x97, 0x44, 0xE6, 0xD7, // WORD6
        0x07, 0x75, 0x57, 0x83, //WORD7
        0x33, 0x0C, 0x80, 0xB5, // WORD8
        0x92, 0x50, 0x42, 0xA1, // WORD9
        0x80, 0x00, 0x16, 0x84, //WORD10
        0x31, 0x2C, 0x30, 0x33,
    ];

    let bytes = from_ublox_be_bytes(&bytes);
    assert_eq!(bytes.len(), 40);
}
