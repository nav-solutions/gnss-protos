#[cfg(feature = "std")]
use std::sync::Once;

#[cfg(feature = "std")]
use log::LevelFilter;

#[cfg(feature = "gps")]
use crate::gps::GpsDataByte;

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

#[cfg(feature = "gps")]
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
#[cfg(feature = "gps")]
fn test_from_ublox_be_bytes() {
    let ubx_bytes = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B,
    ];

    let bytes = from_ublox_be_bytes(&ubx_bytes);
    assert_eq!(bytes.len(), 4);

    assert_eq!(
        bytes,
        [
            GpsDataByte::MsbPadded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B)
        ]
    );

    let ubx_bytes = [
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

    let bytes = from_ublox_be_bytes(&ubx_bytes);

    assert_eq!(bytes.len(), 40);

    assert_eq!(bytes[0], GpsDataByte::MsbPadded(0x22));
    assert_eq!(bytes[1], GpsDataByte::Byte(0xC1));
    assert_eq!(bytes[2], GpsDataByte::Byte(0x3E));
    assert_eq!(bytes[3], GpsDataByte::Byte(0x1B));

    assert_eq!(bytes[4], GpsDataByte::MsbPadded(0x33));
    assert_eq!(bytes[5], GpsDataByte::Byte(0xC9));
    assert_eq!(bytes[6], GpsDataByte::Byte(0x27));
    assert_eq!(bytes[7], GpsDataByte::Byte(0x15));

    assert_eq!(bytes[8], GpsDataByte::MsbPadded(0x13));
    assert_eq!(bytes[9], GpsDataByte::Byte(0xE4));
    assert_eq!(bytes[10], GpsDataByte::Byte(0x00));
    assert_eq!(bytes[11], GpsDataByte::Byte(0x04));

    assert_eq!(bytes[12], GpsDataByte::MsbPadded(0x10));
    assert_eq!(bytes[13], GpsDataByte::Byte(0x4F));
    assert_eq!(bytes[14], GpsDataByte::Byte(0x5D));
    assert_eq!(bytes[15], GpsDataByte::Byte(0x31));

    assert_eq!(bytes[16], GpsDataByte::MsbPadded(0x17));
    assert_eq!(bytes[17], GpsDataByte::Byte(0x44));
    assert_eq!(bytes[18], GpsDataByte::Byte(0xE6));
    assert_eq!(bytes[19], GpsDataByte::Byte(0xD7));
}
