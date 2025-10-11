// #[allow(non_snake_case)]
// mod L1;

// mod burst;
// mod eph1;
// mod eph2;
// mod eph3;

use crate::{
    tests::init_logger,
    {
        gps::{GpsQzssDecoder, GpsQzssFrame, GPS_FRAME_BITS, GPS_FRAME_BYTES},
        Decoder, Message,
    },
};

#[test]
fn encoding_size() {
    assert_eq!(GpsQzssFrame::default().encoding_bits(), 300);
    assert_eq!(GpsQzssFrame::default().encoding_size(), 300 / 8 + 1);
}

#[test]
fn default_frame() {
    init_logger();
    let mut encoded = [0u8; 1024];

    let default = GpsQzssFrame::default();

    let size = default.encode(&mut encoded).unwrap_or_else(|e| {
        panic!("failed to encode default frame: {:?}", e);
    });

    assert_eq!(size, 300 / 8 + 1, "invalid size encoded!");

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x00);
    assert_eq!(encoded[2], 0x00);
    assert_eq!(encoded[3], 0x00);
    assert_eq!(encoded[4], 0x00);
    assert_eq!(encoded[5], 0x00);
    assert_eq!(encoded[6], 0x10);
    assert_eq!(encoded[7], 0x00);
    assert_eq!(encoded[8], 0x00);
    assert_eq!(encoded[9], 0x00);
    assert_eq!(encoded[10], 0x00);
    assert_eq!(encoded[11], 0x00);
    assert_eq!(encoded[12], 0x00);
    assert_eq!(encoded[13], 0x00);
    assert_eq!(encoded[14], 0x00);
    assert_eq!(encoded[15], 0x00);
    assert_eq!(encoded[16], 0x00);
    assert_eq!(encoded[17], 0x00);
    assert_eq!(encoded[18], 0x00);
    assert_eq!(encoded[19], 0x00);
    assert_eq!(encoded[20], 0x00);
    assert_eq!(encoded[21], 0x00);
    assert_eq!(encoded[22], 0x00);
    assert_eq!(encoded[23], 0x00);
    assert_eq!(encoded[24], 0x00);
    assert_eq!(encoded[25], 0x00);
    assert_eq!(encoded[26], 0x00);
    assert_eq!(encoded[27], 0x00);
    assert_eq!(encoded[28], 0x00);
    assert_eq!(encoded[29], 0x00);
    assert_eq!(encoded[30], 0x00);
    assert_eq!(encoded[31], 0x00);
    assert_eq!(encoded[32], 0x00);
    assert_eq!(encoded[33], 0x00);
    assert_eq!(encoded[34], 0x00);
    assert_eq!(encoded[35], 0x00);
    assert_eq!(encoded[36], 0x00);
    assert_eq!(encoded[37], 0x00);

    // reciprocal
    let mut decoder = GpsQzssDecoder::default();

    decoder.fill(&encoded).unwrap_or_else(|e| {
        panic!("Failed to provide data: {:?}", e);
    });

    let decoded = decoder.decode().unwrap_or_else(|| {
        panic!("Decoding failed!");
    });

    assert_eq!(decoded, default, "reciprocal failed");
}
