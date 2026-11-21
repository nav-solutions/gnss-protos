use log::info;

use std::{fs::File, io::Write};

use crate::{
    gps::{
        tests::assert_valid_subframe, GpsQzssDecoder, GpsQzssHow, GpsQzssSatelliteHealth,
        GpsQzssSubframe, GpsQzssSubframe1, GpsQzssSubframeEnum, GpsQzssSubframeId,
        GpsQzssTelemetry, GPS_SUBFRAME_BITS,
    },
    tests::init_logger,
    BigEndian, Message,
};

use bitbuffer::BitWriteStream;

#[test]
fn test1() {
    init_logger();

    let frame = GpsQzssSubframe::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x1234)
                .with_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(18_510 * 6)
                .with_alert_bit()
                .with_anti_spoofing(),
        )
        .with_subframe_enum(GpsQzssSubframeEnum::Ephemeris1(
            GpsQzssSubframe1::default()
                .healthy()
                .with_week(0x123)
                .with_iodc(0x123)
                .with_time_of_clock_seconds(12_000)
                .with_l2p_flag()
                .with_clock_offset_nanoseconds(1.0)
                .with_clock_drift_seconds_s(1E-12)
                .with_clock_drift_rate_seconds_s2(1E-15)
                .with_reserved23_word(0x12_3456)
                .with_reserved24_word1(0x34_5678)
                .with_reserved24_word2(0x98_7654)
                .with_reserved16_word(0x1234)
                .with_total_group_delay_nanos(1.0)
                .with_ca_or_p_l2_mask(0x3)
                .with_user_range_accuracy_m(4.0),
        ));

    let mut encoded = [0; 1024];

    frame.encode(&mut encoded).unwrap_or_else(|e| {
        panic!("Failed to encode frame: {}", e);
    });

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x48);
    assert_eq!(encoded[2], 0xD0 | 0x02 | 0x01);
    // the 300 bits form a valid GPS subframe (parity, t-bits)
    assert_valid_subframe(&encoded);

    // TODO (bit true test)
    // assert_eq!(encoded[4], 0x60);
    // assert_eq!(encoded[6], 0x90);
    // assert_eq!(encoded[7], 0x04);

    // assert_eq!(encoded[8], 0x8f);
    // assert_eq!(encoded[9], 0x20);
    // assert_eq!(encoded[10], 0x10);
    // assert_eq!(encoded[11], 0x24);
    // assert_eq!(encoded[12], 0x8D);
    // assert_eq!(encoded[13], 0x15);
    // assert_eq!(encoded[14], 0x80);
    // assert_eq!(encoded[15], 0x34);
    // assert_eq!(encoded[16], 0x56);
    // assert_eq!(encoded[17], 0x78);
    // assert_eq!(encoded[18], 0x02);
    // assert_eq!(encoded[19], 0x61);
    // assert_eq!(encoded[20], 0xD9);
    // assert_eq!(encoded[21], 0x50);
    // assert_eq!(encoded[22], 0x01);
    // assert_eq!(encoded[23], 0x23);
    // assert_eq!(encoded[24], 0x40);
    // assert_eq!(encoded[25], 0x20);
    // assert_eq!(encoded[26], 0x08);
    // assert_eq!(encoded[27], 0xC0);
    // assert_eq!(encoded[28], 0xBB);
    // assert_eq!(encoded[29], 0x80);
    // assert_eq!(encoded[30], 0x24);
    // assert_eq!(encoded[31], 0x00);
    // assert_eq!(encoded[32], 0x09);
    // assert_eq!(encoded[33], 0x00);
    // assert_eq!(encoded[34], 0x00);
    // assert_eq!(encoded[35], 0x00);
    // assert_eq!(encoded[36], 0x20);
    // assert_eq!(encoded[37], 0x00);

    // reciprocal
    let decoded = GpsQzssSubframe::decode(&encoded).unwrap_or_else(|e| {
        panic!("failed to decode frame: {}", e);
    });

    assert_eq!(decoded, frame, "reciprocal failed, got {:#?}", decoded);
}

#[test]
fn test2() {
    init_logger();

    let frame = GpsQzssSubframe::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x1234)
                .without_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(0x4_6788)
                .with_alert_bit()
                .without_anti_spoofing(),
        )
        .with_subframe_enum(GpsQzssSubframeEnum::Ephemeris1(
            GpsQzssSubframe1::default()
                .healthy()
                .with_week(0x123)
                .with_iodc(0x345)
                .with_ca_or_p_l2_mask(0x1)
                .with_user_range_accuracy_m(24.0)
                .with_clock_offset_nanoseconds(2.0)
                .with_clock_drift_seconds_s(2E-12)
                .with_clock_drift_rate_seconds_s2(2E-15),
        ));

    let mut encoded = [0; 1024];

    frame.encode(&mut encoded).unwrap_or_else(|e| {
        panic!("Failed to encode EPH-1: {}", e);
    });

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x48);
    assert_eq!(encoded[2], 0x34 << 2 | 0x01);
    // the 300 bits form a valid GPS subframe (parity, t-bits)
    assert_valid_subframe(&encoded);

    // TODO (bit true test)
    // assert_eq!(encoded[4], 0xDF);
    // assert_eq!(encoded[5], 0x61);
    // assert_eq!(encoded[6], 0x10);
    // assert_eq!(encoded[7], 0x04);

    // assert_eq!(encoded[8], 0x8D);
    // assert_eq!(encoded[9], 0x60);
    // assert_eq!(encoded[10], 0x30);
    // assert_eq!(encoded[11], 0x00);
    // assert_eq!(encoded[12], 0x00);
    // assert_eq!(encoded[13], 0x00);
    // assert_eq!(encoded[14], 0x00);
    // assert_eq!(encoded[15], 0x00);
    // assert_eq!(encoded[16], 0x00);
    // assert_eq!(encoded[17], 0x00);
    // assert_eq!(encoded[18], 0x00);
    // assert_eq!(encoded[19], 0x00);
    // assert_eq!(encoded[20], 0x00);
    // assert_eq!(encoded[21], 0x00);
    // assert_eq!(encoded[22], 0x00);
    // assert_eq!(encoded[23], 0x00);
    // assert_eq!(encoded[24], 0x00);
    // assert_eq!(encoded[25], 0x00);
    // assert_eq!(encoded[26], 0x11);
    // assert_eq!(encoded[27], 0x40);
    // assert_eq!(encoded[28], 0x00);
    // assert_eq!(encoded[29], 0x00);
    // assert_eq!(encoded[30], 0x48);
    // assert_eq!(encoded[31], 0x00);
    // assert_eq!(encoded[32], 0x12);
    // assert_eq!(encoded[33], 0x00);
    // assert_eq!(encoded[34], 0x00);
    // assert_eq!(encoded[35], 0x00);
    // assert_eq!(encoded[36], 0x40);
    // assert_eq!(encoded[37], 0x00);

    // reciprocal
    let decoded = GpsQzssSubframe::decode(&encoded).unwrap_or_else(|e| {
        panic!("failed to decode frame: {}", e);
    });

    assert_eq!(decoded, frame, "reciprocal failed, got {:#?}", decoded);
}

#[test]
fn test3() {
    let frame = GpsQzssSubframe::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x0123)
                .without_integrity()
                .without_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(15_000)
                .without_alert_bit()
                .without_anti_spoofing(),
        )
        .with_subframe_enum(GpsQzssSubframeEnum::Ephemeris1(
            GpsQzssSubframe1::default()
                .healthy()
                .with_week(0x321)
                .with_iodc(0x321)
                .with_time_of_clock_seconds(24_000)
                .with_clock_offset_nanoseconds(2.0)
                .with_clock_drift_seconds_s(2E-12)
                .with_clock_drift_rate_seconds_s2(2E-15)
                .with_reserved23_word(0x12_3456)
                .with_reserved24_word1(0x34_5678)
                .with_reserved24_word2(0x98_7654)
                .with_reserved16_word(0x1234)
                .with_total_group_delay_nanos(3.0)
                .with_ca_or_p_l2_mask(0x2)
                .with_user_range_accuracy_m(8.0),
        ));

    let mut encoded = [0; 1024];

    frame.encode(&mut encoded).unwrap_or_else(|e| {
        panic!("Failed to encode EPH-1: {}", e);
    });

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x04);
    assert_eq!(encoded[2], 0x23 << 2);
    // the 300 bits form a valid GPS subframe (parity, t-bits)
    assert_valid_subframe(&encoded);

    // TODO (bit true test)
    // assert_eq!(encoded[4], 0x4E);
    // assert_eq!(encoded[5], 0x20);
    // assert_eq!(encoded[6], 0x10);
    // assert_eq!(encoded[7], 0x0C);

    // assert_eq!(encoded[8], 0x86);
    // assert_eq!(encoded[9], 0x00);
    // assert_eq!(encoded[10], 0x00);
    // assert_eq!(encoded[11], 0x00);
    // assert_eq!(encoded[12], 0x00);
    // assert_eq!(encoded[13], 0x00);
    // assert_eq!(encoded[14], 0x00);
    // assert_eq!(encoded[15], 0x00);
    // assert_eq!(encoded[16], 0x00);
    // assert_eq!(encoded[17], 0x00);
    // assert_eq!(encoded[18], 0x00);
    // assert_eq!(encoded[19], 0x00);
    // assert_eq!(encoded[20], 0x00);
    // assert_eq!(encoded[21], 0x00);
    // assert_eq!(encoded[22], 0x00);
    // assert_eq!(encoded[23], 0x00);
    // assert_eq!(encoded[24], 0x00);
    // assert_eq!(encoded[25], 0x00);
    // assert_eq!(encoded[26], 0x00);
    // assert_eq!(encoded[27], 0x00);
    // assert_eq!(encoded[28], 0x00);
    // assert_eq!(encoded[29], 0x00);
    // assert_eq!(encoded[30], 0x00);
    // assert_eq!(encoded[31], 0x00);
    // assert_eq!(encoded[32], 0x00);
    // assert_eq!(encoded[33], 0x00);
    // assert_eq!(encoded[34], 0x00);
    // assert_eq!(encoded[35], 0x00);
    // assert_eq!(encoded[36], 0x00);
    // assert_eq!(encoded[37], 0x00);

    // reciprocal
    let decoded = GpsQzssSubframe::decode(&encoded).unwrap_or_else(|e| {
        panic!("failed to decode frame: {}", e);
    });

    assert_eq!(decoded, frame, "reciprocal failed, got {:#?}", decoded);
}

#[test]
fn reciprocal() {
    init_logger();

    for (
        test_num,
        (
            tow,
            alert,
            anti_spoofing,
            frame_id,
            message,
            integrity,
            tlm_reserved_bit,
            week,
            ca_or_p_l2,
            ura,
            health,
            iodc,
            toc,
            tgd,
            af0,
            af1,
            af2,
        ),
    ) in [
        (
            // 0
            15_000,
            false,
            false,
            GpsQzssSubframeId::Ephemeris1,
            0x13E,
            false,
            false,
            10,
            0x0,
            0,
            0,
            0x10,
            10_000,
            1.0,
            1.0E-9,
            1.0E-12,
            1.0E-14,
        ),
        (
            // 1
            15_000,
            true,
            false,
            GpsQzssSubframeId::Ephemeris1,
            0x13F,
            false,
            true,
            100,
            0x1,
            1,
            1,
            0x0,
            16_000,
            2.0,
            2.0E-9,
            2.0E-12,
            2.0E-14,
        ),
        (
            // 2
            15_000,
            true,
            false,
            GpsQzssSubframeId::Ephemeris1,
            0x13F,
            false,
            true,
            1024,
            0x02,
            2,
            2,
            0x20,
            16_160,
            3.0,
            3.0E-9,
            3.0E-12,
            3.0E-14,
        ),
        // TODO: signed
        // (
        //     15_000,
        //     true,
        //     false,
        //     GpsQzssFrameId::Ephemeris1,
        //     0x13F,
        //     false,
        //     true,
        //     1024,
        //     0x03,
        //     3,
        //     3,
        //     0x123,
        //     16_432,
        //     -4.0,
        //     4.0E-9,
        //     4.0E-12,
        //     4.0E-14,
        // ),
        // TODO: signed
        // (
        //     15_000,
        //     true,
        //     false,
        //     GpsQzssFrameId::Ephemeris1,
        //     0x13F,
        //     false,
        //     true,
        //     1024,
        //     0x03,
        //     3,
        //     3,
        //     0x123,
        //     16_432,
        //     5.0,
        //     -4.0E-9,
        //     4.0E-12,
        //     4.0E-14,
        // ),
        // TODO: signed
        // (
        //     15_000,
        //     true,
        //     false,
        //     GpsQzssFrameId::Ephemeris1,
        //     0x13F,
        //     false,
        //     true,
        //     1024,
        //     0x03,
        //     3,
        //     3,
        //     0x123,
        //     16_432,
        //     5.0,
        //     6.0E-9,
        //     -4.0E-12,
        //     4.0E-14,
        // ),
        // TODO: signed
        // (
        //     15_000,
        //     true,
        //     false,
        //     GpsQzssFrameId::Ephemeris1,
        //     0x13F,
        //     false,
        //     true,
        //     1024,
        //     0x03,
        //     3,
        //     3,
        //     0x123,
        //     16_432,
        //     5.0,
        //     6.0E-9,
        //     4.0E-12,
        //     -4.0E-14,
        // ),
    ]
    .iter()
    .enumerate()
    {
        let mut how = GpsQzssHow::default().with_tow_seconds(*tow);
        let mut telemetry = GpsQzssTelemetry::default().with_message(*message);

        let mut subframe = GpsQzssSubframe1::default()
            .with_week(*week)
            .with_iodc(*iodc)
            // .with_health_mask(*health)
            .with_time_of_clock_seconds(*toc)
            .with_total_group_delay_nanos(*tgd)
            .with_ca_or_p_l2_mask(*ca_or_p_l2)
            .with_clock_offset_seconds(*af0)
            .with_clock_drift_seconds_s(*af1)
            .with_clock_drift_rate_seconds_s2(*af2);

        subframe.ura = *ura;
        subframe.health = GpsQzssSatelliteHealth::from(*health);

        if *alert {
            how = how.with_alert_bit();
        }

        if *anti_spoofing {
            how = how.with_anti_spoofing();
        }

        if *integrity {
            telemetry = telemetry.with_integrity();
        }

        if *tlm_reserved_bit {
            telemetry = telemetry.with_reserved_bit();
        }

        let frame = GpsQzssSubframe::default()
            .with_telemetry(telemetry)
            .with_hand_over_word(how)
            .with_subframe_enum(GpsQzssSubframeEnum::Ephemeris1(subframe));

        let mut encoded = [0; 1024];

        frame.encode(&mut encoded).unwrap_or_else(|e| {
            panic!("Failed to encode frame #{}: {}", test_num, e);
        });

        // reciprocal
        let decoded = GpsQzssSubframe::decode(&encoded).unwrap_or_else(|e| {
            panic!("failed to decode frame: {}", e);
        });

        assert_eq!(decoded, frame, "reciprocal failed");
        assert_eq!(decoded.how.frame_id, *frame_id);
        assert_eq!(decoded.subframe.as_eph1().unwrap().ura, *ura);
        assert_eq!(decoded.subframe.as_eph1().unwrap().health, subframe.health);
        assert_valid_subframe(&encoded);

        // bit exact
        let mut reencoded = [0; 1024];
        decoded.encode(&mut reencoded).unwrap();
        assert_eq!(reencoded, encoded, "bit exact reciprocal failed");

        info!("test ({}): {:?}", test_num, frame);
    }
}

#[test]
fn generate_bin_file() {
    // 128 consecutive frames, forming a continuous GPS bit stream
    // (300-bit frames, not aligned to bytes), as a receiver would output it.
    const NUM_FRAMES: usize = 128;

    let mut frames = Vec::with_capacity(NUM_FRAMES);

    let mut frame = GpsQzssSubframe::default()
        .with_telemetry(GpsQzssTelemetry::model())
        .with_hand_over_word(GpsQzssHow::model(GpsQzssSubframeId::Ephemeris1))
        .with_subframe_enum(GpsQzssSubframeEnum::model(GpsQzssSubframeId::Ephemeris1));

    for _ in 0..NUM_FRAMES {
        frames.push(frame);

        frame.telemetry.message += 1;
        frame.telemetry.integrity = !frame.telemetry.integrity;
        frame.telemetry.reserved_bit = !frame.telemetry.reserved_bit;

        frame.how.tow += 6;
        frame.how.alert = !frame.how.alert;
        frame.how.anti_spoofing = !frame.how.anti_spoofing;

        let subframe = frame.subframe.as_mut_eph1().unwrap();

        subframe.week += 1;
        subframe.ura = (subframe.ura + 1) & 0xf;
        subframe.ca_or_p_l2 ^= 0x3;

        *subframe = subframe.with_iodc(subframe.iodc() + 1);

        subframe.toc += 16;

        subframe.af0 += 1.0E-9;
        subframe.af1 += 1.0E-12;
        // stay within the range of each field (TGD: 8 bits, 2^-31 s)
        subframe.tgd += 1.0E-10;

        subframe.reserved_word4 += 1;
        subframe.reserved_word5 += 1;
        subframe.reserved_word6 += 1;
        subframe.reserved_word7 += 1;

        subframe.l2_p_data_flag = !subframe.l2_p_data_flag;
    }

    let mut buffer = Vec::with_capacity((NUM_FRAMES * GPS_SUBFRAME_BITS + 7) / 8);

    {
        let mut writer = BitWriteStream::new(&mut buffer, BigEndian);

        for frame in frames.iter() {
            writer.write(frame).unwrap();
        }

        assert_eq!(writer.bit_len(), NUM_FRAMES * GPS_SUBFRAME_BITS);
    }

    std::fs::create_dir_all("data/GPS").unwrap();

    let mut fd = File::create("data/GPS/eph1.bin").unwrap_or_else(|e| {
        panic!("Failed to create file: {}", e);
    });

    fd.write_all(&buffer).unwrap();

    // verify: the decoder retrieves every frame
    let mut decoder = GpsQzssDecoder::default();
    let mut position = 0;
    let mut decoded = Vec::new();

    loop {
        let (consumed, frame) = decoder.decode(&buffer, position);
        position += consumed;

        match frame {
            Some(frame) => decoded.push(frame),
            None => break,
        }
    }

    assert_eq!(decoded.len(), frames.len(), "missing frames");

    for (i, (decoded, frame)) in decoded.iter().zip(frames.iter()).enumerate() {
        assert_eq!(decoded, frame, "frame #{} differs", i);
    }
}

// #[test]
// fn ublox1() {
//     init_logger();
//
//     let mut decoder = GpsQzssDecoder::<1024>::default().without_parity_verification();
//
//     decoder.buffer.fill(&[
//         // TLM
//         0x22, 0xC1, 0x3E, 0x1B, // HOW
//         0x15, 0x27, 0xC9, 0x73, // WORD3
//         0x13, 0xE4, 0x00, 0x04, // WORD4
//         0x10, 0x4F, 0x5D, 0x31, // WORD5
//         0x97, 0x44, 0xE6, 0xD7, // WORD6
//         0x07, 0x75, 0x57, 0x83, // WORD7
//         0x33, 0x0C, 0x80, 0xB5, // WORD8
//         0x92, 0x50, 0x42, 0xA1, // WORD9
//         0x80, 0x00, 0x16, 0x84, // WORD10
//         0x31, 0x2C, 0x30, 0x33,
//     ]);
//
//     let frame = decoder.decode().unwrap_or_else(|| {
//         panic!("failed to decode valid frame!");
//     });
//
//     assert_eq!(frame.telemetry.message, 0x13E);
//     assert_eq!(frame.telemetry.integrity, false);
//     assert_eq!(frame.telemetry.reserved_bit, false);
//     assert_eq!(frame.how.alert, false);
//     assert_eq!(frame.how.anti_spoofing, true);
//     assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris1);
// }
//
// #[test]
// fn ublox2() {
//     init_logger();
//
//     let mut decoder = GpsQzssDecoder::<1024>::default().without_parity_verification();
//
//     decoder.buffer.fill(&[
//         // TLM
//         0x22, 0xC1, 0x3E, 0x1B, // HOW
//         0x15, 0x27, 0xC9, 0x73, // WORD3
//         0x00, 0x0A, 0xEA, 0x34, // WORD4
//         0x03, 0x3C, 0xFF, 0xEE, // WORD5
//         0xBF, 0xE5, 0xC9, 0xEB, // WORD6
//         0x13, 0x6F, 0xB6, 0x4E, // WORD7
//         0x86, 0xF4, 0xAB, 0x2C, // WORD8
//         0x06, 0x71, 0xEB, 0x44, // WORD9
//         0x3F, 0xEA, 0xF6, 0x02, // WORD10
//         0x92, 0x45, 0x52, 0x13,
//     ]);
//
//     let frame = decoder.decode().unwrap_or_else(|| {
//         panic!("failed to decode valid frame!");
//     });
//
//     assert_eq!(frame.telemetry.message, 0x13E);
//     assert_eq!(frame.telemetry.integrity, false);
//     assert_eq!(frame.telemetry.reserved_bit, false);
//     assert_eq!(frame.how.alert, false);
//     assert_eq!(frame.how.anti_spoofing, true);
//     assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris1);
// }

// #[test]
// fn parse_bin_file() {
//     init_logger();
//
//     let mut message = 0;
//     let mut gps_size = 0;
//     let mut file_buffer = [0; 8192]; // single read
//
//     let mut file = File::open("data/GPS/eph1.bin").unwrap();
//
//     let mut decoder = GpsQzssDecoder::default();
//
//     let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);
//
//     let mut size = file.read(&mut buffer).unwrap();
//
//     assert!(size > 0, "file is empty");
//
//     // consume everything
//     loop {
//         if message == 128 {
//             // we're done
//             break;
//         }
//
//         // grab a frame
//         let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);
//
//         message += 1;
//
//         if message == 1 {
//             // first RX
//             assert_eq!(processed_size, GPS_FRAME_BITS); // bits!
//         } else {
//             // following RX
//             // TODO +8 expected here, not 16
//             assert_eq!(processed_size, GPS_FRAME_BITS + 16); // bits!
//         }
//
//         let decoded = decoded.unwrap(); // success (we have 128 frames)
//
//         let subf = decoded.subframe.as_eph1().unwrap_or_else(|| {
//             panic!("wrong frame type decoded");
//         });
//
//         if message == 1 {
//             // verify initial values
//             assert_eq!(decoded, model, "invalid initial value");
//         } else {
//             // test pattern
//             assert_eq!(
//                 decoded.telemetry.message,
//                 model.telemetry.message + message - 1,
//                 "error at message {}",
//                 message
//             );
//
//             if message % 2 == 0 {
//                 assert_eq!(decoded.telemetry.integrity, false);
//                 assert_eq!(decoded.telemetry.reserved_bit, false);
//                 assert_eq!(decoded.how.alert, false);
//                 assert_eq!(decoded.how.anti_spoofing, false);
//             } else {
//                 assert_eq!(decoded.telemetry.integrity, true);
//                 assert_eq!(decoded.telemetry.reserved_bit, true);
//                 assert_eq!(decoded.how.alert, true);
//                 assert_eq!(decoded.how.anti_spoofing, true);
//             }
//         }
//
//         info!("EPH-1.bin MESSAGE {}", message + 1);
//
//         ptr += processed_size / 8 - 1;
//         size -= processed_size / 8 - 1;
//
//         if size <= GPS_FRAME_BYTES - 2 {
//             assert_eq!(message, 128, "did not parse enough messages");
//         }
//     }
//     assert_eq!(message, 128, "did not parse enough messages");
// }
//
// #[test]
// fn parse_bin_delayed() {
//     init_logger();
//
//     let mut buffer = [0; 8192];
//
//     // TODO add more cases
//     for zeros in 1..7 {
//         let mut ptr = 0;
//         let mut message = 0;
//         let mut buffer = [0; 8192]; // single read
//
//         let mut file = File::open("data/GPS/eph1.bin").unwrap();
//
//         let mut decoder = GpsQzssDecoder::default();
//
//         let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);
//
//         let mut size = file.read(&mut buffer).unwrap();
//         assert!(size > 0, "file is empty");
//
//         let delayed = insert_zeros(&buffer, zeros);
//
//         // consume everything
//         loop {
//             if message == 128 {
//                 // we're done
//                 break;
//             }
//
//             // grab a frame
//             let (processed_size, decoded) = decoder.decode(&delayed[ptr..], size);
//
//             message += 1;
//
//             if message == 1 {
//                 // first RX
//                 assert_eq!(processed_size, GPS_FRAME_BITS + zeros); // bits!
//             } else {
//                 // following RX
//                 // TODO +8 expected here, not 16
//                 assert_eq!(processed_size, GPS_FRAME_BITS + 16 + zeros); // bits!
//             }
//
//             let decoded = decoded.unwrap(); // success (we have 128 frames)
//
//             let subf = decoded.subframe.as_eph1().unwrap_or_else(|| {
//                 panic!("wrong frame type decoded");
//             });
//
//             if message == 1 {
//                 // verify initial values
//                 assert_eq!(decoded, model, "invalid initial value");
//             } else {
//                 // test pattern
//                 assert_eq!(
//                     decoded.telemetry.message,
//                     model.telemetry.message + message - 1,
//                     "error at message {}",
//                     message
//                 );
//
//                 if message % 2 == 0 {
//                     assert_eq!(decoded.telemetry.integrity, false);
//                     assert_eq!(decoded.telemetry.reserved_bit, false);
//                     assert_eq!(decoded.how.alert, false);
//                     assert_eq!(decoded.how.anti_spoofing, false);
//                 } else {
//                     assert_eq!(decoded.telemetry.integrity, true);
//                     assert_eq!(decoded.telemetry.reserved_bit, true);
//                     assert_eq!(decoded.how.alert, true);
//                     assert_eq!(decoded.how.anti_spoofing, true);
//                 }
//             }
//
//             info!("EPH-1.bin MESSAGE {}", message + 1);
//
//             ptr += processed_size / 8 - 1;
//             size -= processed_size / 8 - 1;
//
//             if size <= GPS_FRAME_BYTES - 2 {
//                 assert_eq!(message, 128, "did not parse enough messages");
//             }
//         }
//         assert_eq!(message, 128, "did not parse enough messages");
//     }
// }
//
