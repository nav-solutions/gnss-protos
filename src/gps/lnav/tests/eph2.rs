use log::info;

use crate::{
    gps::lnav::{
        test_vectors::{on_air, words_to_bytes, UBLOX_EPH2},
        tests::{assert_valid_subframe, GpsQzssLnavSbfBinDecoder},
        GpsQzssLnavHow, GpsQzssLnavSubfDecoder, GpsQzssLnavSubframe, GpsQzssLnavSubframe2,
        GpsQzssLnavSubframeEnum, GpsQzssLnavSubframeId, GpsQzssLnavTelemetry,
        GPS_LNAV_SUBFRAME_BITS, GPS_LNAV_SUBFRAME_BYTES,
    },
    tests::init_logger,
    BigEndian, Message,
};

use bitbuffer::BitWriteStream;

#[test]
fn test1() {
    init_logger();

    let frame = GpsQzssLnavSubframe::default()
        .with_telemetry(
            GpsQzssLnavTelemetry::default()
                .with_message(0x1999)
                .with_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssLnavHow::default()
                .with_tow_seconds(18_516)
                .with_alert_bit()
                .with_anti_spoofing(),
        )
        .with_subframe_enum(GpsQzssLnavSubframeEnum::Ephemeris2(
            GpsQzssLnavSubframe2::default()
                .with_iode(0x12)
                .with_crs_meters(1.8)
                .with_cuc_radians(9e-7)
                .with_cus_radians(2e-6)
                .with_eccentricity(0.001)
                .with_mean_motion_difference_semicircles(3e-9)
                .with_mean_anomaly_semicircles(9.768415465951e-001)
                .with_toe_seconds(345_600)
                .with_square_root_semi_major_axis(5.153602432251e+003)
                .with_fit_interval_flag()
                .with_aodo(0x15),
        ));

    let mut encoded = [0; GPS_LNAV_SUBFRAME_BYTES];

    assert_eq!(
        frame.encode(&mut encoded).unwrap_or_else(|e| {
            panic!("Failed to encode EPH-2: {}", e);
        }),
        GPS_LNAV_SUBFRAME_BYTES
    );

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_valid_subframe(&encoded);

    // reciprocal (Message)
    let decoded = GpsQzssLnavSubframe::decode(&encoded).unwrap();
    assert_eq!(decoded, frame, "reciprocal failed, got {:#?}", decoded);
    assert_eq!(decoded.how.sbf_id, GpsQzssLnavSubframeId::Ephemeris2);

    // reciprocal (decoder)
    let mut decoder = GpsQzssLnavSubfDecoder::default();
    let (consumed, decoded) = decoder.decode(&encoded, 0);
    assert_eq!(consumed, GPS_LNAV_SUBFRAME_BITS);
    assert_eq!(decoded, Some(frame));
}

#[test]
fn reciprocal() {
    init_logger();

    for (
        test_num,
        (tow, alert, message, integrity, tlm_reserved_bit, toe, dn, sqrt_a, iode, cuc, cus, aodo),
    ) in [
        (
            15_000, false, 0x13E, true, false, 435_504, 1.0e-9, 5153.0, 0x12, 1e-6, 2e-6, 0x14,
        ),
        (
            15_006, true, 0x13E, false, true, 435_504, -1.0e-9, 5152.0, 0x21, -2e-6, 1e-6, 0x1f,
        ),
        (
            604_794, false, 0x3fff, true, true, 0, 3.0e-9, 4000.0, 0xff, 3e-6, -3e-6, 0x00,
        ),
    ]
    .iter()
    .enumerate()
    {
        let mut how = GpsQzssLnavHow::default().with_tow_seconds(*tow);
        let mut telemetry = GpsQzssLnavTelemetry::default().with_message(*message);

        let subframe = GpsQzssLnavSubframe2::default()
            .with_toe_seconds(*toe)
            .with_mean_motion_difference_semicircles(*dn)
            .with_square_root_semi_major_axis(*sqrt_a)
            .with_iode(*iode)
            .with_aodo(*aodo)
            .with_cuc_radians(*cuc)
            .with_cus_radians(*cus);

        if *alert {
            how = how.with_alert_bit();
        }

        if *integrity {
            telemetry = telemetry.with_integrity();
        }

        if *tlm_reserved_bit {
            telemetry = telemetry.with_reserved_bit();
        }

        let frame = GpsQzssLnavSubframe::default()
            .with_telemetry(telemetry)
            .with_hand_over_word(how)
            .with_subframe_enum(GpsQzssLnavSubframeEnum::Ephemeris2(subframe));

        let mut encoded = [0; GPS_LNAV_SUBFRAME_BYTES];

        frame.encode(&mut encoded).unwrap_or_else(|e| {
            panic!("Failed to encode frame #{}: {}", test_num, e);
        });

        assert_valid_subframe(&encoded);

        let decoded = GpsQzssLnavSubframe::decode(&encoded).unwrap_or_else(|e| {
            panic!("Failed to decode frame #{}: {}", test_num, e);
        });

        assert_eq!(decoded, frame, "reciprocal failed");

        // bit exact
        let mut reencoded = [0; GPS_LNAV_SUBFRAME_BYTES];
        decoded.encode(&mut reencoded).unwrap();
        assert_eq!(
            reencoded, encoded,
            "bit exact reciprocal failed (#{})",
            test_num
        );

        info!("test ({}): {:?}", test_num, frame);
    }
}

#[test]
fn stream_of_frames() {
    const NUM_FRAMES: usize = 128;

    let mut frames = Vec::with_capacity(NUM_FRAMES);

    let mut frame = GpsQzssLnavSubframe::default()
        .with_telemetry(GpsQzssLnavTelemetry::model())
        .with_hand_over_word(GpsQzssLnavHow::model(GpsQzssLnavSubframeId::Ephemeris2))
        .with_subframe_enum(GpsQzssLnavSubframeEnum::model(
            GpsQzssLnavSubframeId::Ephemeris2,
        ));

    for _ in 0..NUM_FRAMES {
        frames.push(frame);

        frame.telemetry.message = (frame.telemetry.message + 1) & 0x3fff;
        frame.telemetry.integrity = !frame.telemetry.integrity;
        frame.telemetry.reserved_bit = !frame.telemetry.reserved_bit;

        frame.how.tow += 6;
        frame.how.alert = !frame.how.alert;
        frame.how.anti_spoofing = !frame.how.anti_spoofing;

        let subframe = frame.subframe.as_mut_eph2().unwrap();

        subframe.toe += 16;
        subframe.aodo = (subframe.aodo + 1) & 0x1f;
        subframe.iode = subframe.iode.wrapping_add(1);
        subframe.fit_int_flag = !subframe.fit_int_flag;

        subframe.m0 += 1.0e-3;
        subframe.dn += 1.0e-11;
        subframe.cuc += 1.0e-8;
        subframe.cus += 1.0e-9;
        subframe.crs += 1.0;
    }

    let mut buffer = Vec::new();

    {
        let mut writer = BitWriteStream::new(&mut buffer, BigEndian);

        for frame in frames.iter() {
            writer.write(frame).unwrap();
        }
    }

    let mut decoder = GpsQzssLnavSubfDecoder::default();
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

    assert_eq!(decoded, frames);
}

#[test]
fn ublox() {
    init_logger();

    let frame = GpsQzssLnavSubframe::decode(&words_to_bytes(&on_air(UBLOX_EPH2))).unwrap();

    assert_eq!(frame.telemetry.message, 0x13E);
    assert!(!frame.telemetry.integrity);
    assert!(!frame.telemetry.reserved_bit);
    assert!(!frame.how.alert);
    assert!(frame.how.anti_spoofing);
    assert_eq!(frame.how.sbf_id, GpsQzssLnavSubframeId::Ephemeris2);
}

fn run_test(fullpath: &str, items: usize) {
    let mut count = 0;

    let mut decoder = GpsQzssLnavSbfBinDecoder::new(fullpath);
    while let Some(subf2) = decoder.parse_next() {
        assert_eq!(
            subf2.how.sbf_id,
            GpsQzssLnavSubframeId::Ephemeris2,
            "found invalid SUBF"
        );
        let _ = subf2
            .subframe
            .as_eph2()
            .unwrap_or_else(|| panic!("found invalid SUBF"));
        count += 1;
    }
    assert_eq!(count, items);
}

#[test]
fn parse_reconstructed_g15_unsync_subf2() {
    run_test("data/GPS/UNSYNC/ESBC00DNK_G15_eph2.bin", 2398);
}

#[test]
fn parse_reconstructed_g15_sync_subf2() {
    run_test("data/GPS/SYNC/ESBC00DNK_G15_eph2.bin", 2398);
}
