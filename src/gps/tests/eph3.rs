use log::info;

use crate::{
    gps::{
        test_vectors::{on_air, words_to_bytes, UBLOX_EPH3},
        tests::assert_valid_subframe,
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame3, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
        GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
    },
    tests::init_logger,
    BigEndian, Message,
};

use bitbuffer::BitWriteStream;

#[test]
fn test1() {
    init_logger();

    let frame = GpsQzssFrame::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x1999)
                .with_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(18_516)
                .with_alert_bit()
                .with_anti_spoofing(),
        )
        .with_subframe(GpsQzssSubframe::Ephemeris3(
            GpsQzssFrame3::default()
                .with_iode(0x32)
                .with_cic_radians(1.2e-7)
                .with_cis_radians(1.2e-7)
                .with_crc_meters(250.0)
                .with_inclination_semicircles(0.95)
                .with_inclination_rate_semicircles_s(2e-10)
                .with_longitude_ascending_node_semicircles(6.0e-1)
                .with_omega_semicircles(0.5e-1)
                .with_omega_dot_semicircles_s(8e-9),
        ));

    let mut encoded = [0; GPS_FRAME_BYTES];

    assert_eq!(
        frame.encode(&mut encoded).unwrap_or_else(|e| {
            panic!("Failed to encode EPH-3: {}", e);
        }),
        GPS_FRAME_BYTES
    );

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_valid_subframe(&encoded);

    // reciprocal (Message)
    let decoded = GpsQzssFrame::decode(&encoded).unwrap();
    assert_eq!(decoded, frame, "reciprocal failed, got {:#?}", decoded);
    assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris3);

    // reciprocal (decoder)
    let mut decoder = GpsQzssDecoder::default();
    let (consumed, decoded) = decoder.decode(&encoded, 0);
    assert_eq!(consumed, GPS_FRAME_BITS);
    assert_eq!(decoded, Some(frame));
}

#[test]
fn reciprocal() {
    init_logger();

    for (
        test_num,
        (
            tow,
            anti_spoofing,
            message,
            integrity,
            tlm_reserved_bit,
            cic,
            cis,
            crc,
            omega,
            i0,
            omega_dot,
            iode,
            idot,
            omega0,
        ),
    ) in [
        (
            15_000, false, 0x13E, true, false, 1e-6, 2e-6, 12.0, 0.1, 3e-1, 3e-9, 0x34, 1.6e-10,
            6e-1,
        ),
        (
            15_006, true, 0x13E, false, true, 2e-6, 1e-6, 14.0, 0.2, 4e-1, -4e-9, 0x43, -1.5e-10,
            -7e-1,
        ),
        (
            604_794, true, 0x3fff, true, true, -2e-6, -1e-6, -14.0, -0.9, -0.99, 0.0, 0xff, 0.0,
            0.99,
        ),
    ]
    .iter()
    .enumerate()
    {
        let mut how = GpsQzssHow::default().with_tow_seconds(*tow);
        let mut telemetry = GpsQzssTelemetry::default().with_message(*message);

        let subframe = GpsQzssFrame3::default()
            .with_cic_radians(*cic)
            .with_crc_meters(*crc)
            .with_iode(*iode)
            .with_omega_semicircles(*omega)
            .with_omega_dot_semicircles_s(*omega_dot)
            .with_inclination_semicircles(*i0)
            .with_inclination_rate_semicircles_s(*idot)
            .with_longitude_ascending_node_semicircles(*omega0)
            .with_cis_radians(*cis);

        if *anti_spoofing {
            how = how.with_anti_spoofing();
        }

        if *integrity {
            telemetry = telemetry.with_integrity();
        }

        if *tlm_reserved_bit {
            telemetry = telemetry.with_reserved_bit();
        }

        let frame = GpsQzssFrame::default()
            .with_telemetry(telemetry)
            .with_hand_over_word(how)
            .with_subframe(GpsQzssSubframe::Ephemeris3(subframe));

        let mut encoded = [0; GPS_FRAME_BYTES];

        frame.encode(&mut encoded).unwrap_or_else(|e| {
            panic!("Failed to encode frame #{}: {}", test_num, e);
        });

        assert_valid_subframe(&encoded);

        let decoded = GpsQzssFrame::decode(&encoded).unwrap_or_else(|e| {
            panic!("Failed to decode frame #{}: {}", test_num, e);
        });

        assert_eq!(decoded, frame, "reciprocal failed");

        // bit exact
        let mut reencoded = [0; GPS_FRAME_BYTES];
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

    let mut frame = GpsQzssFrame::default()
        .with_telemetry(GpsQzssTelemetry::model())
        .with_hand_over_word(GpsQzssHow::model(GpsQzssFrameId::Ephemeris3))
        .with_subframe(GpsQzssSubframe::model(GpsQzssFrameId::Ephemeris3));

    for _ in 0..NUM_FRAMES {
        frames.push(frame);

        frame.telemetry.message = (frame.telemetry.message + 1) & 0x3fff;
        frame.telemetry.integrity = !frame.telemetry.integrity;
        frame.telemetry.reserved_bit = !frame.telemetry.reserved_bit;

        frame.how.tow += 6;
        frame.how.alert = !frame.how.alert;
        frame.how.anti_spoofing = !frame.how.anti_spoofing;

        let subframe = frame.subframe.as_mut_eph3().unwrap();

        subframe.iode = subframe.iode.wrapping_add(1);
        subframe.crc += 1.0;
        subframe.i0 -= 1.0e-3;
        subframe.omega0 += 1.0e-3;
    }

    let mut buffer = Vec::new();

    {
        let mut writer = BitWriteStream::new(&mut buffer, BigEndian);

        for frame in frames.iter() {
            writer.write(frame).unwrap();
        }
    }

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

    assert_eq!(decoded, frames);
}

#[test]
fn ublox() {
    init_logger();

    let frame = GpsQzssFrame::decode(&words_to_bytes(&on_air(UBLOX_EPH3))).unwrap();

    assert_eq!(frame.telemetry.message, 0x13E);
    assert!(!frame.telemetry.integrity);
    assert!(!frame.telemetry.reserved_bit);
    assert!(!frame.how.alert);
    assert!(frame.how.anti_spoofing);
    assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris3);
}
