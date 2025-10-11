use crate::{
    gps::{GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame1, GpsQzssSubframe, GpsQzssTelemetry},
    tests::init_logger,
    Message,
};

#[test]
fn test1() {
    init_logger();

    let mut decoder = GpsQzssDecoder::default();

    let frame = GpsQzssFrame::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x1234)
                .with_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(18_510)
                .with_alert_bit()
                .with_anti_spoofing(),
        )
        .with_subframe(GpsQzssSubframe::Ephemeris1(
            GpsQzssFrame1::default()
                .with_week(0x123)
                .with_iodc(0x123)
                .with_all_signals_ok()
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

    let encoded = frame.encode_raw();
    let encoded_size = encoded.len();

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x48);
    assert_eq!(encoded[2], 0xD0 | 0x02 | 0x01);
    assert_eq!(encoded[3], 0x4C);

    assert_eq!(encoded[4], 0x60);
    assert_eq!(encoded[5], 0x69);
    assert_eq!(encoded[6], 0x92);
    assert_eq!(encoded[7], 0x04);

    assert_eq!(encoded[8], 0x8f);
    assert_eq!(encoded[9], 0x20);
    assert_eq!(encoded[10], 0x10);
    assert_eq!(encoded[11], 0x24);
    assert_eq!(encoded[12], 0x8D);
    assert_eq!(encoded[13], 0x15);
    assert_eq!(encoded[14], 0x80);
    assert_eq!(encoded[15], 0x34);
    assert_eq!(encoded[16], 0x56);
    assert_eq!(encoded[17], 0x78);
    assert_eq!(encoded[18], 0x02);
    assert_eq!(encoded[19], 0x61);
    assert_eq!(encoded[20], 0xD9);
    assert_eq!(encoded[21], 0x50);
    assert_eq!(encoded[22], 0x01);
    assert_eq!(encoded[23], 0x23);
    assert_eq!(encoded[24], 0x40);
    assert_eq!(encoded[25], 0x20);
    assert_eq!(encoded[26], 0x08);
    assert_eq!(encoded[27], 0xC0);
    assert_eq!(encoded[28], 0xBB);
    assert_eq!(encoded[29], 0x80);
    assert_eq!(encoded[30], 0x24);
    assert_eq!(encoded[31], 0x00);
    assert_eq!(encoded[32], 0x09);
    assert_eq!(encoded[33], 0x00);
    assert_eq!(encoded[34], 0x00);
    assert_eq!(encoded[35], 0x00);
    assert_eq!(encoded[36], 0x20);
    assert_eq!(encoded[37], 0x00);

    // reciprocal
    let mut decoder = GpsQzssDecoder::default();

    let (size, decoded) = decoder.decode(&encoded, encoded_size);

    assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");

    assert_eq!(
        decoded,
        Some(frame),
        "reciprocal failed, got {:#?}",
        decoded
    );
}

#[test]
fn test2() {
    let frame = GpsQzssFrame::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x1234)
                .without_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(0x4_6789)
                .with_alert_bit()
                .without_anti_spoofing(),
        )
        .with_subframe(GpsQzssSubframe::Ephemeris1(
            GpsQzssFrame1::default()
                .with_week(0x123)
                .with_iodc(0x345)
                .with_all_signals_ok()
                .with_ca_or_p_l2_mask(0x1)
                .with_user_range_accuracy_m(24.0)
                .with_clock_offset_nanoseconds(2.0)
                .with_clock_drift_seconds_s(2E-12)
                .with_clock_drift_rate_seconds_s2(2E-15),
        ));

    let encoded = frame.encode_raw();

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x48);
    assert_eq!(encoded[2], 0x34 << 2 | 0x01);
    assert_eq!(encoded[3], 0x4D);

    assert_eq!(encoded[4], 0xDF);
    assert_eq!(encoded[5], 0x61);
    assert_eq!(encoded[6], 0x10);
    assert_eq!(encoded[7], 0x04);

    assert_eq!(encoded[8], 0x8D);
    assert_eq!(encoded[9], 0x60);
    assert_eq!(encoded[10], 0x30);
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
    assert_eq!(encoded[26], 0x11);
    assert_eq!(encoded[27], 0x40);
    assert_eq!(encoded[28], 0x00);
    assert_eq!(encoded[29], 0x00);
    assert_eq!(encoded[30], 0x48);
    assert_eq!(encoded[31], 0x00);
    assert_eq!(encoded[32], 0x12);
    assert_eq!(encoded[33], 0x00);
    assert_eq!(encoded[34], 0x00);
    assert_eq!(encoded[35], 0x00);
    assert_eq!(encoded[36], 0x40);
    assert_eq!(encoded[37], 0x00);

    let frame = GpsQzssFrame::default()
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
        .with_subframe(GpsQzssSubframe::Ephemeris1(
            GpsQzssFrame1::default()
                .with_week(0x321)
                .with_iodc(0x321)
                .with_all_signals_ok()
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

    let encoded = frame.encode_raw();

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x04);
    assert_eq!(encoded[2], 0x23 << 2);
    assert_eq!(encoded[3], 0x30);

    assert_eq!(encoded[4], 0x4E);
    assert_eq!(encoded[5], 0x20);
    assert_eq!(encoded[6], 0x10);
    assert_eq!(encoded[7], 0x0C);

    assert_eq!(encoded[8], 0x86);
    // TODO
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
    let mut decoder = GpsQzssDecoder::default();

    let (size, decoded) = decoder.decode(&encoded, encoded_size);

    assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");

    assert_eq!(
        decoded,
        Some(frame),
        "reciprocal failed, got {:#?}",
        decoded
    );
}

#[test]
fn reciprocal() {
    init_logger();

    let mut decoder = GpsQzssDecoder::default();

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
            15_000,
            false,
            false,
            GpsQzssFrameId::Ephemeris1,
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
            15_000,
            true,
            false,
            GpsQzssFrameId::Ephemeris1,
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
            15_000,
            true,
            false,
            GpsQzssFrameId::Ephemeris1,
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

        let mut subframe = GpsQzssFrame1::default()
            .with_week(*week)
            .with_iodc(*iodc)
            .with_health_mask(*health)
            .with_time_of_clock_seconds(*toc)
            .with_total_group_delay_nanos(*tgd)
            .with_ca_or_p_l2_mask(*ca_or_p_l2)
            .with_clock_offset_seconds(*af0)
            .with_clock_drift_seconds_s(*af1)
            .with_clock_drift_rate_seconds_s2(*af2);

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

        let frame = GpsQzssFrame::default()
            .with_telemetry(telemetry)
            .with_hand_over_word(how)
            .with_subframe(GpsQzssSubframe::Ephemeris1(subframe));

        let encoded = frame.encode_raw();
        let encoded_size = encoded.len();
        assert_eq!(encoded.len(), GPS_FRAME_BYTES, "encoded invalid size!");

        let (size, decoded) = decoder.decode(&encoded, encoded.len());
        assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
        assert_eq!(decoded, Some(frame), "reciprocal failed");

        info!("test ({}): {:?}", test_num, frame);
    }
}

#[test]
fn generate_bin_file() {
    let mut fd = File::create("data/GPS/eph1.bin").unwrap_or_else(|e| {
        panic!("Failed to create file: {}", e);
    });

    let mut frame = GpsQzssFrame::default()
        .with_telemetry(GpsQzssTelemetry::model())
        .with_hand_over_word(GpsQzssHow::model(GpsQzssFrameId::Ephemeris1))
        .with_subframe(GpsQzssSubframe::model(GpsQzssFrameId::Ephemeris1));

    for i in 0..128 {
        let encoded = frame.encode_raw();

        fd.write(&encoded).unwrap_or_else(|e| {
            panic!("Failed to write encoded frame #{}: {}", i, e);
        });

        frame.telemetry.message += 1;
        frame.telemetry.integrity = !frame.telemetry.integrity;
        frame.telemetry.reserved_bit = !frame.telemetry.reserved_bit;

        frame.how.tow += 1;
        frame.how.alert = !frame.how.alert;
        frame.how.anti_spoofing = !frame.how.anti_spoofing;

        let subframe = frame.subframe.as_mut_eph1().unwrap();

        subframe.week += 1;
        subframe.ura += 1;
        subframe.ca_or_p_l2 = subframe.ca_or_p_l2 ^ 0x3;
        subframe.iodc += 1;
        subframe.health += 1;
        subframe.toc += 1;

        subframe.af0 += 1.0E-9;
        subframe.af1 += 1.0E-12;
        subframe.af2 += 1.0E-15;

        subframe.tgd += 1.0E-9;

        subframe.reserved_word4 += 1;
        subframe.reserved_word5 += 1;
        subframe.reserved_word6 += 1;
        subframe.reserved_word7 += 1;

        subframe.l2_p_data_flag = !subframe.l2_p_data_flag;
    }
}

#[test]
fn parse_bin_file() {
    init_logger();

    let mut ptr = 0;
    let mut message = 0;
    let mut buffer = [0; 8192]; // single read

    let mut file = File::open("data/GPS/eph1.bin").unwrap();

    let mut decoder = GpsQzssDecoder::default();

    let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);

    let mut size = file.read(&mut buffer).unwrap();

    assert!(size > 0, "file is empty");

    // consume everything
    loop {
        if message == 128 {
            // we're done
            break;
        }

        // grab a frame
        let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);

        message += 1;

        if message == 1 {
            // first RX
            assert_eq!(processed_size, GPS_FRAME_BITS); // bits!
        } else {
            // following RX
            // TODO +8 expected here, not 16
            assert_eq!(processed_size, GPS_FRAME_BITS + 16); // bits!
        }

        let decoded = decoded.unwrap(); // success (we have 128 frames)

        let subf = decoded.subframe.as_eph1().unwrap_or_else(|| {
            panic!("wrong frame type decoded");
        });

        if message == 1 {
            // verify initial values
            assert_eq!(decoded, model, "invalid initial value");
        } else {
            // test pattern
            assert_eq!(
                decoded.telemetry.message,
                model.telemetry.message + message - 1,
                "error at message {}",
                message
            );

            if message % 2 == 0 {
                assert_eq!(decoded.telemetry.integrity, false);
                assert_eq!(decoded.telemetry.reserved_bit, false);
                assert_eq!(decoded.how.alert, false);
                assert_eq!(decoded.how.anti_spoofing, false);
            } else {
                assert_eq!(decoded.telemetry.integrity, true);
                assert_eq!(decoded.telemetry.reserved_bit, true);
                assert_eq!(decoded.how.alert, true);
                assert_eq!(decoded.how.anti_spoofing, true);
            }
        }

        info!("EPH-1.bin MESSAGE {}", message + 1);

        ptr += processed_size / 8 - 1;
        size -= processed_size / 8 - 1;

        if size <= GPS_FRAME_BYTES - 2 {
            assert_eq!(message, 128, "did not parse enough messages");
        }
    }
    assert_eq!(message, 128, "did not parse enough messages");
}

#[test]
fn parse_bin_delayed() {
    init_logger();

    let mut buffer = [0; 8192];

    // TODO add more cases
    for zeros in 1..7 {
        let mut ptr = 0;
        let mut message = 0;
        let mut buffer = [0; 8192]; // single read

        let mut file = File::open("data/GPS/eph1.bin").unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);

        let mut size = file.read(&mut buffer).unwrap();
        assert!(size > 0, "file is empty");

        let delayed = insert_zeros(&buffer, zeros);

        // consume everything
        loop {
            if message == 128 {
                // we're done
                break;
            }

            // grab a frame
            let (processed_size, decoded) = decoder.decode(&delayed[ptr..], size);

            message += 1;

            if message == 1 {
                // first RX
                assert_eq!(processed_size, GPS_FRAME_BITS + zeros); // bits!
            } else {
                // following RX
                // TODO +8 expected here, not 16
                assert_eq!(processed_size, GPS_FRAME_BITS + 16 + zeros); // bits!
            }

            let decoded = decoded.unwrap(); // success (we have 128 frames)

            let subf = decoded.subframe.as_eph1().unwrap_or_else(|| {
                panic!("wrong frame type decoded");
            });

            if message == 1 {
                // verify initial values
                assert_eq!(decoded, model, "invalid initial value");
            } else {
                // test pattern
                assert_eq!(
                    decoded.telemetry.message,
                    model.telemetry.message + message - 1,
                    "error at message {}",
                    message
                );

                if message % 2 == 0 {
                    assert_eq!(decoded.telemetry.integrity, false);
                    assert_eq!(decoded.telemetry.reserved_bit, false);
                    assert_eq!(decoded.how.alert, false);
                    assert_eq!(decoded.how.anti_spoofing, false);
                } else {
                    assert_eq!(decoded.telemetry.integrity, true);
                    assert_eq!(decoded.telemetry.reserved_bit, true);
                    assert_eq!(decoded.how.alert, true);
                    assert_eq!(decoded.how.anti_spoofing, true);
                }
            }

            info!("EPH-1.bin MESSAGE {}", message + 1);

            ptr += processed_size / 8 - 1;
            size -= processed_size / 8 - 1;

            if size <= GPS_FRAME_BYTES - 2 {
                assert_eq!(message, 128, "did not parse enough messages");
            }
        }
        assert_eq!(message, 128, "did not parse enough messages");
    }
}

