use crate::{
    gps::{
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame2, GpsQzssFrameId, GpsQzssSubframe,
        GpsQzssTelemetry,
    },
    tests::init_logger,
    Message,
};

#[test]
fn test1() {
    #[cfg(all(feature = "std", feature = "log"))]
    init_logger();

    let mut decoder = GpsQzssDecoder::default();

    let frame = GpsQzssFrame::default()
        .with_telemetry(
            GpsQzssTelemetry::default()
                .with_message(0x9999)
                .with_integrity()
                .with_reserved_bit(),
        )
        .with_hand_over_word(
            GpsQzssHow::default()
                .with_tow_seconds(18_516)
                .with_alert_bit()
                .with_anti_spoofing(),
        )
        .with_subframe(GpsQzssSubframe::Ephemeris2(
            GpsQzssFrame2::default()
                .with_iode(0x12)
                .with_crs_meters(1.8)
                .with_cuc_radians(9e-7)
                .with_cus_radians(2e-6)
                .with_eccentricity(0.001)
                .with_mean_motion_difference_semicircles(4e-9)
                .with_mean_anomaly_semicircles(9.768415465951e-001)
                .with_toe_seconds(345_600)
                .with_square_root_semi_major_axis(5.153602432251e+003)
                .with_fit_interval_flag()
                .with_aodo(0x15),
        ));

    let encoded = frame.encode_raw();
    let encoded_size = encoded.len();

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    assert_eq!(encoded[1], 0x66);
    assert_eq!(encoded[2], 0x99 << 2 | 0x02 | 0x01);
    assert_eq!(encoded[3], 0x9C);

    // assert_eq!(encoded[4], 0x33);
    // assert_eq!(encoded[5], 0x33);
    // assert_eq!(encoded[6], 0xA0);
    // assert_eq!(encoded[7], 0x01);

    // assert_eq!(encoded[8], 0x20);
    // assert_eq!(encoded[9], 0x03);
    // assert_eq!(encoded[10], 0xA0);
    // assert_eq!(encoded[11], 0x3F);

    // assert_eq!(encoded[12], 0xFF);
    // assert_eq!(encoded[13], 0xDF);
    // assert_eq!(encoded[14], 0x40);
    // assert_eq!(encoded[15], 0x09);
    // assert_eq!(encoded[16], 0x24);
    // assert_eq!(encoded[17], 0xD0);
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
    // assert_eq!(encoded[28], 40);
    // assert_eq!(encoded[29], 64);
    // assert_eq!(encoded[30], 12);
    // assert_eq!(encoded[31], 209);
    // assert_eq!(encoded[32], 200);
    // assert_eq!(encoded[33], 0x00);
    // assert_eq!(encoded[34], 40);
    // assert_eq!(encoded[35], 0x03);
    // assert_eq!(encoded[36], 0x50);
    // assert_eq!(encoded[37], 0x00);

    // reciprocal
    let mut decoder = GpsQzssDecoder::default();

    let (size, decoded) = decoder.decode(&encoded, encoded_size);

    assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");

    assert_eq!(
        decoded,
        Some(frame),
        "reciprocal failed, got {:#?}",
        decoded,
    );
}

#[test]
fn reciprocal() {
    #[cfg(all(feature = "std", feature = "log"))]
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
            toe,
            dn,
            sqrt_a,
            iode,
            cuc,
            cus,
            aodo,
        ),
    ) in [
        (
            15_000,
            false,
            false,
            GpsQzssFrameId::Ephemeris2,
            0x13E,
            true,
            false,
            435_500,
            1.0e-1,
            5153.0,
            0x12,
            1e-6,
            2e-6,
            0x34,
        ),
        (
            15_000,
            false,
            false,
            GpsQzssFrameId::Ephemeris2,
            0x13E,
            false,
            true,
            435_500,
            1.0e-1,
            5152.0,
            0x21,
            2e-6,
            1e-6,
            0x52,
        ),
    ]
    .iter()
    .enumerate()
    {
        let mut how = GpsQzssHow::default().with_tow_seconds(*tow);
        let mut telemetry = GpsQzssTelemetry::default().with_message(*message);
        let mut subframe = GpsQzssFrame2::default()
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
            .with_subframe(GpsQzssSubframe::Ephemeris2(subframe));

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
    let mut fd = File::create("data/GPS/eph2.bin").unwrap_or_else(|e| {
        panic!("Failed to create file: {}", e);
    });

    let mut frame = GpsQzssFrame::default()
        .with_telemetry(GpsQzssTelemetry::model())
        .with_hand_over_word(GpsQzssHow::model(GpsQzssFrameId::Ephemeris2))
        .with_subframe(GpsQzssSubframe::model(GpsQzssFrameId::Ephemeris2));

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

        let subframe = frame.subframe.as_mut_eph2().unwrap();

        subframe.toe += 1;
        subframe.aodo += 1;
        subframe.iode += 1;
        subframe.fit_int_flag = !subframe.fit_int_flag;

        subframe.m0 += 1.0e-1;
        subframe.dn += 1.0e-1;
        subframe.cuc += 1.0e-8;
        subframe.cus += 1.0e-9;
        subframe.crs += 1.0;
    }
}

#[test]
fn parse_bin_file() {
    init_logger();

    let mut ptr = 0;
    let mut message = 0;
    let mut buffer = [0; 8192]; // single read

    let mut file = File::open("data/GPS/eph2.bin").unwrap();

    let mut decoder = GpsQzssDecoder::default();

    let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris2);

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

        let subf = decoded.subframe.as_eph2().unwrap_or_else(|| {
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

        info!("EPH-2.bin MESSAGE {}", message + 1);

        ptr += processed_size / 8 - 1;
        size -= processed_size / 8 - 1;

        if size <= GPS_FRAME_BYTES - 2 {
            assert_eq!(message, 128, "did not parse enough messages");
        }
    }
    assert_eq!(message, 128, "did not parse enough messages");
}

