use crate::{
    gps::{
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame3, GpsQzssFrameId, GpsQzssSubframe,
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

    let encoded = frame.encode_raw();
    let encoded_size = encoded.len();

    assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");

    let mut decoded = GpsQzssDecoder::default();

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
            15_000,
            false,
            false,
            GpsQzssFrameId::Ephemeris3,
            0x13E,
            true,
            false,
            1e-6,
            2e-6,
            12.0,
            0.1,
            3e-1,
            0.3,
            0x34,
            1.6e-10,
            6e-1,
        ),
        (
            15_000,
            false,
            false,
            GpsQzssFrameId::Ephemeris3,
            0x13E,
            false,
            true,
            2e-6,
            1e-6,
            14.0,
            0.2,
            4e-1,
            0.4,
            0x43,
            1.5e-10,
            7e-1,
        ),
    ]
    .iter()
    .enumerate()
    {
        let mut how = GpsQzssHow::default().with_tow_seconds(*tow);
        let mut telemetry = GpsQzssTelemetry::default().with_message(*message);
        let mut subframe = GpsQzssFrame3::default()
            .with_cic_radians(*cic)
            .with_crc_meters(*crc)
            .with_iode(*iode)
            .with_omega_semicircles(*omega)
            .with_omega_dot_semicircles_s(*omega_dot)
            .with_inclination_semicircles(*i0)
            .with_cis_radians(*cis);

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
            .with_subframe(GpsQzssSubframe::Ephemeris3(subframe));

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
    let mut fd = File::create("data/GPS/eph3.bin").unwrap_or_else(|e| {
        panic!("Failed to create file: {}", e);
    });

    let mut frame = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris3);

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

        let subframe = frame.subframe.as_mut_eph3().unwrap();

        subframe.iode += 1;
        subframe.crc += 1.0;
    }
}

#[test]
fn parse_bin_file() {
    init_logger();

    let mut ptr = 0;
    let mut message = 0;
    let mut buffer = [0; 8192]; // single read

    let mut file = File::open("data/GPS/eph3.bin").unwrap();

    let mut decoder = GpsQzssDecoder::default();

    let model = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris3);

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

        let subf = decoded.subframe.as_eph3().unwrap_or_else(|| {
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

        info!("EPH-3.bin MESSAGE {}", message + 1);

        ptr += processed_size / 8 - 1;
        size -= processed_size / 8 - 1;

        if size <= GPS_FRAME_BYTES - 2 {
            assert_eq!(message, 128, "did not parse enough messages");
        }
    }
    assert_eq!(message, 128, "did not parse enough messages");
}

