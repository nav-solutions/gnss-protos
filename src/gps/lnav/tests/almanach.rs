//! Subframes 4 & 5 (almanac pages) and 12'30" message, on real data.

use crate::{
    gps::lnav::{
        test_vectors::{on_air, words_to_bytes, UBLOX_ALMANACH_PAGES, UBLOX_G25_STREAM},
        tests::{assert_valid_subframe, GpsQzssLnavSbfBinDecoder},
        GpsQzssLnavSubfDecoder, GpsQzssLnavSubframe, GpsQzssLnavSubframe4, GpsQzssLnavSubframe5,
        GpsQzssLnavSubframeEnum, GpsQzssLnavSubframeId, GPS_LNAV_SUBFRAME_BITS,
        GPS_LNAV_SUBFRAME_BYTES, GPS_LNAV_WORD_BITS,
    },
    tests::init_logger,
    BigEndian, BitWriteStream, Message,
};

fn decode(words: [u32; 10]) -> (GpsQzssLnavSubframe, [u8; GPS_LNAV_SUBFRAME_BYTES]) {
    let bytes = words_to_bytes(&on_air(words));
    assert_valid_subframe(&bytes);
    (GpsQzssLnavSubframe::decode(&bytes).unwrap(), bytes)
}

fn frame4(sbf: &GpsQzssLnavSubframe) -> GpsQzssLnavSubframe4 {
    *sbf.subframe.as_alm4().expect("not a subframe 4")
}

fn frame5(sbf: &GpsQzssLnavSubframe) -> GpsQzssLnavSubframe5 {
    *sbf.subframe.as_alm5().expect("not a subframe 5")
}

#[test]
fn real_pages_bit_exact() {
    init_logger();

    for (description, tow, words) in UBLOX_ALMANACH_PAGES.iter() {
        let (frame, bytes) = decode(*words);
        assert_eq!(frame.how.tow, *tow, "{}", description);

        // the page number derives from the time of transmission,
        // and the content matches the IS-GPS-200 pagination
        let page = frame.page_number();

        match frame.subframe {
            GpsQzssLnavSubframeEnum::Almanach4(f) => {
                assert!(f.matches_page(page), "{}: page {}", description, page)
            },
            GpsQzssLnavSubframeEnum::Almanach5(f) => {
                assert!(f.matches_page(page), "{}: page {}", description, page)
            },
            _ => panic!("{}: not an almanac page", description),
        }

        // bit exact re-encoding, parity included
        let mut encoded = [0u8; GPS_LNAV_SUBFRAME_BYTES];
        frame.encode(&mut encoded).unwrap();

        assert_eq!(encoded, bytes, "{}", description);
    }
}

#[test]
fn real_almanach() {
    let (frame, _) = decode(UBLOX_ALMANACH_PAGES[0].2);
    assert_eq!(frame.page_number(), 2);

    let alm = *frame4(&frame).as_almanach().unwrap();
    assert_eq!(alm.data_id, 1);
    assert_eq!(alm.prn(), 25);
    assert_eq!(alm.toa_seconds, 589_824);
    assert_eq!(alm.sv_health, 0);
    assert!((alm.eccentricity - 0.0123).abs() < 1e-4);
    assert!((alm.sqrt_a - 5153.60).abs() < 0.01);

    // GPS orbit: inclination around 55°, period of half a sidereal day
    let inclination = alm.inclination_semicircles() * 180.0;
    assert!((inclination - 55.0).abs() < 2.0, "{}", inclination);
    assert!((alm.orbital_period_s() - 43_080.0).abs() < 60.0);

    // positions all along the orbit: GPS altitude
    for k in 0..48 {
        let tk = alm.time_from_reference_s(frame.how.tow as f64 + 900.0 * k as f64);
        let (x, y, z) = alm.position_ecef_m(tk);
        let r = (x * x + y * y + z * z).sqrt();
        assert!(r > 25_900e3 && r < 27_300e3, "r={}", r);
        assert!(z.abs() <= r * (inclination.to_radians().sin()) + 1.0);
    }

    // clock correction: a few hundred µs at most
    assert!(alm.clock_correction_s(0.0).abs() < 1e-3);

    // subframe 5 almanac
    let (frame, _) = decode(UBLOX_ALMANACH_PAGES[7].2);
    assert_eq!(frame.page_number(), 1);
    let alm = frame5(&frame).as_almanach().copied().unwrap();
    assert_eq!(alm.prn(), 1);
    assert_eq!(alm.toa_seconds, 589_824);
    assert!((alm.sqrt_a - 5153.6).abs() < 1.0);
}

#[test]
fn real_config_and_health() {
    // subframe 4 page 25
    let (frame, _) = decode(UBLOX_ALMANACH_PAGES[5].2);
    assert_eq!(frame.page_number(), 25);

    let config = *frame4(&frame).as_config_health().unwrap();

    let expected = [
        12, 9, 11, 12, 10, 11, 10, 11, 11, 11, 12, 10, 9, 12, 10, 9, 10, 12, 9, 9, 12, 9, 12, 11,
        11, 11, 11, 12, 10, 11, 10, 11,
    ];

    assert_eq!(config.configurations, expected);

    for prn in 1..=32 {
        // anti-spoofing is on, for all satellites
        assert_eq!(config.anti_spoofing(prn), Some(true));
        // Block IIR to GPS III
        let block = config.satellite_configuration(prn).unwrap();
        assert!((1..=4).contains(&block));
    }

    for prn in 25..=32 {
        assert!(config.satellite_health(prn).unwrap().is_healthy());
    }

    assert_eq!(config.reserved_word8, 0);
    assert_eq!(config.reserved_word10, 0);

    // subframe 5 page 25
    let (frame, _) = decode(UBLOX_ALMANACH_PAGES[8].2);
    assert_eq!(frame.page_number(), 25);

    let health = *frame5(&frame).as_health().unwrap();
    assert_eq!(health.toa_seconds, 589_824);
    assert_eq!(health.wna, 59);
    assert_eq!(health.reserved, 0);

    for prn in 1..=24 {
        let sv = health.satellite_health(prn).unwrap();
        assert_eq!(sv.is_healthy(), prn != 5 && prn != 21, "prn {}", prn);
        if !sv.is_healthy() {
            let value: u8 = sv.into();
            assert_eq!(value, 0x3f);
        }
    }
}

#[test]
fn real_nmct_and_reserved() {
    let (frame, _) = decode(UBLOX_ALMANACH_PAGES[1].2);
    assert_eq!(frame.page_number(), 13);

    let nmct = *frame4(&frame).as_nmct().unwrap();
    assert_eq!(nmct.availability, 2);
    assert!(!nmct.is_available());
    assert!(nmct.erd.iter().all(|erd| *erd == -22));

    for (index, sv_id, page) in [(2, 53, 14), (3, 57, 1), (4, 62, 24)] {
        let (frame, _) = decode(UBLOX_ALMANACH_PAGES[index].2);
        let reserved = *frame4(&frame).as_reserved().unwrap();
        assert_eq!(reserved.sv_id, sv_id);
        assert!(!reserved.is_dummy());
        assert_eq!(frame.page_number(), page);
    }

    // dummy page, in place of an almanac
    let (frame, _) = decode(UBLOX_ALMANACH_PAGES[6].2);
    let dummy = *frame5(&frame).as_reserved().unwrap();
    assert!(dummy.is_dummy());
    assert!(frame5(&frame).matches_page(frame.page_number()));
}

#[test]
fn real_stream() {
    init_logger();

    // the transmitted bit stream
    let mut bytes = Vec::new();

    {
        let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
        writer.write_int(0u8, 5).unwrap();

        for words in UBLOX_G25_STREAM.iter() {
            for word in on_air(*words).iter() {
                writer.write_int(*word, GPS_LNAV_WORD_BITS).unwrap();
            }
        }

        writer.write_int(0u8, 8).unwrap();
    }

    let mut decoder = GpsQzssLnavSubfDecoder::default();
    let mut frames = Vec::new();
    let mut position = 0;

    loop {
        let (consumed, frame) = decoder.decode(&bytes, position);
        position += consumed;

        match frame {
            Some(frame) => {
                frames.push(frame);
            },
            None => break,
        }
    }

    assert_eq!(frames.len(), UBLOX_G25_STREAM.len());

    // starts with EPH-1 of page 22 (TOW 455880)
    assert_eq!(frames[0].week_transmission_time_s(), 455_880);
    assert_eq!(frames[0].page_number(), 22);

    // the builder waited for page 1 (4 pages later), and collected the rest
    assert_eq!(frames[20].page_number(), 1);
    assert!(frames[20].starts_superframe_period());

    let mut pages4 = Vec::new();

    for (k, frame) in frames.iter().enumerate() {
        // subframes in sequence
        assert_eq!(frame.subframe.frame_id().encode() as usize, k % 5 + 1);
        assert_eq!(frame.week_transmission_time_s(), 455_880 + 6 * k as u32);

        let page = frame.page_number();

        // every page matches the IS-GPS-200 pagination
        match frame.subframe {
            GpsQzssLnavSubframeEnum::Almanach4(f) => {
                assert!(
                    f.matches_page(page),
                    "SF4 page {}: SV ID {}",
                    page,
                    f.sv_id()
                );
                pages4.push((page, f.sv_id()));
            },
            GpsQzssLnavSubframeEnum::Almanach5(f) => {
                assert!(
                    f.matches_page(page),
                    "SF5 page {}: SV ID {}",
                    page,
                    f.sv_id()
                );
                if let Some(alm) = f.as_almanach() {
                    assert_eq!(alm.prn(), page);
                    assert_eq!(alm.toa_seconds, 589_824);
                }
            },
            _ => {},
        }

        assert_eq!(frame.how.sbf_id, frame.subframe.frame_id());
    }

    assert!(pages4.contains(&(25, 63)));
    assert!(pages4.contains(&(13, 52)));
    assert!(pages4.contains(&(2, 25)));

    // the whole stream is re-encoded bit for bit
    let mut reencoded = Vec::new();

    {
        let mut writer = BitWriteStream::new(&mut reencoded, BigEndian);
        writer.write_int(0u8, 5).unwrap();

        for frame in frames.iter() {
            writer.write(frame).unwrap();
        }

        writer.write_int(0u8, 8).unwrap();
    }

    assert_eq!(reencoded, bytes);

    assert_eq!(
        bytes.len(),
        (5 + UBLOX_G25_STREAM.len() * GPS_LNAV_SUBFRAME_BITS + 8 + 7) / 8
    );

    assert_eq!(
        frames[3].subframe.frame_id(),
        GpsQzssLnavSubframeId::Almanach4
    );
}

fn run_isolated_alm4_test(fullpath: &str, items: usize) {
    let mut count = 0;

    let mut decoder = GpsQzssLnavSbfBinDecoder::new(fullpath);
    while let Some(subf4) = decoder.parse_next() {
        assert_eq!(
            subf4.how.sbf_id,
            GpsQzssLnavSubframeId::Almanach4,
            "found invalid SUBF"
        );
        let _ = subf4
            .subframe
            .as_alm4()
            .unwrap_or_else(|| panic!("found invalid SUBF"));
        count += 1;
    }
    assert_eq!(count, items);
}

fn run_isolated_alm5_test(fullpath: &str, items: usize) {
    let mut count = 0;

    let mut decoder = GpsQzssLnavSbfBinDecoder::new(fullpath);
    while let Some(subf5) = decoder.parse_next() {
        assert_eq!(
            subf5.how.sbf_id,
            GpsQzssLnavSubframeId::Almanach5,
            "found invalid SUBF"
        );
        let _ = subf5
            .subframe
            .as_alm5()
            .unwrap_or_else(|| panic!("found invalid SUBF"));
        count += 1;
    }
    assert_eq!(count, items);
}

#[test]
fn parse_reconstructed_g15_unsync_subf4() {
    run_isolated_alm4_test("data/GPS/UNSYNC/ESBC00DNK_G15_alm4.bin", 2398);
}
#[test]
fn parse_reconstructed_g15_sync_subf4() {
    run_isolated_alm4_test("data/GPS/SYNC/ESBC00DNK_G15_alm4.bin", 2398);
}
#[test]
fn parse_reconstructed_g15_unsync_subf5() {
    run_isolated_alm5_test("data/GPS/UNSYNC/ESBC00DNK_G15_alm5.bin", 2398);
}
#[test]
fn parse_reconstructed_g15_sync_subf5() {
    run_isolated_alm5_test("data/GPS/SYNC/ESBC00DNK_G15_alm5.bin", 2398);
}
