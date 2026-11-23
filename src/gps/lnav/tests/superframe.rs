//! GPS/QZSS 12'30" scale tests, on recorded data

use crate::{
    gps::lnav::{
        GpsQzssLnavSubfDecoder, GpsQzssLnavSuperframeBuilder, GpsQzssLnavSuperframeDecoder,
    },
    tests::init_logger,
};

use log::debug;

const G15_BIN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/data/GPS/esbcdnk_g15.bin");

#[test]
#[ignore]
fn g15_bin_superframes() {
    // this recording only contains EPH-1 subframes (one every 6s):
    // two of them are aligned to a period start (TOW=15000 and 15750), and no
    // 12'30" period may ever complete (no false positive).
    init_logger();

    let bytes = std::fs::read(G15_BIN).unwrap();

    // subframe level: every period found by the builder is valid
    let mut decoder = GpsQzssLnavSubfDecoder::default();
    let mut builder = GpsQzssLnavSuperframeBuilder::default();

    let mut position = 0;
    let mut subframes = 0;
    let mut starts = 0;
    let mut collected = Vec::new();

    loop {
        let (consumed, sbf) = decoder.decode(&bytes, position);
        position += consumed;

        match sbf {
            Some(sbf) => {
                subframes += 1;

                if sbf.starts_superframe_period() {
                    starts += 1;
                }

                if let Some(spf) = builder.push_sbf(sbf) {
                    debug!(
                        "12'30\" period starting at TOW={}",
                        spf.week_transmission_time_s()
                    );
                    collected.push(spf);
                }
            },
            None => break,
        }
    }

    debug!(
        "{} subframes, {} period starts, {} complete periods",
        subframes,
        starts,
        collected.len()
    );

    assert_eq!(subframes, 1810);
    assert_eq!(starts, 2);
    assert!(collected.is_empty());

    for spf in collected.iter() {
        spf.validate().unwrap();
        assert_eq!(spf.week_transmission_time_s() % 750, 0);
    }

    // the superframe decoder gives the very same result
    let mut decoder = GpsQzssLnavSuperframeDecoder::default();
    let mut position = 0;
    let mut decoded = Vec::new();

    loop {
        let (consumed, spf) = decoder.decode(&bytes, position);
        position += consumed;
        match spf {
            Some(spf) => decoded.push(spf),
            None => break,
        }
    }

    assert_eq!(decoded, collected);
}
