//! GPS/QZSS 30s scale tests

use crate::{
    gps::lnav::{
        test_vectors::{on_air, UBLOX_G25_STREAM},
        // GpsQzssLnavSubframe,
        // GpsQzssLnavSubframeId,
        // GpsQzssLnavFrame,
        GpsQzssLnavFrameBuilder,
        GpsQzssLnavSubfDecoder,
        GPS_LNAV_FRAME_ITEMS,
        GPS_LNAV_WORD_BITS,
    },
    tests::init_logger,
    BigEndian, BitWriteStream,
};

use log::debug;

// UBLOX G25
#[test]
fn ublox_g25_stream() {
    init_logger();

    let mut bytes = Vec::new();

    {
        let mut w = BitWriteStream::new(&mut bytes, BigEndian);
        // creates random offset
        w.write_int(0x0, 5).unwrap();

        for words in UBLOX_G25_STREAM.iter() {
            for word in on_air(*words).iter() {
                w.write_int(*word, GPS_LNAV_WORD_BITS).unwrap();
            }
        }

        // creates random 0 insertions
        w.write_int(0, 8).unwrap();
    }

    let mut pos = 0;
    let mut decoder = GpsQzssLnavSubfDecoder::default();
    let mut builder = GpsQzssLnavFrameBuilder::default();

    let mut subframes = 0;
    let mut frames = 0;

    loop {
        let (consumed, sbf) = decoder.decode(&bytes, pos);
        pos += consumed;

        match sbf {
            Some(sbf) => {
                debug!("parsed {:?}", sbf);
                subframes += 1;
                match builder.push_sbf(sbf) {
                    Some(frame) => {
                        frames += 1;
                        debug!("collected {:?}", sbf);
                        assert_eq!(subframes, frames * GPS_LNAV_FRAME_ITEMS + 0);
                    },
                    None => {},
                }
            },
            None => break,
        }
    }
    assert_eq!(frames, 18);
    assert_eq!(subframes, 18 * GPS_LNAV_FRAME_ITEMS + 4);
}
