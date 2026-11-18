/// Parses the EPH-1.2.3 burst extract (3/5 30s) from RINEX v3 files
use log::info;

use std::{fs::File, io::Read};

use crate::{
    gps::{
        tests::assert_valid_subframe, GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrameId,
        GpsQzssHow, GpsQzssSatelliteHealth, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS,
    },
    tests::init_logger,
    BigEndian, Message,
};

fn run_file(path: &str) -> u32 {
    let mut frames = 0;
    let mut position = 0;
    let mut buffer = [0u8; 8192];
    let mut fd = File::open(path).unwrap();
    
    let mut size = fd.read(&mut buffer).unwrap_or_else(|e| {
        panic!("Failed to read file {}: {}", path, e);
    });

    let mut decoder = GpsQzssDecoder::default();

    while size > 0 {
        let (consumed, frame) = decoder.decode(&buffer, position);

        match frame {
            Some(frame) => {
                frames += 1;
                info!("Decoded frame: {:?}", frame);
            },
            _ => {},
        }

        position += consumed;
        size = size.saturating_sub(consumed / 8);
    }
    frames
}

#[test]
fn test_gps_lnav_eph123() {
    init_logger();
    assert_eq!(run_file("data/GPS/esbcdnk_g15_eph123.bin"), 24);
    assert_eq!(run_file("data/GPS/esbcdnk_g24_eph123.bin"), 21);
    assert_eq!(run_file("data/GPS/esbcdnk_g31_eph123.bin"), 27);
}

#[test]
fn test_qzss_lnav_eph123() {
    init_logger();
    assert_eq!(run_file("data/GPS/esbcdnk_j01_eph123.bin"), 12);
    assert_eq!(run_file("data/GPS/esbcdnk_j02_eph123.bin"), 18);
    assert_eq!(run_file("data/GPS/esbcdnk_j03_eph123.bin"), 15);
}
