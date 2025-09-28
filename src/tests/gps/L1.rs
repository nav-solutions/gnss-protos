use std::{fs::File, io::Read};

use crate::{gps::GpsQzssDecoder, tests::init_logger};

#[test]
fn l1_iq4mega() {
    init_logger();
    let file = "data/GPS/iq-4MHz.bin";
    let mut decoder = GpsQzssDecoder::default();

    let mut fd = File::open(file).unwrap_or_else(|e| {
        panic!("Failed to read {}: {}", file, e);
    });

    let mut buffer = [0; 8192];

    loop {
        match fd.read(&mut buffer) {
            Ok(0) => {
                return;
            },
            Ok(size) => {
                let mut ptr = 0;

                // consume everything
                while ptr < size - 128 {
                    let (processed, decoded) = decoder.decode(&buffer[ptr..], size - ptr);
                    ptr += processed / 8;
                    println!("ptr={} {:?}", ptr, decoded);
                }
            },
            Err(e) => {},
        }
    }
}

#[test]
fn l1_i12mega() {
    init_logger();
    let file = "data/GPS/i-12MHz.bin";
    let mut decoder = GpsQzssDecoder::default();

    let mut fd = File::open(file).unwrap_or_else(|e| {
        panic!("Failed to read {}: {}", file, e);
    });

    let mut buffer = [0; 8192];

    loop {
        match fd.read(&mut buffer) {
            Ok(0) => {
                return;
            },
            Ok(size) => {
                let mut ptr = 0;

                // consume everything
                while ptr < size - 128 {
                    let (processed, decoded) = decoder.decode(&buffer[ptr..], size - ptr);
                    ptr += processed / 8;
                    println!("ptr={} {:?}", ptr, decoded);
                }
            },
            Err(e) => {},
        }
    }
}

#[test]
fn l1_i24mega() {
    init_logger();
    let file = "data/GPS/i-24MHz.bin";
    let mut decoder = GpsQzssDecoder::default();

    let mut fd = File::open(file).unwrap_or_else(|e| {
        panic!("Failed to read {}: {}", file, e);
    });

    let mut buffer = [0; 8192];

    loop {
        match fd.read(&mut buffer) {
            Ok(0) => {
                return;
            },
            Ok(size) => {
                let mut ptr = 0;

                // consume everything
                while ptr < size - 128 {
                    let (processed, decoded) = decoder.decode(&buffer[ptr..], size - ptr);
                    ptr += processed / 8;
                    println!("ptr={} {:?}", ptr, decoded);
                }
            },
            Err(e) => {},
        }
    }
}

#[test]
fn l1_gpssim() {
    let file = "data/GPS/gpssim.bin";
    let mut decoder = GpsQzssDecoder::default();

    let mut fd = File::open(file).unwrap_or_else(|e| {
        panic!("Failed to read {}: {}", file, e);
    });

    let mut buffer = [0; 8192];

    loop {
        match fd.read(&mut buffer) {
            Ok(0) => {
                return;
            },
            Ok(size) => {
                let mut ptr = 0;

                // consume everything
                loop {
                    if ptr >= size {
                        break;
                    }

                    let (processed, decoded) = decoder.decode(&buffer[ptr..], size);

                    ptr += 8;

                    if let Some(decoded) = decoded {
                        println!("{:#?}", decoded);
                    }
                }
            },
            Err(e) => {},
        }
    }
}
