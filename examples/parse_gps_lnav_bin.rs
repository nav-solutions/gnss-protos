//! Parses a GPS/QZSS LNAV stream from either a binary
//! or local readable socket, and simply logs each frame.
//! This can be used as a quick bitstream parser.

use gnss_protos::gps::lnav::{GpsQzssLnavSubfDecoder, GPS_LNAV_SUBFRAME_BYTES};

use log::info;

use clap::Parser;

use std::{
    fs::File,
    io::{self, BufRead, BufReader},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    /// Possible local file, missing = stdin
    #[arg(short, long, help = "Local file or defaults to stdin")]
    input: Option<PathBuf>,

    /// Remove parity check
    #[arg(short, long, help = "Turn off parity verification")]
    no_parity: bool,
}

fn open_input(path: Option<PathBuf>) -> io::Result<Box<dyn BufRead>> {
    Ok(match path {
        Some(p) => Box::new(BufReader::new(File::open(p)?)),
        None => Box::new(io::stdin().lock()),
    })
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();

    let mut buf = [0u8; 2 * GPS_LNAV_SUBFRAME_BYTES];

    let args = Args::parse();

    let mut decoder = if args.no_parity {
        GpsQzssLnavSubfDecoder::default().without_parity_verification()
    } else {
        GpsQzssLnavSubfDecoder::default()
    };

    let mut input = open_input(args.input).unwrap_or_else(|e| {
        panic!("failed to open input stream: {}", e);
    });

    loop {
        match input.read_exact(&mut buf) {
            Ok(_) => {
                let mut pos = 0;
                let mut size = buf.len();
                while size >= GPS_LNAV_SUBFRAME_BYTES {
                    let (consumed, subf) = decoder.decode(&buf, pos);
                    pos += consumed;
                    size -= consumed;
                    match subf {
                        Some(subf) => info!("{:?}", subf),
                        _ => {
                            break;
                        },
                    }
                }
            },
            Err(_e) => {
                info!("end of stream");
                return;
            },
        }
    }
}
