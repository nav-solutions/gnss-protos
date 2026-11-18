# GNSS-Protos

[![Rust](https://github.com/nav-solutions/gnss-protos/actions/workflows/rust.yml/badge.svg)](https://github.com/nav-solutions/gnss-protos/actions/workflows/rust.yml)
[![Rust](https://github.com/nav-solutions/gnss-protos/actions/workflows/daily.yml/badge.svg)](https://github.com/nav-solutions/gnss-protos/actions/workflows/daily.yml) 
[![crates.io](https://img.shields.io/crates/v/gnss-protos.svg)](https://crates.io/crates/gnss-protos) 
[![crates.io](https://docs.rs/gnss-protos/badge.svg)](https://docs.rs/gnss-protos/badge.svg)

[![MRSV](https://img.shields.io/badge/MSRV-1.72.0-orange?style=for-the-badge)](https://github.com/rust-lang/rust/releases/tag/1.72.0)
[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/nav-solutions/gnss-protos/blob/main/LICENSE)

GNSS-Protos
===========

This framework aims at providing an easy to use yet complete and efficient
GNSS protocol encoding & decoding.

You can use this framework to learn and go deeper into each protocol, as we strive to write
a compelling documentation ([refer to the online API](https://docs.rs/gnss-protos)).

The framework currently supports GPS / QZSS (unlocked on `gps` option) and Galileo
(unlocked on `galileo` option). We hope to unlock BDS very soon !

Supported protocols
===================

- GPS / QZSS protocol (on `gps` crate feature), note that this feature is activated by default.
- Galileo I/NAV protocol, E1-B and E5b-I (on `galileo` crate feature), note that this feature is activated by default.

Encoding & decoding
===================

Every message implements the `Message` trait, which builds on the [bitbuffer](https://crates.io/crates/bitbuffer)
`BitRead` / `BitWrite` traits. This is the one and only way to encode and decode messages:

- `Message::encode` / `Message::decode` work from / to a byte buffer, starting on its first bit.
- `BitWriteStream::write` / `BitReadStream::read` (re-exported from `bitbuffer`) work at any bit position of a stream.
Messages written one after the other form a valid bit (or symbol) stream, ready for transmission.

The library is strictly reciprocal: decoding then re-encoding any valid message gives back
the very same bits. Equality between messages is "bit true": two messages are equal when they encode to the same bits.

GPS / QZSS
==========

The `gps` compilation option (enabled by default) activates support
for the GPS (US) and QZSS (Jap) protocol, in the `gps` module.

The framework interprets the EPH-1, 2 and 3 subframes, which is enough for real-time applications,
as demonstrated by our [rt-navi (Real Time navigation)](https://github.com/nav-solutions/rt-navi) appplication.
Subframes 4 and 5 (almanac and status pages) are decoded and encoded as raw data bits,
their interpretation is work in progress.

GPS is not aligned to bytes: a subframe is 300 bits long (10 words of 30 bits),
each word carrying 24 data bits and 6 parity bits.

- `GpsQzssFrame` is the only message that deals with the transmitted bits: parity (IS-GPS-200 §20.3.5.2),
data inversion (D30*), bits 23-24 of words 2 and 10, inverted streams (180° carrier phase ambiguity).
Parity is always computed when encoding, and always verified when decoding.
- messages (`GpsQzssTelemetry`, `GpsQzssHow`, `GpsQzssFrame1`..) are read and written on the
240 parity free data bits. Parameters that the ICD splits over two words (M0, e, sqrt(A)...) are contiguous there.

1. The `GpsQzssDecoder` is the solution when working with GPS/QZSS bit streams.
It synchronizes itself on the preamble at any bit position (normal or inverted),
verifies the parity and interprets the frame. It can be fed one bit at a time (`push_bit`, typically
from a tracking loop) or from a byte slice at any bit offset (`decode`). The decoder state is preserved
between calls, so a frame may span several buffers. Positions are expressed in bits, not bytes.

```rust
use gnss_protos::{
    BigEndian, BitWriteStream, GpsQzssDecoder, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow,
    GPS_FRAME_BITS,
};

// Stream 3 frames, contiguously, 5 bits after the start of the buffer
let frame = GpsQzssFrame::default()
    .with_hand_over_word(GpsQzssHow::default().with_tow_seconds(259_956));

let mut buffer = Vec::new();
let mut writer = BitWriteStream::new(&mut buffer, BigEndian);

writer.write_int(0u8, 5).unwrap();

for _ in 0..3 {
    writer.write(&frame).unwrap();
}

// The decoder verifies the parity of each frame by default
let mut decoder = GpsQzssDecoder::default();
let mut position = 0; // in bits!
let mut count = 0;

loop {
    let (consumed, decoded) = decoder.decode(&buffer, position);
    position += consumed;

    match decoded {
        Some(decoded) => {
            // the frame started at position - GPS_FRAME_BITS
            assert_eq!(position - GPS_FRAME_BITS, 5 + count * GPS_FRAME_BITS);
            assert_eq!(decoded, frame);
            assert_eq!(decoded.how.tow, 259_956);
            assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris1);
            count += 1;
        },
        None => break, // buffer exhausted
    }
}

assert_eq!(count, 3);
```

2. `GpsQzssFrame` implements `Message`: it is decoded from / encoded to its 300 transmitted bits (38 bytes,
4 padding bits).

```rust
use gnss_protos::{GpsQzssFrame, GpsQzssFrameId, Message};

// Real EPH-1 subframe, as transmitted (300 bits, parity included)
let bytes = [
    0x8B, 0x04, 0xF8, 0x6E, 0xAD, 0x83, 0x68, 0xC4, 0xF9, 0x00, 0x01, 0x10, 0x4F,
    0x5D, 0x31, 0xA2, 0xEC, 0x64, 0xA0, 0x77, 0x55, 0x78, 0x33, 0x3C, 0xDF, 0xD2,
    0x92, 0x50, 0x42, 0xA1, 0xFF, 0xFF, 0xA5, 0xEC, 0xED, 0x3C, 0xFC, 0xC0,
];

let frame = GpsQzssFrame::decode(&bytes)
    .unwrap();

assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris1);
assert_eq!(frame.how.tow, 259_956);

let eph1 = frame.subframe.as_eph1().unwrap();
assert_eq!(eph1.week, 318);
assert_eq!(eph1.toc, 266_400);

// bit exact re-encoding (parity included)
let mut encoded = [0u8; 38];
frame.encode(&mut encoded).unwrap();
assert_eq!(encoded, bytes);
```

Galileo
=======

The `galileo` compilation option (enabled by default) activates support
for the Galileo I/NAV protocol (E1-B and E5b-I), following the Galileo OS SIS ICD v2.1, in the `galileo` module.

The framework interprets the word types 0 to 6 (ephemeris, clock, ionosphere, BGD, health, GST and GST-UTC),
which is enough for real-time applications. Other word types (almanacs, reduced CED, FEC2, ARAIM..)
are decoded and encoded as raw data bits, their interpretation is work in progress.

Galileo is a symbol based protocol. An I/NAV page is transmitted in 2 seconds, as two page parts
(even, then odd) of 250 symbols each: a 10-symbol synchronization pattern followed by 120 bits
encoded by a rate 1/2 convolutional code (FEC) and interleaved. A CRC-24Q protects the page.

- `GalileoInavPage` is the only message that deals with the transmitted symbols: synchronization pattern,
convolutional encoding and Viterbi decoding, block interleaving, CRC-24Q, inverted streams (180° carrier phase ambiguity).
It carries one 128-bit `GalileoInavWord` and the OSNMA, SAR and SSP fields.
- `GalileoInavWord` is the navigation data, interpreted according to its word type (`GalileoInavWord1`..)

1. The `GalileoInavDecoder` is the solution when working with Galileo symbol streams.
It synchronizes itself on the synchronization pattern at any symbol position (normal or inverted),
corrects symbol errors (hard decision Viterbi decoder), pairs the even and odd page parts and verifies the CRC.
It can be fed one symbol at a time (`push_symbol`, typically from a tracking loop) or from a byte slice
at any symbol offset (`decode`). Positions are expressed in symbols, not bytes.

```rust
use gnss_protos::{
    BigEndian, BitWriteStream, GalileoInavDecoder, GalileoInavPage, GalileoInavWord,
    GalileoInavWord4, GALILEO_INAV_PAGE_SYMBOLS,
};

// Stream 3 pages, contiguously, 7 symbols after the start of the buffer
let page = GalileoInavPage::default()
    .with_word(GalileoInavWord::Ephemeris4(
        GalileoInavWord4::default()
            .with_svid(8)
            .with_toc_seconds(454_800)
            .with_clock_offset_seconds(-4.5e-4),
    ));

let mut buffer = Vec::new();
let mut writer = BitWriteStream::new(&mut buffer, BigEndian);

writer.write_int(0u8, 7).unwrap();

for _ in 0..3 {
    writer.write(&page).unwrap();
}

// some symbol errors: corrected by the Viterbi decoder
buffer[10] ^= 0x01;
buffer[100] ^= 0x10;

let mut decoder = GalileoInavDecoder::default();
let mut position = 0; // in symbols!
let mut count = 0;

loop {
    let (consumed, decoded) = decoder.decode(&buffer, position);
    position += consumed;

    match decoded {
        Some(decoded) => {
            // the page started at position - GALILEO_INAV_PAGE_SYMBOLS
            assert_eq!(position - GALILEO_INAV_PAGE_SYMBOLS, 7 + count * GALILEO_INAV_PAGE_SYMBOLS);
            assert_eq!(decoded, page);
            count += 1;
        },
        None => break, // buffer exhausted
    }
}

assert_eq!(count, 3);
```

2. `GalileoInavPage` implements `Message`: it is decoded from / encoded to its 500 transmitted symbols
(63 bytes, 4 padding bits).

```rust
use gnss_protos::{GalileoInavPage, Message};

// Real E08 page (word type 4), as transmitted (500 symbols)
let symbols = [
    0x58, 0x0B, 0x0D, 0xA5, 0x47, 0xCB, 0xFB, 0xA3, 0x45, 0x51, 0x4A, 0xDD, 0x5B, 0x9A, 0xB6, 0x1C,
    0xD2, 0x89, 0x40, 0x57, 0xC1, 0xBE, 0x9D, 0x69, 0x61, 0x85, 0x51, 0xCB, 0xFF, 0xAB, 0xDC, 0x56,
    0x0D, 0x01, 0xFF, 0x2F, 0x07, 0x31, 0xC3, 0x13, 0xBC, 0x01, 0x03, 0x7B, 0x5A, 0x8F, 0xD5, 0x0B,
    0xF1, 0x4F, 0x7F, 0xCB, 0x67, 0x41, 0x27, 0x8E, 0xBC, 0x09, 0xB4, 0x41, 0xFF, 0xEA, 0xC0,
];

let page = GalileoInavPage::decode(&symbols)
    .unwrap();

let eph4 = page.word.as_word4().unwrap();
assert_eq!(eph4.svid, 8);
assert_eq!(eph4.iodnav, 123);
assert_eq!(eph4.toc, 454_800);
assert!((eph4.af0 - -4.54849563539E-04).abs() < 1E-14);

// bit exact re-encoding (FEC and CRC included)
let mut encoded = [0u8; 63];
page.encode(&mut encoded).unwrap();
assert_eq!(encoded, symbols);
```

License
=======

This library is part of the [Nav-solutions framework](https://github.com/nav-solutions) and is
licensed under [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
