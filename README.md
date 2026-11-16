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

GPS / QZSS
==========

The `gps` compilation option (enabled by default) activates support
for the GPS (US) and QZSS (Jap) protocol.

The framework interprets the EPH-1, 2 and 3 subframes, which is enough for real-time applications,
as demonstrated by our [rt-navi (Real Time navigation)](https://github.com/nav-solutions/rt-navi) appplication.
Subframes 4 and 5 (almanac and status pages) are decoded and encoded as raw data bits,
their interpretation is work in progress.

GPS is not aligned to bytes: a subframe is 300 bits long (10 words of 30 bits),
each word carrying 24 data bits and 6 parity bits. The library is organized in two layers:

- the `gps::link` layer is the only place that deals with the transmitted bits: parity (IS-GPS-200 §20.3.5.2),
data inversion (D30*), bits 23-24 of words 2 and 10, inverted streams (180° carrier phase ambiguity).
It converts 10 words of 30 bits into 240 contiguous, parity free, data bits (and back).
- messages (`GpsQzssTelemetry`, `GpsQzssHow`, `GpsQzssFrame1`..) are read and written on those
240 data bits. Parameters that the ICD splits over two words (M0, e, sqrt(A)...) are contiguous there.

1. The `GpsQzssDecoder` is the solution when working with GPS/QZSS bit streams.
It synchronizes itself on the preamble at any bit position (normal or inverted),
verifies the parity and interprets the frame. It can be fed one bit at a time (`push_bit`, typically
from a tracking loop) or from a byte slice at any bit offset (`decode`). The decoder state is preserved
between calls, so a frame may span several buffers. Positions are expressed in bits, not bytes.

```rust
use gnss_protos::{GpsQzssDecoder, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GPS_FRAME_BITS};

// Encode 3 frames, contiguously, 5 bits after the start of the buffer
let frame = GpsQzssFrame::default()
    .with_hand_over_word(GpsQzssHow::default().with_tow_seconds(259_956));

let mut buffer = [0u8; 128];

for i in 0..3 {
    frame.encode_at(&mut buffer, 5 + i * GPS_FRAME_BITS)
        .unwrap();
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
            assert_eq!(decoded.how.tow, 259_956);
            assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris1);
            count += 1;
        },
        None => break, // buffer exhausted
    }
}

assert_eq!(count, 3);
```

2. `GpsQzssFrame` can be decoded from / encoded to 10 words of 30 bits (`decode_words`, `encode_words`),
from / to any bit position of a buffer (`decode_at`, `encode_at`), or using the `bitbuffer`
`BitRead` / `BitWrite` traits. Some receivers report words whose polarity was already corrected
(u-blox UBX-RXM-SFRBX for example): use `decode_polarity_corrected_words` for those.

```rust
use gnss_protos::{GpsQzssFrame, GpsQzssFrameId};

// EPH-1 subframe, as reported by a u-blox receiver (UBX-RXM-SFRBX)
let words = [
    0x22C13E1B, 0x1527C973, 0x13E40004, 0x104F5D31, 0x1744E6D7,
    0x07755783, 0x330C80B5, 0x125042A1, 0x00001684, 0x312C3033,
];

let frame = GpsQzssFrame::decode_polarity_corrected_words(&words, true)
    .unwrap();

assert_eq!(frame.how.frame_id, GpsQzssFrameId::Ephemeris1);
assert_eq!(frame.how.tow, 259_956);

let eph1 = frame.subframe.as_eph1().unwrap();
assert_eq!(eph1.week, 318);
assert_eq!(eph1.toc, 266_400);
```

Galileo
=======

The `galileo` compilation option (enabled by default) activates support
for the Galileo I/NAV protocol (E1-B and E5b-I), following the Galileo OS SIS ICD v2.1.

The framework interprets the word types 0 to 6 (ephemeris, clock, ionosphere, BGD, health, GST and GST-UTC),
which is enough for real-time applications. Other word types (almanacs, reduced CED, FEC2, ARAIM..)
are decoded and encoded as raw data bits, their interpretation is work in progress.

Galileo is a symbol based protocol. An I/NAV page is transmitted in 2 seconds, as two page parts
(even, then odd) of 250 symbols each: a 10-symbol synchronization pattern followed by 120 bits
encoded by a rate 1/2 convolutional code (FEC) and interleaved. A CRC-24Q protects the page.
The library is organized in three layers:

- the `galileo::link` layer is the only place that deals with the transmitted symbols: synchronization pattern,
convolutional encoding and Viterbi decoding, block interleaving, CRC-24Q, inverted streams (180° carrier phase ambiguity).
It converts 250 symbols into a 120-bit page part (and back).
- `GalileoInavPage` gathers two page parts and verifies the CRC. It carries one 128-bit `GalileoInavWord`
and the OSNMA, SAR and SSP fields.
- `GalileoInavWord` is the navigation data, interpreted according to its word type (`GalileoInavWord1`..)

1. The `GalileoInavDecoder` is the solution when working with Galileo symbol streams.
It synchronizes itself on the synchronization pattern at any symbol position (normal or inverted),
corrects symbol errors (hard decision Viterbi decoder), pairs the even and odd page parts and verifies the CRC.
It can be fed one symbol at a time (`push_symbol`, typically from a tracking loop) or from a byte slice
at any symbol offset (`decode`). Positions are expressed in symbols, not bytes.

```rust
use gnss_protos::{
    GalileoInavDecoder, GalileoInavPage, GalileoInavWord, GalileoInavWord4,
    GALILEO_INAV_PAGE_SYMBOLS,
};

// Encode 3 pages, contiguously, 7 symbols after the start of the buffer
let page = GalileoInavPage::default()
    .with_word(GalileoInavWord::Ephemeris4(
        GalileoInavWord4::default()
            .with_svid(8)
            .with_toc_seconds(454_800)
            .with_clock_offset_seconds(-4.5e-4),
    ));

let mut buffer = [0u8; 256];

for i in 0..3 {
    page.encode_at(&mut buffer, 7 + i * GALILEO_INAV_PAGE_SYMBOLS)
        .unwrap();
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

2. `GalileoInavPage` can be decoded from / encoded to its two 120-bit page parts (`decode_parts`, `encode_parts`),
as most receivers report them after FEC decoding (u-blox UBX-RXM-SFRBX: `decode_ublox_words`, `encode_ublox_words`),
from / to any symbol position of a buffer (`decode_at`, `encode_at`), or using the `bitbuffer`
`BitRead` / `BitWrite` traits.

```rust
use gnss_protos::GalileoInavPage;

// E08 word type 4, as reported by a u-blox receiver (UBX-RXM-SFRBX)
let words = [
    0x041EC800, 0x07000F76, 0x73FC461C, 0x87FF8000,
    0x8F804986, 0x6BD3872A, 0xAAAA423B, 0x4ECAC000,
];

let page = GalileoInavPage::decode_ublox_words(&words)
    .unwrap();

let eph4 = page.word.as_word4().unwrap();
assert_eq!(eph4.svid, 8);
assert_eq!(eph4.iodnav, 123);
assert_eq!(eph4.toc, 454_800);
assert!((eph4.af0 - -4.54849563539E-04).abs() < 1E-14);

// bit exact re-encoding (CRC included)
assert_eq!(page.encode_ublox_words().unwrap(), words);
```

License
=======

This library is part of the [Nav-solutions framework](https://github.com/nav-solutions) and is
licensed under [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
