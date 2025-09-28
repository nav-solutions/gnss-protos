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

The framework is currently limited to GPS / QZSS (unlocked on `gps` option) but we
hope to unlock Galileo and BDS very soon !

Supported protocols
===================

- GPS / QZSS protocol (on `gps` crate feature), note that this feature is activated by default.

GPS / QZSS
==========

The `gps` compilation option (enabled by default) activates support
for the GPS (US) and QZSS (Jap) protocol.

The framework is currently limited to EPH-1, 2 and 3 frames, which is enough for real-time applications,
as demonstrated by our [rt-navi (Real Time navigation)](https://github.com/nav-solutions/rt-navi) appplication.

We hope to unlock the Almanach and status frames soon.

We provide methods to both encode and decode GPS frames, and methods
(1) that may apply to raw stream (like in typical real-time decoding working with a stream of bits),
or bytes that were inveavitably aligned to bytes (GPS is not aligned to bytes):

1. The `GpsQzssDecoder` is the solution when working with real-time GPS/QZSS streams.
It will synchronize itself to the synchronization bit and start the decoding process.
When working with a stream of bits, you need to think in terms of bits not bytes.
Each frame is 300-bit long, the API returns the number of processed and encoded bits (not like
in standard `std::io` interfaces), which allows you to create an efficient interface that can
operate on every single bit. But obviously, this will require accurate and correct buffer management.

In this example, we simulate the real-time decoder example, and we apply a "lazy" buffer management technique,
where we remove all the entire bytes that were processed. Because the frame is 37.5 byte long, 0.5 bytes
get processed twice the next time we decode the next frame, so it's a little bit "inefficient", but still
very simple buffer management.

```rust
use gnss_protos::GpsQzssDecoder;

let mut buffer = [0; 2048]; 

// 128 messages example, as provided by a real-time decoder
let mut file = File::open("data/GPS/burst.bin")
    .unwrap();

// Grab a few frames
let size = file.read(&mut buffer)
    .unwrap();

// The decoder verifies the parity of each frame by default,
// but you can easily turn that off if you need to.
let mut decoder = GpsQzssDecoder::default();

let (processed_size, decoded) = decoder.decode(&buffer, size);

assert_eq!(processed_size, 300); // !!bits not bytes!!

let decoded = decoded
    .unwrap(); // we have 128 frames

let subf = subf.subframe.as_eph1()
    .unwrap(); // our burst pattern is EPH-1..2..3

assert_eq!(decoded.telemetry.message, 0x1234); // test pattern
assert_eq!(decoded.telemetry.integriry, true); // test pattern

// lazy management
let ptr = processed_size / 8 -1; // !bytes! -1 so we do not lose data

let (processed_size, decoded) = decoder.decode(&buffer[ptr..], size);

assert_eq!(processed_size, 316); // !!bits not bytes!!

let decoded = decoded
    .unwrap(); // we have 128 frames

let subf = subf.subframe.as_eph2()
    .unwrap(); // our burst pattern is EPH-1..2..3

assert_eq!(decoded.telemetry.message, 0x1234); // test pattern
assert_eq!(decoded.telemetry.integriry, true); // test pattern
```

2. `GpsQzssFrame` supports a `decode()` that works with a possibly padded Byte.
This is the prefered option when working with a stream that was already manipulated by a machine
and therefore, re-aligned to bytes. Each byte may have padding (or not). The stream must
start with the sync byte.
We used this approach to interface correctly to U-Blox receivers for example
(that pad and align the frames internally).

```rust
use gnss_protos::GpsQzssDecoder;

// TODO
```

License
=======

This library is part of the [Nav-solutions framework](https://github.com/nav-solutions) and is
licensed under [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
