# GNSS-Protos

[![Rust](https://github.com/nav-solutions/gnss-protos/actions/workflows/rust.yml/badge.svg)](https://github.com/nav-solutions/gnss-protos/actions/workflows/rust.yml)
[![Rust](https://github.com/nav-solutions/gnss-protos/actions/workflows/daily.yml/badge.svg)](https://github.com/nav-solutions/gnss-protos/actions/workflows/daily.yml) 
[![crates.io](https://img.shields.io/crates/v/gnss-protos.svg)](https://crates.io/crates/gnss-protos) 
[![crates.io](https://docs.rs/gnss-protos/badge.svg)](https://docs.rs/gnss-protos/badge.svg)

[![MRSV](https://img.shields.io/badge/MSRV-1.72.0-orange?style=for-the-badge)](https://github.com/rust-lang/rust/releases/tag/1.72.0)
[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/nav-solutions/gnss-protos/blob/main/LICENSE)

GNSS-Protos
===========

The idea behind this library is to offer an easy to use, efficient and regrouped
framework to encode & decode GNSS protocols.

You can use this API to learn and go deeper into each protocol, as we strive to document
each data bit correctly (refer to the online API).

All protocols are gated under a library feature, so you can narrow it down to your use case.

Supported protocols
===================

- GPS / QZSS protocol available on `gps` crate feature, activated by default.

GPS (US) / QZSS (Jap) protocol
==============================

The `gps` library feature activates support of GPS/QZSS protocol.

Currently we support Ephemeris frames 1, 2 and 3, which is sufficient for real-time
navigation. Protocol parity is not fully implemented yet.

We provide methods to both encode and decode data frames, and methods
to work from a single byte or a buffer which is most suited for real-time interfaces.

The parser is flexible and efficient enough. It supports both frame encoding & decoding.
The GPS/QZSS protocol being "thoughtout" and redacted in the 70s/80s, it is not convenient to deal with.
The data stream is not aligned to 8bit, hence not compatible with any machine.
To work around this, this framework proposes two different interfaces that should suite all use cases:

1. The `GpsDecoder.parse_buffer(&[u8])` works from a stream of bytes and is the goto method
when working with a real time GPS/QZSS decoder. It will automatically lock to the first valid GPS message found in your buffer,
which is not [u8] aligned. But you have to carefully mange your buffer.
The methods will return the number of processed _bits_ (not bytes!) and the possibly identified GPS message.
You must discard all processed _bits_ (not bytes) not to process the same GPS message twice. If you happen to discard
this amount of bytes (not bits) you will automatically loose data. 
If you follow this principle, you can decode successive GPS messages and not loose any data from your streamer.

```rust
use gnss_protos::gps::GpsDecoder;

let mut decoder = GpsDecoder::default()
    .without_parity_verification(); // until further notice
```

2. The `GpsDecoder.consume_byte()` let's you process one byte at a time. For each byte, you can 
describe whether this byte contains padding (which is most likely going to happen at some point in your stream)
or not (pure data bits or noise). We used this approach to interface correctly to U-Blox receivers for example
(that pad and align the frames internally).

```rust
use gnss_protos::gps::GpsDecoder;

let mut decoder = GpsDecoder::default()
    .without_parity_verification(); // until further notice
```

License
=======

This library is part of the [Nav-solutions framework](https://github.com/nav-solutions) and is
licensed under [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
