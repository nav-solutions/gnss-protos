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
The GPS/QZSS protocol is not ideal to handle because it is not aligned to 32-bits, which is incompatible with any machine.
To work around this (design flaw), this framework proposes two different interfaces that should suite all use cases:

1. The `GpsDecoder.parse_buffer(&[u8])` works from a stream of bytes and is the goto method
when working with a real time GPS/QZSS decoder. It is capable to align itself to the first valid GPS bit found.
[u8] being not aligned to GPS (design flaw) you have to carefully manage your buffer.
Anytime you invoke this method we will return the _first_ frame we identify in the buffer, and its offset position in the buffer
(in bits, not bytes!). You are expected to discard this amount _of bits_ (_not bytes_), not to process the same frame twice.
If you discard this amount of _bytes_ you will lose data. If you respect this rule, you will be able to process all successive
GPS/QZSS messages from your streamer.

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
