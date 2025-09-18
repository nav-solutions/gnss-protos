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
navigation (we do not support the Almanach frames).
Frames parity is not fully implemented either.

We provide methods to both encode and decode data frames, and methods
to work from a stream of padded bytes (re-aligned) (2) or a bit stream buffer for real-time interfaces (1):

1. The `GpsDecoder` is the solution when working with real-time GPS/QZSS streams.
It is capable to synchronize itself to the frame start (which is not aligned to a byte).
But you have to manage your buffer and operate the API correctly.
This method returns the number of processed _bits_ (not bytes). You are expected
to discard all processed _bits_ each time you invoke the decoder, not to process
the same frame twice. If you discard bytes not bits, you will inevitably loose messages.

```rust
use gnss_protos::gps::GpsDecoder;

// The decoder does not verify parity at the moment
let mut decoder = GpsDecoder::default();

// Feed one of our test frames into it,
// which is equivalent to real-time acquisition
```

2. `GpsQzssFrame` supports a `decode()` that works with a possibly padded Byte.
This is the prefered option when working with a stream that was already manipulated by a machine
and therefore, re-aligned to bytes. Each byte may have padding (or not). The stream must
start with the sync byte.
We used this approach to interface correctly to U-Blox receivers for example
(that pad and align the frames internally).

```rust
use gnss_protos::gps::GpsDecoder;
```

License
=======

This library is part of the [Nav-solutions framework](https://github.com/nav-solutions) and is
licensed under [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
