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

- `GPS (+QZSS)`: available on `gps` crate feature, activated by default

GPS and QZSS protocol
=====================

The `gps` library feature activates support of part of the GPS and QZSS protocol.  

Currently we support Ephemeris frames 1, 2 and 3, which is sufficient for real-time
navigation. We're still lacking support for Almanach frames, which help converge faster
in case you are doing all the work manually.

License
=======

This library is part of the [Nav-solutions framework](https://github.com/nav-solutions) and is
licensed under [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
