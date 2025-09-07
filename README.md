# GNSS-Protos

[![Rust](https://github.com/nav-solutions/gnss-protos/actions/workflows/rust.yml/badge.svg)](https://github.com/nav-solutions/gnss-protos/actions/workflows/rust.yml)
[![Rust](https://github.com/nav-solutions/gnss-protos/actions/workflows/daily.yml/badge.svg)](https://github.com/nav-solutions/gnss-protos/actions/workflows/daily.yml) 
[![crates.io](https://img.shields.io/crates/v/gnss-protos.svg)](https://crates.io/crates/gnss-protos) 
[![crates.io](https://docs.rs/gnss-protos/badge.svg)](https://docs.rs/gnss-protos/badge.svg)

[![License](https://img.shields.io/badge/license-MPL_2.0-orange?style=for-the-badge&logo=mozilla)](https://github.com/nav-solutions/gnss-protos/blob/main/LICENSE)

GNSS-Protos
===========

Ths library offers small and efficient decoders for GNSS protocols.

This library currently supports the following protos:

- `GPS`: available on `gps` crate feature.

GPS proto
=========

Available on `gps` crate feature.

We support the following GPS frames:

- Ephemeris #1
- Ephemeris #2
- Ephemeris #3

License
=======

This library is part of the [RTK-rs framework](https://github.com/nav-solutions) which
is delivered under the [Mozilla V2 Public](https://www.mozilla.org/en-US/MPL/2.0) license.
