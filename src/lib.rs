#![doc(
    html_logo_url = "https://raw.githubusercontent.com/nav-solutions/.github/master/logos/logo2.jpg"
)]
#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

/*
 * gnss-protos is part of the NAV-Solutions framework.
 *
 * Authors: Guillaume W. Bres <guillaume.bressaix@gmail.com> et al.
 * (cf. https://github.com/nav-solutions/gnss-protos/graphs/contributors)
 *
 * This framework is shipped under Mozilla Public V2 license.
 */

mod errors;
pub use errors::Error;

#[cfg(feature = "gps")]
mod gps;

#[cfg(feature = "bds")]
mod bds;

#[cfg(feature = "galileo")]
mod galileo;

#[cfg(feature = "gps")]
pub use gps::*;

#[cfg(feature = "bds")]
pub use bds::*;

#[cfg(feature = "galileo")]
pub use galileo::*;

#[cfg(test)]
mod tests;

/// Two's complement parsing & interpretation.
/// ## Input
/// - raw bytes as [u32]
/// - bits_mask: masking u32
/// - sign_bit_mask: sign bit
#[cfg(feature = "gps")]
pub(crate) fn twos_complement(value: u32, bits_mask: u32, sign_bit_mask: u32) -> i32 {
    let value = value & bits_mask;

    let signed = (value & sign_bit_mask) > 0;

    if signed {
        (value | !bits_mask) as i32
    } else {
        value as i32
    }
}

#[cfg(test)]
#[cfg(feature = "gps")]
mod test {
    use crate::twos_complement;

    #[test]
    fn test_twos_complement() {
        let value = 0x3fff;
        let parsed = twos_complement(value, 0x3fff, 0x2000);
        assert_eq!(parsed, 0xffffffffu32 as i32);
    }
}
