#![doc(html_logo_url = "https://raw.githubusercontent.com/rtk-rs/.github/master/logos/logo2.jpg")]
#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

/*
 * gnss-protos is part of the rtk-rs framework.
 *
 * Authors: Guillaume W. Bres <guillaume.bressaix@gmail.com> et al.
 * (cf. https://github.com/rtk-rs/gnss-protos/graphs/contributors)
 *
 * This framework is shipped under Mozilla Public V2 license.
 */

mod errors;

mod buffer;

#[cfg(feature = "gps")]
mod gps;

#[cfg(test)]
mod tests;

pub use crate::{buffer::BufferingError, errors::Error};

#[cfg(feature = "gps")]
pub use gps::*;

/// All GNSS messages implement the [Message] trait
pub trait Message: Default {
    /// Returns the total number of bytes required to encode this [Message].
    /// Most [Message]s are not aligned to [u8], so the returned value here
    /// is more than needed.
    fn encoding_size(&self) -> usize;

    /// Returns the total number of bits required to encode this [Message].
    /// For aligned protocol, this value is strictly equals to [Self::encoding_size].
    fn encoding_bits(&self) -> usize;

    /// Encode this [Message] which must completely fit into the buffer,
    /// otherwise returns [BufferingError::StorageFull].
    /// Only a decided encoder (not yet provided) may support streamed/partial encoding.
    fn encode(&self, dest: &mut [u8]) -> Result<usize, BufferingError>;

    /// Generates a realistic frame model for testing purposes.
    #[cfg(test)]
    fn model(&self) -> Self;
}

/// All our GNSS decoders implement the [Decoder] trait.
pub trait Decoder {
    /// [Message] type returned by [Self::decode].
    type M: Message;

    /// Provide new data to this [Decoder].
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError>;

    /// Tries to decode a valid [Self::Message] using actual buffered content.
    ///
    /// You need to provide more content in between this call, either using
    /// - [Self::fill] which is always available
    /// - [std::io::Write] when feasible
    ///
    /// ## Ouput
    /// - [Self::Message] on decoding sucess.
    fn decode(&mut self) -> Option<Self::M>;
}

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
