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

mod buffer;
pub use buffer::Buffering;

mod errors;

#[cfg(test)]
mod tests;

#[cfg(feature = "gps")]
mod gps;

pub use errors::{BufferingError, Error};

#[cfg(feature = "gps")]
pub use gps::*;

/// All our GNSS decoders implement the [Decoder] trait.
pub trait Decoder {
    /// [Message] type returned by [Self::decode].
    type M: Message;

    /// Provide new data to this [Decoder].
    ///
    /// Most of these protocols are not aligned to [u8], for example
    /// a GPS burst is 300 bit long. You may insert padding bits (blanking)
    /// in between frames but not inside frames. Otherwise, the binary content
    /// would be corrupt and impossible to decode.
    ///
    /// If you don't need 100% efficiency and can afford to loose one frame
    /// from time to time (say one per receiver capture), then you are fine and may
    /// use padding whenever that suites you.
    ///
    /// Note that, even real-time navigation does not require 100% efficiency,
    /// because information is regurlarly updated, but that is closely related to the
    /// velocity of your receiver, and will not work well in the case of fast moving rovers.
    ///
    /// In between frame padding is tolerated and our [Decoder]s will naturally adapt,
    /// because all these protocols use synchronization bytes to mark
    /// the beginning of frame. This library exposes all the synchronization
    /// bytes, that you may use to create an efficient padded receiver.
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError>;

    /// Process internal buffer and try to decode a [Message].
    /// You can use the following methods to provide new data:
    /// - [Self::fill] which is always available
    /// - [std::io::Write] when feasible
    ///
    /// ## Ouput
    /// - [Message] on decoding success.
    fn decode(&mut self) -> Option<Self::M>;
}

/// All GNSS messages implement the [Message] trait
pub trait Message: Copy + Clone + Default + PartialEq {
    /// Error type for this messaging.
    type Err;

    /// Specific [Message] buffer storage.
    /// Each message may be encoded into a specific structure, following
    /// the underlying protocol endianness.
    type B: Buffering;

    /// Returns the total number of bytes required to encode this [Message].
    /// Most [Message]s are not aligned to [u8], so the returned value here
    /// is more than needed, meaning that padding was introduced at some point.
    fn encoding_size(&self) -> usize;

    /// Returns the total number of bits required to encode this [Message].
    /// For aligned protocol, this value strictly equals [Self::encoding_size].
    fn encoding_bitsize(&self) -> usize;

    /// [Message] encoding attempt to mutable [Self::B],
    /// which must fit completely in current buffer state.
    ///
    /// Returns total number of encoded bytes on success,
    /// depending on protocol, this may include padding bits.
    /// Returns [Self::Err] on encoding issues.
    fn encode(&self, buffer: &mut Self::B) -> Result<usize, Self::Err>;

    /// Conveniently encodes this [Message] as a slice of [u8].
    /// Depending on the protocol, the slice might be terminated with padding bits (zeros).
    #[cfg(test)]
    fn to_slice(&self) -> Vec<u8>;
}

/// Two's complement parsing & interpretation.
/// ## Input
/// - raw bytes as [u32]
/// - bits_mask: masking u32
/// - sign_bit_mask: sign bit
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
