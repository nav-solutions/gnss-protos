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
pub use buffer::{Buffer, StaticBuffer};

mod errors;

#[cfg(test)]
mod tests;

#[cfg(feature = "gps")]
mod gps;

pub use errors::{BufferingError, Error};

#[cfg(feature = "gps")]
pub use gps::*;

pub use bitbuffer::{BitError, BitRead, BitWrite, Endianness};

use bitbuffer::{BitReadBuffer, BitReadStream, BitWriteStream};

#[cfg(feature = "std")]
use std::io::{Read, Write};

/// All our GNSS decoders implement the [Decoder] trait.
pub trait Decoder<E: Endianness>: Default {
    /// [Message] type returned by [Self::decode].
    type M: Message<E>;

    /// Process internal buffer and try to decode a [Message].
    /// You can use the following methods to provide new data:
    /// - [Self::fill] which is always available
    /// - [std::io::Write] when feasible
    ///
    /// ## Ouput
    /// - [Message] on decoding success.
    fn decode(&mut self) -> Option<Self::M>;
}

/// All GNSS messages implement the [Message] trait, which
/// implicitely means:
///
/// - [Copy] and [Clone] objects
/// - [PartialEq] comparison method
/// - Simple yet efficient [Default] builder
/// - [Message::encode] to dump to bytes
/// - [Message::decode] to read from bytes
pub trait Message<E: Endianness>:
    Copy + Clone + Default + PartialEq + BitWrite<E> + for<'a> BitRead<'a, E>
{
    /// Error type for this messaging.
    type Err;

    /// Bytewise encoding size. Some protocols may use padding,
    /// in these cases, we use zero terminations.
    fn encoding_size(&self) -> usize;

    /// Bitwise encoding size. Some protocols may be unaligned.
    fn encoding_bits(&self) -> usize;

    /// [Message] encoding attempt to mutable buffer.
    /// [Message] must fit entirely.
    ///
    /// Returns total number of encoded bytes on success,
    /// depending on protocol, this may include padding bits.
    /// Returns [Self::Err] on encoding issues.
    fn encode(&self, buffer: &mut [u8]) -> Result<usize, BitError> {
        let mut writer = BitWriteStream::from_slice(buffer, E::endianness());
        writer.write(self)?;
        Ok(self.encoding_size())
    }

    /// [Message] decoding attempt, from read-only buffer state.
    /// This is not compatible with a real-time decoder, for this task you are
    /// expected to run a mutable [Decoder] implementation.
    fn decode(buffer: &[u8]) -> Result<Self, BitError> {
        let mut reader = BitReadStream::new(BitReadBuffer::new(buffer, E::endianness()));
        let decoded = reader.read::<Self>()?;
        Ok(decoded)
    }

    /// Generates a realistic (physically valid) frame, for testing purposes.
    #[cfg(test)]
    fn model() -> Self;
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
