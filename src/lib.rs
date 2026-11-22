#![doc(
    html_logo_url = "https://raw.githubusercontent.com/nav-solutions/.github/master/logos/logo2.jpg"
)]
#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

/*
 * gnss-protos is part of the nav-solutions framework.
 *
 * Authors: Guillaume W. Bres <guillaume.bressaix@gmail.com> et al.
 * (cf. https://github.com/nav-solutions/gnss-protos/graphs/contributors)
 *
 * This framework is shipped under Mozilla Public V2 license.
 */

pub mod bits;
pub mod constants;

pub(crate) mod modulation;

#[cfg(feature = "gps")]
#[cfg_attr(docsrs, doc(cfg(feature = "gps")))]
pub mod gps;

#[cfg(feature = "galileo")]
#[cfg_attr(docsrs, doc(cfg(feature = "galileo")))]
pub mod galileo;

#[cfg(test)]
pub mod tests;

mod buffer;
mod errors;

pub use buffer::{Buffer, StaticBuffer};
pub use errors::{BufferingError, Error, ModulationError};

pub use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadBuffer, BitReadStream, BitWrite, BitWriteStream,
    Endianness, LittleEndian,
};

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
    /// Bytewise encoding size. Some protocols may use padding,
    /// in these cases, we use zero terminations.
    fn encoding_size(&self) -> usize;

    /// Bitwise encoding size. Some protocols may be unaligned.
    fn encoding_bits(&self) -> usize;

    /// [Message] encoding attempt to mutable buffer, starting on its first bit.
    /// [Message] must fit entirely, otherwise [BitError::NotEnoughData] is returned.
    /// To encode at any bit position, or to stream several messages back to back,
    /// write the [Message] to a [BitWriteStream] (see [BitWrite]).
    ///
    /// Returns total number of encoded bytes on success,
    /// depending on protocol, this may include padding bits.
    /// Returns [BitError] on encoding issues.
    fn encode(&self, buffer: &mut [u8]) -> Result<usize, BitError> {
        let bits = self.encoding_bits();

        if buffer.len() * 8 < bits {
            return Err(BitError::NotEnoughData {
                requested: bits,
                bits_left: buffer.len() * 8,
            });
        }

        let mut writer = BitWriteStream::from_slice(buffer, E::endianness());
        writer.write(self)?;
        Ok(self.encoding_size())
    }

    /// [Message] decoding attempt, from read-only buffer state, starting on its first bit.
    /// To decode at any bit position, read the [Message] from a [BitReadStream] (see [BitRead]).
    /// This is not compatible with a real-time decoder, for this task you are
    /// expected to run a mutable [Decoder] implementation.
    fn decode(buffer: &[u8]) -> Result<Self, BitError> {
        let mut reader = BitReadStream::new(BitReadBuffer::new(buffer, E::endianness()));
        let decoded = reader.read::<Self>()?;
        Ok(decoded)
    }
}

use crate::bits::get_bit;

/// Generate [Modulation] trait
pub trait Modulation {
    /// Symbol rate, number of modulated symbols per second
    fn symbol_rate_hz(&self) -> f64;

    /// Modulator output sample rate in Hz
    fn sample_rate_hz(&self) -> f64;

    /// Number of output samples generated for each input bit/symbol
    fn samples_per_symbol(&self) -> usize;

    /// PRN number or satellite ID
    fn prn_number(&self) -> u8;

    /// Sample `index` (`0..`[Self::period_samples]) of one primary code period,
    /// sub-carrier included, data and secondary code excluded. This is the local replica
    /// a receiver correlates against.
    fn replica_sample(&self, index: usize) -> i8;

    /// Secondary code: one chip (logic level) per primary code period, its length
    /// being the number of primary code periods per data bit / symbol.
    /// GPS has no secondary code, but a data bit lasts 20 code periods: it is
    /// described as a 20-chip null secondary code.
    fn secondary_code(&self) -> &'static [bool];

    /// Spreads a single data bit / symbol into mutable output buffer.
    /// Returns the number of samples written (always [Self::samples_per_symbol]).
    fn modulate_symbol(&self, symbol: bool, bufer: &mut [i8]) -> Result<usize, ModulationError>;

    /// Spreads `num_symbols` data bits / symbols (depending on protocol),
    /// from readable input buffer starting at absolute bit position
    /// `bit_offset` (MSB first), into mutable output buffer.
    ///
    /// Output buffer must be able to store `num_symbols *` [Self::samples_per_symbol] samples.
    /// Returns the number of samples written.
    fn modulate(
        &self,
        input: &[u8],
        bit_offset: usize,
        num_symbols: usize,
        output: &mut [i8],
    ) -> Result<usize, ModulationError> {
        let samples_per_symbol = self.samples_per_symbol();

        if bit_offset + num_symbols > input.len() * 8 {
            return Err(ModulationError::NotEnoughBits);
        }

        if output.len() < num_symbols * samples_per_symbol {
            return Err(ModulationError::WouldNotFit);
        }

        for (i, chunk) in output
            .chunks_exact_mut(samples_per_symbol)
            .take(num_symbols)
            .enumerate()
        {
            self.modulate_symbol(get_bit(input, bit_offset + i), chunk)?;
        }

        Ok(num_symbols * samples_per_symbol)
    }

    /// Number of output saples per primary code period
    fn period_samples(&self) -> usize {
        self.samples_per_symbol() / self.secondary_code().len()
    }
}

// /// Two's complement parsing & interpretation.
// /// ## Input
// /// - raw bytes as [u32]
// /// - bits_mask: masking u32
// /// - sign_bit_mask: sign bit
// pub(crate) fn twos_complement(value: u32, bits_mask: u32, sign_bit_mask: u32) -> i32 {
//     let value = value & bits_mask;
//
//     let signed = (value & sign_bit_mask) > 0;
//
//     if signed {
//         (value | !bits_mask) as i32
//     } else {
//         value as i32
//     }
// }
//
// #[cfg(test)]
// #[cfg(feature = "gps")]
// mod test {
//     use crate::twos_complement;
//
//     #[test]
//     fn test_twos_complement() {
//         let value = 0x3fff;
//         let parsed = twos_complement(value, 0x3fff, 0x2000);
//         assert_eq!(parsed, 0xffffffffu32 as i32);
//     }
// }
