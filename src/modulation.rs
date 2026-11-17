//! Spreading modulation, shared by all protocols.
//!
//! Encoding a message (a [crate::gps::GpsQzssFrame], a [crate::galileo::GalileoInavPage]..)
//! produces a stream of data bits (GPS, 50 bit/s) or of channel symbols (Galileo I/NAV,
//! 250 sym/s, after FEC and interleaving). This stream cannot be transmitted as is:
//! every GNSS signal spreads it with a pseudo random noise (PRN) code, a.k.a. ranging code,
//! chipped at a much higher rate (1.023 MHz for GPS L1 C/A, 10.23 MHz for Galileo E5b..).
//!
//! Each protocol provides its own `Modulator` ([crate::gps::Modulator],
//! [crate::galileo::Modulator]) that implements the [Modulation] trait. They all follow
//! the same conventions:
//!
//! - the input is the bit (or symbol) stream, packed MSB first, as produced by the
//!   `encode_at` methods of each protocol, starting at any bit position;
//! - the output is a baseband stream of samples, one per chip (or half chip, when a
//!   sub-carrier applies), with the usual BPSK mapping: logic `0` is `+1`, logic `1` is `-1`.
//!   The output sample is therefore `(-1)^(data ⊕ code)`;
//! - each data bit / symbol is spread over an integer number of code periods, so the
//!   modulators are stateless: you can modulate a stream in as many pieces as you want,
//!   as long as each piece starts on a bit / symbol boundary.
//!
//! The output is a *baseband* signal: carrier generation (or I/Q upconversion),
//! pulse shaping, filtering and amplitude are left to the transmitter.

use thiserror::Error;

use crate::bits::get_bit;

/// Errors that may happen when building a modulator, or while modulating.
#[derive(Error, Debug, Copy, Clone, PartialEq)]
pub enum ModulationError {
    /// The PRN number does not exist for this signal.
    #[error("invalid PRN number {0}")]
    InvalidPrn(u8),

    /// A user provided spreading code is invalid (bad length or non hexadecimal character).
    #[error("invalid spreading code")]
    InvalidCode,

    /// The input slice does not contain enough bits / symbols.
    #[error("not enough input bits")]
    NotEnoughBits,

    /// The output buffer is too small.
    #[error("output buffer too small")]
    WouldNotFit,
}

/// Maps a logic level to a BPSK sample (logic `0` → `+1`, logic `1` → `-1`).
#[inline]
pub(crate) const fn bpsk(logic: bool) -> i8 {
    if logic {
        -1
    } else {
        1
    }
}

/// Spreading modulation, implemented by every protocol `Modulator`.
///
/// This trait allows to write a transmitter that is generic over the GNSS protocol.
pub trait Modulation {
    /// Rate of the input data bits / symbols, in Hz.
    fn symbol_rate_hz(&self) -> f64;

    /// Rate of the output samples, in Hz.
    fn sample_rate_hz(&self) -> f64;

    /// Number of output samples generated for each input data bit / symbol.
    fn samples_per_symbol(&self) -> usize;

    /// Spreads a single data bit / symbol into `out`.
    /// Returns the number of samples written (always [Self::samples_per_symbol]).
    fn modulate_symbol(&self, symbol: bool, out: &mut [i8]) -> Result<usize, ModulationError>;

    /// Spreads `num_symbols` data bits / symbols, read from `slice` starting at
    /// absolute bit position `bit_offset` (MSB first), into `out`.
    ///
    /// `out` must be able to store `num_symbols *` [Self::samples_per_symbol] samples.
    /// Returns the number of samples written.
    fn modulate(
        &self,
        slice: &[u8],
        bit_offset: usize,
        num_symbols: usize,
        out: &mut [i8],
    ) -> Result<usize, ModulationError> {
        let samples_per_symbol = self.samples_per_symbol();

        if bit_offset + num_symbols > slice.len() * 8 {
            return Err(ModulationError::NotEnoughBits);
        }

        if out.len() < num_symbols * samples_per_symbol {
            return Err(ModulationError::WouldNotFit);
        }

        for (i, chunk) in out
            .chunks_exact_mut(samples_per_symbol)
            .take(num_symbols)
            .enumerate()
        {
            self.modulate_symbol(get_bit(slice, bit_offset + i), chunk)?;
        }

        Ok(num_symbols * samples_per_symbol)
    }
}

/// Number of chips per byte, in packed code storage.
const PACKED_CHIPS: usize = 8;

/// Reads chip `index` (logic level) of a code packed MSB first.
#[inline]
pub(crate) fn packed_chip(code: &[u8], index: usize) -> bool {
    (code[index / PACKED_CHIPS] >> (7 - index % PACKED_CHIPS)) & 0x1 > 0
}

/// Writes chip `index` (logic level) of a code packed MSB first.
#[inline]
pub(crate) fn set_packed_chip(code: &mut [u8], index: usize, chip: bool) {
    if chip {
        code[index / PACKED_CHIPS] |= 0x80 >> (index % PACKED_CHIPS);
    }
}

#[cfg(test)]
pub(crate) mod test {
    /// Packs `count` (<= 32) chips of a code, starting at `start`, MSB first.
    pub(crate) fn chips_word(chip: impl Fn(usize) -> bool, start: usize, count: usize) -> u32 {
        (start..start + count).fold(0u32, |acc, i| (acc << 1) | chip(i) as u32)
    }
}
