//! Galileo I/NAV forward error correction (Galileo OS SIS ICD v2.1, §4.1.4 and §4.3.2).
//!
//! Private to the crate: [crate::galileo::GalileoInavPage] is the only public
//! entry point to the transmitted symbols, through the [crate::Message] trait.
//!
//! Each 120-bit page part (114 data bits and 6 tail bits set to zero) goes through:
//!
//! 1. a rate 1/2, constraint length 7 convolutional encoder
//!    (G1 = 171o, G2 = 133o, G1 first, G2 inverted) producing 240 symbols;
//! 2. a 30 columns x 8 rows block interleaver: the 240 symbols are written
//!    column by column, and read row by row;
//! 3. the (non encoded) 10-symbol synchronization pattern `0101100000`
//!    is then prepended by the page, giving 250 symbols (1 s at 250 sps).
//!
//! Symbols are hard symbols, packed MSB first: a symbol is a bit
//! (`1` meaning the chip sequence was transmitted with a `-1` sign, as in the ICD binary convention).

use crate::{
    bits::{get_bit, set_bit},
    galileo::GALILEO_INAV_SYNC_PATTERN,
};

/// Number of bits in a page part (data and tail bits).
pub(crate) const GALILEO_INAV_PART_BITS: usize = 120;

/// Number of bytes needed to store a page part.
pub(crate) const GALILEO_INAV_PART_BYTES: usize = GALILEO_INAV_PART_BITS / 8;

/// Number of data bits in a page part (tail bits excluded).
pub(crate) const GALILEO_INAV_PART_DATA_BITS: usize = 114;

/// Number of tail bits (set to zero) at the end of each page part.
pub(crate) const GALILEO_INAV_TAIL_BITS: usize = 6;

/// Number of encoded symbols in a page part (sync pattern excluded).
pub(crate) const GALILEO_INAV_PART_ENCODED_SYMBOLS: usize = 2 * GALILEO_INAV_PART_BITS;

/// Number of bytes needed to store the encoded symbols of a page part.
pub(crate) const GALILEO_INAV_PART_ENCODED_BYTES: usize = GALILEO_INAV_PART_ENCODED_SYMBOLS / 8;

/// Inverted I/NAV synchronization pattern, as observed when the carrier phase
/// is locked with a 180° ambiguity (the whole symbol stream is then inverted).
pub(crate) const GALILEO_INAV_INVERTED_SYNC_PATTERN: u32 = !GALILEO_INAV_SYNC_PATTERN & 0x3ff;

/// Block interleaver: number of columns (I/NAV).
pub(crate) const GALILEO_INAV_INTERLEAVER_COLUMNS: usize = 30;

/// Block interleaver: number of rows (I/NAV).
pub(crate) const GALILEO_INAV_INTERLEAVER_ROWS: usize = 8;

/// Convolutional code: first generator polynomial (171 octal).
pub(crate) const GALILEO_FEC_G1: u8 = 0o171;

/// Convolutional code: second generator polynomial (133 octal). This branch is inverted.
pub(crate) const GALILEO_FEC_G2: u8 = 0o133;

/// Number of states of the convolutional code (constraint length 7).
const NUM_STATES: usize = 64;

/// Returns the two output symbols (G1, G2 inverted) of the convolutional encoder,
/// for a 7-bit register whose MSB (bit 6) is the newest input bit.
#[inline]
fn fec_outputs(register: u8) -> (bool, bool) {
    let g1 = (register & GALILEO_FEC_G1).count_ones() & 0x1 > 0;
    let g2 = (register & GALILEO_FEC_G2).count_ones() & 0x1 > 0;
    (g1, !g2)
}

/// Convolutional encoding (rate 1/2, K = 7) of a 120-bit page part.
/// The encoder starts in the zero state: the 6 tail bits of the previous page part
/// always flush it. Returns 240 symbols, G1 first, then (inverted) G2.
pub(crate) fn convolutional_encode(
    part: &[u8; GALILEO_INAV_PART_BYTES],
) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];
    let mut register = 0u8;

    for i in 0..GALILEO_INAV_PART_BITS {
        register = (register >> 1) | ((get_bit(part, i) as u8) << 6);

        let (g1, g2) = fec_outputs(register);
        set_bit(&mut symbols, 2 * i, g1);
        set_bit(&mut symbols, 2 * i + 1, g2);
    }

    symbols
}

/// Hard decision Viterbi decoder, mirror operation of [convolutional_encode].
///
/// The trellis starts and ends in the zero state (tail bits), so the 6 last
/// decoded bits are always zero. Returns the 120 decoded bits and the number of
/// symbols that disagree with the re-encoded sequence (number of corrected
/// symbol errors, a good quality indicator: 0 on a clean channel, high on noise).
pub(crate) fn viterbi_decode(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> ([u8; GALILEO_INAV_PART_BYTES], usize) {
    const UNREACHABLE: u32 = u32::MAX / 2;

    // state = 6 last input bits, the most recent being bit 5
    let mut metrics = [UNREACHABLE; NUM_STATES];
    metrics[0] = 0;

    // decisions[t] bit #s: LSB of the predecessor of state s at step t
    let mut decisions = [0u64; GALILEO_INAV_PART_BITS];

    for (t, decision) in decisions.iter_mut().enumerate() {
        let r1 = get_bit(symbols, 2 * t);
        let r2 = get_bit(symbols, 2 * t + 1);

        let mut next = [UNREACHABLE; NUM_STATES];

        for (state, next_metric) in next.iter_mut().enumerate() {
            let input = (state >> 5) as u8;

            for lsb in 0..2usize {
                let previous = ((state & 0x1f) << 1) | lsb;
                let metric = metrics[previous];

                if metric >= UNREACHABLE {
                    continue;
                }

                let register = (input << 6) | (previous as u8);
                let (g1, g2) = fec_outputs(register);
                let cost = metric + (g1 != r1) as u32 + (g2 != r2) as u32;

                if cost < *next_metric {
                    *next_metric = cost;

                    if lsb == 1 {
                        *decision |= 1 << state;
                    } else {
                        *decision &= !(1 << state);
                    }
                }
            }
        }

        metrics = next;
    }

    // trace back from the zero state (flushed by the tail bits)
    let mut bits = [0u8; GALILEO_INAV_PART_BYTES];
    let mut state = 0usize;

    for t in (0..GALILEO_INAV_PART_BITS).rev() {
        set_bit(&mut bits, t, (state >> 5) > 0);
        let lsb = ((decisions[t] >> state) & 0x1) as usize;
        state = ((state & 0x1f) << 1) | lsb;
    }

    (bits, metrics[0] as usize)
}

/// Block interleaving (30 columns x 8 rows) of 240 encoded symbols:
/// symbols are written column by column (8 symbols per column)
/// and read row by row (30 symbols per row).
pub(crate) fn interleave(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    let mut out = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];

    for row in 0..GALILEO_INAV_INTERLEAVER_ROWS {
        for col in 0..GALILEO_INAV_INTERLEAVER_COLUMNS {
            let bit = get_bit(symbols, col * GALILEO_INAV_INTERLEAVER_ROWS + row);
            set_bit(&mut out, row * GALILEO_INAV_INTERLEAVER_COLUMNS + col, bit);
        }
    }

    out
}

/// Block de-interleaving, mirror operation of [interleave].
pub(crate) fn deinterleave(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    let mut out = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];

    for row in 0..GALILEO_INAV_INTERLEAVER_ROWS {
        for col in 0..GALILEO_INAV_INTERLEAVER_COLUMNS {
            let bit = get_bit(symbols, row * GALILEO_INAV_INTERLEAVER_COLUMNS + col);
            set_bit(&mut out, col * GALILEO_INAV_INTERLEAVER_ROWS + row, bit);
        }
    }

    out
}

/// Encodes a 120-bit page part (tail bits included) into the 240 symbols
/// to be transmitted after the synchronization pattern (FEC + interleaving).
pub(crate) fn encode_part(
    part: &[u8; GALILEO_INAV_PART_BYTES],
) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    interleave(&convolutional_encode(part))
}

/// Decodes the 240 symbols that follow a synchronization pattern into a 120-bit
/// page part (de-interleaving + Viterbi decoding). Symbols must be of normal polarity
/// (see [invert_symbols]). Returns the page part and the number of corrected symbol errors.
pub(crate) fn decode_part(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> ([u8; GALILEO_INAV_PART_BYTES], usize) {
    viterbi_decode(&deinterleave(symbols))
}

/// Inverts all symbols (180° carrier phase ambiguity).
pub(crate) fn invert_symbols(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    let mut out = *symbols;

    for byte in out.iter_mut() {
        *byte = !*byte;
    }

    out
}

#[cfg(test)]
pub(crate) mod test {
    use super::*;

    /// Builds a page part from its bits as a string of '0' and '1'.
    fn part_from_str(bits: &str) -> [u8; GALILEO_INAV_PART_BYTES] {
        let mut part = [0u8; GALILEO_INAV_PART_BYTES];
        for (i, c) in bits.chars().enumerate() {
            set_bit(&mut part, i, c == '1');
        }
        part
    }

    /// Simple deterministic pseudo random generator
    pub(crate) fn lfsr(seed: &mut u32) -> u32 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 17;
        *seed ^= *seed << 5;
        *seed
    }

    pub(crate) fn random_part(seed: &mut u32) -> [u8; GALILEO_INAV_PART_BYTES] {
        let mut part = [0u8; GALILEO_INAV_PART_BYTES];
        for byte in part.iter_mut() {
            *byte = lfsr(seed) as u8;
        }
        // tail bits
        part[GALILEO_INAV_PART_BYTES - 1] &= 0xC0;
        part
    }

    #[test]
    fn sync_patterns() {
        assert_eq!(GALILEO_INAV_SYNC_PATTERN, 0x160);
        assert_eq!(GALILEO_INAV_INVERTED_SYNC_PATTERN, 0b10_1001_1111);
        assert_eq!(GALILEO_INAV_PART_ENCODED_SYMBOLS, 240);
    }

    #[test]
    fn invert() {
        let mut seed = 0x0101_0101;
        let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];
        for byte in symbols.iter_mut() {
            *byte = lfsr(&mut seed) as u8;
        }
        let inverted = invert_symbols(&symbols);
        assert!(symbols.iter().zip(inverted.iter()).all(|(a, b)| *a == !*b));
        assert_eq!(invert_symbols(&inverted), symbols);
    }

    /// OS SIS ICD v2.1, Table 83: the 8 SSP bits are the last data bits of the
    /// odd page part on E1-B, followed by the 6 tail bits: the 16 last encoded
    /// symbols (before interleaving) only depend on them.
    #[test]
    fn convolutional_encoding_icd_ssp() {
        for (ssp, expected) in [
            ("00000100", "1110100100100101"),
            ("00101011", "0110110001001110"),
            ("00101111", "1101000000111110"),
        ] {
            let mut bits = String::from("1");
            bits.push_str(&"0".repeat(105));
            bits.push_str(ssp);
            bits.push_str("000000");

            let part = part_from_str(&bits);
            let symbols = convolutional_encode(&part);

            let last: String = (224..240)
                .map(|i| if get_bit(&symbols, i) { '1' } else { '0' })
                .collect();

            assert_eq!(last, expected, "SSP {}", ssp);
        }
    }

    #[test]
    fn convolutional_encoding_zero_input() {
        // G2 is inverted: a null input produces 0101...
        let symbols = convolutional_encode(&[0; GALILEO_INAV_PART_BYTES]);
        assert!(symbols.iter().all(|b| *b == 0x55));
    }

    #[test]
    fn interleaver() {
        let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];
        // symbol #1 (column 0, row 1) is read in 2nd row: position 30
        set_bit(&mut symbols, 1, true);
        let interleaved = interleave(&symbols);
        assert!(get_bit(&interleaved, 30));
        assert_eq!(interleaved.iter().map(|b| b.count_ones()).sum::<u32>(), 1);

        // symbol #8 (column 1, row 0) is transmitted in second position
        let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];
        set_bit(&mut symbols, 8, true);
        assert!(get_bit(&interleave(&symbols), 1));

        let mut seed = 0x1234_5678;
        for _ in 0..16 {
            let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];
            for byte in symbols.iter_mut() {
                *byte = lfsr(&mut seed) as u8;
            }
            assert_eq!(deinterleave(&interleave(&symbols)), symbols);
            assert_eq!(interleave(&deinterleave(&symbols)), symbols);
        }
    }

    #[test]
    fn viterbi_clean_channel() {
        let mut seed = 0xCAFE_0001;
        for _ in 0..32 {
            let part = random_part(&mut seed);
            let (decoded, errors) = viterbi_decode(&convolutional_encode(&part));
            assert_eq!(decoded, part);
            assert_eq!(errors, 0);

            let (decoded, errors) = decode_part(&encode_part(&part));
            assert_eq!(decoded, part);
            assert_eq!(errors, 0);
        }
    }

    #[test]
    fn viterbi_corrects_symbol_errors() {
        let mut seed = 0xBEEF_0002;

        for num_errors in 1..=8 {
            for _ in 0..32 {
                let part = random_part(&mut seed);
                let mut symbols = encode_part(&part);

                // errors are spread by the de-interleaver
                let mut flipped = 0;
                while flipped < num_errors {
                    let pos = (lfsr(&mut seed) as usize) % GALILEO_INAV_PART_ENCODED_SYMBOLS;
                    if get_bit(&symbols, pos) == get_bit(&encode_part(&part), pos) {
                        let bit = get_bit(&symbols, pos);
                        set_bit(&mut symbols, pos, !bit);
                        flipped += 1;
                    }
                }

                let (decoded, errors) = decode_part(&symbols);
                assert_eq!(decoded, part, "{} symbol errors", num_errors);
                assert_eq!(errors, num_errors);
            }
        }
    }

    #[test]
    fn viterbi_on_noise() {
        // random symbols: the residual error count is high
        // (never below 25 over 20 000 trials)
        let mut seed = 0x0BAD_F00D;
        for _ in 0..256 {
            let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];
            for byte in symbols.iter_mut() {
                *byte = lfsr(&mut seed) as u8;
            }
            let (_, errors) = decode_part(&symbols);
            assert!(errors > 20, "only {} errors on noise", errors);
        }
    }
}
