//! Galileo I/NAV "link layer" (Galileo OS SIS ICD v2.1, §4.1.4 and §4.3.2).
//!
//! An I/NAV page (2 s on E1-B or E5b-I) is transmitted as two *page parts*
//! (even then odd). Each page part is made of 120 bits (114 data bits and
//! 6 tail bits set to zero) that go through:
//!
//! 1. a rate 1/2, constraint length 7 convolutional encoder
//!    (G1 = 171o, G2 = 133o, G1 first, G2 inverted) producing 240 symbols;
//! 2. a 30 columns x 8 rows block interleaver: the 240 symbols are written
//!    column by column, and read row by row;
//! 3. the (non encoded) 10-symbol synchronization pattern `0101100000`
//!    is then prepended, giving 250 symbols (1 s at 250 sps).
//!
//! The 24-bit CRC (CRC-24Q) of a page is computed over both page parts
//! and transmitted in the odd page part.
//!
//! This module is the only place where symbols, FEC and CRC are handled.
//! It converts 120-bit page parts (15 bytes) into 240 symbols (30 bytes) and back.
//! Messages ([crate::galileo::GalileoInavPage], [crate::galileo::GalileoInavWord]) are read
//! and written on clean data bits only.
//!
//! Symbols are hard symbols, packed MSB first: a symbol is a bit
//! (`1` meaning the chip sequence was transmitted with a `-1` sign, as in the ICD binary convention).

use crate::bits::{get_bit, read_bits, set_bit, write_bits};

/// Number of bits in a page part (data and tail bits).
pub const GALILEO_INAV_PART_BITS: usize = 120;

/// Number of bytes needed to store a page part.
pub const GALILEO_INAV_PART_BYTES: usize = GALILEO_INAV_PART_BITS / 8;

/// Number of data bits in a page part (tail bits excluded).
pub const GALILEO_INAV_PART_DATA_BITS: usize = 114;

/// Number of tail bits (set to zero) at the end of each page part.
pub const GALILEO_INAV_TAIL_BITS: usize = 6;

/// Number of encoded symbols in a page part (sync pattern excluded).
pub const GALILEO_INAV_PART_ENCODED_SYMBOLS: usize = 2 * GALILEO_INAV_PART_BITS;

/// Number of bytes needed to store the encoded symbols of a page part.
pub const GALILEO_INAV_PART_ENCODED_BYTES: usize = GALILEO_INAV_PART_ENCODED_SYMBOLS / 8;

/// Number of symbols in the I/NAV synchronization pattern.
pub const GALILEO_INAV_SYNC_SYMBOLS: usize = 10;

/// I/NAV synchronization pattern (`0101100000`), right aligned.
pub const GALILEO_INAV_SYNC_PATTERN: u32 = 0b01_0110_0000;

/// Inverted I/NAV synchronization pattern, as observed when the carrier phase
/// is locked with a 180° ambiguity (the whole symbol stream is then inverted).
pub const GALILEO_INAV_INVERTED_SYNC_PATTERN: u32 = !GALILEO_INAV_SYNC_PATTERN & 0x3ff;

/// Total number of symbols of a page part, as transmitted (sync pattern included).
pub const GALILEO_INAV_PART_SYMBOLS: usize =
    GALILEO_INAV_SYNC_SYMBOLS + GALILEO_INAV_PART_ENCODED_SYMBOLS;

/// Block interleaver: number of columns (I/NAV).
pub const GALILEO_INAV_INTERLEAVER_COLUMNS: usize = 30;

/// Block interleaver: number of rows (I/NAV).
pub const GALILEO_INAV_INTERLEAVER_ROWS: usize = 8;

/// Convolutional code: first generator polynomial (171 octal).
pub const GALILEO_FEC_G1: u8 = 0o171;

/// Convolutional code: second generator polynomial (133 octal). This branch is inverted.
pub const GALILEO_FEC_G2: u8 = 0o133;

/// Number of states of the convolutional code (constraint length 7).
const NUM_STATES: usize = 64;

/// CRC-24Q generator polynomial, X^24 term omitted:
/// G(X) = (1 + X).P(X) with P(X) = X^23 + X^17 + X^13 + X^12 + X^11 + X^9 + X^8 + X^7 + X^5 + X^3 + 1.
pub const CRC24Q_POLY: u32 = 0x86_4CFB;

/// Computes the CRC-24Q of `count` bits of `slice`, starting at absolute
/// bit position `bit_pos` (MSB first). The register is initialized to zero.
pub fn crc24q(slice: &[u8], bit_pos: usize, count: usize) -> u32 {
    let mut crc = 0u32;

    for pos in bit_pos..bit_pos + count {
        let top = ((crc >> 23) & 0x1) > 0;
        crc = (crc << 1) & 0xff_ffff;

        if top ^ get_bit(slice, pos) {
            crc ^= CRC24Q_POLY;
        }
    }

    crc
}

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
pub fn convolutional_encode(
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
pub fn viterbi_decode(
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
pub fn interleave(
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
pub fn deinterleave(
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
pub fn encode_part(part: &[u8; GALILEO_INAV_PART_BYTES]) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    interleave(&convolutional_encode(part))
}

/// Decodes the 240 symbols that follow a synchronization pattern into a 120-bit
/// page part (de-interleaving + Viterbi decoding). Symbols must be of normal polarity
/// (see [invert_symbols]). Returns the page part and the number of corrected symbol errors.
pub fn decode_part(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> ([u8; GALILEO_INAV_PART_BYTES], usize) {
    viterbi_decode(&deinterleave(symbols))
}

/// Inverts all symbols (180° carrier phase ambiguity).
pub fn invert_symbols(
    symbols: &[u8; GALILEO_INAV_PART_ENCODED_BYTES],
) -> [u8; GALILEO_INAV_PART_ENCODED_BYTES] {
    let mut out = *symbols;

    for byte in out.iter_mut() {
        *byte = !*byte;
    }

    out
}

/// Writes a complete page part (sync pattern + 240 encoded symbols, 250 symbols)
/// at absolute symbol position `symbol_pos` of `slice`. Surrounding symbols are preserved.
/// Returns None if the slice is too short.
pub fn write_part_symbols(
    slice: &mut [u8],
    symbol_pos: usize,
    part: &[u8; GALILEO_INAV_PART_BYTES],
) -> Option<()> {
    if symbol_pos + GALILEO_INAV_PART_SYMBOLS > slice.len() * 8 {
        return None;
    }

    write_bits(
        slice,
        symbol_pos,
        GALILEO_INAV_SYNC_PATTERN,
        GALILEO_INAV_SYNC_SYMBOLS,
    )?;

    let encoded = encode_part(part);

    for (i, byte) in encoded.iter().enumerate() {
        write_bits(
            slice,
            symbol_pos + GALILEO_INAV_SYNC_SYMBOLS + 8 * i,
            *byte as u32,
            8,
        )?;
    }

    Some(())
}

/// Reads the 240 encoded symbols of a page part, whose synchronization pattern
/// starts at absolute symbol position `symbol_pos` of `slice`.
/// Returns the symbols, and true if the sync pattern was inverted
/// (symbols are then returned already inverted back), or None if the slice
/// is too short or no sync pattern (normal or inverted) is found at this position.
pub fn read_part_symbols(
    slice: &[u8],
    symbol_pos: usize,
) -> Option<([u8; GALILEO_INAV_PART_ENCODED_BYTES], bool)> {
    let sync = read_bits(slice, symbol_pos, GALILEO_INAV_SYNC_SYMBOLS)?;

    let inverted = if sync == GALILEO_INAV_SYNC_PATTERN {
        false
    } else if sync == GALILEO_INAV_INVERTED_SYNC_PATTERN {
        true
    } else {
        return None;
    };

    let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];

    for (i, byte) in symbols.iter_mut().enumerate() {
        *byte = read_bits(slice, symbol_pos + GALILEO_INAV_SYNC_SYMBOLS + 8 * i, 8)? as u8;
    }

    if inverted {
        symbols = invert_symbols(&symbols);
    }

    Some((symbols, inverted))
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
        assert_eq!(GALILEO_INAV_PART_SYMBOLS, 250);
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

    #[test]
    fn crc24q_reference() {
        // CRC-24Q of "123456789" (ASCII)
        assert_eq!(crc24q(b"123456789", 0, 72), 0xCDE703);
        assert_eq!(crc24q(&[0u8; 8], 0, 64), 0);

        // leading zeros do not modify the CRC
        let data = [0x00, 0x0A, 0xBC, 0xDE, 0xF0];
        assert_eq!(crc24q(&data, 0, 40), crc24q(&data, 12, 28));
    }

    #[test]
    fn part_symbols_any_offset() {
        let mut seed = 0x5555_1234;
        let part = random_part(&mut seed);

        for offset in 0..16 {
            let mut buffer = [0xA5u8; 40];
            write_part_symbols(&mut buffer, offset, &part).unwrap();

            // preserved surrounding symbols
            assert_eq!(
                read_bits(&buffer, 0, offset),
                Some(read_bits(&[0xA5u8; 4], 0, offset).unwrap())
            );

            let (symbols, inverted) = read_part_symbols(&buffer, offset).unwrap();
            assert!(!inverted);
            assert_eq!(decode_part(&symbols).0, part);

            // inverted stream
            let inverted_buffer: Vec<u8> = buffer.iter().map(|b| !b).collect();
            let (symbols, inverted) = read_part_symbols(&inverted_buffer, offset).unwrap();
            assert!(inverted);
            assert_eq!(decode_part(&symbols).0, part);
        }

        assert!(write_part_symbols(&mut [0u8; 31], 3, &part).is_none());
        assert!(read_part_symbols(&[0u8; 40], 0).is_none());
    }
}
