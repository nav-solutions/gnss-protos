//! GPS / QZSS LNAV "link layer".
//!
//! A GPS subframe is 300 bits = 10 words of 30 bits. Each word carries
//! 24 data bits (d1..d24) followed by 6 parity bits (D25..D30), computed
//! from the data bits *and* the last two parity bits (D29*, D30*) of the
//! previous word (IS-GPS-200, §20.3.5.2, table 20-XIV). When D30* is set,
//! the transmitted data bits are inverted.
//!
//! This module is the only place where parity and polarity are handled.
//! It converts a 300-bit subframe (10 x 30 bits) into 240 contiguous,
//! parity-free data bits (30 bytes), and back. Messages (TLM, HOW, subframes)
//! are then read and written on those 240 clean bits: no parity bits to skip,
//! and fields that the ICD splits over two words (M0, e, sqrt(A), Omega0..)
//! become contiguous.

use crate::gps::{GpsError, GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME, GPS_WORD_BITS};

/// Number of data bits per GPS word.
pub const GPS_WORD_DATA_BITS: usize = 24;

/// Number of data (parity free) bits per GPS subframe.
pub const GPS_FRAME_DATA_BITS: usize = GPS_WORDS_PER_FRAME * GPS_WORD_DATA_BITS;

/// Number of data (parity free) bytes per GPS subframe.
pub const GPS_FRAME_DATA_BYTES: usize = GPS_FRAME_DATA_BITS / 8;

/// 30-bit mask.
pub(crate) const GPS_WORD_MASK: u32 = 0x3fff_ffff;

/// 24-bit data mask.
const DATA_MASK: u32 = 0x00ff_ffff;

/// Inverted preamble, as observed when the carrier phase is locked
/// with a 180° ambiguity: the whole bit stream is then inverted.
pub const GPS_INVERTED_PREAMBLE_BYTE: u8 = !GPS_PREAMBLE_BYTE;

/// Parity equations (IS-GPS-200 table 20-XIV) expressed as bit masks,
/// applied to a 32-bit value laid out as `[D29*, D30*, d1..d24, 0 x 6]`.
const PARITY_MASKS: [u32; 6] = [
    0xBB1F_3480, // D25
    0x5D8F_9A40, // D26
    0xAEC7_CD00, // D27
    0x5763_E680, // D28
    0x6BB1_F340, // D29
    0x8B7A_89C0, // D30
];

/// Computes the 6 parity bits (D25..D30, D25 being the MSB) of 24 *source*
/// data bits (right aligned), given the last two parity bits of the previous word.
pub fn parity(data: u32, prev_d29: bool, prev_d30: bool) -> u8 {
    let mut value = (data & DATA_MASK) << 6;

    if prev_d29 {
        value |= 0x8000_0000;
    }

    if prev_d30 {
        value |= 0x4000_0000;
    }

    let mut parity = 0u8;

    for mask in PARITY_MASKS.iter() {
        parity <<= 1;
        parity |= ((value & mask).count_ones() & 0x1) as u8;
    }

    parity
}

/// Returns (D29, D30) of a 30-bit (right aligned) word.
#[inline]
fn d29_d30(word: u32) -> (bool, bool) {
    (word & 0x2 > 0, word & 0x1 > 0)
}

/// Encodes 24 source data bits into a 30-bit transmitted word (right aligned),
/// following the previous transmitted word `prev` (only its 2 LSB matter).
pub fn encode_word(data: u32, prev: u32) -> u32 {
    let (d29, d30) = d29_d30(prev);
    let data = data & DATA_MASK;
    let parity = parity(data, d29, d30) as u32;

    let transmitted = if d30 { !data & DATA_MASK } else { data };

    (transmitted << 6) | parity
}

/// Encodes 22 source data bits (right aligned, bits 23 and 24 being left free),
/// solving bits 23-24 so the resulting word ends with D29 = D30 = 0.
/// This is required for the HOW (word 2) and word 10 (IS-GPS-200 §20.3.5.2),
/// so that every subframe starts with D29* = D30* = 0.
pub fn encode_word_with_t_bits(data: u32, prev: u32) -> u32 {
    let data = data & !0x3 & DATA_MASK;

    for t in 0..4 {
        let word = encode_word(data | t, prev);

        if word & 0x3 == 0 {
            return word;
        }
    }

    // There is always one combination of the two free bits
    // that nulls D29 and D30 (the system is invertible).
    unreachable!("GPS t-bits: no solution");
}

/// Decodes a 30-bit received word (right aligned) into its 24 source data bits,
/// following the previous received word `prev` (only its 2 LSB matter).
/// When `check_parity` is set, the parity is verified.
pub fn decode_word(word: u32, prev: u32, check_parity: bool) -> Result<u32, GpsError> {
    let (d29, d30) = d29_d30(prev);

    let mut data = (word >> 6) & DATA_MASK;

    if d30 {
        data = !data & DATA_MASK;
    }

    if check_parity && parity(data, d29, d30) != (word & 0x3f) as u8 {
        return Err(GpsError::Parity);
    }

    Ok(data)
}

/// Encodes a complete subframe, from its 240 data bits (30 bytes),
/// into 10 transmitted words (30-bit right aligned) with parity.
/// Words 2 (HOW) and 10 have their bits 23-24 solved, as required by the ICD.
pub fn encode_frame(data: &[u8; GPS_FRAME_DATA_BYTES]) -> [u32; GPS_WORDS_PER_FRAME] {
    let mut words = [0u32; GPS_WORDS_PER_FRAME];

    // Every subframe is preceded by a word 10 ending with 00
    let mut prev = 0u32;

    for (i, word) in words.iter_mut().enumerate() {
        let d = ((data[3 * i] as u32) << 16)
            | ((data[3 * i + 1] as u32) << 8)
            | (data[3 * i + 2] as u32);

        *word = if i == 1 || i == GPS_WORDS_PER_FRAME - 1 {
            encode_word_with_t_bits(d, prev)
        } else {
            encode_word(d, prev)
        };

        prev = *word;
    }

    words
}

/// Decodes a complete subframe, from 10 received words (30-bit right aligned),
/// into its 240 data bits (30 bytes). The polarity is resolved from the preamble
/// (normal 0x8B or inverted 0x74): an inverted subframe is decoded transparently.
pub fn decode_frame(
    words: &[u32; GPS_WORDS_PER_FRAME],
    check_parity: bool,
) -> Result<[u8; GPS_FRAME_DATA_BYTES], GpsError> {
    let preamble = (words[0] >> 22) as u8;

    // Inverted stream <=> previous word ended with D30* = 1
    let mut prev = if preamble == GPS_PREAMBLE_BYTE {
        0
    } else if preamble == GPS_INVERTED_PREAMBLE_BYTE {
        GPS_WORD_MASK
    } else {
        return Err(GpsError::InvalidPreamble);
    };

    let mut data = [0u8; GPS_FRAME_DATA_BYTES];

    for (i, word) in words.iter().enumerate() {
        let word = word & GPS_WORD_MASK;
        let d = decode_word(word, prev, check_parity)?;

        data[3 * i] = (d >> 16) as u8;
        data[3 * i + 1] = (d >> 8) as u8;
        data[3 * i + 2] = d as u8;

        // Inverting the whole stream inverts D29*/D30* as well, and
        // parity(d, !D29*, !D30*) == !parity(d, D29*, D30*): an inverted
        // stream is self consistent, nothing special to do from here on.
        prev = word;
    }

    Ok(data)
}

/// Some receivers (u-blox UBX-RXM-SFRBX for example) do not report the words
/// as transmitted: each 30-bit word (data *and* parity) is XOR'ed with the D30*
/// bit of the previous word, so that the data bits come out non-inverted.
/// This restores the transmitted words, so they can go through [decode_frame].
pub fn from_polarity_corrected(words: &[u32; GPS_WORDS_PER_FRAME]) -> [u32; GPS_WORDS_PER_FRAME] {
    let mut ret = [0u32; GPS_WORDS_PER_FRAME];
    let mut prev = 0u32;

    for (i, word) in words.iter().enumerate() {
        ret[i] = if prev & 0x1 > 0 {
            !word & GPS_WORD_MASK
        } else {
            word & GPS_WORD_MASK
        };

        prev = ret[i];
    }

    ret
}

/// Mirror operation of [from_polarity_corrected]: converts transmitted words
/// to the "polarity corrected" representation used by some receivers.
pub fn to_polarity_corrected(words: &[u32; GPS_WORDS_PER_FRAME]) -> [u32; GPS_WORDS_PER_FRAME] {
    let mut ret = [0u32; GPS_WORDS_PER_FRAME];
    let mut prev = 0u32;

    for (i, word) in words.iter().enumerate() {
        let word = word & GPS_WORD_MASK;

        ret[i] = if prev & 0x1 > 0 {
            !word & GPS_WORD_MASK
        } else {
            word
        };

        prev = word;
    }

    ret
}

/// Reads `count` (<= 32) bits, MSB first, starting at absolute bit position `bit_pos`.
/// Returns None if the slice is too short.
pub fn read_bits(slice: &[u8], bit_pos: usize, count: usize) -> Option<u32> {
    debug_assert!(count <= 32);

    if bit_pos + count > slice.len() * 8 {
        return None;
    }

    let mut value = 0u64;
    let first = bit_pos / 8;
    let last = (bit_pos + count + 7) / 8;

    for byte in slice[first..last].iter() {
        value = (value << 8) | (*byte as u64);
    }

    let total = (last - first) * 8;
    let shift = total - (bit_pos % 8) - count;

    Some(((value >> shift) & ((1u64 << count) - 1)) as u32)
}

/// Writes the `count` (<= 32) LSB of `value`, MSB first, at absolute bit position
/// `bit_pos`. Surrounding bits are preserved. Returns None if the slice is too short.
pub fn write_bits(slice: &mut [u8], bit_pos: usize, value: u32, count: usize) -> Option<()> {
    debug_assert!(count <= 32);

    if bit_pos + count > slice.len() * 8 {
        return None;
    }

    let mut pos = bit_pos;
    let mut remaining = count;

    while remaining > 0 {
        let byte = pos / 8;
        let avail = 8 - pos % 8;
        let take = avail.min(remaining);

        let chunk = ((value >> (remaining - take)) & ((1u32 << take) - 1)) as u8;
        let shift = avail - take;
        let mask = (((1u16 << take) - 1) as u8) << shift;

        slice[byte] = (slice[byte] & !mask) | (chunk << shift);

        pos += take;
        remaining -= take;
    }

    Some(())
}

/// Reads 10 consecutive 30-bit words starting at absolute bit position `bit_pos`.
pub fn read_words(slice: &[u8], bit_pos: usize) -> Option<[u32; GPS_WORDS_PER_FRAME]> {
    let mut words = [0u32; GPS_WORDS_PER_FRAME];

    for (i, word) in words.iter_mut().enumerate() {
        *word = read_bits(slice, bit_pos + i * GPS_WORD_BITS, GPS_WORD_BITS)?;
    }

    Some(words)
}

/// Writes 10 consecutive 30-bit words starting at absolute bit position `bit_pos`.
pub fn write_words(
    slice: &mut [u8],
    bit_pos: usize,
    words: &[u32; GPS_WORDS_PER_FRAME],
) -> Option<()> {
    for (i, word) in words.iter().enumerate() {
        write_bits(slice, bit_pos + i * GPS_WORD_BITS, *word, GPS_WORD_BITS)?;
    }

    Some(())
}

#[cfg(test)]
pub(crate) mod test {
    use super::*;

    /// Real subframes (EPH-1, 2, 3) as reported by a u-blox receiver
    /// (UBX-RXM-SFRBX), 30-bit words, right aligned.
    pub(crate) const UBLOX_EPH1: [u32; 10] = [
        0x22C13E1B, 0x1527C973, 0x13E40004, 0x104F5D31, 0x1744E6D7, 0x07755783, 0x330C80B5,
        0x125042A1, 0x00001684, 0x312C3033,
    ];

    pub(crate) const UBLOX_EPH2: [u32; 10] = [
        0x22C13E1B, 0x1527EA1B, 0x127FF165, 0x0C681F7C, 0x02493415, 0x3FF8811E, 0x191B8114,
        0x043E686E, 0x03347221, 0x10429F7B,
    ];

    pub(crate) const UBLOX_EPH3: [u32; 10] = [
        0x22C13E1B, 0x15280BDB, 0x000AEA34, 0x033CFFEE, 0x3FE5C9EB, 0x136FB64E, 0x06F4AB2C,
        0x0671EB44, 0x3FEAF602, 0x12455213,
    ];

    /// u-blox words, converted back to the transmitted bit stream
    pub(crate) fn on_air(frame: [u32; 10]) -> [u32; 10] {
        from_polarity_corrected(&frame)
    }

    #[test]
    fn parity_real_ublox_frames() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            let frame = on_air(frame);
            let mut prev = 0;
            for (i, word) in frame.iter().enumerate() {
                assert!(
                    decode_word(*word, prev, true).is_ok(),
                    "parity error on word #{} (0x{:08X})",
                    i + 1,
                    word
                );
                prev = *word;
            }

            // HOW and word 10 have D29 = D30 = 0
            assert_eq!(frame[1] & 0x3, 0);
            assert_eq!(frame[9] & 0x3, 0);
        }
    }

    #[test]
    fn polarity_corrected_reciprocal() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            assert_eq!(
                to_polarity_corrected(&from_polarity_corrected(&frame)),
                frame
            );
        }
    }

    #[test]
    fn frame_reciprocal_real_ublox() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            let frame = on_air(frame);
            let data = decode_frame(&frame, true).unwrap();
            assert_eq!(data[0], GPS_PREAMBLE_BYTE);

            // re-encoding the data bits must give back the exact same bit stream
            let encoded = encode_frame(&data);
            assert_eq!(encoded, frame);
        }
    }

    #[test]
    fn inverted_frame() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            let frame = on_air(frame);
            let mut inverted = frame;
            for word in inverted.iter_mut() {
                *word = !*word & GPS_WORD_MASK;
            }

            assert_eq!(
                decode_frame(&inverted, true).unwrap(),
                decode_frame(&frame, true).unwrap()
            );
        }
    }

    #[test]
    fn single_bit_errors_are_detected() {
        for frame in [UBLOX_EPH1, UBLOX_EPH2, UBLOX_EPH3] {
            let frame = on_air(frame);
            // skip preamble bits: those turn into InvalidPreamble
            for bit in 8..300 {
                let mut corrupted = frame;
                corrupted[bit / 30] ^= 1 << (29 - bit % 30);
                assert!(
                    decode_frame(&corrupted, true).is_err(),
                    "bit #{} flip not detected",
                    bit
                );
            }
        }
    }

    #[test]
    fn t_bits() {
        for (data, prev) in [(0x123400, 0), (0xABCDEF, 1), (0x000000, 2), (0xFFFFFF, 3)] {
            let word = encode_word_with_t_bits(data, prev);
            assert_eq!(word & 0x3, 0);
            let decoded = decode_word(word, prev, true).unwrap();
            assert_eq!(decoded & !0x3, data & !0x3);
        }
    }

    #[test]
    fn bit_io() {
        let mut buf = [0xffu8; 8];
        write_bits(&mut buf, 3, 0, 10).unwrap();
        assert_eq!(buf[0], 0xE0);
        assert_eq!(buf[1], 0x07);
        assert_eq!(read_bits(&buf, 3, 10), Some(0));
        assert_eq!(read_bits(&buf, 0, 3), Some(0x7));
        assert_eq!(read_bits(&buf, 13, 3), Some(0x7));

        for offset in 0..16 {
            let mut buf = [0u8; 16];
            write_bits(&mut buf, offset, 0x2AB_CDEF5 & GPS_WORD_MASK, 30).unwrap();
            assert_eq!(
                read_bits(&buf, offset, 30),
                Some(0x2AB_CDEF5 & GPS_WORD_MASK)
            );
            write_bits(&mut buf, offset, 0xDEAD_BEEF, 32).unwrap();
            assert_eq!(read_bits(&buf, offset, 32), Some(0xDEAD_BEEF));
        }

        assert!(read_bits(&[0u8; 4], 3, 30).is_none());
        assert!(write_bits(&mut [0u8; 4], 3, 0, 30).is_none());
    }
}
