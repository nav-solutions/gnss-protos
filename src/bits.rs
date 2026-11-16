//! Bit level helpers, shared by all protocols.
//!
//! GNSS messages are rarely aligned to bytes: these helpers read and write
//! bit fields at any (absolute) bit position of a byte slice, MSB first,
//! and convert physical values to / from their scaled binary representation.

use bitbuffer::{BigEndian, BitError, BitWriteStream};

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

/// Reads a single bit at absolute bit position `bit_pos` (no bound verification
/// other than the slice indexing).
#[inline]
pub(crate) fn get_bit(slice: &[u8], bit_pos: usize) -> bool {
    (slice[bit_pos / 8] >> (7 - bit_pos % 8)) & 0x1 > 0
}

/// Writes a single bit at absolute bit position `bit_pos`.
#[inline]
pub(crate) fn set_bit(slice: &mut [u8], bit_pos: usize, bit: bool) {
    let mask = 0x80u8 >> (bit_pos % 8);

    if bit {
        slice[bit_pos / 8] |= mask;
    } else {
        slice[bit_pos / 8] &= !mask;
    }
}

/// Converts an angle in radians to semicircles (the unit used by GNSS ICDs).
pub(crate) fn rad_to_semicircles(rad: f64) -> f64 {
    rad / core::f64::consts::PI
}

/// Writes a signed integer on `bits` bits (two's complement), clamped
/// to the range that this field can represent.
pub(crate) fn write_signed(
    stream: &mut BitWriteStream<'_, BigEndian>,
    value: i64,
    bits: usize,
) -> Result<(), BitError> {
    let max = (1i64 << (bits - 1)) - 1;
    let min = -(1i64 << (bits - 1));
    let value = value.clamp(min, max);
    let mask = if bits == 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    };
    stream.write_int((value as u32) & mask, bits)
}

/// Writes an unsigned integer on `bits` bits, clamped
/// to the range that this field can represent.
pub(crate) fn write_unsigned(
    stream: &mut BitWriteStream<'_, BigEndian>,
    value: i64,
    bits: usize,
) -> Result<(), BitError> {
    let max = if bits == 32 {
        u32::MAX as i64
    } else {
        (1i64 << bits) - 1
    };
    let value = value.clamp(0, max);
    stream.write_int(value as u32, bits)
}

/// Converts a physical value to its binary representation, using a 2^`pow2` scale factor.
pub(crate) fn to_scaled(value: f64, pow2: i32) -> i64 {
    (value * 2.0_f64.powi(-pow2)).round() as i64
}

/// Converts a binary value to its physical representation, using a 2^`pow2` scale factor.
pub(crate) fn from_scaled<T: Into<f64>>(value: T, pow2: i32) -> f64 {
    value.into() * 2.0_f64.powi(pow2)
}

#[cfg(test)]
mod test {
    use super::*;

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
            write_bits(&mut buf, offset, 0xDEAD_BEEF, 32).unwrap();
            assert_eq!(read_bits(&buf, offset, 32), Some(0xDEAD_BEEF));
        }

        assert!(read_bits(&[0u8; 4], 3, 30).is_none());
        assert!(write_bits(&mut [0u8; 4], 3, 0, 30).is_none());
    }

    #[test]
    fn single_bit_io() {
        let mut buf = [0u8; 4];
        for pos in [0, 5, 7, 8, 19, 31] {
            set_bit(&mut buf, pos, true);
            assert!(get_bit(&buf, pos));
            assert_eq!(read_bits(&buf, pos, 1), Some(1));
            set_bit(&mut buf, pos, false);
            assert!(!get_bit(&buf, pos));
        }
        assert_eq!(buf, [0u8; 4]);
    }

    #[test]
    fn scaling() {
        assert_eq!(to_scaled(1.0, -3), 8);
        assert_eq!(from_scaled(8, -3), 1.0);
        assert_eq!(to_scaled(-0.5, -1), -1);
        assert!((rad_to_semicircles(core::f64::consts::PI) - 1.0).abs() < 1e-15);
    }
}
