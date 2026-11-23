/// Number of chips per byte, in packed code storage.
const PACKED_CHIPS: usize = 8;

/// Writes chip `index` (logic level) of a code packed MSB first.
#[inline]
pub(crate) fn set_packed_chip(code: &mut [u8], index: usize, chip: bool) {
    if chip {
        code[index / PACKED_CHIPS] |= 0x80 >> (index % PACKED_CHIPS);
    }
}

/// Reads chip `index` (logic level) of a code packed MSB first.
#[inline]
pub(crate) fn packed_chip(code: &[u8], index: usize) -> bool {
    (code[index / PACKED_CHIPS] >> (7 - index % PACKED_CHIPS)) & 0x1 > 0
}

/// Maps a logic level to a BPSK sample (logic `0` -> `+1`, logic `1` -> `-1`).
#[inline]
pub(crate) const fn bpsk(logic: bool) -> i8 {
    if logic {
        -1
    } else {
        1
    }
}
