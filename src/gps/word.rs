use crate::gps::{GpsDataByte, GpsError, GPS_PARITY_MASK, GPS_PARITY_SIZE, GPS_PAYLOAD_MASK};

/// Counters number of bits set to '1'
fn count_bits(value: u32) -> u32 {
    let mut count = 0;
    let mut mask = 0x01u32;

    for i in 0..32 {
        if value & mask > 0 {
            count += 1;
        }

        mask <<= 1;
    }

    count
}

/// [GpsDataWord] is used to pack [GpsDataByte]s.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsDataWord {
    /// [GpsDataByte]s stored with "big endianness"
    bytes: [GpsDataByte; 4],
}

impl Default for GpsDataWord {
    /// Creates a default null [GpsDataWord].
    fn default() -> Self {
        Self {
            bytes: Default::default(),
        }
    }
}

impl core::ops::BitOrAssign<u8> for GpsDataWord {
    fn bitor_assign(&mut self, rhs: u8) {
        let mut value = self.value();
        value |= (rhs as u32);
        *self = Self::from(value << 2)
    }
}

impl core::ops::BitOrAssign<u32> for GpsDataWord {
    fn bitor_assign(&mut self, rhs: u32) {
        let mut value = self.value();
        value |= rhs;
        *self = Self::from(value << 2)
    }
}

impl From<u32> for GpsDataWord {
    fn from(value: u32) -> GpsDataWord {
        GpsDataWord::from_be_bytes(&value.to_be_bytes())
    }
}

impl Into<u32> for GpsDataWord {
    fn into(self) -> u32 {
        self.value()
    }
}

impl GpsDataWord {
    /// Creates a [GpsDataWord] from a 30-bit slice stored as 4 big-endian bytes.
    pub fn from_be_bytes(slice: &[u8; 4]) -> Self {
        Self {
            bytes: [
                GpsDataByte::Byte(slice[0]),
                GpsDataByte::Byte(slice[1]),
                GpsDataByte::Byte(slice[2]),
                GpsDataByte::LsbPadded(slice[3]),
            ],
        }
    }

    /// Creates a [GpsDataWord] from a 30-bit slice stored as 4 little-endian bytes.
    pub fn from_le_bytes(slice: &[u8; 4]) -> Self {
        Self {
            bytes: [
                GpsDataByte::Byte(slice[3]),
                GpsDataByte::Byte(slice[2]),
                GpsDataByte::Byte(slice[1]),
                GpsDataByte::LsbPadded(slice[0]),
            ],
        }
    }

    /// Converts this [GpsDataWord] to [u32]
    pub fn value(&self) -> u32 {
        let mut value = self.bytes[3].as_u32();
        value |= self.bytes[2].as_u32() << 6;
        value |= self.bytes[1].as_u32() << 14;
        value |= self.bytes[0].as_u32() << 22;
        value
    }

    /// Evaluates the parity of this data word,
    /// using provided NIB which must be set to zero on initial cycle.
    /// 6-bit parity is encoded as [u8].
    pub fn parity(&self, rhs: &Self, nib: bool) -> u8 {
        const BITMASKS: [u32; 6] = [
            0x3B1F3480, 0x1D8F9A40, 0x2EC7CD00, 0x1763E680, 0x2BB1F340, 0x0B7A89C0,
        ];

        let prev = rhs.value();

        let (b29, b30) = ((prev & 0x2) >> 1, prev & 0x01);

        let mut d = (self.value() & GPS_PAYLOAD_MASK) >> GPS_PARITY_SIZE;

        if nib {
            if ((b30 + count_bits(BITMASKS[4] & d)) % 2) > 0 {
                d ^= 1 << 6;
            }

            if ((b29 + count_bits(BITMASKS[5]) & d) % 2) > 0 {
                d ^= 1 << 7;
            }
        }

        let mut b = d;

        if b30 > 0 {
            b ^= 0x3fffffc0;
        }

        b |= (b29 + count_bits(BITMASKS[0]) & d % 2) << 5;
        b |= (b30 + count_bits(BITMASKS[1]) & d % 2) << 4;
        b |= (b29 + count_bits(BITMASKS[2]) & d % 2) << 3;
        b |= (b30 + count_bits(BITMASKS[3]) & d % 2) << 2;
        b |= (b30 + count_bits(BITMASKS[4]) & d % 2) << 1;
        b |= b29 + count_bits(BITMASKS[5]) & d % 2;

        b &= 0x3fffffff;

        (b & GPS_PARITY_MASK) as u8
    }

    /// Verifies the parity of this [GpsDataWord], possibly invalidating this word.
    pub fn parity_check(&self, rhs: &Self, nib: bool) -> Result<(), GpsError> {
        let parity = self.parity(rhs, nib);

        if (self.value() & GPS_PARITY_MASK) as u8 == parity {
            Ok(())
        } else {
            Err(GpsError::Parity)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn from_be_bytes() {
        for (bytes, value) in [
            ([0x8B, 0x12, 0x48, 0xCA], 0x22C49232),
            ([0x8B, 0xAA, 0xAA, 0xAA], 0x22EAAAAA),
            ([0x8B, 0xA5, 0xA5, 0xA5], 0x22E96969),
            ([0x8B, 0x5A, 0x5A, 0x5A], 0x22D69696),
        ] {
            let word = GpsDataWord::from_be_bytes(&bytes);
            let found = word.value();
            assert_eq!(
                found, value,
                "expected 0x{:08X}, got 0x{:08X}",
                value, found
            );
        }
    }

    #[test]
    fn from_le_bytes() {
        for (bytes, value) in [
            ([0xCA, 0x48, 0x12, 0x8B], 0x22C49232),
            ([0xAA, 0xAA, 0xAA, 0x8B], 0x22EAAAAA),
            ([0xA5, 0xA5, 0xA5, 0x8B], 0x22E96969),
            ([0x5A, 0x5A, 0x5A, 0x8B], 0x22D69696),
        ] {
            let word = GpsDataWord::from_le_bytes(&bytes);
            let found = word.value();
            assert_eq!(
                found, value,
                "expected 0x{:08X}, got 0x{:08X}",
                value, found
            );
        }
    }

    #[test]
    fn from_u32() {
        for (value, expected_value) in [
            (0x32563412u32, 0x32563412 >> 2),
            (0xF2563412u32, 0xF2563412 >> 2),
            (0xFFFF3412u32, 0xFFFF3412 >> 2),
            (0xFFFF34FFu32, 0xFFFF34FF >> 2),
        ] {
            let word = GpsDataWord::from(value);
            assert_eq!(
                word.value(),
                expected_value,
                "got 0x{:08X} expecting 0x{:08X}",
                word.value(),
                expected_value
            );
        }
    }

    #[test]
    fn binmask() {
        for (dword, mask, initial_value, final_value) in [
            (0x00000004, 0x03u8, 0x1, 0x00000003),
            (0x00000004, 0x08u8, 0x1, 0x00000009),
            (0x00000004, 0x20u8, 0x1, 0x00000021),
        ] {
            let mut word = GpsDataWord::from(dword);
            assert_eq!(word.value(), initial_value);

            word |= mask;
            assert_eq!(word.value(), final_value);
        }
    }

    // #[test]
    // fn parity_calc() {
    //     for (bin, b29, b30, expected) in [
    //         (
    //             0b101101001001011010010101110110u32,
    //             0,
    //             0,
    //             0b000000u8,
    //         ),
    //         ] {

    //         let word = GpsDataWord::from(bin << 6);
    //         let parity = word.parity(&Default::default(), false);

    //         assert_eq!(
    //             parity, expected,
    //             "got 0x{:02X}, expecting 0x{:02X}",
    //             parity, expected
    //         );
    //     }
    // }

    // #[test]
    // fn parity_ok_checker() {
    //     for (b29, b30, dword) in [(0, 0, 0x8B1248CA)] {
    //         let word = GpsDataWord::from(dword);

    //         assert!(
    //             word.parity_check(&Default::default(), false).is_ok(),
    //             "failed for 0x{:08X}",
    //             dword
    //         );
    //     }
    // }

    // #[test]
    // fn parity_nok_checker() {
    //     for dword in [0x8B1248CA] {
    //         let word = GpsDataWord::from(dword);
    //         assert!(
    //             word.parity_check(&Default::default(), false).is_err(),
    //             "failed for 0x{:08X}",
    //             dword
    //         );
    //     }
    // }

    #[test]
    fn test_asserted_bits() {
        for (dword, bits) in [
            (0x00000001, 1),
            (0x00000010, 1),
            (0x00000011, 2),
            (0x00000100, 1),
            (0x00000101, 2),
            (0x00000110, 2),
            (0x00000111, 3),
            (0x10000000, 1),
            (0x01000000, 1),
            (0x00100000, 1),
            (0x11000000, 2),
            (0x11100000, 3),
            (0x11100111, 6),
            (0x11100113, 7),
            (0x11100333, 9),
            (0x81100333, 9),
            (0xC1100333, 10),
            (0xF110033F, 14),
        ] {
            assert_eq!(count_bits(dword), bits, "failed for input 0x{:08X}", dword);
        }
    }
}
