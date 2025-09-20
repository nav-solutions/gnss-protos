use crate::gps::GpsDataByte;

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
    /// Creates a [GpsDataWord] from a 30-bit slice stored as 4 big-endian
    /// bytes. We consider two upper padding bits on the MSB, which are inevitably lost.
    /// We do not supported padding on any other position (all other bits are preserved).
    pub fn from_be_bytes(slice: &[u8; 4]) -> Self {
        Self {
            bytes: [
                GpsDataByte::MsbPadded(slice[0]),
                GpsDataByte::Byte(slice[1]),
                GpsDataByte::Byte(slice[2]),
                GpsDataByte::Byte(slice[3]),
            ],
        }
    }

    /// Creates a [GpsDataWord] from a 30-bit slice stored as 4 little-endian
    /// bytes. We consider two upper padding bits on the MSB, which are inevitably lost.
    /// We do not supported padding on any other position (all other bits are preserved).
    pub fn from_le_bytes(slice: &[u8; 4]) -> Self {
        Self {
            bytes: [
                GpsDataByte::MsbPadded(slice[3]),
                GpsDataByte::Byte(slice[2]),
                GpsDataByte::Byte(slice[1]),
                GpsDataByte::Byte(slice[0]),
            ],
        }
    }

    /// Converts this [GpsDataWord] to [u32]
    pub fn value(&self) -> u32 {
        let mut value = self.bytes[3].as_u32();
        value |= (self.bytes[2].as_u32()) << 8;
        value |= (self.bytes[1].as_u32()) << 16;
        value |= (self.bytes[0].as_u32()) << 24;

        value
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn from_be_bytes() {
        for (bytes, value) in [
            ([0x12, 0x34, 0x56, 0x78], 0x12345678),
            ([0x78, 0x56, 0x34, 0x12], 0x38563412),
            ([0x22, 0xC1, 0x3E, 0x1B], 0x22C13E1B),
            ([0xF2, 0xC1, 0x3E, 0x1B], 0x32C13E1B),
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
            ([0x12, 0x34, 0x56, 0x78], 0x38563412),
            ([0x12, 0x34, 0x56, 0x38], 0x38563412),
            ([0xF2, 0x34, 0x56, 0x38], 0x385634F2),
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
            (0x32563412u32, 0x32563412),
            (0xF2563412u32, 0x32563412),
            (0x00563410u32, 0x00563410),
            (0x00333410u32, 0x00333410),
        ] {
            let word = GpsDataWord::from(value);
            assert_eq!(word.value(), expected_value);
        }
    }
}
