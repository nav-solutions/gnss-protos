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
