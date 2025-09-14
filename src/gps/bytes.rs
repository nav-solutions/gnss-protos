pub(crate) struct ByteArray<'a> {
    slice: &'a [GpsDataByte],
}

impl<'a> ByteArray<'a> {
    pub fn new(slice: &'a [GpsDataByte]) -> Self {
        Self { slice }
    }

    /// Converts a slice of MSBF [GpsDataByte]s to [u32] correctly LSB aligned,
    /// stripped of termination padding.
    pub fn value_u32(&self) -> u32 {
        let mut value = 0;

        match self.slice[0] {
            GpsDataByte::MsbPadded(byte) => {
                value |= ((byte & 0x3f) as u32) << 24;
            },
            GpsDataByte::Byte(byte) => {
                value |= (byte as u32) << 24;
            },
            GpsDataByte::LsbPadded(byte) => {},
        }

        match self.slice[1] {
            GpsDataByte::Byte(byte) => {
                value |= (byte as u32) << 16;
            },
            GpsDataByte::MsbPadded(byte) => {
                value >>= 2;
                value |= ((byte & 0x3f) as u32) << 16;
            },
            GpsDataByte::LsbPadded(byte) => {
                value >>= 2;
                value |= (((byte >> 2) & 0x3f) as u32) << 16;
            },
        }

        match self.slice[2] {
            GpsDataByte::Byte(byte) => {
                value |= (byte as u32) << 8;
            },
            GpsDataByte::MsbPadded(byte) => {
                value >>= 2;
                value |= ((byte & 0x3f) as u32) << 8;
            },
            GpsDataByte::LsbPadded(byte) => {
                value >>= 2;
                value |= (((byte >> 2) & 0x3f) as u32) << 8;
            },
        }

        match self.slice[3] {
            GpsDataByte::Byte(byte) => {
                value |= byte as u32;
            },
            GpsDataByte::MsbPadded(byte) => {
                value >>= 2;
                value |= (byte & 0x3f) as u32;
            },
            GpsDataByte::LsbPadded(byte) => {
                value >>= 2;
                value |= ((byte >> 2) & 0x3f) as u32;
            },
        }

        value
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
/// [GpsDataByte] aligned to 32 bits
pub enum GpsDataByte {
    /// 2-bit MSB padding.
    /// Usually used at the beginning or end of GPS word to align GPS stream correctly.
    MsbPadded(u8),

    /// 2-bit LSB padding.
    /// Usually used at the beginning or end of GPS word to align GPS stream correctly.
    LsbPadded(u8),

    /// Plain byte (no padding)
    Byte(u8),
}

impl Default for GpsDataByte {
    fn default() -> Self {
        Self::Byte(0)
    }
}

impl core::fmt::LowerExp for GpsDataByte {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MsbPadded(value) => write!(f, "0x{:02}X", value),
            Self::LsbPadded(value) => write!(f, "0x{:02}X", (value << 2)),
            Self::Byte(value) => write!(f, "0x{:02}X", value),
        }
    }
}

impl GpsDataByte {
    /// Stores provided byte as 2-bit MSB padded [u8]
    pub fn msb_padded(byte: u8) -> Self {
        Self::MsbPadded(byte & 0x3f)
    }

    /// Stores provided byte as 2-bit LSB padded [u8]
    pub fn lsb_padded(byte: u8) -> Self {
        Self::LsbPadded((byte & 0xfc) >> 2)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn bytearray() {
        for (bytes, value) in [
            (
                [
                    GpsDataByte::MsbPadded(0x00),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                ],
                0x00000000,
            ),
            (
                [
                    GpsDataByte::MsbPadded(0x90),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                ],
                0x10000000,
            ),
            (
                [
                    GpsDataByte::MsbPadded(0x10),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                ],
                0x10000000,
            ),
            (
                [
                    GpsDataByte::MsbPadded(0xB0),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                ],
                0x30000000,
            ),
            (
                [
                    GpsDataByte::MsbPadded(0x01),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                    GpsDataByte::Byte(0x00),
                ],
                0x01000000,
            ),
            (
                [
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::MsbPadded(0x01),
                ],
                0x00404041,
            ),
            (
                [
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::LsbPadded(0x01),
                ],
                0x00404040,
            ),
            (
                [
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::LsbPadded(0x03),
                ],
                0x00404040,
            ),
            (
                [
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::Byte(0x01),
                    GpsDataByte::LsbPadded(0x04),
                ],
                0x00404041,
            ),
        ] {
            let array = ByteArray::new(&bytes);

            let result = array.value_u32();
            assert_eq!(
                value, result,
                "failed to test value 0x{:08X} got 0x{:08X}",
                value, result
            );
        }
    }
}
