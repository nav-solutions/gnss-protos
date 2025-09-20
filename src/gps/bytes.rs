#[derive(Debug, Copy, Clone, PartialEq)]
/// [GpsDataByte] aligned to 32 bits
pub enum GpsDataByte {
    /// 2-bit MSB padding.
    /// Usually used at the beginning or end of GPS word to align GPS stream correctly.
    MsbPadded(u8),

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
            Self::Byte(value) => write!(f, "0x{:02}X", value),
        }
    }
}

impl GpsDataByte {
    /// Stores provided byte as 2-bit MSB padded [u8]
    pub fn msb_padded(byte: u8) -> Self {
        Self::MsbPadded(byte & 0x3f)
    }

    pub(crate) fn as_u8(&self) -> u8 {
        match self {
            Self::MsbPadded(value) => value & 0x3f,
            Self::Byte(value) => *value,
        }
    }

    pub(crate) fn as_u32(&self) -> u32 {
        match self {
            Self::MsbPadded(value) => (value & 0x3f) as u32,
            Self::Byte(value) => *value as u32,
        }
    }
}
