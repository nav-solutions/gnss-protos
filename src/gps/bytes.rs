#[derive(Debug)]
/// [GpsDataByte] aligned to 32 bits
pub enum GpsDataByte {
    /// 2-bit MSB padding.
    /// Usually used at the beginning or stream termination by computers.
    MsbPadded(u8),

    /// 2-bit LSB padding.
    /// Usually used at the beginning or stream termination by computers.
    LsbPadded(u8),

    /// Plain byte
    Byte(u8),
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
