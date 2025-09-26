#[derive(Copy, Clone, PartialEq)]
/// [GpsDataByte] aligned to 32 bits
pub enum GpsDataByte {
    /// 2-bit LSB padding, used to align the received bits to [u32].
    LsbPadded(u8),

    /// Plain byte (no padding, only meaningful bits).
    Byte(u8),
}

impl std::fmt::Debug for GpsDataByte {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:x}", self)
    }
}

impl core::fmt::LowerHex for GpsDataByte {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "0x{:02X}", self.as_u8())
    }
}

impl Default for GpsDataByte {
    /// Creates a default null [GpsDataByte::Byte].
    fn default() -> Self {
        Self::Byte(0)
    }
}

impl GpsDataByte {
    /// Stores provided un-padded byte with 2-bit LSB padding,
    /// to align one word to [u32].
    pub fn padded(byte: u8) -> Self {
        Self::LsbPadded(byte << 2)
    }

    /// Interprets internal value as [u8] to process
    /// internal byte correctly.
    pub(crate) fn as_u8(&self) -> u8 {
        match self {
            Self::LsbPadded(value) => (value >> 2) & 0x3f,
            Self::Byte(value) => *value,
        }
    }

    /// Interprets internal value as [u32] to process
    /// internal byte correctly.
    pub(crate) fn as_u32(&self) -> u32 {
        self.as_u8() as u32
    }
}
