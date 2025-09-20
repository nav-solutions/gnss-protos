use crate::gps::{GpsDataWord, GpsError};

const PREAMBLE_MASK: u32 = 0x42C00000;

const MESSAGE_MASK: u32 = 0x003fff00;
const MESSAGE_SHIFT: u32 = 8;

const INTEGRITY_BIT_MASK: u32 = 0x00000080;
const RESERVED_BIT_MASK: u32 = 0x00000040;

/// [GpsQzssTelemetry] marks the beginning of each frame
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssTelemetry {
    /// 14-bit TLM Message
    pub message: u16,

    /// Integrity bit is asserted means the conveying signal is provided
    /// with an enhanced level of integrity assurance.
    pub integrity: bool,

    /// Reserved bit
    pub reserved_bit: bool,
}

#[cfg(feature = "std")]
impl std::fmt::Display for GpsQzssTelemetry {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "INTEGRITY={} - MSG=0x{:08X} - reserved={}",
            self.integrity, self.message, self.reserved_bit
        )
    }
}

impl GpsQzssTelemetry {
    /// Copies and returns new [GpsQzssTelemetry] with updated 14-bit TLM message
    pub fn with_message(mut self, message_14b: u16) -> Self {
        self.message = message_14b & 0x3fff;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with message integrity asserted
    pub fn with_integrity(mut self) -> Self {
        self.integrity = true;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with message integrity deasserted
    pub fn without_integrity(mut self) -> Self {
        self.integrity = false;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with reserved bit asserted
    pub fn with_reserved_bit(mut self) -> Self {
        self.reserved_bit = true;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with reserved bit deasserted
    pub fn without_reserved_bit(mut self) -> Self {
        self.reserved_bit = false;
        self
    }

    /// [GpsQzssTelemetry] decoding attempt from a [GpsDataWord].
    /// The special GPS marker must be present (MSB) for this to pass.
    pub(crate) fn from_word(word: GpsDataWord) -> Result<Self, GpsError> {
        let value = word.value();

        if value & PREAMBLE_MASK == PREAMBLE_MASK {
            return Err(GpsError::InvalidPreamble);
        };

        let message = ((value & MESSAGE_MASK) >> MESSAGE_SHIFT) as u16;
        let integrity = (value & INTEGRITY_BIT_MASK) > 0;
        let reserved_bit = (value & RESERVED_BIT_MASK) > 0;

        Ok(Self {
            message,
            integrity,
            reserved_bit,
        })
    }

    /// Encodes this [GpsQzssTelemetry] as [GpsDataWord].
    pub(crate) fn to_word(&self) -> GpsDataWord {
        let mut value = 0x22C00000u32;

        value |= (self.message as u32) << MESSAGE_SHIFT;

        if self.integrity {
            value |= INTEGRITY_BIT_MASK;
        }

        if self.reserved_bit {
            value |= RESERVED_BIT_MASK;
        }

        GpsDataWord::from(value)
    }
}

#[cfg(test)]
mod tlm {
    use crate::gps::{GpsDataWord, GpsQzssTelemetry};

    #[test]
    fn tlm_encoding() {
        for (dword, message, integrity, reserved_bit) in [
            (0x22C13E00u32, 0x13E, false, false),
            (0x22C13F80u32, 0x13F, true, false),
            (0x22C13FC0u32, 0x13F, true, true),
            (0x22C13F40u32, 0x13F, false, true),
        ] {
            let gps_word = GpsDataWord::from(dword);

            let tlm = GpsQzssTelemetry::from_word(gps_word).unwrap_or_else(|e| {
                panic!("failed to decode gps-tlm from 0x{:08X} - {}", dword, e);
            });

            assert_eq!(tlm.message, message);
            assert_eq!(tlm.integrity, integrity);
            assert_eq!(tlm.reserved_bit, reserved_bit);

            let encoded = tlm.to_word();
            assert_eq!(tlm.to_word(), gps_word, "Reciprocal issue");
        }
    }
}
