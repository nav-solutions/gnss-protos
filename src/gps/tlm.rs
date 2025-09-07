use crate::gps::GpsError;

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

    /// Reserved bits
    pub reserved_bits: bool,
}

impl GpsQzssTelemetry {
    /// Copies and returns new [GpsQzssTelemetry] with updated 14-bit TLM message
    pub fn with_message(mut self, message_14b: u16) -> Self {
        self.message = message_14b & 0x3fff;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry with updated message integrity
    pub fn with_integrity(mut self, integrity: bool) -> Self {
        self.integrity = integrity;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry with updated reserved bits
    pub fn with_reserved_bits(mut self, reserved_bits: bool) -> Self {
        self.reserved_bits = reserved_bits;
        self
    }

    /// [GpsQzssTelemetry] decoding attempt.   
    /// The special GPS marker must be present on the MSB for this to pass.   
    /// When parity_check is requested, the parity check must pass as well.
    pub(crate) fn decode(dword: u32) -> Result<Self, GpsError> {
        if dword & PREAMBLE_MASK == PREAMBLE_MASK {
            return Err(GpsError::InvalidPreamble);
        };

        let message = ((dword & MESSAGE_MASK) >> MESSAGE_SHIFT) as u16;
        let integrity = (dword & INTEGRITY_BIT_MASK) > 0;
        let reserved_bits = (dword & RESERVED_BIT_MASK) > 0;

        Ok(Self {
            message,
            integrity,
            reserved_bits,
        })
    }

    /// [GpsQzssTelemetry] encoding as [u32]
    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0x22C00000;

        value |= (self.message as u32) << MESSAGE_SHIFT;

        if self.integrity {
            value |= INTEGRITY_BIT_MASK;
        }

        if self.reserved_bits {
            value |= RESERVED_BIT_MASK;
        }

        value
    }
}

#[cfg(test)]
mod tlm {
    use crate::gps::GpsQzssTelemetry;

    #[test]
    fn tlm_encoding() {
        for (dword, message, integrity, reserved_bit) in [
            (0x22C13E00, 0x13E, false, false),
            (0x22C13F80, 0x13F, true, false),
        ] {
            let tlm = GpsQzssTelemetry::decode(dword).unwrap_or_else(|e| {
                panic!("failed to decode gps-tlm from 0x{:08X} - {}", dword, e);
            });

            assert_eq!(tlm.message, message);
            assert_eq!(tlm.integrity, integrity);
            assert_eq!(tlm.reserved_bits, reserved_bit);

            let encoded = tlm.encode();

            assert_eq!(
                encoded, dword,
                "{:?} encoded to 0x{:08X} but 0x{:08X} is expected",
                tlm, encoded, dword
            );
        }
    }
}
