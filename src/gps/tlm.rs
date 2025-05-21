use crate::gps::{decoder::GPS_PARITY_MASK, GpsError};

const PREAMBLE_MASK: u32 = 0x42C00000;

const MESSAGE_MASK: u32 = 0x003fff00;
const MESSAGE_SHIFT: u32 = 8;

const INTEGRITY_BIT_MASK: u32 = 0x00000080;
const RESERVED_BIT_MASK: u32 = 0x00000040;

#[cfg(feature = "log")]
use log::debug;

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
    /// [GpsQzssTelemetry] decoding attempt.   
    /// The special GPS marker must be present on the MSB for this to pass.   
    /// When parity_check is requested, the parity check must pass as well.
    pub fn decode(dword: u32) -> Result<Self, GpsError> {
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
}
