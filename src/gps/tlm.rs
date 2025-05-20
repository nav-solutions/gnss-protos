use crate::gps::GpsError;

pub(crate) const GPS_TLM_PREAMBLE_MASK: u32 = 0x8b000000;
const GPS_TLM_MESSAGE_MASK: u32 = 0x00fff800;
const GPS_TLM_MESSAGE_SHIFT: u32 = 11;
const GPS_TLM_INTEGRITY_BIT_MASK: u32 = 0x00000400;
const GPS_TLM_RESERVED_BIT_MASK: u32 = 0x00000200;

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
