use crate::gps::GpsError;

pub(crate) const GPS_TLM_PREAMBLE_MASK: u32 = 0x8b000000;
const GPS_TLM_MESSAGE_MASK: u32 = 0x00fff800;
const GPS_TLM_MESSAGE_SHIFT: u32 = 11;
const GPS_TLM_INTEGRITY_BIT_MASK: u32 = 0x00000400;
const GPS_TLM_RESERVED_BIT_MASK: u32 = 0x00000200;

#[cfg(feature = "log")]
use log::trace;

impl GpsQzssTelemetry {
    pub(crate) fn decode(dword: u32, _: bool) -> Result<Self, GpsError> {
        #[cfg(feature = "log")]
        trace!("GPS TLM dword=0x{:08x}", dword);

        // preamble verification
        if dword & GPS_TLM_PREAMBLE_MASK == GPS_TLM_PREAMBLE_MASK {
            return Err(GpsError::InvalidPreamble);
        }

        let tlm_message = ((dword & GPS_TLM_MESSAGE_MASK) >> GPS_TLM_MESSAGE_SHIFT) as u16;
        let integrity = (dword & GPS_TLM_INTEGRITY_BIT_MASK) > 0;
        let reserved = (dword & GPS_TLM_RESERVED_BIT_MASK) > 0;

        Ok(Self {
            tlm_message,
            integrity,
            reserved,
        })
    }
}

/// [GpsQzssTelemetry] marks the beginning of each frame
#[derive(Debug, Default, Clone)]
pub struct GpsQzssTelemetry {
    /// TLM Message
    pub tlm_message: u16,

    /// Integrity bit is asserted means the conveying signal is provided
    /// with an enhanced level of integrity assurance.
    pub integrity: bool,

    /// Reserved bit
    pub reserved: bool,
}
