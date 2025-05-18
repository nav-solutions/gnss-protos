use crate::gps::{GpsQzssFrameId, GpsError};

#[cfg(doc)]
use crate::gps::GpsQzssTelemetry;

/// [GpsQzssHow] marks the beginning of each frame, following [GpsQzssTelemetry]
#[derive(Debug, Default, Clone)]
/// [GpsHowWord]
pub struct GpsQzssHow {
    /// TOW in seconds
    pub tow_s: u32,

    /// Following Frame ID (to decode following data words)
    pub frame_id: GpsQzssFrameId,

    /// When alert is asserted, the SV URA may be worse than indicated in subframe 1
    /// and user shall use this SV at their own risk.
    pub alert: bool,

    /// A-S mode is ON in that SV
    pub anti_spoofing: bool,
}

impl GpsQzssHow {
    pub(crate) fn decode(dword: u32) -> Result<Self, GpsError> {
        let tow = ((dword & 0x3fffe000) >> 13) as u32;

        let frame_id = GpsQzssFrameId::decode(((dword >> 8) & 0x07) as u8)?;

        let anti_spoofing = (dword & 0x08) > 0;
        let alert = (dword & 0x10) > 0;

        Ok(Self {
            alert,
            frame_id,
            anti_spoofing,
            tow_s: tow * 6,
        })
    }
}