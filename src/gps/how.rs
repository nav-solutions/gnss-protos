use crate::gps::{GpsError, GpsQzssFrameId};

#[cfg(doc)]
use crate::gps::GpsQzssTelemetry;

#[cfg(feature = "log")]
use log::trace;

const TOW_MASK: u32 = 0xffff8000;
const TOW_SHIFT: u32 = 15;
const ALERT_MASK: u32 = 0x00004000;
const AS_MASK: u32 = 0x00002000;
const FRAMEID_MASK: u32 = 0x00001C00;
const FRAMEID_SHIFT: u32 = 10;

/// [GpsQzssHow] marks the beginning of each frame, following [GpsQzssTelemetry]
#[derive(Debug, Default, Clone)]
/// [GpsHowWord]
pub struct GpsQzssHow {
    /// TOW (in seconds)
    pub tow: u32,

    /// Following Frame ID (to decode following data words)
    pub frame_id: GpsQzssFrameId,

    /// When alert is asserted, the SV URA may be worse than indicated in subframe 1
    /// and user shall use this SV at their own risk.
    pub alert: bool,

    /// A-S mode is ON in that SV
    pub anti_spoofing: bool,
}

impl GpsQzssHow {
    pub(crate) fn decode(dword: u32, _: bool) -> Result<Self, GpsError> {
        let tow = ((dword & TOW_MASK) >> TOW_SHIFT) as u32;
        let frame_id = GpsQzssFrameId::decode(((dword & FRAMEID_MASK) >> FRAMEID_SHIFT) as u8)?;
        let anti_spoofing = (dword & ALERT_MASK) > 0;
        let alert = (dword & AS_MASK) > 0;

        #[cfg(feature = "log")]
        trace!("GPS HOW dword=0x{:08x} id={:?}", dword, frame_id);

        Ok(Self {
            alert,
            frame_id,
            anti_spoofing,
            tow: tow * 6,
        })
    }
}
