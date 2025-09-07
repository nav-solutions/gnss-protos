use crate::gps::GpsError;

const TOW_MASK: u32 = 0x3fffE000;
const TOW_SHIFT: u32 = 13;

const ALERT_MASK: u32 = 0x00001000;
const AS_MASK: u32 = 0x00000800;

const FRAMEID_MASK: u32 = 0x00000700;
const FRAMEID_SHIFT: u32 = 8;

use crate::gps::GpsQzssFrameId;

#[cfg(doc)]
use crate::gps::GpsQzssTelemetry;

/// [GpsQzssHow] marks the beginning of each frame, following [GpsQzssTelemetry]
#[derive(Debug, Default, Copy, Clone, PartialEq)]
/// [GpsHowWord]
pub struct GpsQzssHow {
    /// TOW (in seconds)
    pub tow: u32,

    /// When alert is asserted, the SV URA may be worse than indicated in subframe 1
    /// and user shall use this SV at their own risk.
    pub alert: bool,

    /// A-S mode is ON in that SV
    pub anti_spoofing: bool,

    /// Following Frame ID (to decode following data words)
    pub frame_id: GpsQzssFrameId,
}

impl GpsQzssHow {
    /// Copies and returns [GpsQzssHow] with updated TOW in seconds
    pub fn with_tow_seconds(mut self, tow_seconds: u32) -> Self {
        self.tow = tow_seconds;
        self
    }

    /// Copies and returns [GpsQzssHow] with updated [GpsQzssFrameId]
    pub fn with_frame_id(mut self, frame_id: GpsQzssFrameId) -> Self {
        self.frame_id = frame_id;
        self
    }
    /// Copies and returns [GpsQzssHow] with updated alert bit
    pub fn with_alert_bit(mut self, alert: bool) -> Self {
        self.alert = alert;
        self
    }

    /// Copies and returns [GpsQzssHow] with updated A/S bit
    pub fn with_anti_spoofing(mut self, anti_spoofing: bool) -> Self {
        self.anti_spoofing = anti_spoofing;
        self
    }

    /// Builds an Ephemeris #1 [GpsQzssHow]
    pub fn ephemeris1() -> Self {
        Self {
            tow: 0,
            alert: false,
            anti_spoofing: false,
            frame_id: GpsQzssFrameId::Ephemeris1,
        }
    }

    /// Builds an Ephemeris #2 [GpsQzssHow]
    pub fn ephemeris2() -> Self {
        Self {
            tow: 0,
            alert: false,
            anti_spoofing: false,
            frame_id: GpsQzssFrameId::Ephemeris2,
        }
    }

    /// Builds an Ephemeris #3 [GpsQzssHow]
    pub fn ephemeris3() -> Self {
        Self {
            tow: 0,
            alert: false,
            anti_spoofing: false,
            frame_id: GpsQzssFrameId::Ephemeris3,
        }
    }

    pub(crate) fn decode(dword: u32) -> Result<Self, GpsError> {
        let tow = ((dword & TOW_MASK) >> TOW_SHIFT) as u32;
        let frame_id = GpsQzssFrameId::decode(((dword & FRAMEID_MASK) >> FRAMEID_SHIFT) as u8)?;
        let alert = (dword & ALERT_MASK) > 0;
        let anti_spoofing = (dword & AS_MASK) > 0;

        Ok(Self {
            alert,
            frame_id,
            anti_spoofing,
            tow: tow * 6,
        })
    }
}
