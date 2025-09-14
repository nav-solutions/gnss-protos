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

/// [GpsQzssHow] (GPS HandOver Word) marks the beginning of each frame, following [GpsQzssTelemetry],
/// and defines the content to follow.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssHow {
    /// 17-bit TOW (in seconds)
    pub tow: u32,

    /// When alert is asserted, the SV URA may be worse than indicated in subframe 1
    /// and user shall use this SV at their own risk.
    pub alert: bool,

    /// A-S mode is ON in that SV
    pub anti_spoofing: bool,

    /// Following Frame ID (to decode following data words)
    pub frame_id: GpsQzssFrameId,
}

#[cfg(feature = "std")]
impl std::fmt::Display for GpsQzssHow {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "TOW={} - ALERT={} - A/S={} - ID={}",
            self.tow, self.alert, self.anti_spoofing, self.frame_id
        )
    }
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
        let tow = (dword & TOW_MASK) >> TOW_SHIFT;
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

    /// Encodes this [GpsQzssHow] word as [u32]
    pub(crate) fn encode(&self) -> u32 {
        let mut value = 0;

        if self.alert {
            value |= ALERT_MASK;
        }

        if self.anti_spoofing {
            value |= AS_MASK;
        }

        value |= ((self.tow / 6) & 0x1ffff) << TOW_SHIFT;
        value += (self.frame_id.encode() as u32) << FRAMEID_SHIFT;

        value
    }
}

#[cfg(test)]
mod test {
    use super::{TOW_MASK, TOW_SHIFT};
    use crate::gps::{GpsQzssFrameId, GpsQzssHow};

    #[test]
    fn how_encoding() {
        for (dword, tow, frame_id, alert, anti_spoofing) in [
            (0x1527C173, 259956, GpsQzssFrameId::Ephemeris1, false, false),
            (0x1527C973, 259956, GpsQzssFrameId::Ephemeris1, false, true),
            (0x1527EA1B, 259962, GpsQzssFrameId::Ephemeris2, false, true),
            (0x1527E21B, 259962, GpsQzssFrameId::Ephemeris2, false, false),
            (0x1527F21B, 259962, GpsQzssFrameId::Ephemeris2, true, false),
            (0x1527FA1B, 259962, GpsQzssFrameId::Ephemeris2, true, true),
            (0x15280BDB, 259968, GpsQzssFrameId::Ephemeris3, false, true),
            (0x152803DB, 259968, GpsQzssFrameId::Ephemeris3, false, false),
            (0x152813DB, 259968, GpsQzssFrameId::Ephemeris3, true, false),
        ] {
            let gps_how = GpsQzssHow::decode(dword).unwrap_or_else(|e| {
                panic!("failed to decode gps-how from 0x{:08X} : {}", dword, e);
            });

            let expected_tow = (dword & TOW_MASK) >> TOW_SHIFT;
            assert_eq!(tow, expected_tow * 6);

            assert_eq!(gps_how.tow, tow);
            assert_eq!(gps_how.alert, alert);
            assert_eq!(gps_how.frame_id, frame_id);
            assert_eq!(gps_how.anti_spoofing, anti_spoofing);

            let encoded = gps_how.encode();

            let decoded = GpsQzssHow::decode(encoded).unwrap_or_else(|e| {
                panic!(
                    "failed to decode previously encoded gps-how (0x{:08X}) : {}",
                    encoded, e
                );
            });

            assert_eq!(decoded, gps_how);
        }
    }
}
