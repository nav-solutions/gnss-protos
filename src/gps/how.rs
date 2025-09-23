use crate::gps::{GpsDataWord, GpsError};

const ZCOUNT_MASK: u32 = 0x3fffE000;
const ZCOUNT_SHIFT: u32 = 13;

const ALERT_MASK: u32 = 0x00001000;
const AS_MASK: u32 = 0x00000800;

const FRAMEID_MASK: u32 = 0x00000700;
const FRAMEID_SHIFT: u32 = 8;

use crate::gps::GpsQzssFrameId;

#[cfg(doc)]
use crate::gps::GpsQzssTelemetry;

/// [GpsQzssHow] (GPS Hand Over Word) marks the beginning of each frame, following [GpsQzssTelemetry],
/// and defines the content to follow.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssHow {
    /// TOW: elapsed time within current GPS week (in seconds),
    /// at the instant of transmission of the 1st bit of the next frame to follow
    /// this [GpsQzssHow] word.
    pub tow: u32,

    /// The alert bit serves two purposes.
    /// For block 000 satellites, '1' here indicates a maneuver.
    /// For other satellites, '1' here means the URA may be worse than indicated in subframe 1
    /// and user shall use this SV at their own risk.
    pub alert: bool,

    /// The A/S bit serves two purposes.
    /// For block 000 satellites, '1' here means the satellite is "synchronous",
    /// the leading edge of the TLM sync is the 1.5 second epoch instant, otherwise it
    /// is asynchronous.   
    /// For other satellite, this indicates A/S is active.
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
    /// Copies and returns [GpsQzssHow] with updated TOW in seconds.
    /// This value should be aligned to midnight and always a multiple of 6 seconds,
    /// the message transmission rate.
    pub fn with_tow_seconds(mut self, tow_seconds: u32) -> Self {
        self.tow = tow_seconds;
        self
    }

    /// Copies and returns [GpsQzssHow] with updated [GpsQzssFrameId]
    pub fn with_frame_id(mut self, frame_id: GpsQzssFrameId) -> Self {
        self.frame_id = frame_id;
        self
    }

    /// Copies and returns [GpsQzssHow] with updated alert bit asserted
    pub fn with_alert_bit(mut self) -> Self {
        self.alert = true;
        self
    }

    /// Copies and returns [GpsQzssHow] with updated alert bit deasserted
    pub fn without_alert_bit(mut self) -> Self {
        self.alert = false;
        self
    }

    /// Copies and returns [GpsQzssHow] with A/S bit asserted
    pub fn with_anti_spoofing(mut self) -> Self {
        self.anti_spoofing = true;
        self
    }

    /// Copies and returns [GpsQzssHow] with A/S bit deasserted
    pub fn without_anti_spoofing(mut self) -> Self {
        self.anti_spoofing = false;
        self
    }

    /// Constructs a default EPH-1 [GpsQzssHow]
    pub fn ephemeris1() -> Self {
        Self::default()
            .with_frame_id(GpsQzssFrameId::Ephemeris1)
    }

    /// Constructs a default EPH-2 [GpsQzssHow]
    pub fn ephemeris2() -> Self {
        Self::default()
            .with_frame_id(GpsQzssFrameId::Ephemeris1)
    }

    /// Constructs a default EPH-3 [GpsQzssHow]
    pub fn ephemeris3() -> Self {
        Self::default()
            .with_frame_id(GpsQzssFrameId::Ephemeris3)
    }

    /// Decodes [GpsQzssHow] from this [GpsDataWord].
    /// Subframe must be supported for this to work.
    pub(crate) fn from_word(word: GpsDataWord) -> Result<Self, GpsError> {
        let value = word.value();

        let zcount = (value & ZCOUNT_MASK) >> ZCOUNT_SHIFT;
        let frame_id = GpsQzssFrameId::decode(((value & FRAMEID_MASK) >> FRAMEID_SHIFT) as u8)?;
        let alert = (value & ALERT_MASK) > 0;
        let anti_spoofing = (value & AS_MASK) > 0;

        Ok(Self {
            alert,
            frame_id,
            anti_spoofing,
            tow: zcount * 3 /2,
        })
    }

    /// Encodes this [GpsQzssHow] word as [GpsDataWord].
    pub(crate) fn to_word(&self) -> GpsDataWord {
        let mut value = 0u32;

        if self.alert {
            value |= ALERT_MASK;
        }

        if self.anti_spoofing {
            value |= AS_MASK;
        }

        value |= ((self.tow * 2 / 3) & 0x1ffff) << ZCOUNT_SHIFT;
        value += (self.frame_id.encode() as u32) << FRAMEID_SHIFT;

        // TODO parity

        value <<= 2;

        GpsDataWord::from(value)
    }
}

#[cfg(test)]
mod test {
    use crate::gps::{GpsDataWord, GpsQzssFrameId, GpsQzssHow};

    #[test]
    fn encoding() {
        for (dword, tow, frame_id, alert, anti_spoofing) in [
            (0x12344400, 0x02468, GpsQzssFrameId::Ephemeris1, true, false),
            (
                0x12340400,
                0x02468,
                GpsQzssFrameId::Ephemeris1,
                false,
                false,
            ),
            (0x12342400, 0x02468, GpsQzssFrameId::Ephemeris1, false, true),
            (0x12342800, 0x02468, GpsQzssFrameId::Ephemeris2, false, true),
            (0x12342C00, 0x02468, GpsQzssFrameId::Ephemeris3, false, true),
            (0x32342C00, 0x06468, GpsQzssFrameId::Ephemeris3, false, true),
            (0x42342C00, 0x08468, GpsQzssFrameId::Ephemeris3, false, true),
            (0x82342C00, 0x10468, GpsQzssFrameId::Ephemeris3, false, true),
            (0x82342C00, 0x16789, GpsQzssFrameId::Ephemeris3, false, true),
        ] {
            let gps_word = GpsDataWord::from(dword);

            let gps_how = GpsQzssHow::from_word(gps_word).unwrap_or_else(|e| {
                panic!("failed to decode gps-how from 0x{:08X} : {}", dword, e);
            });

            assert_eq!(gps_how.tow, tow);
            assert_eq!(gps_how.alert, alert);
            assert_eq!(gps_how.frame_id, frame_id);
            assert_eq!(gps_how.anti_spoofing, anti_spoofing);

            assert_eq!(
                gps_how.to_word(),
                gps_word,
                "Encoding reciprocal issue for dword=0x{:08X}",
                dword
            );
        }
    }
}
