use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    gps::{GpsQzssFrameId, GPS_WORD_BITS, GPS_WORD_BYTES},
    Message,
};

#[cfg(doc)]
use crate::gps::GpsQzssTelemetry;

/// [GpsQzssHow] (GPS Hand Over Word) marks the beginning of each frame, following [GpsQzssTelemetry],
/// and defines the content to follow.
#[derive(Debug, Copy, Clone, PartialEq)]
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
    /// the leading edge of the TLM sync is the 0.5 second epoch instant, otherwise it
    /// is asynchronous.   
    /// For other satellite, this indicates A/S is active.
    pub anti_spoofing: bool,

    /// Following Frame ID (to decode following data words)
    pub frame_id: GpsQzssFrameId,
}

impl Message<BigEndian> for GpsQzssHow {
    fn encoding_size(&self) -> usize {
        GPS_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GPS_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GpsQzssHow {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let zcount = stream.read_int::<u32>(17)?;
        let alert = stream.read_bool()?;
        let anti_spoofing = stream.read_bool()?;
        let frame_id = stream.read_int::<u8>(3)?;

        if let Ok(frame_id) = GpsQzssFrameId::decode(frame_id) {
            let nib = stream.read_int::<u8>(2)?; // TODO (parity)
            let parity = stream.read_int::<u8>(6)?; // TODO (parity)

            Ok(Self {
                alert,
                frame_id,
                anti_spoofing,
                tow: zcount * 3 / 2,
            })
        } else {
            Err(BitError::UnmatchedDiscriminant {
                discriminant: frame_id as usize,
                enum_name: "frame-id".to_string(),
            })
        }
    }
}

impl BitWrite<BigEndian> for GpsQzssHow {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.tow * 2 / 3, 17)?;
        stream.write_bool(self.alert)?;
        stream.write_bool(self.anti_spoofing)?;
        stream.write_int(self.frame_id.encode() & 0x07, 3)?;

        stream.write_int(0, 2)?; // TODO (parity)
        stream.write_int(0, 6)?; // TODO (parity)

        Ok(())
    }
}

impl Default for GpsQzssHow {
    /// Generates a default Null [GpsQzssHow] (Eph-1).
    fn default() -> Self {
        Self {
            tow: Default::default(),
            alert: Default::default(),
            frame_id: Default::default(),
            anti_spoofing: Default::default(),
        }
    }
}

#[cfg(feature = "std")]
impl std::fmt::Display for GpsQzssHow {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "{} TOW={} - ALERT={} - A/S={}",
            self.frame_id, self.tow, self.alert, self.anti_spoofing,
        )
    }
}

impl GpsQzssHow {
    #[cfg(test)]
    pub fn model(frame_id: GpsQzssFrameId) -> Self {
        Self::default()
            .with_frame_id(frame_id)
            .with_tow_seconds(15_000)
            .with_alert_bit()
            .with_anti_spoofing()
    }

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
        Self::default().with_frame_id(GpsQzssFrameId::Ephemeris1)
    }

    /// Constructs a default EPH-2 [GpsQzssHow]
    pub fn ephemeris2() -> Self {
        Self::default().with_frame_id(GpsQzssFrameId::Ephemeris1)
    }

    /// Constructs a default EPH-3 [GpsQzssHow]
    pub fn ephemeris3() -> Self {
        Self::default().with_frame_id(GpsQzssFrameId::Ephemeris3)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{GpsQzssFrameId, GpsQzssHow},
        Buffer, Message, StaticBuffer,
    };

    use bitbuffer::{BigEndian, BitReadStream};

    #[test]
    fn default_reciprocal() {
        let default = GpsQzssHow::default();

        let mut buffer = [0; 1024];

        assert!(
            default.encode(&mut buffer).is_ok(),
            "failed to encode frame"
        );

        let decoded = GpsQzssHow::decode(&buffer).unwrap_or_else(|e| {
            panic!("failed to decode HOW: {}", e);
        });

        assert_eq!(decoded, default);
    }

    #[test]
    fn reciprocal() {
        for (tow, frame_id, alert, anti_spoofing) in [
            (0x05DC, GpsQzssFrameId::Ephemeris1, true, false),
            (0x0708, GpsQzssFrameId::Ephemeris1, false, false),
            (0x0_1194, GpsQzssFrameId::Ephemeris1, false, true),
            (0x0_1194, GpsQzssFrameId::Ephemeris2, false, true),
            (0x0_1194, GpsQzssFrameId::Ephemeris3, false, true),
        ] {
            let how = GpsQzssHow {
                tow,
                frame_id,
                anti_spoofing,
                alert,
            };

            let mut buffer = [0; 1024];

            assert!(how.encode(&mut buffer).is_ok(), "failed to encode frame");

            let decoded = GpsQzssHow::decode(&buffer).unwrap_or_else(|e| {
                panic!("GPS HOW reciprocal failed: {}", e);
            });

            assert_eq!(decoded.tow, tow);
            assert_eq!(decoded.alert, alert);
            assert_eq!(decoded.frame_id, frame_id);
            assert_eq!(decoded.anti_spoofing, anti_spoofing);
        }
    }
}
