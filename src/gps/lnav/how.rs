use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    gps::lnav::{GpsQzssLnavSubframeId, GPS_LNAV_WORD_DATA_BITS},
    Message,
};

#[cfg(doc)]
use crate::gps::lnav::GpsQzssLnavTelemetry;

/// [GpsQzssLnavHow] (GPS Hand Over Word) marks the beginning of each frame, following [GpsQzssTelemetry],
/// and defines the content to follow.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsQzssLnavHow {
    /// TOW: elapsed time within current GPS week (in seconds),
    /// at the instant of transmission of the 1st bit of the next frame to follow
    /// this [GpsQzssLnavHow] word.
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

    /// Indenfities binary content to follow
    pub sbf_id: GpsQzssLnavSubframeId,
}

impl Message<BigEndian> for GpsQzssLnavHow {
    fn encoding_size(&self) -> usize {
        GPS_LNAV_WORD_DATA_BITS / 8
    }

    fn encoding_bits(&self) -> usize {
        GPS_LNAV_WORD_DATA_BITS
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavHow {
    /// Reads the 24 data bits of the HOW (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let tow_count = stream.read_int::<u32>(17)?;
        let alert = stream.read_bool()?;
        let anti_spoofing = stream.read_bool()?;
        let sbf_id = stream.read_int::<u8>(3)?;
        let _t = stream.read_int::<u8>(2)?; // solved for parity, no meaning

        match GpsQzssLnavSubframeId::decode(sbf_id) {
            Ok(sbf_id) => Ok(Self {
                alert,
                sbf_id,
                anti_spoofing,
                tow: tow_count * 6,
            }),
            Err(_) => Err(BitError::UnmatchedDiscriminant {
                discriminant: sbf_id as usize,
                enum_name: "frame-id".to_string(),
            }),
        }
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavHow {
    /// Writes the 24 data bits of the HOW (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    /// The two last bits (t) are solved when the frame is encoded.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int((self.tow / 6) & 0x1ffff, 17)?;
        stream.write_bool(self.alert)?;
        stream.write_bool(self.anti_spoofing)?;
        stream.write_int(self.sbf_id.encode() & 0x07, 3)?;
        stream.write_int(0u8, 2)?;
        Ok(())
    }
}

impl Default for GpsQzssLnavHow {
    /// Generates a default Null [GpsQzssLnavHow] (Eph-1).
    fn default() -> Self {
        Self {
            tow: Default::default(),
            alert: Default::default(),
            sbf_id: Default::default(),
            anti_spoofing: Default::default(),
        }
    }
}

impl core::fmt::Display for GpsQzssLnavHow {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "{} TOW={} - ALERT={} - A/S={}",
            self.sbf_id, self.tow, self.alert, self.anti_spoofing,
        )
    }
}

impl GpsQzssLnavHow {
    #[cfg(test)]
    pub fn model(sbf_id: GpsQzssLnavSubframeId) -> Self {
        Self::default()
            .with_subframe_id(sbf_id)
            .with_tow_seconds(15_000)
            .with_alert_bit()
            .with_anti_spoofing()
    }

    /// Copies and returns [GpsQzssLnavHow] with updated TOW in seconds.
    /// This value should be aligned to midnight and always a multiple of 6 seconds,
    /// the message transmission rate.
    pub fn with_tow_seconds(mut self, tow_seconds: u32) -> Self {
        self.tow = tow_seconds;
        self
    }

    /// Copies and returns [GpsQzssLnavHow] with updated [GpsQzssLnavSubframeId]
    pub fn with_subframe_id(mut self, sbf_id: GpsQzssLnavSubframeId) -> Self {
        self.sbf_id = sbf_id;
        self
    }

    /// Copies and returns [GpsQzssLnavHow] with updated alert bit asserted
    pub fn with_alert_bit(mut self) -> Self {
        self.alert = true;
        self
    }

    /// Copies and returns [GpsQzssLnavHow] with updated alert bit deasserted
    pub fn without_alert_bit(mut self) -> Self {
        self.alert = false;
        self
    }

    /// Copies and returns [GpsQzssLnavHow] with A/S bit asserted
    pub fn with_anti_spoofing(mut self) -> Self {
        self.anti_spoofing = true;
        self
    }

    /// Copies and returns [GpsQzssLnavHow] with A/S bit deasserted
    pub fn without_anti_spoofing(mut self) -> Self {
        self.anti_spoofing = false;
        self
    }

    /// Constructs a default EPH-1 [GpsQzssLnavHow]
    pub fn ephemeris1() -> Self {
        Self::default().with_subframe_id(GpsQzssLnavSubframeId::Ephemeris1)
    }

    /// Constructs a default EPH-2 [GpsQzssLnavHow]
    pub fn ephemeris2() -> Self {
        Self::default().with_subframe_id(GpsQzssLnavSubframeId::Ephemeris2)
    }

    /// Constructs a default EPH-3 [GpsQzssLnavHow]
    pub fn ephemeris3() -> Self {
        Self::default().with_subframe_id(GpsQzssLnavSubframeId::Ephemeris3)
    }

    /// Constructs a default ALM-4 [GpsQzssLnavHow]
    pub fn almanach4() -> Self {
        Self::default().with_subframe_id(GpsQzssLnavSubframeId::Almanach4)
    }

    /// Constructs a default ALM-5 [GpsQzssLnavHow]
    pub fn almanach5() -> Self {
        Self::default().with_subframe_id(GpsQzssLnavSubframeId::Almanach5)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::lnav::{GpsQzssLnavHow, GpsQzssLnavSubframeId},
        Message,
    };

    #[test]
    fn default_reciprocal() {
        let default = GpsQzssLnavHow::default();

        let mut buffer = [0; 1024];

        assert!(
            default.encode(&mut buffer).is_ok(),
            "failed to encode frame"
        );

        let decoded = GpsQzssLnavHow::decode(&buffer).unwrap_or_else(|e| {
            panic!("failed to decode HOW: {}", e);
        });

        assert_eq!(decoded, default);
    }

    #[test]
    fn reciprocal() {
        for (tow, sbf_id, alert, anti_spoofing) in [
            (0x05DC, GpsQzssLnavSubframeId::Ephemeris1, true, false),
            (0x0708, GpsQzssLnavSubframeId::Ephemeris1, false, false),
            (0x0_1194, GpsQzssLnavSubframeId::Ephemeris1, false, true),
            (0x0_1194, GpsQzssLnavSubframeId::Ephemeris2, false, true),
            (0x0_1194, GpsQzssLnavSubframeId::Ephemeris3, false, true),
        ] {
            let how = GpsQzssLnavHow {
                tow,
                sbf_id,
                anti_spoofing,
                alert,
            };

            let mut buffer = [0; 1024];

            assert!(how.encode(&mut buffer).is_ok(), "failed to encode frame");

            let decoded = GpsQzssLnavHow::decode(&buffer).unwrap_or_else(|e| {
                panic!("GPS HOW reciprocal failed: {}", e);
            });

            assert_eq!(decoded.tow, tow);
            assert_eq!(decoded.alert, alert);
            assert_eq!(decoded.sbf_id, sbf_id);
            assert_eq!(decoded.anti_spoofing, anti_spoofing);
        }
    }
}
