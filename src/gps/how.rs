use crate::{
    gps::{GpsError, GPS_PREAMBLE_BYTE, GPS_WORD_BITS},
    BufferingError, Message,
};

use bitbuffer::{BigEndian, BitRead, BitReadBuffer, BitWrite, BitWriteStream, Endianness};

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

    /// 6 Parity bits
    parity: u8,
}

impl Message for GpsQzssHow {
    type Err = GpsError; 
    
    fn encoding_size(&self) -> usize {
        4
    }

    fn encoding_bitsize(&self) -> usize {
        GPS_WORD_BITS
    }
    
    /// [GpsQzssHow] decoding attempt from a burst of
    /// GPS bits, where first received bits were stored in MSB position.
    fn decode(buf: &[u8]) -> Result<Self, GpsError> {
        let buf = BitReadBuffer::new(buf, BigEndian);

        let tow = buf.read_int::<u8>(0, 17)? * 3 / 2;
        let alert = buf.read_bool(18)?;
        let anti_spoofing = buf.read_bool(19)?;
        let frame_id = buf.read_int::<u8>(20, 3)?;
        let parity = buf.read_int::<u8>(24, 6);
        
        let frame_id = GpsFrameId::decode(frame_id)?;

        Ok(Self {
            tow,
            alert,
            frame_id,
            parity,
            anti_spoofing,
        })
    }

    /// Encodes this [GpsQzssHow] as big-endian stream,
    /// last byte will be padded because a GPS word is not aligned to [u8].
    fn encode(&self, buf: &mut [u8]) -> Result<usize, GpsError> {
        let len = buf.len();
        let encoding_size = self.encoding_size();

        if len < encoding_size {
            return Err(GpsError::Buffering(BufferingError::StorageFull));
        }

        let tow = (self.tow  * 2 / 3) 0x1ffff;

        encoded[3] |= ((tow & 0x1_8000) >> 15) as u8;
        encoded[4] = ((tow & 0x0_7f80) >> 7) as u8;
        encoded[5] = (tow & 0x0_007f) as u8;
        encoded[5] <<= 1;

        if self.how.alert {
            encoded[5] |= 0x01;
        }

        if self.how.anti_spoofing {
            encoded[6] |= 0x80;
        }

        buf[3] = (self.parity & 0x003f) << 2;

        Ok(encoding_size)
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
    /// Generates a realitic frame model for testing purpose
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
            tow: zcount * 3 / 2,
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
mod how {
    use crate::gps::{GpsDataWord, GpsQzssFrameId, GpsQzssHow};

    #[test]
    fn encoding() {
        for (tow, frame_id, alert, anti_spoofing) in [
            (0x05DC, GpsQzssFrameId::Ephemeris1, true, false),
            (0x0708, GpsQzssFrameId::Ephemeris1, false, false),
            (0x0_1194, GpsQzssFrameId::Ephemeris1, false, true),
        ] {
            let how = GpsQzssHow {
                tow,
                frame_id,
                anti_spoofing,
                alert,
            };

            let gps_word = how.to_word();

            let decoded = GpsQzssHow::from_word(gps_word).unwrap_or_else(|e| {
                panic!("failed to decode gps-how from {:?} : {}", gps_word, e);
            });

            assert_eq!(decoded.tow, tow);
            assert_eq!(decoded.alert, alert);
            assert_eq!(decoded.frame_id, frame_id);
            assert_eq!(decoded.anti_spoofing, anti_spoofing);
        }
    }
}
