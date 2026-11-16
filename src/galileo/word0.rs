use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    galileo::{word::read_word_type, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES},
    Message,
};

/// Spare data of a [GalileoInavWord0] (88 bits).
pub const GALILEO_INAV_WORD0_SPARE_BYTES: usize = 11;

/// [GalileoInavWord0] I/NAV word type 0: spare word, that may carry the
/// Galileo System Time (OS SIS ICD v2.1, table 52).
///
/// | Type | Time | Spare | WN | TOW |
/// |------|------|-------|----|-----|
/// | 6    | 2    | 88    | 12 | 20  |
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct GalileoInavWord0 {
    /// 2-bit time field: WN and TOW are only valid when set to `0b10`
    /// (see [Self::has_valid_time]).
    pub time: u8,

    /// 88 spare bits, MSB first.
    pub spare: [u8; GALILEO_INAV_WORD0_SPARE_BYTES],

    /// GST week number (modulo 4096).
    pub wn: u16,

    /// GST time of week, in seconds.
    pub tow: u32,
}

impl Default for GalileoInavWord0 {
    /// Builds a [GalileoInavWord0] without valid time and null spare bits.
    fn default() -> Self {
        Self {
            time: 0,
            spare: [0; GALILEO_INAV_WORD0_SPARE_BYTES],
            wn: 0,
            tow: 0,
        }
    }
}

impl GalileoInavWord0 {
    /// Word type of [GalileoInavWord0]
    pub const WORD_TYPE: u8 = 0;

    /// Value of the time field when WN and TOW are valid.
    pub const VALID_TIME: u8 = 0b10;

    /// True if this word carries a valid GST (WN and TOW).
    pub fn has_valid_time(&self) -> bool {
        self.time == Self::VALID_TIME
    }

    /// Copies and returns [GalileoInavWord0] with valid GST week number and time of week (s).
    pub fn with_gst(mut self, wn: u16, tow_seconds: u32) -> Self {
        self.time = Self::VALID_TIME;
        self.wn = wn;
        self.tow = tow_seconds;
        self
    }

    /// Copies and returns [GalileoInavWord0] with updated spare bits.
    pub fn with_spare(mut self, spare: [u8; GALILEO_INAV_WORD0_SPARE_BYTES]) -> Self {
        self.spare = spare;
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_gst(1339, 455_901)
            .with_spare([0x55; 11])
    }
}

impl Message<BigEndian> for GalileoInavWord0 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord0 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let time = stream.read_int::<u8>(2)?;

        let mut spare = [0u8; GALILEO_INAV_WORD0_SPARE_BYTES];
        spare.copy_from_slice(&stream.read_bytes(GALILEO_INAV_WORD0_SPARE_BYTES)?);

        let wn = stream.read_int::<u16>(12)?;
        let tow = stream.read_int::<u32>(20)?;

        Ok(Self {
            time,
            spare,
            wn,
            tow,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord0 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        stream.write_int(self.time & 0x3, 2)?;
        stream.write_bytes(&self.spare)?;
        stream.write_int(self.wn & 0xfff, 12)?;
        stream.write_int(self.tow & 0xf_ffff, 20)?;
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use crate::{
        galileo::{GalileoInavWord0, GalileoInavWord1},
        Message,
    };

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord0::default();
        let mut buffer = [0u8; 16];
        assert_eq!(default.encode(&mut buffer).unwrap(), 16);
        assert_eq!(buffer, [0; 16]);
        assert!(!default.has_valid_time());
        assert_eq!(GalileoInavWord0::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord0::model();
        assert!(model.has_valid_time());

        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        assert_eq!(buffer[0], 0x02); // type 0, time = 10
        assert_eq!(GalileoInavWord0::decode(&buffer).unwrap(), model);
    }

    #[test]
    fn wrong_word_type() {
        let mut buffer = [0u8; 16];
        GalileoInavWord1::default().encode(&mut buffer).unwrap();
        assert!(GalileoInavWord0::decode(&buffer).is_err());
    }
}
