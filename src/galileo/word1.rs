use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{from_scaled, rad_to_semicircles, to_scaled, write_signed, write_unsigned},
    galileo::{word::read_word_type, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES},
    Message,
};

/// [GalileoInavWord1] I/NAV word type 1: Ephemeris (1/4)
/// (OS SIS ICD v2.1, table 40).
///
/// | Type | IODnav | t0e | M0  | e  | sqrt(A) | Reserved |
/// |------|--------|-----|-----|----|---------|----------|
/// | 6    | 10     | 14  | 32* | 32 | 32      | 2        |
///
/// (*) two's complement. Equality is tested at the ICD resolution:
/// two words are equal when their binary representations are identical.
#[derive(Debug, Default, Copy, Clone)]
pub struct GalileoInavWord1 {
    /// Issue Of Data of the ephemeris batch (10 bits).
    pub iodnav: u16,

    /// Ephemeris reference time, in seconds of GST week (60 s resolution).
    pub toe: u32,

    /// Mean anomaly at reference time (in semicircles).
    pub m0: f64,

    /// Eccentricity (unitless).
    pub e: f64,

    /// Square root of the semi major axis (in sqrt(m)).
    pub sqrt_a: f64,

    /// Reserved bits (2 bits)
    pub reserved: u8,
}

impl GalileoInavWord1 {
    /// Word type of [GalileoInavWord1]
    pub const WORD_TYPE: u8 = 1;

    /// Copies and returns [GalileoInavWord1] with updated IODnav (10 bits).
    pub fn with_iodnav(mut self, iodnav: u16) -> Self {
        self.iodnav = iodnav;
        self
    }

    /// Copies and returns [GalileoInavWord1] with updated reference time
    /// (in seconds of week, should be a multiple of 60 s).
    pub fn with_toe_seconds(mut self, toe: u32) -> Self {
        self.toe = toe;
        self
    }

    /// Copies and returns [GalileoInavWord1] with updated mean anomaly (in semicircles).
    pub fn with_mean_anomaly_semicircles(mut self, m0: f64) -> Self {
        self.m0 = m0;
        self
    }

    /// Copies and returns [GalileoInavWord1] with updated mean anomaly (in radians).
    pub fn with_mean_anomaly_radians(self, m0: f64) -> Self {
        self.with_mean_anomaly_semicircles(rad_to_semicircles(m0))
    }

    /// Copies and returns [GalileoInavWord1] with updated eccentricity.
    pub fn with_eccentricity(mut self, e: f64) -> Self {
        self.e = e;
        self
    }

    /// Copies and returns [GalileoInavWord1] with updated square root
    /// of the semi major axis (in sqrt(m)).
    pub fn with_sqrt_semi_major_axis(mut self, sqrt_a: f64) -> Self {
        self.sqrt_a = sqrt_a;
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_iodnav(123)
            .with_toe_seconds(454_800)
            .with_mean_anomaly_semicircles(0.93)
            .with_eccentricity(3.8e-4)
            .with_sqrt_semi_major_axis(5440.6)
    }
}

impl Message<BigEndian> for GalileoInavWord1 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord1 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let iodnav = stream.read_int::<u16>(10)?;
        let toe = stream.read_int::<u32>(14)? * 60;
        let m0 = from_scaled(stream.read_int::<i32>(32)?, -31);
        let e = from_scaled(stream.read_int::<u32>(32)?, -33);
        let sqrt_a = from_scaled(stream.read_int::<u32>(32)?, -19);
        let reserved = stream.read_int::<u8>(2)?;

        Ok(Self {
            iodnav,
            toe,
            m0,
            e,
            sqrt_a,
            reserved,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord1 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        stream.write_int(self.iodnav & 0x3ff, 10)?;
        write_unsigned(stream, (self.toe / 60) as i64, 14)?;
        write_signed(stream, to_scaled(self.m0, -31), 32)?;
        write_unsigned(stream, to_scaled(self.e, -33), 32)?;
        write_unsigned(stream, to_scaled(self.sqrt_a, -19), 32)?;
        stream.write_int(self.reserved & 0x3, 2)?;
        Ok(())
    }
}

crate::bits::impl_eq_by_encoding!(GalileoInavWord1, crate::galileo::GALILEO_INAV_WORD_BYTES);

#[cfg(test)]
mod test {
    use crate::{galileo::GalileoInavWord1, Message};

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord1::default();
        let mut buffer = [0u8; 16];
        default.encode(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x04); // word type 1
        assert_eq!(GalileoInavWord1::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord1::model();
        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord1::decode(&buffer).unwrap();
        assert_eq!(decoded, model);
        assert_eq!(decoded.iodnav, 123);
        assert_eq!(decoded.toe, 454_800);
        assert!((decoded.m0 - 0.93).abs() < 1e-9);
        assert!((decoded.e - 3.8e-4).abs() < 1e-9);
        assert!((decoded.sqrt_a - 5440.6).abs() < 1e-5);
    }

    #[test]
    fn values_are_clamped() {
        // values out of range saturate rather than wrap around
        let word = GalileoInavWord1::default()
            .with_mean_anomaly_semicircles(2.0)
            .with_eccentricity(-1.0);

        let mut buffer = [0u8; 16];
        word.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord1::decode(&buffer).unwrap();
        assert!((decoded.m0 - 1.0).abs() < 1e-9);
        assert_eq!(decoded.e, 0.0);
    }
}
