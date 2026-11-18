use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{from_scaled, to_scaled, write_signed, write_unsigned},
    galileo::{word::read_word_type, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES},
    Message,
};

/// [GalileoInavWord4] I/NAV word type 4: Ephemeris (4/4) and clock correction parameters
/// (OS SIS ICD v2.1, table 43).
///
/// | Type | IODnav | SVID | Cic | Cis | t0c | af0 | af1 | af2 | Spare |
/// |------|--------|------|-----|-----|-----|-----|-----|-----|-------|
/// | 6    | 10     | 6    | 16* | 16* | 14  | 31* | 21* | 6*  | 2     |
///
/// (*) two's complement. Equality is tested at the ICD resolution:
/// two words are equal when their binary representations are identical.
#[derive(Debug, Default, Copy, Clone)]
pub struct GalileoInavWord4 {
    /// Issue Of Data of the ephemeris batch (10 bits).
    pub iodnav: u16,

    /// Satellite identifier (1..=63).
    pub svid: u8,

    /// Amplitude of the cosine harmonic correction term to the angle of inclination (in radians).
    pub cic: f64,

    /// Amplitude of the sine harmonic correction term to the angle of inclination (in radians).
    pub cis: f64,

    /// Clock correction data reference time, in seconds of GST week (60 s resolution).
    pub toc: u32,

    /// Clock bias correction (in seconds).
    pub af0: f64,

    /// Clock drift correction (in s.s⁻¹).
    pub af1: f64,

    /// Clock drift rate correction (in s.s⁻²).
    pub af2: f64,

    /// Spare bits (2 bits)
    pub spare: u8,
}

impl GalileoInavWord4 {
    /// Word type of [GalileoInavWord4]
    pub const WORD_TYPE: u8 = 4;

    /// Copies and returns [GalileoInavWord4] with updated IODnav (10 bits).
    pub fn with_iodnav(mut self, iodnav: u16) -> Self {
        self.iodnav = iodnav;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated satellite identifier.
    pub fn with_svid(mut self, svid: u8) -> Self {
        self.svid = svid;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated Cic correction term (in radians).
    pub fn with_cic_radians(mut self, cic: f64) -> Self {
        self.cic = cic;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated Cis correction term (in radians).
    pub fn with_cis_radians(mut self, cis: f64) -> Self {
        self.cis = cis;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated clock reference time
    /// (in seconds of week, should be a multiple of 60 s).
    pub fn with_toc_seconds(mut self, toc: u32) -> Self {
        self.toc = toc;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated clock bias (in seconds).
    pub fn with_clock_offset_seconds(mut self, af0: f64) -> Self {
        self.af0 = af0;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated clock drift (in s.s⁻¹).
    pub fn with_clock_drift_seconds_s(mut self, af1: f64) -> Self {
        self.af1 = af1;
        self
    }

    /// Copies and returns [GalileoInavWord4] with updated clock drift rate (in s.s⁻²).
    pub fn with_clock_drift_rate_seconds_s2(mut self, af2: f64) -> Self {
        self.af2 = af2;
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_iodnav(123)
            .with_svid(8)
            .with_cic_radians(1.3e-8)
            .with_cis_radians(2.8e-8)
            .with_toc_seconds(454_800)
            .with_clock_offset_seconds(-4.5e-4)
            .with_clock_drift_seconds_s(-6.4e-12)
            .with_clock_drift_rate_seconds_s2(3.4e-17)
    }
}

impl Message<BigEndian> for GalileoInavWord4 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord4 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let iodnav = stream.read_int::<u16>(10)?;
        let svid = stream.read_int::<u8>(6)?;
        let cic = from_scaled(stream.read_int::<i16>(16)?, -29);
        let cis = from_scaled(stream.read_int::<i16>(16)?, -29);
        let toc = stream.read_int::<u32>(14)? * 60;
        let af0 = from_scaled(stream.read_int::<i32>(31)?, -34);
        let af1 = from_scaled(stream.read_int::<i32>(21)?, -46);
        let af2 = from_scaled(stream.read_int::<i8>(6)?, -59);
        let spare = stream.read_int::<u8>(2)?;

        Ok(Self {
            iodnav,
            svid,
            cic,
            cis,
            toc,
            af0,
            af1,
            af2,
            spare,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord4 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        stream.write_int(self.iodnav & 0x3ff, 10)?;
        stream.write_int(self.svid & 0x3f, 6)?;
        write_signed(stream, to_scaled(self.cic, -29), 16)?;
        write_signed(stream, to_scaled(self.cis, -29), 16)?;
        write_unsigned(stream, (self.toc / 60) as i64, 14)?;
        write_signed(stream, to_scaled(self.af0, -34), 31)?;
        write_signed(stream, to_scaled(self.af1, -46), 21)?;
        write_signed(stream, to_scaled(self.af2, -59), 6)?;
        stream.write_int(self.spare & 0x3, 2)?;
        Ok(())
    }
}

crate::bits::impl_eq_by_encoding!(GalileoInavWord4, crate::galileo::GALILEO_INAV_WORD_BYTES);

#[cfg(test)]
mod test {
    use crate::{galileo::GalileoInavWord4, Message};

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord4::default();
        let mut buffer = [0u8; 16];
        default.encode(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x10); // word type 4
        assert_eq!(GalileoInavWord4::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord4::model();
        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord4::decode(&buffer).unwrap();
        assert_eq!(decoded, model);
        assert_eq!(decoded.iodnav, 123);
        assert_eq!(decoded.svid, 8);
        assert_eq!(decoded.toc, 454_800);
        assert!((decoded.cic - 1.3e-8).abs() < 1e-9);
        assert!((decoded.cis - 2.8e-8).abs() < 1e-9);
        assert!((decoded.af0 - -4.5e-4).abs() < 1e-10);
        assert!((decoded.af1 - -6.4e-12).abs() < 1e-14);
        assert!((decoded.af2 - 3.4e-17).abs() < 1e-18);
    }

    #[test]
    fn negative_clock_terms() {
        // af0 is a 31-bit field, af2 a 6-bit field: sign extension
        let word = GalileoInavWord4::default()
            .with_clock_offset_seconds(-2.0_f64.powi(-34))
            .with_clock_drift_rate_seconds_s2(-2.0_f64.powi(-59) * 32.0);

        let mut buffer = [0u8; 16];
        word.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord4::decode(&buffer).unwrap();
        assert_eq!(decoded.af0, -2.0_f64.powi(-34));
        assert_eq!(decoded.af2, -2.0_f64.powi(-54));
    }
}
