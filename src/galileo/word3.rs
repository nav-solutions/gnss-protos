use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{from_scaled, rad_to_semicircles, to_scaled, write_signed},
    galileo::{word::read_word_type, GalileoSisa, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES},
    Message,
};

/// [GalileoInavWord3] I/NAV word type 3: Ephemeris (3/4) and SISA
/// (OS SIS ICD v2.1, table 42).
///
/// | Type | IODnav | Omega_dot | Δn  | Cuc | Cus | Crc | Crs | SISA(E1,E5b) |
/// |------|--------|-----------|-----|-----|-----|-----|-----|--------------|
/// | 6    | 10     | 24*       | 16* | 16* | 16* | 16* | 16* | 8            |
///
/// (*) two's complement. Equality is tested at the ICD resolution:
/// two words are equal when their binary representations are identical.
#[derive(Debug, Default, Copy, Clone)]
pub struct GalileoInavWord3 {
    /// Issue Of Data of the ephemeris batch (10 bits).
    pub iodnav: u16,

    /// Rate of change of right ascension (in semicircles.s⁻¹).
    pub omega_dot: f64,

    /// Mean motion difference from computed value (in semicircles.s⁻¹).
    pub dn: f64,

    /// Amplitude of the cosine harmonic correction term to the argument of latitude (in radians).
    pub cuc: f64,

    /// Amplitude of the sine harmonic correction term to the argument of latitude (in radians).
    pub cus: f64,

    /// Amplitude of the cosine harmonic correction term to the orbit radius (in meters).
    pub crc: f64,

    /// Amplitude of the sine harmonic correction term to the orbit radius (in meters).
    pub crs: f64,

    /// Signal In Space Accuracy for the (E1, E5b) dual frequency combination.
    pub sisa: GalileoSisa,
}

impl GalileoInavWord3 {
    /// Word type of [GalileoInavWord3]
    pub const WORD_TYPE: u8 = 3;

    /// Copies and returns [GalileoInavWord3] with updated IODnav (10 bits).
    pub fn with_iodnav(mut self, iodnav: u16) -> Self {
        self.iodnav = iodnav;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated rate of right ascension (in semicircles.s⁻¹).
    pub fn with_omega_dot_semicircles_s(mut self, omega_dot: f64) -> Self {
        self.omega_dot = omega_dot;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated rate of right ascension (in radians.s⁻¹).
    pub fn with_omega_dot_radians_s(self, omega_dot: f64) -> Self {
        self.with_omega_dot_semicircles_s(rad_to_semicircles(omega_dot))
    }

    /// Copies and returns [GalileoInavWord3] with updated mean motion difference (in semicircles.s⁻¹).
    pub fn with_mean_motion_difference_semicircles_s(mut self, dn: f64) -> Self {
        self.dn = dn;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated mean motion difference (in radians.s⁻¹).
    pub fn with_mean_motion_difference_radians_s(self, dn: f64) -> Self {
        self.with_mean_motion_difference_semicircles_s(rad_to_semicircles(dn))
    }

    /// Copies and returns [GalileoInavWord3] with updated Cuc correction term (in radians).
    pub fn with_cuc_radians(mut self, cuc: f64) -> Self {
        self.cuc = cuc;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated Cus correction term (in radians).
    pub fn with_cus_radians(mut self, cus: f64) -> Self {
        self.cus = cus;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated Crc correction term (in meters).
    pub fn with_crc_meters(mut self, crc: f64) -> Self {
        self.crc = crc;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated Crs correction term (in meters).
    pub fn with_crs_meters(mut self, crs: f64) -> Self {
        self.crs = crs;
        self
    }

    /// Copies and returns [GalileoInavWord3] with updated [GalileoSisa].
    pub fn with_sisa(mut self, sisa: GalileoSisa) -> Self {
        self.sisa = sisa;
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_iodnav(123)
            .with_omega_dot_semicircles_s(-1.9e-9)
            .with_mean_motion_difference_semicircles_s(1.2e-9)
            .with_cuc_radians(-2.9e-6)
            .with_cus_radians(2.7e-6)
            .with_crc_meters(288.6875)
            .with_crs_meters(-64.90625)
            .with_sisa(GalileoSisa::from_meters(3.12))
    }
}

impl Message<BigEndian> for GalileoInavWord3 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord3 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let iodnav = stream.read_int::<u16>(10)?;
        let omega_dot = from_scaled(stream.read_int::<i32>(24)?, -43);
        let dn = from_scaled(stream.read_int::<i16>(16)?, -43);
        let cuc = from_scaled(stream.read_int::<i16>(16)?, -29);
        let cus = from_scaled(stream.read_int::<i16>(16)?, -29);
        let crc = from_scaled(stream.read_int::<i16>(16)?, -5);
        let crs = from_scaled(stream.read_int::<i16>(16)?, -5);
        let sisa = GalileoSisa(stream.read_int::<u8>(8)?);

        Ok(Self {
            iodnav,
            omega_dot,
            dn,
            cuc,
            cus,
            crc,
            crs,
            sisa,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord3 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        stream.write_int(self.iodnav & 0x3ff, 10)?;
        write_signed(stream, to_scaled(self.omega_dot, -43), 24)?;
        write_signed(stream, to_scaled(self.dn, -43), 16)?;
        write_signed(stream, to_scaled(self.cuc, -29), 16)?;
        write_signed(stream, to_scaled(self.cus, -29), 16)?;
        write_signed(stream, to_scaled(self.crc, -5), 16)?;
        write_signed(stream, to_scaled(self.crs, -5), 16)?;
        stream.write_int(self.sisa.0, 8)?;
        Ok(())
    }
}

crate::bits::impl_eq_by_encoding!(GalileoInavWord3, crate::galileo::GALILEO_INAV_WORD_BYTES);

#[cfg(test)]
mod test {
    use crate::{
        galileo::{GalileoInavWord3, GalileoSisa},
        Message,
    };

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord3::default();
        assert!(default.sisa.is_napa());

        let mut buffer = [0u8; 16];
        default.encode(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x0C); // word type 3
        assert_eq!(buffer[15], 0xff); // NAPA
        assert_eq!(GalileoInavWord3::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord3::model();
        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord3::decode(&buffer).unwrap();
        assert_eq!(decoded, model);
        assert_eq!(decoded.iodnav, 123);
        assert!((decoded.omega_dot - -1.9e-9).abs() < 1e-13);
        assert!((decoded.dn - 1.2e-9).abs() < 1e-13);
        assert!((decoded.cuc - -2.9e-6).abs() < 1e-8);
        assert!((decoded.cus - 2.7e-6).abs() < 1e-8);
        assert_eq!(decoded.crc, 288.6875);
        assert_eq!(decoded.crs, -64.90625);
        assert_eq!(decoded.sisa, GalileoSisa(107));
    }
}
