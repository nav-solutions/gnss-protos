use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{from_scaled, rad_to_semicircles, to_scaled, write_signed},
    galileo::{word::read_word_type, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES},
    Message,
};

/// [GalileoInavWord2] I/NAV word type 2: Ephemeris (2/4)
/// (OS SIS ICD v2.1, table 41).
///
/// | Type | IODnav | Omega0 | i0  | omega | IDOT | Reserved |
/// |------|--------|--------|-----|-------|------|----------|
/// | 6    | 10     | 32*    | 32* | 32*   | 14*  | 2        |
///
/// (*) two's complement. Equality is tested at the ICD resolution:
/// two words are equal when their binary representations are identical.
#[derive(Debug, Default, Copy, Clone)]
pub struct GalileoInavWord2 {
    /// Issue Of Data of the ephemeris batch (10 bits).
    pub iodnav: u16,

    /// Longitude of ascending node of orbital plane at weekly epoch (in semicircles).
    pub omega0: f64,

    /// Inclination angle at reference time (in semicircles).
    pub i0: f64,

    /// Argument of perigee (in semicircles).
    pub omega: f64,

    /// Rate of change of inclination angle (in semicircles.s⁻¹).
    pub idot: f64,

    /// Reserved bits (2 bits)
    pub reserved: u8,
}

impl GalileoInavWord2 {
    /// Word type of [GalileoInavWord2]
    pub const WORD_TYPE: u8 = 2;

    /// Copies and returns [GalileoInavWord2] with updated IODnav (10 bits).
    pub fn with_iodnav(mut self, iodnav: u16) -> Self {
        self.iodnav = iodnav;
        self
    }

    /// Copies and returns [GalileoInavWord2] with updated longitude
    /// of ascending node at weekly epoch (in semicircles).
    pub fn with_longitude_ascending_node_semicircles(mut self, omega0: f64) -> Self {
        self.omega0 = omega0;
        self
    }

    /// Copies and returns [GalileoInavWord2] with updated longitude
    /// of ascending node at weekly epoch (in radians).
    pub fn with_longitude_ascending_node_radians(self, omega0: f64) -> Self {
        self.with_longitude_ascending_node_semicircles(rad_to_semicircles(omega0))
    }

    /// Copies and returns [GalileoInavWord2] with updated inclination angle (in semicircles).
    pub fn with_inclination_semicircles(mut self, i0: f64) -> Self {
        self.i0 = i0;
        self
    }

    /// Copies and returns [GalileoInavWord2] with updated inclination angle (in radians).
    pub fn with_inclination_radians(self, i0: f64) -> Self {
        self.with_inclination_semicircles(rad_to_semicircles(i0))
    }

    /// Copies and returns [GalileoInavWord2] with updated argument of perigee (in semicircles).
    pub fn with_omega_semicircles(mut self, omega: f64) -> Self {
        self.omega = omega;
        self
    }

    /// Copies and returns [GalileoInavWord2] with updated argument of perigee (in radians).
    pub fn with_omega_radians(self, omega: f64) -> Self {
        self.with_omega_semicircles(rad_to_semicircles(omega))
    }

    /// Copies and returns [GalileoInavWord2] with updated inclination rate (in semicircles.s⁻¹).
    pub fn with_inclination_rate_semicircles_s(mut self, idot: f64) -> Self {
        self.idot = idot;
        self
    }

    /// Copies and returns [GalileoInavWord2] with updated inclination rate (in radians.s⁻¹).
    pub fn with_inclination_rate_radians_s(self, idot: f64) -> Self {
        self.with_inclination_rate_semicircles_s(rad_to_semicircles(idot))
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_iodnav(123)
            .with_longitude_ascending_node_semicircles(-0.53)
            .with_inclination_semicircles(0.31)
            .with_omega_semicircles(-0.11)
            .with_inclination_rate_semicircles_s(-6.3e-11)
    }
}

impl Message<BigEndian> for GalileoInavWord2 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord2 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let iodnav = stream.read_int::<u16>(10)?;
        let omega0 = from_scaled(stream.read_int::<i32>(32)?, -31);
        let i0 = from_scaled(stream.read_int::<i32>(32)?, -31);
        let omega = from_scaled(stream.read_int::<i32>(32)?, -31);
        let idot = from_scaled(stream.read_int::<i16>(14)?, -43);
        let reserved = stream.read_int::<u8>(2)?;

        Ok(Self {
            iodnav,
            omega0,
            i0,
            omega,
            idot,
            reserved,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord2 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        stream.write_int(self.iodnav & 0x3ff, 10)?;
        write_signed(stream, to_scaled(self.omega0, -31), 32)?;
        write_signed(stream, to_scaled(self.i0, -31), 32)?;
        write_signed(stream, to_scaled(self.omega, -31), 32)?;
        write_signed(stream, to_scaled(self.idot, -43), 14)?;
        stream.write_int(self.reserved & 0x3, 2)?;
        Ok(())
    }
}

crate::galileo::word::impl_eq_by_encoding!(GalileoInavWord2);

#[cfg(test)]
mod test {
    use crate::{galileo::GalileoInavWord2, Message};

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord2::default();
        let mut buffer = [0u8; 16];
        default.encode(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x08); // word type 2
        assert_eq!(GalileoInavWord2::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord2::model();
        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord2::decode(&buffer).unwrap();
        assert_eq!(decoded, model);
        assert_eq!(decoded.iodnav, 123);
        assert!((decoded.omega0 - -0.53).abs() < 1e-9);
        assert!((decoded.i0 - 0.31).abs() < 1e-9);
        assert!((decoded.omega - -0.11).abs() < 1e-9);
        assert!((decoded.idot - -6.3e-11).abs() < 1e-13);
    }

    #[test]
    fn radians() {
        let word =
            GalileoInavWord2::default().with_inclination_radians(core::f64::consts::PI / 4.0);
        assert!((word.i0 - 0.25).abs() < 1e-12);
    }
}
