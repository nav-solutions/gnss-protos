use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{from_scaled, to_scaled, write_signed, write_unsigned},
    galileo::{word::read_word_type, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES},
    Message,
};

/// [GalileoInavWord6] I/NAV word type 6: GST-UTC conversion parameters
/// (OS SIS ICD v2.1, table 45).
///
/// | Type | A0  | A1  | ΔtLS | t0t | WN0t | WNLSF | DN | ΔtLSF | TOW | Spare |
/// |------|-----|-----|------|-----|------|-------|----|-------|-----|-------|
/// | 6    | 32* | 24* | 8*   | 8   | 8    | 8     | 3  | 8*    | 20  | 3     |
///
/// (*) two's complement. Equality is tested at the ICD resolution:
/// two words are equal when their binary representations are identical.
#[derive(Debug, Default, Copy, Clone)]
pub struct GalileoInavWord6 {
    /// Constant term of the GST-UTC polynomial (in seconds).
    pub a0: f64,

    /// First order term of the GST-UTC polynomial (in s.s⁻¹).
    pub a1: f64,

    /// Leap seconds count before leap second adjustment (in seconds).
    pub dt_ls: i8,

    /// UTC data reference time of week, in seconds (3600 s resolution).
    pub t0t: u32,

    /// UTC data reference week number (modulo 256).
    pub wn0t: u8,

    /// Week number of leap second adjustment (modulo 256).
    pub wn_lsf: u8,

    /// Day number at the end of which a leap second adjustment becomes effective
    /// (1 = Sunday to 7 = Saturday).
    pub dn: u8,

    /// Leap seconds count after leap second adjustment (in seconds).
    pub dt_lsf: i8,

    /// GST time of week, in seconds.
    pub tow: u32,

    /// Spare bits (3 bits)
    pub spare: u8,
}

impl GalileoInavWord6 {
    /// Word type of [GalileoInavWord6]
    pub const WORD_TYPE: u8 = 6;

    /// Copies and returns [GalileoInavWord6] with updated GST-UTC polynomial
    /// (a0 in seconds, a1 in s.s⁻¹).
    pub fn with_utc_polynomial(mut self, a0: f64, a1: f64) -> Self {
        self.a0 = a0;
        self.a1 = a1;
        self
    }

    /// Copies and returns [GalileoInavWord6] with updated UTC reference time:
    /// week number and time of week (in seconds, multiple of 3600 s).
    pub fn with_utc_reference(mut self, wn0t: u8, t0t_seconds: u32) -> Self {
        self.wn0t = wn0t;
        self.t0t = t0t_seconds;
        self
    }

    /// Copies and returns [GalileoInavWord6] with current leap seconds count.
    pub fn with_leap_seconds(mut self, dt_ls: i8) -> Self {
        self.dt_ls = dt_ls;
        self
    }

    /// Copies and returns [GalileoInavWord6] announcing a leap second adjustment:
    /// effective at the end of day `dn` (1..=7) of week `wn_lsf`,
    /// leading to `dt_lsf` leap seconds.
    pub fn with_future_leap_seconds(mut self, wn_lsf: u8, dn: u8, dt_lsf: i8) -> Self {
        self.wn_lsf = wn_lsf;
        self.dn = dn;
        self.dt_lsf = dt_lsf;
        self
    }

    /// Copies and returns [GalileoInavWord6] with updated GST time of week (in seconds).
    pub fn with_tow_seconds(mut self, tow: u32) -> Self {
        self.tow = tow;
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_utc_polynomial(-9.3e-10, 1.7e-15)
            .with_utc_reference(59, 345_600)
            .with_leap_seconds(18)
            .with_future_leap_seconds(137, 7, 18)
            .with_tow_seconds(455_909)
    }
}

impl Message<BigEndian> for GalileoInavWord6 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord6 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let a0 = from_scaled(stream.read_int::<i32>(32)?, -30);
        let a1 = from_scaled(stream.read_int::<i32>(24)?, -50);
        let dt_ls = stream.read_int::<i8>(8)?;
        let t0t = stream.read_int::<u32>(8)? * 3600;
        let wn0t = stream.read_int::<u8>(8)?;
        let wn_lsf = stream.read_int::<u8>(8)?;
        let dn = stream.read_int::<u8>(3)?;
        let dt_lsf = stream.read_int::<i8>(8)?;
        let tow = stream.read_int::<u32>(20)?;
        let spare = stream.read_int::<u8>(3)?;

        Ok(Self {
            a0,
            a1,
            dt_ls,
            t0t,
            wn0t,
            wn_lsf,
            dn,
            dt_lsf,
            tow,
            spare,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord6 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        write_signed(stream, to_scaled(self.a0, -30), 32)?;
        write_signed(stream, to_scaled(self.a1, -50), 24)?;
        stream.write_int(self.dt_ls as u8, 8)?;
        write_unsigned(stream, (self.t0t / 3600) as i64, 8)?;
        stream.write_int(self.wn0t, 8)?;
        stream.write_int(self.wn_lsf, 8)?;
        stream.write_int(self.dn & 0x7, 3)?;
        stream.write_int(self.dt_lsf as u8, 8)?;
        stream.write_int(self.tow & 0xf_ffff, 20)?;
        stream.write_int(self.spare & 0x7, 3)?;
        Ok(())
    }
}

crate::bits::impl_eq_by_encoding!(GalileoInavWord6, crate::galileo::GALILEO_INAV_WORD_BYTES);

#[cfg(test)]
mod test {
    use crate::{galileo::GalileoInavWord6, Message};

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord6::default();
        let mut buffer = [0u8; 16];
        default.encode(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x18); // word type 6
        assert_eq!(GalileoInavWord6::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord6::model();
        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord6::decode(&buffer).unwrap();
        assert_eq!(decoded, model);
        assert!((decoded.a0 - -9.3e-10).abs() < 2.0_f64.powi(-31)); // LSB/2
        assert!((decoded.a1 - 1.7e-15).abs() < 2.0_f64.powi(-51)); // LSB/2
        assert_eq!(decoded.dt_ls, 18);
        assert_eq!(decoded.t0t, 345_600);
        assert_eq!(decoded.wn0t, 59);
        assert_eq!(decoded.wn_lsf, 137);
        assert_eq!(decoded.dn, 7);
        assert_eq!(decoded.dt_lsf, 18);
        assert_eq!(decoded.tow, 455_909);
    }

    #[test]
    fn negative_leap_seconds() {
        let word = GalileoInavWord6::default()
            .with_leap_seconds(-3)
            .with_future_leap_seconds(1, 1, -4);
        let mut buffer = [0u8; 16];
        word.encode(&mut buffer).unwrap();
        let decoded = GalileoInavWord6::decode(&buffer).unwrap();
        assert_eq!(decoded.dt_ls, -3);
        assert_eq!(decoded.dt_lsf, -4);
    }
}
