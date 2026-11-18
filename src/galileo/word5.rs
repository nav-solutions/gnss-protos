use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{from_scaled, to_scaled, write_signed, write_unsigned},
    galileo::{
        word::read_word_type, GalileoSignalHealth, GALILEO_INAV_WORD_BITS, GALILEO_INAV_WORD_BYTES,
    },
    Message,
};

/// Number of ionospheric regions of the NeQuick G model
/// (disturbance flags of [GalileoInavWord5]).
pub const GALILEO_IONO_REGIONS: usize = 5;

/// [GalileoInavWord5] I/NAV word type 5: Ionospheric correction, BGD,
/// signal health and data validity status, GST (OS SIS ICD v2.1, table 44).
///
/// | Type | ai0 | ai1 | ai2 | Region 1..5 | BGD(E1,E5a) | BGD(E1,E5b) | E5b HS | E1-B HS | E5b DVS | E1-B DVS | WN | TOW | Spare |
/// |------|-----|-----|-----|-------------|-------------|-------------|--------|---------|---------|----------|----|-----|-------|
/// | 6    | 11  | 11* | 14* | 5 x 1       | 10*         | 10*         | 2      | 2       | 1       | 1        | 12 | 20  | 23    |
///
/// (*) two's complement. Equality is tested at the ICD resolution:
/// two words are equal when their binary representations are identical.
#[derive(Debug, Default, Copy, Clone)]
pub struct GalileoInavWord5 {
    /// Effective ionisation level, 1st order parameter (in sfu).
    pub ai0: f64,

    /// Effective ionisation level, 2nd order parameter (in sfu.deg⁻¹).
    pub ai1: f64,

    /// Effective ionisation level, 3rd order parameter (in sfu.deg⁻²).
    pub ai2: f64,

    /// Ionospheric disturbance flags, for regions 1 to 5.
    pub iono_disturbance: [bool; GALILEO_IONO_REGIONS],

    /// E1-E5a Broadcast Group Delay (in seconds).
    pub bgd_e1_e5a: f64,

    /// E1-E5b Broadcast Group Delay (in seconds).
    pub bgd_e1_e5b: f64,

    /// E5b signal health status.
    pub e5b_health: GalileoSignalHealth,

    /// E1-B signal health status.
    pub e1b_health: GalileoSignalHealth,

    /// E5b data validity status: true means "working without guarantee".
    pub e5b_dvs: bool,

    /// E1-B data validity status: true means "working without guarantee".
    pub e1b_dvs: bool,

    /// GST week number (modulo 4096).
    pub wn: u16,

    /// GST time of week, in seconds.
    pub tow: u32,

    /// Spare bits (23 bits).
    pub spare: u32,
}

impl GalileoInavWord5 {
    /// Word type of [GalileoInavWord5]
    pub const WORD_TYPE: u8 = 5;

    /// Copies and returns [GalileoInavWord5] with updated NeQuick G effective
    /// ionisation level coefficients (ai0 in sfu, ai1 in sfu.deg⁻¹, ai2 in sfu.deg⁻²).
    pub fn with_ionisation_level(mut self, ai0: f64, ai1: f64, ai2: f64) -> Self {
        self.ai0 = ai0;
        self.ai1 = ai1;
        self.ai2 = ai2;
        self
    }

    /// Copies and returns [GalileoInavWord5] with ionospheric disturbance asserted
    /// for given `region` (1..=5). Other values are ignored.
    pub fn with_iono_disturbance(mut self, region: usize) -> Self {
        if (1..=GALILEO_IONO_REGIONS).contains(&region) {
            self.iono_disturbance[region - 1] = true;
        }
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated E1-E5a Broadcast Group Delay (in seconds).
    pub fn with_bgd_e1_e5a_seconds(mut self, bgd: f64) -> Self {
        self.bgd_e1_e5a = bgd;
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated E1-E5b Broadcast Group Delay (in seconds).
    pub fn with_bgd_e1_e5b_seconds(mut self, bgd: f64) -> Self {
        self.bgd_e1_e5b = bgd;
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated E5b signal health status.
    pub fn with_e5b_health(mut self, health: GalileoSignalHealth) -> Self {
        self.e5b_health = health;
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated E1-B signal health status.
    pub fn with_e1b_health(mut self, health: GalileoSignalHealth) -> Self {
        self.e1b_health = health;
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated E5b data validity status
    /// (true: working without guarantee).
    pub fn with_e5b_dvs(mut self, dvs: bool) -> Self {
        self.e5b_dvs = dvs;
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated E1-B data validity status
    /// (true: working without guarantee).
    pub fn with_e1b_dvs(mut self, dvs: bool) -> Self {
        self.e1b_dvs = dvs;
        self
    }

    /// Copies and returns [GalileoInavWord5] with updated GST week number and time of week (s).
    pub fn with_gst(mut self, wn: u16, tow_seconds: u32) -> Self {
        self.wn = wn;
        self.tow = tow_seconds;
        self
    }

    /// True if both E1-B and E5b signals are healthy and their data valid.
    pub fn is_healthy(&self) -> bool {
        self.e1b_health.is_ok() && self.e5b_health.is_ok() && !self.e1b_dvs && !self.e5b_dvs
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_ionisation_level(48.5, 0.2, 0.004)
            .with_iono_disturbance(2)
            .with_bgd_e1_e5a_seconds(-7.0e-9)
            .with_bgd_e1_e5b_seconds(-7.9e-9)
            .with_e5b_health(GalileoSignalHealth::InTest)
            .with_e1b_dvs(true)
            .with_gst(1339, 455_905)
    }
}

impl Message<BigEndian> for GalileoInavWord5 {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_WORD_BITS
    }
}

impl BitRead<'_, BigEndian> for GalileoInavWord5 {
    /// Reads the 128 bits of the word (word type included).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        read_word_type(stream, Self::WORD_TYPE)?;

        let ai0 = from_scaled(stream.read_int::<u16>(11)?, -2);
        let ai1 = from_scaled(stream.read_int::<i16>(11)?, -8);
        let ai2 = from_scaled(stream.read_int::<i16>(14)?, -15);

        let mut iono_disturbance = [false; GALILEO_IONO_REGIONS];
        for flag in iono_disturbance.iter_mut() {
            *flag = stream.read_bool()?;
        }

        let bgd_e1_e5a = from_scaled(stream.read_int::<i16>(10)?, -32);
        let bgd_e1_e5b = from_scaled(stream.read_int::<i16>(10)?, -32);
        let e5b_health = GalileoSignalHealth::decode(stream.read_int::<u8>(2)?);
        let e1b_health = GalileoSignalHealth::decode(stream.read_int::<u8>(2)?);
        let e5b_dvs = stream.read_bool()?;
        let e1b_dvs = stream.read_bool()?;
        let wn = stream.read_int::<u16>(12)?;
        let tow = stream.read_int::<u32>(20)?;
        let spare = stream.read_int::<u32>(23)?;

        Ok(Self {
            ai0,
            ai1,
            ai2,
            iono_disturbance,
            bgd_e1_e5a,
            bgd_e1_e5b,
            e5b_health,
            e1b_health,
            e5b_dvs,
            e1b_dvs,
            wn,
            tow,
            spare,
        })
    }
}

impl BitWrite<BigEndian> for GalileoInavWord5 {
    /// Writes the 128 bits of the word (word type included).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(Self::WORD_TYPE, 6)?;
        write_unsigned(stream, to_scaled(self.ai0, -2), 11)?;
        write_signed(stream, to_scaled(self.ai1, -8), 11)?;
        write_signed(stream, to_scaled(self.ai2, -15), 14)?;

        for flag in self.iono_disturbance.iter() {
            stream.write_bool(*flag)?;
        }

        write_signed(stream, to_scaled(self.bgd_e1_e5a, -32), 10)?;
        write_signed(stream, to_scaled(self.bgd_e1_e5b, -32), 10)?;
        stream.write_int(self.e5b_health.encode(), 2)?;
        stream.write_int(self.e1b_health.encode(), 2)?;
        stream.write_bool(self.e5b_dvs)?;
        stream.write_bool(self.e1b_dvs)?;
        stream.write_int(self.wn & 0xfff, 12)?;
        stream.write_int(self.tow & 0xf_ffff, 20)?;
        stream.write_int(self.spare & 0x7f_ffff, 23)?;
        Ok(())
    }
}

crate::bits::impl_eq_by_encoding!(GalileoInavWord5, crate::galileo::GALILEO_INAV_WORD_BYTES);

#[cfg(test)]
mod test {
    use crate::{
        galileo::{GalileoInavWord5, GalileoSignalHealth},
        Message,
    };

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavWord5::default();
        assert!(default.is_healthy());

        let mut buffer = [0u8; 16];
        default.encode(&mut buffer).unwrap();
        assert_eq!(buffer[0], 0x14); // word type 5
        assert_eq!(GalileoInavWord5::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn model_reciprocal() {
        let model = GalileoInavWord5::model();
        assert!(!model.is_healthy());

        let mut buffer = [0u8; 16];
        model.encode(&mut buffer).unwrap();

        let decoded = GalileoInavWord5::decode(&buffer).unwrap();
        assert_eq!(decoded, model);
        assert_eq!(decoded.ai0, 48.5);
        assert!((decoded.ai1 - 0.2).abs() < 2.0e-3);
        assert!((decoded.ai2 - 0.004).abs() < 1.0e-4);
        assert_eq!(decoded.iono_disturbance, [false, true, false, false, false]);
        assert!((decoded.bgd_e1_e5a - -7.0e-9).abs() < 1e-10);
        assert!((decoded.bgd_e1_e5b - -7.9e-9).abs() < 1e-10);
        assert_eq!(decoded.e5b_health, GalileoSignalHealth::InTest);
        assert_eq!(decoded.e1b_health, GalileoSignalHealth::Ok);
        assert!(!decoded.e5b_dvs);
        assert!(decoded.e1b_dvs);
        assert_eq!(decoded.wn, 1339);
        assert_eq!(decoded.tow, 455_905);
    }
}
