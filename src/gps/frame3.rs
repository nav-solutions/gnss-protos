use crate::{
    gps::{rad_to_semicircles, GPS_WORDS_PER_FRAME},
    twos_complement,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

const WORD10_IODE_MASK: u32 = 0x3fc00000;
const WORD10_IODE_SHIFT: u32 = 22;
const WORD10_IDOT_MASK: u32 = 0x003fff00;
const WORD10_IDOT_SHIFT: u32 = 8;

/// [GpsQzssFrame3] Ephemeris #3 frame interpretation.
#[derive(Debug, Default, Copy, Clone)]
pub struct GpsQzssFrame3 {
    /// Inclination angle cosine harmonic in radians.
    pub cic: f64,

    /// Inclination angle sine harmonic in radians.
    pub cis: f64,

    /// Orbit radius cosine harmonic in meters.
    pub crc: f64,

    /// Inclination angle at reference time  (in semicircles)
    pub i0: f64,

    /// IODE: Issue of Data (Ephemeris)
    pub iode: u8,

    /// Rate of inclination angle (in semicircles.s⁻¹)
    pub idot: f64,

    /// Longitude of ascending node of orbit plane at weekly epoch (in semicircles)
    pub omega0: f64,

    /// Omega (in semicircles)
    pub omega: f64,

    /// Omega_dot (in semicircles.s⁻¹)
    pub omega_dot: f64,
}

impl GpsQzssFrame3 {
    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
            .with_cic_radians(1.0e-6)
            .with_cis_radians(2.0e-6)
            .with_crc_meters(122.0)
            .with_iode(0x12)
            .with_omega_semicircles(4e-1)
            .with_omega_dot_semicircles_s(1e-3)
            .with_inclination_semicircles(1e-3)
            .with_inclination_rate_semicircles_s(1e-9)
            .with_longitude_ascending_node_semicircles(3e-1)
    }

    /// Copies and returns [GpsQzssFrame3] with updated IODE
    pub fn with_iode(mut self, iode: u8) -> Self {
        self.iode = iode;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated Cic correction term
    pub fn with_cic_radians(mut self, cic_rad: f64) -> Self {
        self.cic = cic_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated Cic correction term
    pub fn with_cis_radians(mut self, cis_rad: f64) -> Self {
        self.cis = cis_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated inclination angle (in semicircles) at reference time.
    pub fn with_inclination_semicircles(mut self, angle_semicircles: f64) -> Self {
        self.i0 = angle_semicircles;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated inclination angle (in radians) at reference time.
    pub fn with_inclination_radians(mut self, angle_rad: f64) -> Self {
        self.with_inclination_semicircles(rad_to_semicircles(angle_rad))
    }

    /// Copies and returns [GpsQzssFrame3] with updated inclination rate (in semicircles/s).
    pub fn with_inclination_rate_semicircles_s(mut self, rate_semicircles_sec: f64) -> Self {
        self.idot = rate_semicircles_sec;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated inclination rate (in radians/s).
    pub fn with_inclination_rate_radians_s(mut self, rate_rad_sec: f64) -> Self {
        self.idot = rate_rad_sec;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated orbit radius cosine harmonic term (meters)
    pub fn with_crc_meters(mut self, crc_m: f64) -> Self {
        self.crc = crc_m;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated longitude of ascending node (in semicircles) at reference time.
    pub fn with_longitude_ascending_node_semicircles(mut self, angle_semicircles: f64) -> Self {
        self.omega0 = angle_semicircles;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated longitude of ascending node (in radians) at reference time.
    pub fn with_longitude_ascending_node_radians(mut self, angle_rad: f64) -> Self {
        self.with_longitude_ascending_node_semicircles(rad_to_semicircles(angle_rad))
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega (in semicircles)
    pub fn with_omega_semicircles(mut self, angle_semicircles: f64) -> Self {
        self.omega = angle_semicircles;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega (in radians)
    pub fn with_omega_radians(mut self, angle_rad: f64) -> Self {
        self.with_omega_semicircles(rad_to_semicircles(angle_rad))
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega rate (in semicircles.s⁻¹)
    pub fn with_omega_dot_semicircles_s(mut self, omega_dot: f64) -> Self {
        self.omega_dot = omega_dot;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega velocity (in radians.s⁻¹)
    pub fn with_omega_dot_rad_s(mut self, omega_dot_rad: f64) -> Self {
        self.with_omega_dot_semicircles_s(rad_to_semicircles(omega_dot_rad))
    }
}

impl PartialEq for GpsQzssFrame3 {
    fn eq(&self, rhs: &Self) -> bool {
        if (self.cic - rhs.cic).abs() > 1e-9 {
            return false;
        }

        if (self.cis - rhs.cis).abs() > 1e-8 {
            return false;
        }

        if (self.crc - rhs.crc).abs() > 1e-2 {
            return false;
        }

        if (self.i0 - rhs.i0).abs() > 1e-3 {
            return false;
        }

        if self.iode != rhs.iode {
            return false;
        }

        if (self.idot - rhs.idot).abs() > 1e-3 {
            return false;
        }

        if (self.omega0 - rhs.omega0).abs() > 1e-3 {
            return false;
        }

        if (self.omega - rhs.omega).abs() > 1e-3 {
            return false;
        }

        if (self.omega_dot - rhs.omega_dot).abs() > 1e-3 {
            return false;
        }

        true
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame3 {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        let omega0 = (self.omega0 * 2.0_f64.powi(31)).round() as u32;
        let omega0_msb = ((omega0 & 0xff00_0000) >> 24) as u8;
        let omega0_lsb = omega0 & 0x00ff_ffff;

        let cic = (self.cic * 2.0_f64.powi(29)).round() as i16;

        stream.write_int(cic, 16)?;
        stream.write_int(omega0_msb, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(omega0_lsb, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let cis = (self.cis * 2.0_f64.powi(29)).round() as i16;
        let i0 = (self.i0 * 2.0_f64.powi(31)).round() as u32;
        let i0_msb = ((i0 & 0xff00_0000) >> 24) as u8;
        let i0_lsb = i0 & 0x00ff_ffff;

        stream.write_int(cis, 16)?;
        stream.write_int(i0_msb, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(i0_lsb, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let crc = (self.crc * 2.0_f64.powi(5)).round() as i16;
        let omega = (self.omega * 2.0_f64.powi(31)).round() as u32;
        let omega_msb = ((omega & 0xff00_0000) >> 24) as u8;
        let omega_lsb = omega & 0x00ff_ffff;

        stream.write_int(crc, 16)?;
        stream.write_int(omega_msb, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(omega_lsb, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let omega_dot = (self.omega_dot * 2.0_f64.powi(43)).round() as i32;
        stream.write_int(omega_dot, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let idot = (self.idot * 2.0_f64.powi(43)).round() as i16;
        stream.write_int(self.iode, 8)?;
        stream.write_int(idot, 14)?;
        stream.write_int(0, 2)?; // TODO (parity)
        stream.write_int(0, 6)?; // TODO (parity)

        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame3 {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let cic = stream.read_int::<i16>(16)?;
        let cic = (cic as f64) * 2.0_f64.powi(-29);

        let omega0_msb = stream.read_int::<u8>(8)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let omega0_lsb = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let mut omega0 = omega0_msb as u32;
        omega0 <<= 24;
        omega0 |= omega0_lsb;
        let omega0 = (omega0 as f64) * 2.0_f64.powi(-31);

        let cis = stream.read_int::<i16>(16)?;
        let cis = (cis as f64) * 2.0_f64.powi(-29);

        let i0_msb = stream.read_int::<u8>(8)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let i0_lsb = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let mut i0 = i0_msb as u32;
        i0 <<= 24;
        i0 |= i0_lsb;
        let i0 = (i0 as f64) * 2.0_f64.powi(-31);

        let crc = stream.read_int::<i16>(16)?;
        let crc = (crc as f64) * 2.0_f64.powi(-5);

        let omega_msb = stream.read_int::<u8>(8)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let omega_lsb = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let mut omega = omega_msb as u32;
        omega <<= 24;
        omega |= omega_lsb;
        let omega = (omega as f64) * 2.0_f64.powi(-31);

        let omega_dot = stream.read_int::<u32>(24)?;
        let omega_dot = (omega_dot as f64) * 2.0_f64.powi(-43);
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let iode = stream.read_int::<u8>(8)?;
        let idot = stream.read_int::<i16>(14)?;
        let idot = (idot as f64) * 2.0_f64.powi(-43);

        let nib = stream.read_int::<u8>(2)?; // TODO (parity)
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        Ok(Self {
            cic,
            cis,
            omega,
            omega0,
            omega_dot,
            crc,
            i0,
            idot,
            iode,
        })
    }
}

#[cfg(test)]
mod frame3 {
    use crate::{
        gps::{GpsBuffer, GpsQzssFrame3},
        Buffering,
    };

    #[test]
    fn reciprocal() {
        for (cic, cis, crc, i0, iode, idot, omega0, omega, omega_dot) in [
            (
                1.0e-9, 2.0e-9, 3.0e-3, 3.0e-1, 20, 3.0e-10, 6.0e-1, 6.0e-1, 3.0e-9,
            ),
            (
                2.0e-9, 3.0e-9, 3.0e-3, 2.0e-1, 25, 4.0e-10, 7.0e-1, 7.0e-1, 4.0e-9,
            ),
        ] {
            let frame = GpsQzssFrame3 {
                cic,
                cis,
                crc,
                i0,
                iode,
                idot,
                omega0,
                omega,
                omega_dot,
            };

            let mut buf = GpsBuffer::default();
            let mut writer = buf.bit_write_stream();
            assert!(writer.write(&frame).is_ok(), "failed to encode frame");

            let mut reader = buf.bit_read_stream();

            let decoded = reader.read::<GpsQzssFrame3>().unwrap_or_else(|e| {
                panic!("failed to decode GPS EPH-3: {}", e);
            });

            assert_eq!(decoded.iode, iode);
            assert!((decoded.cic - cic).abs() < 1e-9);
            assert!((decoded.cis - cis).abs() < 1e-9);
            assert!((decoded.i0 - i0).abs() < 1e-9);
            assert!((decoded.idot - idot).abs() < 1e-9);
            assert!((decoded.omega0 - omega0).abs() < 1e-9);
            assert!((decoded.omega - omega).abs() < 1e-9);
            assert!((decoded.omega_dot - omega_dot).abs() < 1e-9);
        }
    }
}
