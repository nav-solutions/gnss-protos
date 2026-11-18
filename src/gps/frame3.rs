use crate::{
    bits::{from_scaled, rad_to_semicircles, to_scaled, write_signed},
    gps::GPS_WORD_DATA_BITS,
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

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
    pub fn with_inclination_radians(self, angle_rad: f64) -> Self {
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
    pub fn with_longitude_ascending_node_radians(self, angle_rad: f64) -> Self {
        self.with_longitude_ascending_node_semicircles(rad_to_semicircles(angle_rad))
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega (in semicircles)
    pub fn with_omega_semicircles(mut self, angle_semicircles: f64) -> Self {
        self.omega = angle_semicircles;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega (in radians)
    pub fn with_omega_radians(self, angle_rad: f64) -> Self {
        self.with_omega_semicircles(rad_to_semicircles(angle_rad))
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega rate (in semicircles.s⁻¹)
    pub fn with_omega_dot_semicircles_s(mut self, omega_dot: f64) -> Self {
        self.omega_dot = omega_dot;
        self
    }

    /// Copies and returns [GpsQzssFrame3] with updated omega velocity (in radians.s⁻¹)
    pub fn with_omega_dot_rad_s(self, omega_dot_rad: f64) -> Self {
        self.with_omega_dot_semicircles_s(rad_to_semicircles(omega_dot_rad))
    }

    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
            .with_cic_radians(1.0e-6)
            .with_cis_radians(2.0e-6)
            .with_crc_meters(122.0)
            .with_iode(0x12)
            .with_omega_semicircles(4e-1)
            .with_omega_dot_semicircles_s(-2.5e-9)
            .with_inclination_semicircles(3e-1)
            .with_inclination_rate_semicircles_s(1.5e-10)
            .with_longitude_ascending_node_semicircles(3e-1)
    }
}

crate::bits::impl_eq_by_encoding!(GpsQzssFrame3, crate::gps::GPS_SUBFRAME_DATA_BYTES);

impl Message<BigEndian> for GpsQzssFrame3 {
    fn encoding_size(&self) -> usize {
        8 * GPS_WORD_DATA_BITS / 8
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame3 {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    /// Parameters split over two words by the ICD (Omega0, i0, omega) are contiguous here.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        // words 3 & 4
        write_signed(stream, to_scaled(self.cic, -29), 16)?;
        write_signed(stream, to_scaled(self.omega0, -31), 32)?;

        // words 5 & 6
        write_signed(stream, to_scaled(self.cis, -29), 16)?;
        write_signed(stream, to_scaled(self.i0, -31), 32)?;

        // words 7 & 8
        write_signed(stream, to_scaled(self.crc, -5), 16)?;
        write_signed(stream, to_scaled(self.omega, -31), 32)?;

        // word 9
        write_signed(stream, to_scaled(self.omega_dot, -43), 24)?;

        // word 10 (last 2 bits are solved when the frame is encoded)
        stream.write_int(self.iode, 8)?;
        write_signed(stream, to_scaled(self.idot, -43), 14)?;
        stream.write_int(0u8, 2)?;

        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame3 {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    /// Parameters split over two words by the ICD (Omega0, i0, omega) are contiguous here.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        // words 3 & 4
        let cic = from_scaled(stream.read_int::<i16>(16)?, -29);
        let omega0 = from_scaled(stream.read_int::<i32>(32)?, -31);

        // words 5 & 6
        let cis = from_scaled(stream.read_int::<i16>(16)?, -29);
        let i0 = from_scaled(stream.read_int::<i32>(32)?, -31);

        // words 7 & 8
        let crc = from_scaled(stream.read_int::<i16>(16)?, -5);
        let omega = from_scaled(stream.read_int::<i32>(32)?, -31);

        // word 9
        let omega_dot = from_scaled(stream.read_int::<i32>(24)?, -43);

        // word 10
        let iode = stream.read_int::<u8>(8)?;
        let idot = from_scaled(stream.read_int::<i16>(14)?, -43);
        let _t = stream.read_int::<u8>(2)?; // solved for parity, no meaning

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
    use crate::{gps::GpsQzssFrame3, Message};

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

            let mut buf = [0; 1024];

            frame.encode(&mut buf).unwrap_or_else(|e| {
                panic!("Failed to encode frame: {}", e);
            });

            let decoded = GpsQzssFrame3::decode(&buf).unwrap_or_else(|e| {
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
