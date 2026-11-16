use crate::{
    gps::{from_scaled, link::GPS_WORD_DATA_BITS, to_scaled, write_signed, write_unsigned},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// [GpsQzssAlmanach] page, found in Frame-5 pages 1 to 24,
/// and Frame-4 pages 2 to 5 and 7 to 10 (IS-GPS-200 §20.3.3.5.1.2).
/// Read and written on the 192 parity free data bits of words 3 to 10.
#[derive(Debug, Copy, Clone, Default)]
pub struct GpsQzssAlmanach {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// SV (page) ID (6 bits): satellite PRN this almanac applies to.
    pub sv_id: u8,

    /// Eccentricity
    pub eccentricity: f64,

    /// Almanac reference time (in seconds of week)
    pub toa_seconds: u32,

    /// Correction to inclination, relative to 0.3 semi-circles (in semi-circles)
    pub di: f64,

    /// Rate of right ascension (in semi circles.s⁻¹)
    pub omega_dot: f64,

    /// SV health (8-bit)
    pub sv_health: u8,

    /// Square root of semi-major axis, in square root of meters.
    pub sqrt_a: f64,

    /// Longitude of ascending node of orbit plane at weekly epoch (in semi circles)
    pub omega0: f64,

    /// Argument of perigee (in semi circles)
    pub omega: f64,

    /// Mean anomaly at reference time (in semi-circles)
    pub m0: f64,

    /// af1 (in seconds per second)
    pub af1: f64,

    /// af0 (in seconds)
    pub af0: f64,
}

impl PartialEq for GpsQzssAlmanach {
    fn eq(&self, rhs: &Self) -> bool {
        self.data_id == rhs.data_id
            && self.sv_id == rhs.sv_id
            && self.toa_seconds == rhs.toa_seconds
            && self.sv_health == rhs.sv_health
            && (self.eccentricity - rhs.eccentricity).abs() < 1e-6
            && (self.di - rhs.di).abs() < 1e-5
            && (self.omega_dot - rhs.omega_dot).abs() < 1e-11
            && (self.sqrt_a - rhs.sqrt_a).abs() < 1e-3
            && (self.omega0 - rhs.omega0).abs() < 1e-6
            && (self.omega - rhs.omega).abs() < 1e-6
            && (self.m0 - rhs.m0).abs() < 1e-6
            && (self.af0 - rhs.af0).abs() < 1e-6
            && (self.af1 - rhs.af1).abs() < 1e-11
    }
}

impl Message<BigEndian> for GpsQzssAlmanach {
    fn encoding_size(&self) -> usize {
        8 * GPS_WORD_DATA_BITS / 8
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssAlmanach {
    /// Writes the 192 data bits of words 3 to 10 (parity free, see [crate::gps::link]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        // word 3
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(self.sv_id & 0x3f, 6)?;
        write_unsigned(stream, to_scaled(self.eccentricity, -21), 16)?;

        // word 4
        write_unsigned(stream, (self.toa_seconds >> 12) as i64, 8)?;
        write_signed(stream, to_scaled(self.di, -19), 16)?;

        // word 5
        write_signed(stream, to_scaled(self.omega_dot, -38), 16)?;
        stream.write_int(self.sv_health, 8)?;

        // words 6 to 9
        write_unsigned(stream, to_scaled(self.sqrt_a, -11), 24)?;
        write_signed(stream, to_scaled(self.omega0, -23), 24)?;
        write_signed(stream, to_scaled(self.omega, -23), 24)?;
        write_signed(stream, to_scaled(self.m0, -23), 24)?;

        // word 10: af0 is split around af1 (8 MSB, then 3 LSB)
        let af0 = to_scaled(self.af0, -20).clamp(-1024, 1023);
        let af0 = (af0 as u32) & 0x7ff;

        stream.write_int(af0 >> 3, 8)?;
        write_signed(stream, to_scaled(self.af1, -38), 11)?;
        stream.write_int(af0 & 0x7, 3)?;
        stream.write_int(0u8, 2)?; // solved by the link layer

        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssAlmanach {
    /// Reads the 192 data bits of words 3 to 10 (parity free, see [crate::gps::link]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        // word 3
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;
        let eccentricity = from_scaled(stream.read_int::<u16>(16)?, -21);

        // word 4
        let toa_seconds = stream.read_int::<u32>(8)? << 12;
        let di = from_scaled(stream.read_int::<i16>(16)?, -19);

        // word 5
        let omega_dot = from_scaled(stream.read_int::<i16>(16)?, -38);
        let sv_health = stream.read_int::<u8>(8)?;

        // words 6 to 9
        let sqrt_a = from_scaled(stream.read_int::<u32>(24)?, -11);
        let omega0 = from_scaled(stream.read_int::<i32>(24)?, -23);
        let omega = from_scaled(stream.read_int::<i32>(24)?, -23);
        let m0 = from_scaled(stream.read_int::<i32>(24)?, -23);

        // word 10: af0 is split around af1 (8 MSB, then 3 LSB)
        let af0_msb = stream.read_int::<u16>(8)?;
        let af1 = from_scaled(stream.read_int::<i16>(11)?, -38);
        let af0_lsb = stream.read_int::<u16>(3)?;
        let _t = stream.read_int::<u8>(2)?;

        // 11-bit two's complement
        let af0 = (((af0_msb << 3) | af0_lsb) << 5) as i16 >> 5;
        let af0 = from_scaled(af0, -20);

        Ok(Self {
            data_id,
            sv_id,
            eccentricity,
            toa_seconds,
            di,
            omega_dot,
            sv_health,
            sqrt_a,
            omega0,
            omega,
            m0,
            af1,
            af0,
        })
    }
}

impl GpsQzssAlmanach {
    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
            .with_sv_id(12)
            .with_eccentricity(0.01)
            .with_toa_seconds(405_504)
    }

    /// Copies and returns [GpsQzssAlmanach] with updated SV (page) ID.
    pub fn with_sv_id(mut self, sv_id: u8) -> Self {
        self.sv_id = sv_id & 0x3f;
        self
    }

    /// Copies and returns [GpsQzssAlmanach] with updated eccentricity.
    pub fn with_eccentricity(mut self, e: f64) -> Self {
        self.eccentricity = e;
        self
    }

    /// Copies and returns [GpsQzssAlmanach] with updated reference time
    /// (in seconds of week, multiple of 4096).
    pub fn with_toa_seconds(mut self, toa_seconds: u32) -> Self {
        self.toa_seconds = toa_seconds;
        self
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn reciprocal() {
        for (e, toa, di, omega_dot, sqrt_a, omega0, omega, m0, af0, af1) in [
            (
                0.01, 405_504, 0.01, -2.5e-9, 5153.6, 0.5, -0.3, 0.9, 1.0e-4, 3.0e-12,
            ),
            (
                0.001, 4096, -0.02, 2.5e-9, 5153.1, -0.5, 0.3, -0.9, -4.5e-4, -1.0e-11,
            ),
        ] {
            let alm = GpsQzssAlmanach {
                data_id: 1,
                sv_id: 25,
                eccentricity: e,
                toa_seconds: toa,
                di,
                omega_dot,
                sv_health: 0x3f,
                sqrt_a,
                omega0,
                omega,
                m0,
                af0,
                af1,
            };

            let mut buffer = [0; 1024];

            alm.encode(&mut buffer).unwrap_or_else(|e| {
                panic!("failed to encode almanach: {}", e);
            });

            let decoded = GpsQzssAlmanach::decode(&buffer).unwrap_or_else(|e| {
                panic!("failed to decode almanach: {}", e);
            });

            assert_eq!(decoded, alm, "got {:#?}", decoded);
        }
    }
}
