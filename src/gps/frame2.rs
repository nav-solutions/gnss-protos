use crate::{
    gps::{rad_to_semicircles, GpsError, GPS_WORDS_PER_FRAME, GPS_WORD_BITS, GPS_WORD_BYTES},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use core::f64::consts::PI;

/// [GpsQzssFrame2] Ephemeris #2 frame interpretation.
#[derive(Debug, Default, Copy, Clone)]
pub struct GpsQzssFrame2 {
    /// Time of issue of ephemeris (in seconds of week)
    /// at instant of transmission of the next MSB.
    /// Must be a multiple of 16 to correctly be encoded.
    pub toe: u32,

    /// IODE (Issue of Data)
    pub iode: u8,

    /// Mean anomaly at reference time (in semicircles)
    pub m0: f64,

    /// Mean motion difference from computed value (in semicircles)
    pub dn: f64,

    /// Latitude (cosine harmonic) in radians.
    pub cuc: f64,

    /// Latitude (sine harmonic) in radians.
    pub cus: f64,

    /// Orbit radius (sine harmonic) in meters.
    pub crs: f64,

    /// Orbit eccentricity.
    pub e: f64,

    /// Square root of semi-major axis, in square root of meters.
    pub sqrt_a: f64,

    /// Fit interval flag
    pub fit_int_flag: bool,

    /// AODO
    pub aodo: u8,
}

impl PartialEq for GpsQzssFrame2 {
    fn eq(&self, rhs: &Self) -> bool {
        if self.toe != rhs.toe {
            return false;
        }

        if self.iode != rhs.iode {
            return false;
        }

        if (self.m0 - rhs.m0).abs() > 1e-3 {
            return false;
        }

        if (self.dn - rhs.dn).abs() > 1e-8 {
            return false;
        }

        if (self.cuc - rhs.cuc).abs() > 1e-9 {
            return false;
        }

        if (self.cus - rhs.cus).abs() > 1e-9 {
            return false;
        }

        if (self.crs - rhs.crs).abs() > 1e-9 {
            return false;
        }

        if (self.e - rhs.e).abs() > 1e-10 {
            return false;
        }

        if (self.sqrt_a - rhs.sqrt_a).abs() > 1e-5 {
            return false;
        }

        if self.fit_int_flag != rhs.fit_int_flag {
            return false;
        }

        if self.aodo != rhs.aodo {
            return false;
        }

        true
    }
}

impl Message<BigEndian> for GpsQzssFrame2 {
    fn encoding_size(&self) -> usize {
        8 * GPS_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame2 {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        let crs = (self.crs * 2.0_f64.powi(5)).round() as i16;
        stream.write_int(self.iode, 8)?;
        stream.write_int(crs, 16)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let m0 = (self.m0 * 2.0_f64.powi(31)).round() as u32;
        let m0_msb = (m0 & 0xff00_0000) >> 24;
        let m0_lsb = m0 & 0x00ff_ffff;
        let dn = (self.dn * 2.0_f64.powi(43)).round() as i16;

        stream.write_int(dn, 16)?;
        stream.write_int(m0_msb, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(m0_lsb, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let e = (self.e * 2.0_f64.powi(33)).round() as u32;
        let e_msb = (e & 0xff00_0000) >> 24;
        let e_lsb = e & 0x00ff_ffff;

        let cuc = (self.cuc * 2.0_f64.powi(29)).round() as i16;

        stream.write_int(cuc, 16)?;
        stream.write_int(e_msb, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let e = (self.e * 2.0_f64.powi(33)).round() as i32;
        stream.write_int(e_lsb, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        let sqrt_a = (self.sqrt_a * 2.0_f64.powi(19)).round() as u32;
        let sqrt_a_msb = (sqrt_a & 0xff00_0000) >> 24;
        let sqrt_a_lsb = sqrt_a & 0x00ff_ffff;

        let cus = (self.cus * 2.0_f64.powi(29)).round() as i32;

        stream.write_int(cus, 16)?;
        stream.write_int(sqrt_a_msb, 8)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int(sqrt_a_lsb, 24)?;
        stream.write_int(0, 6)?; // TODO (parity)

        stream.write_int((self.toe / 16) as u16, 16)?;
        stream.write_bool(self.fit_int_flag)?;
        stream.write_int(self.aodo & 0x1f, 5)?;
        stream.write_int(0, 2)?; // TODO (parity)
        stream.write_int(0, 6)?; // TODO (parity)

        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame2 {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let iode = stream.read_int::<u8>(8)?;
        let crs = stream.read_int::<i16>(16)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let dn = stream.read_int::<i16>(16)?;
        let m0_msb = stream.read_int::<u8>(8)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let m0_lsb = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let cuc = stream.read_int::<i16>(16)?;
        let e_msb = stream.read_int::<u8>(8)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let e_lsb = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let cus = stream.read_int::<i16>(16)?;
        let sqrt_a_msb = stream.read_int::<u8>(8)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let sqrt_a_lsb = stream.read_int::<u32>(24)?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let toe = stream.read_int::<u16>(16)?;
        let fit_int_flag = stream.read_bool()?;
        let aodo = stream.read_int::<u8>(5)?;
        let parity = stream.read_int::<u8>(2)?; // TODO (parity)
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        let cuc = (cuc as f64) * 2.0_f64.powi(-29);
        let cus = (cus as f64) * 2.0_f64.powi(-29);
        let crs = (crs as f64) * 2.0_f64.powi(-5);
        let dn = (dn as f64) * 2.0_f64.powi(-43);

        let toe = (toe as u32) * 16;

        let mut m0 = m0_msb as u32;
        m0 <<= 24;
        m0 |= m0_lsb;
        let m0 = (m0 as f64) * 2.0_f64.powi(-31);

        let mut e = e_msb as u32;
        e <<= 24;
        e |= e_lsb;
        let e = (e as f64) * 2.0_f64.powi(-33);

        let mut sqrt_a = sqrt_a_msb as u32;
        sqrt_a <<= 24;
        sqrt_a |= sqrt_a_lsb;
        let sqrt_a = (sqrt_a as f64) * 2.0_f64.powi(-19);

        Ok(Self {
            iode,
            m0,
            cuc,
            cus,
            crs,
            dn,
            e,
            aodo,
            sqrt_a,
            toe,
            fit_int_flag,
        })
    }
}

impl GpsQzssFrame2 {
    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
            .with_toe_seconds(54_320)
            .with_iode(0x01)
            .with_mean_anomaly_semicircles(1.0e-1)
            .with_mean_motion_difference_semicircles(2.0e-1)
            .with_square_root_semi_major_axis(5353.0)
            .with_eccentricity(1.0e-1)
            .with_aodo(0x12)
            .with_cuc_radians(1e-6)
            .with_cus_radians(2e-6)
            .with_crs_meters(87.0)
            .with_fit_interval_flag()
    }

    /// Copies and returns [GpsQzssFrame2] with updated time of issue of Ephemeris
    /// in seconds of week.
    pub fn with_toe_seconds(mut self, toe_seconds: u32) -> Self {
        self.toe = toe_seconds;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated IODE value.
    pub fn with_iode(mut self, iode: u8) -> Self {
        self.iode = iode;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean anomaly (in semicircles) at reference time.
    pub fn with_mean_anomaly_semicircles(mut self, m0_semicircles: f64) -> Self {
        self.m0 = m0_semicircles;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean anomaly (in radians) at reference time.
    pub fn with_mean_anomaly_radians(mut self, m0_rad: f64) -> Self {
        self.with_mean_anomaly_semicircles(rad_to_semicircles(m0_rad))
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean motion difference (in semicircles)
    pub fn with_mean_motion_difference_semicircles(mut self, dn_semicircles: f64) -> Self {
        self.dn = dn_semicircles;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated mean motion difference (in radians)
    pub fn with_mean_motion_difference_radians(mut self, dn_rad: f64) -> Self {
        self.with_mean_motion_difference_semicircles(rad_to_semicircles(dn_rad))
    }

    /// Copies and returns [GpsQzssFrame2] with updated semi-major axis (in meters)
    pub fn with_semi_major_axis_meters(mut self, semi_major_m: f64) -> Self {
        self.sqrt_a = semi_major_m.sqrt();
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated square root of semi-major axis (in square root meters)
    pub fn with_square_root_semi_major_axis(mut self, sqrt_semi_major_m: f64) -> Self {
        self.sqrt_a = sqrt_semi_major_m;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated orbit eccentricity.
    pub fn with_eccentricity(mut self, e: f64) -> Self {
        self.e = e;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated 5-bit AODO mask.
    pub fn with_aodo(mut self, aodo: u8) -> Self {
        self.aodo = aodo & 0x1f;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated radius (sine component) in meters.
    pub fn with_crs_meters(mut self, crs_m: f64) -> Self {
        self.crs = crs_m;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated latitude cosine harmnoic correction term.
    pub fn with_cuc_radians(mut self, cuc_rad: f64) -> Self {
        self.cuc = cuc_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with updated latitude sine harmnoic correction term.
    pub fn with_cus_radians(mut self, cus_rad: f64) -> Self {
        self.cus = cus_rad;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with fit interval flag asserted.
    pub fn with_fit_interval_flag(mut self) -> Self {
        self.fit_int_flag = true;
        self
    }

    /// Copies and returns [GpsQzssFrame2] with fit interval flag deasserted.
    pub fn without_fit_interval_flag(mut self) -> Self {
        self.fit_int_flag = false;
        self
    }
}

#[cfg(test)]
mod frame2 {
    use crate::{gps::GpsQzssFrame2, Buffer, Message, StaticBuffer};

    use bitbuffer::{BigEndian, BitReadStream};

    #[test]
    fn reciprocal() {
        for (toe, iode, m0, dn, cuc, cus, crs, e, sqrt_a, fit_int_flag, aodo) in [
            (
                345_600, 10, 9.76E-1, 4.0e-9, 9.3e-7, 2.8E-6, -88.0, 0.01, 5153.639, false, 10,
            ),
            (
                2320, 10, 9.76E-1, 5.0e-9, 9.4e-7, 2.9e-6, -87.0, 0.010234, 5153.64, false, 1,
            ),
            (
                4800, 11, 9.78E-1, 6.0e-9, 9.8e-7, 3.0e-6, 87.0, 0.02, 5153.65, true, 0x1f,
            ),
            (
                4800, 11, 9.78E-1, 6.0e-9, 9.8e-7, 3.0e-6, 87.0, 0.02, 5153.65, true, 0x0f,
            ),
        ] {
            let frame = GpsQzssFrame2 {
                toe,
                iode,
                m0,
                dn,
                cuc,
                cus,
                crs,
                e,
                sqrt_a,
                fit_int_flag,
                aodo,
            };

            let mut buf = [0; 1024];

            frame.encode(&mut buf).unwrap_or_else(|e| {
                panic!("failed to encode frame: {}", e);
            });

            let decoded = GpsQzssFrame2::decode(&buf).unwrap_or_else(|e| {
                panic!("failed to decode GPS EPH-2: {}", e);
            });

            assert_eq!(decoded.toe, toe);
            assert_eq!(decoded.iode, iode);
            assert_eq!(decoded.aodo, aodo);
            assert_eq!(decoded.fit_int_flag, fit_int_flag);
            assert!(
                (decoded.cus - cus).abs() < 1e-5,
                "expcting {:.3E} got {:.3E}",
                cus,
                decoded.cus
            );
            assert!(
                (decoded.crs - crs).abs() < 1e-5,
                "expcting {:.3E} got {:.3E}",
                crs,
                decoded.crs
            );

            assert!(
                (decoded.e - e).abs() < 1e-5,
                "expecting {:.3E} got {:.3E}",
                e,
                decoded.e
            );

            assert!(
                (decoded.dn - dn).abs() < 1e-8,
                "expecting {:.3E} got {:.3E}",
                dn,
                decoded.dn
            );

            assert!(
                (decoded.m0 - m0).abs() < 1e-9,
                "expecting {:.3E} got {:.3E}",
                m0,
                decoded.m0
            );

            assert!(
                (decoded.sqrt_a - sqrt_a).abs() < 1e-6,
                "expecting {:.3E} got {:.3E}",
                sqrt_a,
                decoded.sqrt_a
            );
        }
    }
}
