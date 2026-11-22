use crate::{
    bits::{from_scaled, to_scaled, write_signed, write_unsigned},
    gps::lnav::{GPS_LNAV_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

#[cfg(test)]
use crate::gps::lnav::GPS_LNAV_CONFIG_DID;

/// [GpsQzssLnavAlmanach] found in SF-5 pages 1 to 24,
/// and SF-4 pages 2 to 5 and 7 to 10 (IS-GPS-200 §20.3.3.5.1.2).
/// Read and written on the 192 parity free data bits of words 3 to 10.
#[derive(Debug, Copy, Clone, Default)]
pub struct GpsQzssLnavAlmanach {
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

crate::bits::impl_eq_by_encoding!(GpsQzssLnavAlmanach, GPS_LNAV_WORD_DATA_BITS);

impl Message<BigEndian> for GpsQzssLnavAlmanach {
    fn encoding_size(&self) -> usize {
        8 * GPS_LNAV_WORD_DATA_BITS / 8
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_LNAV_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavAlmanach {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
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
        stream.write_int(0u8, 2)?; // solved when the frame is encoded

        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavAlmanach {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
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

/// WGS-84 earth gravitational constant (m³.s⁻²), as defined by IS-GPS-200
const GM: f64 = 3.986005e14;

/// WGS-84 earth rotation rate (rad.s⁻¹), as defined by IS-GPS-200
const OMEGA_E_DOT: f64 = 7.2921151467e-5;

/// Reference inclination of the almanac (in semicircles): i = 0.30 + δi
const I0_SEMICIRCLES: f64 = 0.30;

/// Half a week (in seconds)
const HALF_WEEK_S: f64 = 302_400.0;

impl GpsQzssLnavAlmanach {
    /// Returns the PRN (SV ID) this almanac applies to.
    pub fn prn(&self) -> u8 {
        self.sv_id
    }

    /// Returns the orbit inclination (in semicircles): 0.30 + δi.
    pub fn inclination_semicircles(&self) -> f64 {
        I0_SEMICIRCLES + self.di
    }

    /// Returns the semi-major axis (in meters).
    pub fn semi_major_axis_m(&self) -> f64 {
        self.sqrt_a * self.sqrt_a
    }

    /// Returns the orbital period (in seconds).
    pub fn orbital_period_s(&self) -> f64 {
        2.0 * core::f64::consts::PI / (GM / self.semi_major_axis_m().powi(3)).sqrt()
    }

    /// Returns the time from the almanac reference time t_oa (in seconds), for a GPS time
    /// `tow_seconds` (in seconds of week) of the almanac week, accounting for the
    /// beginning or end of week crossovers (IS-GPS-200 Table 20-IV).
    pub fn time_from_reference_s(&self, tow_seconds: f64) -> f64 {
        let mut tk = tow_seconds - self.toa_seconds as f64;

        if tk > HALF_WEEK_S {
            tk -= 2.0 * HALF_WEEK_S;
        } else if tk < -HALF_WEEK_S {
            tk += 2.0 * HALF_WEEK_S;
        }

        tk
    }

    /// Returns the satellite clock correction (in seconds), `tk` seconds after
    /// the almanac reference time: af0 + af1 tk.
    pub fn clock_correction_s(&self, tk: f64) -> f64 {
        self.af0 + self.af1 * tk
    }

    /// Returns the eccentric anomaly (in radians), `tk` seconds after the almanac
    /// reference time, solving Kepler's equation.
    fn eccentric_anomaly_rad(&self, tk: f64) -> f64 {
        use core::f64::consts::PI;

        let n0 = (GM / self.semi_major_axis_m().powi(3)).sqrt();
        let mk = self.m0 * PI + n0 * tk;

        let mut ek = mk;

        for _ in 0..16 {
            let next = mk + self.eccentricity * ek.sin();
            let converged = (next - ek).abs() < 1.0e-14;
            ek = next;
            if converged {
                break;
            }
        }

        ek
    }

    /// Returns the satellite position in the ECEF WGS-84 frame (x, y, z, in meters),
    /// `tk` seconds after the almanac reference time (see [Self::time_from_reference_s]).
    ///
    /// This is the user algorithm of IS-GPS-200 Table 20-IV, applied to the almanac
    /// parameters (no harmonic corrections, null inclination rate and mean motion
    /// correction): accuracy is a few kilometers, which is suitable for
    /// satellite visibility prediction, acquisition aiding and constellation studies.
    pub fn position_ecef_m(&self, tk: f64) -> (f64, f64, f64) {
        use core::f64::consts::PI;

        let a = self.semi_major_axis_m();
        let e = self.eccentricity;

        let ek = self.eccentric_anomaly_rad(tk);

        // true anomaly, argument of latitude, radius
        let nu = ((1.0 - e * e).sqrt() * ek.sin()).atan2(ek.cos() - e);
        let phi = nu + self.omega * PI;
        let r = a * (1.0 - e * ek.cos());
        let i = self.inclination_semicircles() * PI;

        // position in the orbital plane
        let x_p = r * phi.cos();
        let y_p = r * phi.sin();

        // corrected longitude of ascending node
        let omega = self.omega0 * PI + (self.omega_dot * PI - OMEGA_E_DOT) * tk
            - OMEGA_E_DOT * self.toa_seconds as f64;

        (
            x_p * omega.cos() - y_p * i.cos() * omega.sin(),
            x_p * omega.sin() + y_p * i.cos() * omega.cos(),
            y_p * i.sin(),
        )
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated Keplerian parameters:
    /// square root of the semi-major axis (√m), inclination correction δi, longitude of
    /// ascending node Ω0, argument of perigee ω and mean anomaly M0 (in semicircles).
    pub fn with_keplerian_parameters(
        mut self,
        sqrt_a: f64,
        di: f64,
        omega0: f64,
        omega: f64,
        m0: f64,
    ) -> Self {
        self.sqrt_a = sqrt_a;
        self.di = di;
        self.omega0 = omega0;
        self.omega = omega;
        self.m0 = m0;
        self
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated rate of right ascension
    /// (in semicircles.s⁻¹).
    pub fn with_omega_dot(mut self, omega_dot: f64) -> Self {
        self.omega_dot = omega_dot;
        self
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated clock correction:
    /// af0 (s) and af1 (s.s⁻¹).
    pub fn with_clock(mut self, af0: f64, af1: f64) -> Self {
        self.af0 = af0;
        self.af1 = af1;
        self
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated SV health (8 bits).
    pub fn with_sv_health(mut self, sv_health: u8) -> Self {
        self.sv_health = sv_health;
        self
    }

    #[cfg(test)]
    pub fn model() -> Self {
        Self {
            data_id: GPS_LNAV_CONFIG_DID,
            ..Self::default()
        }
        .with_sv_id(12)
        .with_eccentricity(0.01)
        .with_toa_seconds(405_504)
        .with_keplerian_parameters(5153.6, 0.0128, -0.43, 0.27, 0.61)
        .with_omega_dot(-2.6e-9)
        .with_clock(1.2e-4, -3.6e-12)
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated SV (page) ID.
    pub fn with_sv_id(mut self, sv_id: u8) -> Self {
        self.sv_id = sv_id & 0x3f;
        self
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated eccentricity.
    pub fn with_eccentricity(mut self, e: f64) -> Self {
        self.eccentricity = e;
        self
    }

    /// Copies and returns [GpsQzssLnavAlmanach] with updated reference time
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
            let alm = GpsQzssLnavAlmanach {
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

            let decoded = GpsQzssLnavAlmanach::decode(&buffer).unwrap_or_else(|e| {
                panic!("failed to decode almanach: {}", e);
            });

            assert_eq!(decoded, alm, "got {:#?}", decoded);
        }
    }

    #[test]
    fn layout() {
        use crate::bits::read_bits;

        let alm = GpsQzssLnavAlmanach {
            data_id: 1,
            sv_id: 25,
            eccentricity: 0x1234 as f64 * 2f64.powi(-21),
            toa_seconds: 0x90 << 12,
            di: -(0x10 as f64) * 2f64.powi(-19),
            omega_dot: -(2.0 * 2f64.powi(-38)),
            sv_health: 0x3f,
            sqrt_a: 0xA1_0D2E as f64 * 2f64.powi(-11),
            omega0: 0x12_3456 as f64 * 2f64.powi(-23),
            omega: -(0x10 as f64) * 2f64.powi(-23),
            m0: 0x65_4321 as f64 * 2f64.powi(-23),
            af0: -(3.0 * 2f64.powi(-20)),
            af1: 5.0 * 2f64.powi(-38),
        };

        let mut bytes = [0u8; 24];
        alm.encode(&mut bytes).unwrap();

        // word 3: data ID, SV ID, e
        assert_eq!(read_bits(&bytes, 0, 2), Some(1));
        assert_eq!(read_bits(&bytes, 2, 6), Some(25));
        assert_eq!(read_bits(&bytes, 8, 16), Some(0x1234));
        // word 4: toa, di
        assert_eq!(read_bits(&bytes, 24, 8), Some(0x90));
        assert_eq!(read_bits(&bytes, 32, 16), Some(0xfff0));
        // word 5: omega_dot, health
        assert_eq!(read_bits(&bytes, 48, 16), Some(0xfffe));
        assert_eq!(read_bits(&bytes, 64, 8), Some(0x3f));
        // words 6 to 9
        assert_eq!(read_bits(&bytes, 72, 24), Some(0xA1_0D2E));
        assert_eq!(read_bits(&bytes, 96, 24), Some(0x12_3456));
        assert_eq!(read_bits(&bytes, 120, 24), Some(0xff_fff0));
        assert_eq!(read_bits(&bytes, 144, 24), Some(0x65_4321));
        // word 10: af0 (8 MSB), af1, af0 (3 LSB), t
        let af0 = (-3i32 as u32) & 0x7ff;
        assert_eq!(read_bits(&bytes, 168, 8), Some(af0 >> 3));
        assert_eq!(read_bits(&bytes, 176, 11), Some(5));
        assert_eq!(read_bits(&bytes, 187, 3), Some(af0 & 0x7));
        assert_eq!(read_bits(&bytes, 190, 2), Some(0));

        assert_eq!(GpsQzssLnavAlmanach::decode(&bytes).unwrap(), alm);
    }

    #[test]
    fn model_reciprocal() {
        let alm = GpsQzssLnavAlmanach::model();

        let mut bytes = [0u8; 24];
        alm.encode(&mut bytes).unwrap();

        let decoded = GpsQzssLnavAlmanach::decode(&bytes).unwrap();
        assert_eq!(decoded, alm);
        assert_eq!(decoded.prn(), 12);

        let mut reencoded = [0u8; 24];
        decoded.encode(&mut reencoded).unwrap();
        assert_eq!(reencoded, bytes);
    }

    #[test]
    fn circular_orbit() {
        // circular orbit, everything at the origin: the satellite starts
        // on the ascending node, which lies on the x axis
        let alm = GpsQzssLnavAlmanach::default()
            .with_sv_id(1)
            .with_keplerian_parameters(5153.6, 0.0, 0.0, 0.0, 0.0);

        let a = alm.semi_major_axis_m();

        let (x, y, z) = alm.position_ecef_m(0.0);
        assert!((x - a).abs() < 1e-6);
        assert!(y.abs() < 1e-6);
        assert!(z.abs() < 1e-6);

        // half a sidereal day: about 11h58
        let period = alm.orbital_period_s();
        assert!((period - 43_082.0).abs() < 60.0, "period: {}", period);

        let i = alm.inclination_semicircles() * core::f64::consts::PI;
        assert!((i.to_degrees() - 54.0).abs() < 1e-9);

        for tk in [0.0, 1000.0, period / 4.0, period / 2.0, 30_000.0, -5000.0] {
            let (x, y, z) = alm.position_ecef_m(tk);

            // constant radius
            let r = (x * x + y * y + z * z).sqrt();
            assert!((r - a).abs() < 1e-3, "tk={}", tk);

            // z only depends on the argument of latitude
            let u = 2.0 * core::f64::consts::PI * tk / period;
            assert!((z - a * u.sin() * i.sin()).abs() < 1e-3, "tk={}", tk);
        }

        // a quarter of orbit: at the highest latitude
        let (_, _, z) = alm.position_ecef_m(period / 4.0);
        assert!((z - a * i.sin()).abs() < 1e-3);
    }

    #[test]
    fn eccentric_orbit() {
        let e = 0.02;
        let alm = GpsQzssLnavAlmanach::default()
            .with_eccentricity(e)
            .with_keplerian_parameters(5153.6, 0.01, 0.3, 0.2, 0.0);

        let a = alm.semi_major_axis_m();
        let period = alm.orbital_period_s();

        // perigee at M0 = 0, apogee half an orbit later
        let r = |tk| {
            let (x, y, z) = alm.position_ecef_m(tk);
            (x * x + y * y + z * z).sqrt()
        };

        assert!((r(0.0) - a * (1.0 - e)).abs() < 1e-3);
        assert!((r(period / 2.0) - a * (1.0 + e)).abs() < 1e-3);

        for k in 0..100 {
            let r = r(k as f64 * period / 100.0);
            assert!(r >= a * (1.0 - e) - 1e-3 && r <= a * (1.0 + e) + 1e-3);
        }

        // periodic in the orbital plane (earth rotation aside)
        let alm = alm.with_keplerian_parameters(5153.6, 0.01, 0.3, 0.2, 0.4);
        let (x0, y0, z0) = alm.position_ecef_m(0.0);
        let (x1, y1, z1) = alm.position_ecef_m(period);
        assert!((z0 - z1).abs() < 1e-3);
        assert!(((x0 * x0 + y0 * y0) - (x1 * x1 + y1 * y1)).abs() / (x0 * x0 + y0 * y0) < 1e-9);
    }

    #[test]
    fn earth_rotation() {
        // a geostationary like check: the ECEF longitude of the node drifts
        // at the earth rotation rate
        let alm =
            GpsQzssLnavAlmanach::default().with_keplerian_parameters(5153.6, 0.0, 0.0, 0.0, 0.0);
        let period = alm.orbital_period_s();

        let (x, y, z) = alm.position_ecef_m(period);
        assert!(z.abs() < 1e-3);

        let longitude = y.atan2(x);
        let expected = -OMEGA_E_DOT * period;
        assert!((longitude - expected).abs() < 1e-9);
    }

    #[test]
    fn time_from_reference() {
        let alm = GpsQzssLnavAlmanach::default().with_toa_seconds(589_824);
        assert_eq!(alm.time_from_reference_s(589_824.0), 0.0);
        assert_eq!(alm.time_from_reference_s(600_000.0), 10_176.0);
        // beginning of next week
        assert_eq!(
            alm.time_from_reference_s(1000.0),
            1000.0 + 604_800.0 - 589_824.0
        );

        let alm = GpsQzssLnavAlmanach::default().with_toa_seconds(4096);
        // end of the previous week
        assert_eq!(
            alm.time_from_reference_s(604_000.0),
            604_000.0 - 604_800.0 - 4096.0
        );

        let alm = GpsQzssLnavAlmanach::default().with_clock(1.0e-4, 1.0e-11);
        assert!((alm.clock_correction_s(1.0e3) - (1.0e-4 + 1.0e-8)).abs() < 1e-15);
    }
}
