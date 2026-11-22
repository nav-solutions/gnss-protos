//! Ionospheric (Klobuchar) and UTC parameters (subframe 4, page 18, SV ID 56).

use crate::{
    bits::{from_scaled, to_scaled, write_signed, write_unsigned},
    gps::lnav::{GPS_LNAV_CONFIG_DID, GPS_LNAV_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// Scale factors (power of 2) of the Klobuchar alpha coefficients.
const ALPHA_SCALES: [i32; 4] = [-30, -27, -24, -24];

/// Scale factors (power of 2) of the Klobuchar beta coefficients.
const BETA_SCALES: [i32; 4] = [11, 14, 16, 16];

/// Seconds per week
const WEEK_SECONDS: f64 = 604_800.0;

/// Seconds per day
const DAY_SECONDS: f64 = 86_400.0;

/// [GpsQzssLnavIonosphereUtc] (IS-GPS-200 §20.3.3.5.1.6 and §20.3.3.5.1.7), transmitted
/// in subframe 4 page 18 (SV ID 56): the Klobuchar ionospheric model coefficients
/// and the GPS to UTC conversion parameters.
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssLnavIonosphereUtc {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// Klobuchar alpha coefficients (amplitude of the vertical delay), in
    /// s, s.semicircle⁻¹, s.semicircle⁻², s.semicircle⁻³.
    pub alpha: [f64; 4],

    /// Klobuchar beta coefficients (period of the model), in
    /// s, s.semicircle⁻¹, s.semicircle⁻², s.semicircle⁻³.
    pub beta: [f64; 4],

    /// A1: drift of the GPS / UTC offset (in seconds per second)
    pub a1: f64,

    /// A0: GPS / UTC offset at reference time (in seconds)
    pub a0: f64,

    /// t_ot: UTC data reference time (in seconds of week, multiple of 4096)
    pub tot: u32,

    /// WN_t: UTC data reference week (8 LSB of the week number)
    pub wnt: u8,

    /// Δt_LS: current number of leap seconds (in seconds)
    pub delta_t_ls: i8,

    /// WN_LSF: week of the next (or last) leap second event (8 LSB of the week number)
    pub wn_lsf: u8,

    /// DN: day of week of the leap second event, 1 (Sunday) to 7: the leap second
    /// occurs at the end of this day.
    pub dn: u8,

    /// Δt_LSF: number of leap seconds, after the leap second event (in seconds)
    pub delta_t_lsf: i8,

    /// Reserved bits (14 bits) of word 10
    pub reserved: u16,
}

crate::bits::impl_eq_by_encoding!(GpsQzssLnavIonosphereUtc, 24); // TODO improve defs

impl Default for GpsQzssLnavIonosphereUtc {
    fn default() -> Self {
        Self {
            data_id: GPS_LNAV_CONFIG_DID,
            alpha: [0.0; 4],
            beta: [0.0; 4],
            a1: 0.0,
            a0: 0.0,
            tot: 0,
            wnt: 0,
            delta_t_ls: 0,
            wn_lsf: 0,
            dn: 0,
            delta_t_lsf: 0,
            reserved: 0,
        }
    }
}

impl Message<BigEndian> for GpsQzssLnavIonosphereUtc {
    fn encoding_size(&self) -> usize {
        24 // TODO improve defs
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_LNAV_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavIonosphereUtc {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        // word 3
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(Self::SV_ID, 6)?;

        // words 3 to 5: alpha0..3, beta0..3
        for (value, scale) in self.alpha.iter().zip(ALPHA_SCALES.iter()) {
            write_signed(stream, to_scaled(*value, *scale), 8)?;
        }

        for (value, scale) in self.beta.iter().zip(BETA_SCALES.iter()) {
            write_signed(stream, to_scaled(*value, *scale), 8)?;
        }

        // word 6: A1, words 7-8: A0 (24 MSB, then 8 LSB)
        write_signed(stream, to_scaled(self.a1, -50), 24)?;
        write_signed(stream, to_scaled(self.a0, -30), 32)?;

        // word 8
        write_unsigned(stream, (self.tot >> 12) as i64, 8)?;
        stream.write_int(self.wnt, 8)?;

        // word 9
        stream.write_int(self.delta_t_ls as u8, 8)?;
        stream.write_int(self.wn_lsf, 8)?;
        stream.write_int(self.dn, 8)?;

        // word 10
        stream.write_int(self.delta_t_lsf as u8, 8)?;
        stream.write_int(self.reserved & 0x3fff, 14)?;
        stream.write_int(0u8, 2) // t-bits: solved when the frame is encoded
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavIonosphereUtc {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;

        if sv_id != Self::SV_ID {
            return Err(BitError::UnmatchedDiscriminant {
                discriminant: sv_id as usize,
                enum_name: "GpsQzssLnavIonosphereUtc".to_string(),
            });
        }

        let mut alpha = [0.0; 4];
        let mut beta = [0.0; 4];

        for (value, scale) in alpha.iter_mut().zip(ALPHA_SCALES.iter()) {
            *value = from_scaled(stream.read_int::<i8>(8)?, *scale);
        }

        for (value, scale) in beta.iter_mut().zip(BETA_SCALES.iter()) {
            *value = from_scaled(stream.read_int::<i8>(8)?, *scale);
        }

        let a1 = from_scaled(stream.read_int::<i32>(24)?, -50);
        let a0 = from_scaled(stream.read_int::<i32>(32)?, -30);

        let tot = stream.read_int::<u32>(8)? << 12;
        let wnt = stream.read_int::<u8>(8)?;

        let delta_t_ls = stream.read_int::<i8>(8)?;
        let wn_lsf = stream.read_int::<u8>(8)?;
        let dn = stream.read_int::<u8>(8)?;

        let delta_t_lsf = stream.read_int::<i8>(8)?;
        let reserved = stream.read_int::<u16>(14)?;
        let _t = stream.read_int::<u8>(2)?;

        Ok(Self {
            data_id,
            alpha,
            beta,
            a1,
            a0,
            tot,
            wnt,
            delta_t_ls,
            wn_lsf,
            dn,
            delta_t_lsf,
            reserved,
        })
    }
}

/// Signed difference between two 8-bit truncated week numbers, in -128..=127.
fn week_difference(week: u32, reference: u8) -> i64 {
    let diff = (week as i64 - reference as i64).rem_euclid(256);

    if diff >= 128 {
        diff - 256
    } else {
        diff
    }
}

impl GpsQzssLnavIonosphereUtc {
    /// SV (page) ID of the ionosphere / UTC page.
    pub const SV_ID: u8 = 56;

    /// Copies and returns [GpsQzssLnavIonosphereUtc] with updated Klobuchar coefficients.
    pub fn with_klobuchar(mut self, alpha: [f64; 4], beta: [f64; 4]) -> Self {
        self.alpha = alpha;
        self.beta = beta;
        self
    }

    /// Copies and returns [GpsQzssLnavIonosphereUtc] with updated GPS / UTC polynomial:
    /// offset `a0` (s) and drift `a1` (s.s⁻¹) at reference time `tot` (seconds of week,
    /// multiple of 4096) of week `wnt` (8 LSB are kept).
    pub fn with_utc_polynomial(mut self, a0: f64, a1: f64, tot: u32, wnt: u32) -> Self {
        self.a0 = a0;
        self.a1 = a1;
        self.tot = tot;
        self.wnt = (wnt & 0xff) as u8;
        self
    }

    /// Copies and returns [GpsQzssLnavIonosphereUtc] with updated leap seconds information:
    /// current leap seconds, leap seconds after the event of week `wn_lsf` (8 LSB are kept)
    /// at the end of day `dn` (1 to 7).
    pub fn with_leap_seconds(
        mut self,
        delta_t_ls: i8,
        delta_t_lsf: i8,
        wn_lsf: u32,
        dn: u8,
    ) -> Self {
        self.delta_t_ls = delta_t_ls;
        self.delta_t_lsf = delta_t_lsf;
        self.wn_lsf = (wn_lsf & 0xff) as u8;
        self.dn = dn;
        self
    }

    /// Returns the GPS - UTC offset (in seconds), at GPS time `tow_seconds` of
    /// week `week` (complete or truncated: only the 8 LSB are used),
    /// as defined in IS-GPS-200 §20.3.3.5.2.4:
    ///
    /// Δt_UTC = Δt_LS + A0 + A1 (t - t_ot + 604800 (WN - WN_t))
    ///
    /// Δt_LSF replaces Δt_LS once the leap second event (end of day DN of week WN_LSF)
    /// is past. The week numbers are compared on their 8 LSB, which is valid
    /// within ±127 weeks from the reference.
    pub fn gps_utc_offset_seconds(&self, week: u32, tow_seconds: f64) -> f64 {
        let dt =
            tow_seconds - self.tot as f64 + WEEK_SECONDS * week_difference(week, self.wnt) as f64;

        let event = WEEK_SECONDS * week_difference(week, self.wn_lsf) as f64 + tow_seconds
            - self.dn as f64 * DAY_SECONDS;

        let leap_seconds = if event >= 0.0 {
            self.delta_t_lsf
        } else {
            self.delta_t_ls
        };

        leap_seconds as f64 + self.a0 + self.a1 * dt
    }

    /// Returns the UTC time of day (in seconds, 0 to 86400) for given GPS time
    /// (see [Self::gps_utc_offset_seconds]).
    pub fn utc_time_of_day_seconds(&self, week: u32, tow_seconds: f64) -> f64 {
        (tow_seconds - self.gps_utc_offset_seconds(week, tow_seconds)).rem_euclid(DAY_SECONDS)
    }

    /// Klobuchar ionospheric delay model (IS-GPS-200 §20.3.3.5.2.5):
    /// returns the L1 ionospheric delay (in seconds).
    ///
    /// - `latitude`, `longitude`: user geodetic position (in semicircles)
    /// - `elevation`, `azimuth`: of the satellite seen from the user (in semicircles)
    /// - `tow_seconds`: GPS time (in seconds of week)
    ///
    /// Multiply by the speed of light to obtain meters, and by (f_L1/f)² for another frequency.
    pub fn klobuchar_delay_seconds(
        &self,
        latitude: f64,
        longitude: f64,
        elevation: f64,
        azimuth: f64,
        tow_seconds: f64,
    ) -> f64 {
        use core::f64::consts::PI;

        // earth centered angle (semicircles)
        let psi = 0.0137 / (elevation + 0.11) - 0.022;

        // sub-ionospheric latitude (semicircles)
        let phi_i = (latitude + psi * (azimuth * PI).cos()).clamp(-0.416, 0.416);

        // sub-ionospheric longitude (semicircles)
        let lambda_i = longitude + psi * (azimuth * PI).sin() / (phi_i * PI).cos();

        // geomagnetic latitude (semicircles)
        let phi_m = phi_i + 0.064 * ((lambda_i - 1.617) * PI).cos();

        // local time (s)
        let t = (4.32e4 * lambda_i + tow_seconds).rem_euclid(DAY_SECONDS);

        // obliquity factor
        let f = 1.0 + 16.0 * (0.53 - elevation).powi(3);

        let mut amplitude = 0.0;
        let mut period = 0.0;

        for n in 0..4 {
            amplitude += self.alpha[n] * phi_m.powi(n as i32);
            period += self.beta[n] * phi_m.powi(n as i32);
        }

        let amplitude = amplitude.max(0.0);
        let period = period.max(72_000.0);

        let x = 2.0 * PI * (t - 50_400.0) / period;

        if x.abs() < 1.57 {
            f * (5.0e-9 + amplitude * (1.0 - x.powi(2) / 2.0 + x.powi(4) / 24.0))
        } else {
            f * 5.0e-9
        }
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default()
            .with_klobuchar(
                [1.1176e-8, 7.4506e-9, -5.9605e-8, -5.9605e-8],
                [90112.0, 0.0, -196608.0, -65536.0],
            )
            .with_utc_polynomial(-2.79396772385e-9, -8.881784197e-16, 233_472, 2087)
            .with_leap_seconds(18, 18, 1929, 7)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bits::read_bits;

    #[test]
    fn reciprocal() {
        for page in [
            GpsQzssLnavIonosphereUtc::default(),
            GpsQzssLnavIonosphereUtc::model(),
            GpsQzssLnavIonosphereUtc::model()
                .with_leap_seconds(-5, 127, 255, 1)
                .with_utc_polynomial(-1.5, 7.0e-9, 4096 * 255, 0),
        ] {
            let mut bytes = [0u8; 24];
            assert_eq!(page.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssLnavIonosphereUtc::decode(&bytes).unwrap();
            assert_eq!(decoded, page, "got {:#?}", decoded);

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes);
        }
    }

    #[test]
    fn model_values() {
        let page = GpsQzssLnavIonosphereUtc::model();

        let mut bytes = [0u8; 24];
        page.encode(&mut bytes).unwrap();
        let decoded = GpsQzssLnavIonosphereUtc::decode(&bytes).unwrap();

        // the model coefficients are on the ICD grid (within half a LSB)
        for n in 0..4 {
            assert!((decoded.alpha[n] - page.alpha[n]).abs() <= 2f64.powi(ALPHA_SCALES[n] - 1));
            assert!((decoded.beta[n] - page.beta[n]).abs() <= 2f64.powi(BETA_SCALES[n] - 1));
        }

        assert!((decoded.a0 - page.a0).abs() < 2f64.powi(-31));
        assert!((decoded.a1 - page.a1).abs() < 2f64.powi(-51));
        assert_eq!(decoded.tot, 233_472);
        assert_eq!(decoded.wnt, (2087 & 0xff) as u8);
        assert_eq!(decoded.delta_t_ls, 18);
        assert_eq!(decoded.wn_lsf, (1929 & 0xff) as u8);
        assert_eq!(decoded.dn, 7);
    }

    #[test]
    fn layout() {
        let page = GpsQzssLnavIonosphereUtc {
            data_id: 1,
            alpha: [
                -2f64.powi(-30),
                2f64.powi(-27),
                2.0 * 2f64.powi(-24),
                3.0 * 2f64.powi(-24),
            ],
            beta: [4.0 * 2048.0, 5.0 * 16384.0, 6.0 * 65536.0, -7.0 * 65536.0],
            a1: 9.0 * 2f64.powi(-50),
            a0: (0x1234_5678 as f64) * 2f64.powi(-30),
            tot: 0xAB << 12,
            wnt: 0xCD,
            delta_t_ls: 18,
            wn_lsf: 0xEF,
            dn: 5,
            delta_t_lsf: -2,
            reserved: 0x2aaa,
        };

        let mut bytes = [0u8; 24];
        page.encode(&mut bytes).unwrap();

        // word 3: data ID, SV ID, alpha0, alpha1
        assert_eq!(read_bits(&bytes, 0, 2), Some(1));
        assert_eq!(read_bits(&bytes, 2, 6), Some(56));
        assert_eq!(read_bits(&bytes, 8, 8), Some(0xff));
        assert_eq!(read_bits(&bytes, 16, 8), Some(1));
        // word 4: alpha2, alpha3, beta0
        assert_eq!(read_bits(&bytes, 24, 8), Some(2));
        assert_eq!(read_bits(&bytes, 32, 8), Some(3));
        assert_eq!(read_bits(&bytes, 40, 8), Some(4));
        // word 5: beta1, beta2, beta3
        assert_eq!(read_bits(&bytes, 48, 8), Some(5));
        assert_eq!(read_bits(&bytes, 56, 8), Some(6));
        assert_eq!(read_bits(&bytes, 64, 8), Some(0xf9));
        // word 6: A1
        assert_eq!(read_bits(&bytes, 72, 24), Some(9));
        // word 7: A0 (24 MSB)
        assert_eq!(read_bits(&bytes, 96, 24), Some(0x12_3456));
        // word 8: A0 (8 LSB), t_ot, WN_t
        assert_eq!(read_bits(&bytes, 120, 8), Some(0x78));
        assert_eq!(read_bits(&bytes, 128, 8), Some(0xAB));
        assert_eq!(read_bits(&bytes, 136, 8), Some(0xCD));
        // word 9: delta_t_ls, WN_LSF, DN
        assert_eq!(read_bits(&bytes, 144, 8), Some(18));
        assert_eq!(read_bits(&bytes, 152, 8), Some(0xEF));
        assert_eq!(read_bits(&bytes, 160, 8), Some(5));
        // word 10: delta_t_lsf, reserved, t
        assert_eq!(read_bits(&bytes, 168, 8), Some(0xfe));
        assert_eq!(read_bits(&bytes, 176, 14), Some(0x2aaa));
        assert_eq!(read_bits(&bytes, 190, 2), Some(0));

        assert_eq!(GpsQzssLnavIonosphereUtc::decode(&bytes).unwrap(), page);
    }

    #[test]
    fn gps_utc_offset() {
        let page = GpsQzssLnavIonosphereUtc::default()
            .with_utc_polynomial(1.0e-8, 1.0e-14, 4096 * 10, 300)
            .with_leap_seconds(18, 19, 302, 3);

        // at reference time
        let offset = page.gps_utc_offset_seconds(300, 40_960.0);
        assert!((offset - (18.0 + 1.0e-8)).abs() < 1e-12);

        // one week later: drift
        let offset = page.gps_utc_offset_seconds(301, 40_960.0);
        assert!((offset - (18.0 + 1.0e-8 + 1.0e-14 * 604_800.0)).abs() < 1e-12);

        // truncated week numbers give the same result
        assert_eq!(
            page.gps_utc_offset_seconds(301 + 1024, 40_960.0),
            page.gps_utc_offset_seconds(301, 40_960.0)
        );

        // leap second event: end of day 3 of week 302
        assert!(page.gps_utc_offset_seconds(302, 3.0 * 86_400.0 - 1.0) < 18.1);
        assert!(page.gps_utc_offset_seconds(302, 3.0 * 86_400.0) > 18.9);
        assert!(page.gps_utc_offset_seconds(303, 0.0) > 18.9);

        // UTC time of day
        let tod = page.utc_time_of_day_seconds(300, 40_960.0);
        assert!((tod - (40_960.0 - 18.0 - 1.0e-8)).abs() < 1e-9);

        let tod = page.utc_time_of_day_seconds(300, 10.0);
        assert!((tod - (86_400.0 - 8.0 - 1.0e-8)).abs() < 1e-6);
    }

    #[test]
    fn week_differences() {
        assert_eq!(week_difference(300, 44), 0);
        assert_eq!(week_difference(301, 44), 1);
        assert_eq!(week_difference(299, 44), -1);
        assert_eq!(week_difference(255, 0), -1);
        assert_eq!(week_difference(256, 255), 1);
        assert_eq!(week_difference(2087 + 127, (2087 & 0xff) as u8), 127);
        assert_eq!(week_difference(2087 - 128, (2087 & 0xff) as u8), -128);
    }

    #[test]
    fn klobuchar() {
        let page = GpsQzssLnavIonosphereUtc::model();

        // at night: constant 5 ns, scaled by the obliquity factor
        let zenith = page.klobuchar_delay_seconds(0.25, 0.0, 0.5, 0.0, 0.0);
        let f = 1.0 + 16.0 * (0.53f64 - 0.5).powi(3);
        assert!((zenith - f * 5.0e-9).abs() < 1e-15);

        // maximum around 14:00 local time, larger than the night time value
        let day = page.klobuchar_delay_seconds(0.25, 0.0, 0.5, 0.0, 50_400.0);
        assert!(day > zenith);
        assert!(day < 100.0e-9);

        // symmetric around 14:00
        let before = page.klobuchar_delay_seconds(0.25, 0.0, 0.5, 0.0, 50_400.0 - 3600.0);
        let after = page.klobuchar_delay_seconds(0.25, 0.0, 0.5, 0.0, 50_400.0 + 3600.0);
        assert!((before - after).abs() < 1e-15);
        assert!(before < day);

        // lower elevation: longer path
        let low = page.klobuchar_delay_seconds(0.25, 0.0, 0.05, 0.0, 50_400.0);
        assert!(low > day);
        assert!(low < 3.5 * day);

        // longitude shifts the local time: 90°E is 6 hours ahead,
        // so the maximum occurs 6 hours earlier in GPS time
        let peak = 50_400.0 - 21_600.0;
        let east = page.klobuchar_delay_seconds(0.25, 0.5, 0.5, 0.0, peak);
        let east_before = page.klobuchar_delay_seconds(0.25, 0.5, 0.5, 0.0, peak - 3600.0);
        let east_after = page.klobuchar_delay_seconds(0.25, 0.5, 0.5, 0.0, peak + 3600.0);
        assert!(east > east_before);
        assert!((east_before - east_after).abs() < 1e-15);

        // null coefficients: the night time value, always
        let null = GpsQzssLnavIonosphereUtc::default();
        for t in [0.0, 30_000.0, 50_400.0, 80_000.0] {
            let delay = null.klobuchar_delay_seconds(0.1, -0.3, 0.2, 0.7, t);
            let f = 1.0 + 16.0 * (0.53f64 - 0.2).powi(3);
            assert!((delay - f * 5.0e-9).abs() < 1e-15);
        }
    }
}
