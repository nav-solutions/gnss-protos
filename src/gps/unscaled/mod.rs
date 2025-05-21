#[cfg(not(feature = "std"))]
use num_traits::float::FloatCore;

pub(crate) mod frame1;
pub(crate) mod frame2;
pub(crate) mod frame3;

pub(crate) use frame1::*;
pub(crate) use frame2::*;
pub(crate) use frame3::*;

use super::{scaled::*, RxmSfrbxGpsQzssHow, RxmSfrbxGpsQzssTelemetry};

impl GpsUnscaledEph3 {
    pub fn scale(&self) -> RxmSfrbxGpsQzssFrame3 {
        RxmSfrbxGpsQzssFrame3 {
            iode: self.word10.iode,
            cic: (self.word3.cic as f64) / 2.0_f64.powi(29),
            cis: (self.word5.cis as f64) / 2.0_f64.powi(29),
            crc: (self.word7.crc as f64) / 2.0_f64.powi(5),

            i0: {
                let mut i0 = self.word5.i0_msb as u32;
                i0 <<= 24;
                i0 |= self.word6.i0_lsb;

                let i0 = (i0 as i32) as f64;
                i0 / 2.0_f64.powi(31)
            },

            omega0: {
                let mut omega0 = self.word3.omega0_msb as u32;
                omega0 <<= 24;
                omega0 |= self.word4.omega0_lsb;

                let omega0 = (omega0 as i32) as f64;
                omega0 / 2.0_f64.powi(31)
            },

            idot: {
                let idot = self.word10.idot as f64;
                idot / 2.0_f64.powi(43)
            },

            omega_dot: {
                let omega_dot = self.word9.omega_dot as f64;
                omega_dot / 2.0_f64.powi(43)
            },

            omega: {
                // form the u32 raw word
                let mut omega = self.word7.omega_msb as u32;
                omega <<= 24;
                omega |= self.word8.omega_lsb;

                let omega = (omega as i32) as f64;
                omega / 2.0_f64.powi(31)
            },
        }
    }
}

/// Interpreted [GpsUnscaledSubframe]s (not scaled yet)
#[derive(Debug, Clone)]
pub(crate) enum GpsUnscaledSubframe {
    /// GPS Ephemeris #1 frame
    Eph1(GpsUnscaledEph1),

    /// GPS - Unscaled Subframe #2
    Eph2(GpsUnscaledEph2),

    /// GPS - Unscaled Subframe #3
    Eph3(GpsUnscaledEph3),

    /// Non supported subframe
    NonSupported,
}

impl GpsUnscaledSubframe {
    pub fn scale(&self) -> Option<RxmSfrbxGpsQzssSubframe> {
        match self {
            Self::Eph1(subframe) => Some(RxmSfrbxGpsQzssSubframe::Eph1(subframe.scale())),
            Self::Eph2(subframe) => Some(RxmSfrbxGpsQzssSubframe::Eph2(subframe.scale())),
            Self::Eph3(subframe) => Some(RxmSfrbxGpsQzssSubframe::Eph3(subframe.scale())),
            Self::NonSupported => None,
        }
    }
}

impl Default for GpsUnscaledSubframe {
    fn default() -> Self {
        Self::NonSupported
    }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct GpsUnscaledFrame {
    pub how: RxmSfrbxGpsQzssHow,
    pub subframe: GpsUnscaledSubframe,
    pub telemetry: RxmSfrbxGpsQzssTelemetry,
}

impl GpsUnscaledFrame {
    /// Scale this [GpsUnscaledFrame] into [RxmSfrbxGpsQzssFrame], if it is correctly supported.
    pub(crate) fn scale(&self) -> Option<RxmSfrbxGpsQzssFrame> {
        Some(RxmSfrbxGpsQzssFrame {
            how: self.how.clone(),
            subframe: self.subframe.scale()?,
            telemetry: self.telemetry.clone(),
        })
    }
}
