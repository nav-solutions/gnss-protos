//! GPS / QZSS protocol

mod frame1;
mod frame2;
// mod frame3;
mod decoder;
mod errors;
mod frame_id;
mod how;
mod tlm;

pub use decoder::GpsQzssDecoder;
pub use errors::GpsError;
pub use frame_id::GpsQzssFrameId;
pub use how::GpsQzssHow;
pub use tlm::GpsQzssTelemetry;

pub use frame1::GpsQzssFrame1;
pub use frame2::GpsQzssFrame2;

use frame1::UnscaledFrame as UnscaledFrame1;
use frame2::UnscaledFrame as UnscaledFrame2;

/// GPS / QZSS interpreted frame.
#[derive(Debug, Clone)]
pub struct GpsQzssFrame {
    /// [GpsQzssTelemetry] describes following frame.
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,

    /// [GpsQzssSubframe] depends on associated How.
    pub subframe: GpsQzssSubframe,
}

/// GPS / QZSS Interpreted subframes
#[derive(Debug, Clone)]
pub enum GpsQzssSubframe {
    /// GPS Ephemeris Frame #1
    Eph1(GpsQzssFrame1),

    /// GPS Ephemeris Frame #2
    Eph2(GpsQzssFrame2),
}

impl Default for GpsQzssSubframe {
    fn default() -> Self {
        Self::Eph1(Default::default())
    }
}

impl GpsQzssSubframe {
    /// Unwraps self as [GpsQzssFrame1] reference (if feasible)
    pub fn as_eph1(&self) -> Option<&GpsQzssFrame1> {
        match self {
            Self::Eph1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as mutable [GpsQzssFrame1] reference (if feasible)
    pub fn as_mut_eph1(&mut self) -> Option<&mut GpsQzssFrame1> {
        match self {
            Self::Eph1(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    pub fn as_eph2(&self) -> Option<&GpsQzssFrame2> {
        match self {
            Self::Eph2(frame) => Some(frame),
            _ => None,
        }
    }

    /// Unwraps self as [GpsQzssFrame2] reference (if feasible)
    pub fn as_mut_eph2(&mut self) -> Option<&mut GpsQzssFrame2> {
        match self {
            Self::Eph2(frame) => Some(frame),
            _ => None,
        }
    }
}
