//! GPS / QZSS protocol

mod frame1;
// mod frame2;
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

use frame1::UnscaledFrame as UnscaledFrame1;

const GPS_BITMASK: u32 = 0x3fffffff;
const GPS_PARITY_SIZE: u32 = 6;

/// Unscaled subframe
#[derive(Debug, Clone)]
pub(crate) enum UnscaledSubframe {
    /// GPS Ephemeris Frame #1
    Eph1(UnscaledFrame1),
    // /// GPS Ephemeris Frame #2
    // Eph2(GpsQzssFrame2),

    // /// GPS Ephemeris Frame #3
    // Eph3(GpsQzssFrame3),
}

impl Default for UnscaledSubframe {
    fn default() -> Self {
        Self::Eph1(UnscaledFrame1::default())
    }
}

impl UnscaledSubframe {
    pub fn scale(&self) -> GpsQzssSubframe {
        match self {
            Self::Eph1(frame1) => GpsQzssSubframe::Eph1(frame1.scale()),
        }
    }
}

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
    // /// GPS Ephemeris Frame #2
    // Eph2(GpsQzssFrame2),

    // /// GPS Ephemeris Frame #3
    // Eph3(GpsQzssFrame3),
}
