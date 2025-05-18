//! GPS / QZSS protocol

// mod frame1;
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

// pub use frame1::GpsQzssFrame1;
// pub use frame2::GpsQzssFrame2;
// pub use frame3::GpsQzssFrame3;

const GPS_BITMASK: u32 = 0x3fffffff;
const GPS_PARITY_SIZE: u32 = 6;

// /// GPS / QZSS Interpreted subframes
// #[derive(Debug, Clone)]
// pub enum GpsQzssSubframe {
//     /// GPS Ephemeris Frame #1
//     Eph1(GpsQzssFrame1),

//     /// GPS Ephemeris Frame #2
//     Eph2(GpsQzssFrame2),

//     /// GPS Ephemeris Frame #3
//     Eph3(GpsQzssFrame3),
// }

/// GPS / QZSS interpreted frame.
#[derive(Debug, Clone)]
pub struct GpsQzssFrame {
    /// [GpsQzssTelemetry] describes following frame.
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,
    // /// [GpsQzssSubframe] depends on associated How.
    // pub subframe: GpsQzssSubframe,
}
