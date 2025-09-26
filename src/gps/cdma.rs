use crate::gps::GpsQzssFrame;

/// CDMA modulator, used to scramble a stream of bits
/// that are then GPS/QZSS compatible.
#[derive(Copy, Clone, PartialEq)]
pub struct CDMA {
    /// SV ID
    sat_id: u8,
}

impl CDMA {
    /// Creates a new [CDMA] modulator ready to scramble a stream of bits
    /// for this particular SV-ID
    pub fn from_satellite_id(sat_id: u8) -> Self {
        Self { sat_id }
    }

    /// Returns the

    /// [CDMA] encode this buffer, ready to transmit
    pub fn scramble(buffer: &mut [u8], size: usize) {}
}
