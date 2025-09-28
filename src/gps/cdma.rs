use crate::gps::GpsQzssFrame;

/// CDMA modulator, used to scramble a stream of bits
/// then compatible with GPS/QZSS data stream.
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

    /// [CDMA] encode read-only rx buffer into tx (transmission) buffer.
    /// You should preallocate the tx buffer with zeros, so your
    /// transmission is correct. Correct GPS / QZSS transmission
    /// still requires to synchronously stream these bits
    /// at the rate of 50 bits per second.
    ///
    /// ## Input
    /// - rx: read-only buffer to encoded
    /// - tx: transmission, encoded buffer
    /// - size: number of bytes contained in rx buffer
    ///
    /// ## Returns
    /// - total number of _bits_ encoded in tx buffer.
    pub fn scramble(rx: &[u8], size: usize, tx: &mut [u8]) -> usize {

    }
}
