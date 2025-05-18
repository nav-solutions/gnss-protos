#[cfg(feature = "gps")]
use crate::gps::GpsError;

/// GNSS-Proto error
pub enum Error {
    #[cfg(feature = "gps")]
    Gps(GpsError),
}