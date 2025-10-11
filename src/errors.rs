#[cfg(feature = "gps")]
use crate::gps::GpsError;

/// GNSS-Proto error wrapper
#[derive(Debug)]
pub enum Error {
    #[cfg(feature = "gps")]
    Gps(GpsError),
}
