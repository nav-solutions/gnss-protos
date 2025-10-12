use thiserror::Error;

#[cfg(feature = "gps")]
use crate::gps::GpsError;

#[derive(Error, Debug, Copy, Clone)]
pub enum BufferingError {
    /// std-io WouldBlock equivalent
    #[error("buffer would block")]
    WouldBlock,

    /// Storage is full, target will not fit entirely (std-io equivalent).
    #[error("buffer is full")]
    StorageFull,
}

#[cfg(feature = "std")]
impl Into<std::io::Error> for BufferingError {
    fn into(self) -> std::io::Error {
        match self {
            Self::WouldBlock => std::io::ErrorKind::WouldBlock.into(),
            Self::StorageFull => std::io::ErrorKind::StorageFull.into(),
        }
    }
}

/// GNSS-Proto error
#[derive(Debug)]
pub enum Error {
    #[cfg(feature = "gps")]
    Gps(GpsError),
}
