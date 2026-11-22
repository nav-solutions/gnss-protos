use thiserror::Error;

#[cfg(feature = "gps")]
use crate::gps::lnav::GpsError;

#[cfg(feature = "galileo")]
use crate::galileo::GalileoError;

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

/// Protocol decoding / encoding errors
#[derive(Debug, Error)]
pub enum Error {
    #[cfg(feature = "gps")]
    #[error("gps error: {0}")]
    #[cfg(feature = "gps")]
    Gps(GpsError),

    #[cfg(feature = "galileo")]
    #[error("galileo error: {0}")]
    #[cfg(feature = "gps")]
    Galileo(GalileoError),
}

/// Protocol modulation errors
#[derive(Debug, Error)]
pub enum ModulationError {
    #[error("output buffer too small")]
    WouldNotFit,

    #[error("not enough input bits to consume")]
    NotEnoughBits,

    #[error("not enough input samples to consume")]
    NotEnoughSamples,

    #[error("invalid PRN number {0}")]
    InvalidPRN(u8),
}
