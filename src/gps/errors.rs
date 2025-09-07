use thiserror::Error;

#[derive(Error, Debug)]
pub enum GpsError {
    /// Not a valid GPS preamble
    #[error("invalid GPS preamble")]
    InvalidPreamble,

    /// Frame Type is either invalid or not supported
    #[error("unknown GPS subframe type")]
    UnknownFrameType,

    /// Size is too small to encode a correct data frame
    #[error("buffer to small for this GPS frame")]
    WouldNotFit,
}
