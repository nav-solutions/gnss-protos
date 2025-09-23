use thiserror::Error;

#[derive(Error, Debug)]
pub enum GpsError {
    /// Not a valid GPS preamble
    #[error("invalid GPS preamble")]
    InvalidPreamble,

    /// Frame Type is either invalid or not supported
    #[error("unknown GPS subframe type")]
    UnknownFrameType,

    /// Internal error: internal FSM reached invalid state,
    /// most likely due to corruption in the handling of successive
    /// data words in the stream. Should never happen.
    #[error("internal FSM error")]
    InternalFSM,

    /// Size is too small to encode a correct data frame
    #[error("buffer to small for this GPS frame")]
    WouldNotFit,
    
    /// Invalid Word Parity 
    #[error("invalid word parity")]
    Parity,
}
