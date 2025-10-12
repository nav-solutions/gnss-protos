use bitbuffer::BitError;
use thiserror::Error;

use crate::BufferingError;

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

    /// Invalid Word Parity
    #[error("invalid word parity")]
    Parity,

    /// Buffer read issue
    #[error("read issue: not enough bytes?")]
    BufferRead(#[from] BitError),

    /// Buffering Error
    #[error("buffering error: {0}")]
    Buffering(#[from] BufferingError),
}
