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

    /// [crate::gps::GpsQzssMessage] does not start on the first subframe
    /// of page 1 (at a multiple of 12'30" since the beginning of the week)
    #[error("GPS message: does not start with page 1")]
    MessageAlignment,

    /// [crate::gps::GpsQzssMessage]: subframes are not in sequence
    /// (subframe ID or time of week discontinuity) at given subframe index
    #[error("GPS message: discontinuity at subframe #{0}")]
    MessageSequence(usize),

    /// [crate::gps::GpsQzssMessage]: the content of a subframe 4 or 5 page
    /// does not match the SV (page) ID expected on this page
    #[error("GPS message: unexpected content on page #{0} of subframe {1}")]
    UnexpectedPage(u8, u8),

    /// Buffer read issue
    #[error("read issue: not enough bytes?")]
    BufferRead(#[from] BitError),

    /// Buffering Error
    #[error("buffering error: {0}")]
    Buffering(#[from] BufferingError),
}
