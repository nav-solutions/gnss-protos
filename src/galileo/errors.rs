use bitbuffer::BitError;
use thiserror::Error;

/// Errors that may happen while encoding or decoding Galileo messages.
#[derive(Error, Debug)]
pub enum GalileoError {
    /// No synchronization pattern (normal or inverted) where a page part should start.
    #[error("invalid Galileo synchronization pattern")]
    InvalidSyncPattern,

    /// The page parts are not in the expected order (even part first, then odd part).
    #[error("invalid Galileo even/odd page parts sequence")]
    EvenOddMismatch,

    /// The two page parts do not agree on the page type (nominal / alert).
    #[error("inconsistent Galileo page type")]
    PageTypeMismatch,

    /// Tail bits of a page part are not null.
    #[error("non null Galileo tail bits")]
    TailBits,

    /// CRC-24Q verification failed.
    #[error("Galileo CRC error")]
    Crc,

    /// The word type does not match the structure it is decoded into.
    #[error("unexpected Galileo word type {0}")]
    WordType(u8),

    /// Buffer read / write issue
    #[error("bit stream issue: {0}")]
    BitStream(#[from] BitError),
}
