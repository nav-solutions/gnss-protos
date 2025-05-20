#[derive(Debug)]
pub enum GpsError {
    /// Not a valid GPS preamble
    InvalidPreamble,

    /// Frame Type is either invalid or not supported
    UnknownFrameType,

    /// Size is too small to encode a correct data frame
    WouldNotFit,
}
