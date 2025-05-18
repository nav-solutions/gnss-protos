#[derive(Debug)]
pub enum GpsError {
    /// Not a valid GPS preamble
    InvalidPreamble,

    /// Frame Type is either invalid or not supported
    UnknownFrameType,
}
