mod inav;
// mod fnav;
mod decoder;

pub use inav::*;
// pub use fnav::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GalileoMessage {
    /// Galileo I/NAV frame is 250-bit long
    INAV(GalINAV),
    // /// Galileo F/NAV E5a-I message
    // FNAV(GalFNavFrame),
}
