use crate::gps::{GpsQzssFrame, GPS_WORDS_PER_FRAME, GpsError};

/// [GpsQzssFrameRotation] is a structure to help
/// generate and transmit the correct [GpsQzssFrame] rotation.
#[derive(Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrameRotation {
    current: GpsQzssFrameId,
}

impl Iterator for GpsQzssFrameRotation {
    type Item = GpsQzssFrameId;

    /// Returns the next [GpsQzssFrame] to generate according to the
    /// standard frame rotation. The message stops and returns None
    /// once the last Frame 5 has been generated.
    /// To generate the entire period, prefer the [DoubleEndedIterator]
    fn next(&mut self) -> Option<Self::Item> {
        match self.current {
            GpsQzssFrameId::Alamach5 => None,
            current => current.next(),
        }
    }
}

impl DoubleEndedIterator for GpsQzssFrameRotation {
    type Item = GpsQzssFrameId;

    /// A never ending [GpsQzssFrameRotation]. In the real world,
    /// the rotation period should be 30s.
    fn next_back(&mut self) -> Option<Self::Item> {
        self.current.next()
    }
}
