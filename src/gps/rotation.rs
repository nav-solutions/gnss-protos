use crate::gps::{GpsError, GpsQzssFrame, GpsQzssFrameId, GPS_WORDS_PER_FRAME};

/// [GpsQzssFrameRotation] is a tiny structure that helps you
/// generate the correct message sequence in a transmitter.
///
/// ```
/// use gnss_protos::{GpsQzssFrameRotation, GpsQzssFrameId};
///
/// let mut finite_helper = GpsQzssFrameRotation::default().into_iter();
///
/// assert_eq!(finite_helper.next(), Some(GpsQzssFrameId::Ephemeris2)); // first is EPH-1
/// assert_eq!(finite_helper.next(), Some(GpsQzssFrameId::Ephemeris3));
/// // assert_eq!(finite_helper.next(), Some(GpsQzssFrameId::Almanach4));
/// assert_eq!(finite_helper.next(), Some(GpsQzssFrameId::Almanach5));
/// assert_eq!(finite_helper.next(), None); // end of sequence
///
/// let mut periodic_helper = GpsQzssFrameRotation::default().into_iter();
///
/// assert_eq!(periodic_helper.next_back(), Some(GpsQzssFrameId::Ephemeris2)); // first is EPH-1
/// assert_eq!(periodic_helper.next_back(), Some(GpsQzssFrameId::Ephemeris3));
/// // assert_eq!(periodic_helper.next_back(), Some(GpsQzssFrameId::Almanach4));
/// assert_eq!(periodic_helper.next_back(), Some(GpsQzssFrameId::Almanach5));
/// assert_eq!(periodic_helper.next_back(), Some(GpsQzssFrameId::Ephemeris1)); // should repeat every 30s
/// assert_eq!(periodic_helper.next_back(), Some(GpsQzssFrameId::Ephemeris2));
/// ```
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
        let ret = match self.current {
            GpsQzssFrameId::Almanach5 => None,
            current => Some(current.next()),
        };

        if let Some(ret) = ret {
            self.current = ret;
        }

        ret
    }
}

impl DoubleEndedIterator for GpsQzssFrameRotation {
    /// A never ending [GpsQzssFrameRotation]. In the real world,
    /// the rotation period should be 30s.
    fn next_back(&mut self) -> Option<GpsQzssFrameId> {
        let next = self.current.next();
        self.current = next;
        Some(next)
    }
}

#[cfg(test)]
mod test {
    use crate::gps::{GpsQzssFrameId, GpsQzssFrameRotation};

    #[test]
    fn message_rotation() {
        let mut rot = GpsQzssFrameRotation::default().into_iter();

        assert_eq!(rot.next(), Some(GpsQzssFrameId::Ephemeris2));
        assert_eq!(rot.next(), Some(GpsQzssFrameId::Ephemeris3));
        // assert_eq!(rot.next(), Some(GpsQzssFrameId::Almanach4));
        assert_eq!(rot.next(), Some(GpsQzssFrameId::Almanach5));
        assert_eq!(rot.next(), None);
    }
}
