use crate::gps::{GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry};

/// GPS / QZSS interpreted frame.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame {
    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,

    /// [GpsQzssTelemetry] describes following frame.
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssSubframe] depends on associated How.
    pub subframe: GpsQzssSubframe,
}

impl GpsQzssFrame {
    /// Copies and returns with updated [GpsQzssHow] data word
    pub fn with_how_word(mut self, how: GpsQzssHow) -> Self {
        self.how = how;
        self
    }

    /// Copies and returns with updated [GpsQzssTelemetry] data word
    pub fn with_telemetry(mut self, telemetry: GpsQzssTelemetry) -> Self {
        self.telemetry = telemetry;
        self
    }

    /// Copies and returns an updated [GpsQzssSubframe]
    pub fn with_subframe(mut self, subframe: GpsQzssSubframe) -> Self {
        self.subframe = subframe;

        match subframe {
            GpsQzssSubframe::Ephemeris1(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris1,
            // GpsQzssSubframe::Ephemeris2(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris2,
            // GpsQzssSubframe::Ephemeris3(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris3,
        }

        self
    }
}
