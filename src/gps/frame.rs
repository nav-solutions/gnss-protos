use crate::{
    gps::{
        GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS,
        GPS_FRAME_BYTES,
    },
    BufferingError, Message,
};

/// GPS / QZSS interpreted frame.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame {
    /// [GpsQzssTelemetry] describes the following frame and contains
    /// the sync-byter, therefore initiates a [GpsQzssFrame].
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,

    /// [GpsQzssSubframe] depends on associated [GpsQzssHow].
    pub subframe: GpsQzssSubframe,
}

impl Message for GpsQzssFrame {
    #[cfg(test)]
    fn model(frame_id: GpsQzssFrameId) -> Self {
        Self::default()
            .with_telemetry(GpsQzssTelemetry::model())
            .with_hand_over_word(GpsQzssHow::model(frame_id))
            .with_subframe(GpsQzssSubframe::model(frame_id))
    }

    fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GPS_FRAME_BITS
    }

    fn encode(&self, dest: &mut [u8]) -> Result<usize, BufferingError> {
        let avail = dest.len();
        let needed_size = self.encoding_size();

        if avail < needed_size {
            return Err(BufferingError::StorageFull);
        }

        let subf = self.subframe.to_words();

        let words = [
            self.telemetry.to_word(),
            self.how.to_word(),
            subf[0],
            subf[1],
            subf[2],
            subf[3],
            subf[4],
            subf[5],
            subf[6],
            subf[7],
        ];

        Ok(needed_size)
    }
}

impl GpsQzssFrame {
    /// Copies and returns with updated [GpsQzssHow].
    pub fn with_hand_over_word(mut self, how: GpsQzssHow) -> Self {
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
            GpsQzssSubframe::Ephemeris2(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris2,
            GpsQzssSubframe::Ephemeris3(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris3,
        }

        self
    }
}
