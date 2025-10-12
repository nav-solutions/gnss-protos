use crate::{
    gps::{
        GpsBuffer, GpsError, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
        GPS_FRAME_BITS, GPS_FRAME_BYTES,
    },
    Buffering, Message,
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
    type Err = GpsError;
    type B = GpsBuffer;

    fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }

    fn encoding_bitsize(&self) -> usize {
        GPS_FRAME_BITS
    }

    fn encode(&self, buffer: &mut Self::B) -> Result<usize, Self::Err> {
        let mut stream = buffer.bit_write_stream();
        stream.write(&self.telemetry)?;
        stream.write(&self.how)?;

        match self.subframe {
            GpsQzssSubframe::Ephemeris1(eph1) => stream.write(&eph1)?,
        }

        Ok(GPS_FRAME_BYTES)
    }

    #[cfg(test)]
    fn to_slice(&self) -> Vec<u8> {
        let mut buf = GpsBuffer::default();
        self.encode(&mut buf).unwrap();
        buf.to_slice().to_vec()
    }
}

impl GpsQzssFrame {
    /// Generates a realistic frame model for testing purposes.
    #[cfg(test)]
    pub fn model(frame_id: GpsQzssFrameId) -> Self {
        Self::default()
            .with_telemetry(GpsQzssTelemetry::model())
            .with_hand_over_word(GpsQzssHow::model(frame_id))
            .with_subframe(GpsQzssSubframe::model(frame_id))
    }

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
            // GpsQzssSubframe::Ephemeris2(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris2,
            // GpsQzssSubframe::Ephemeris3(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris3,
        }

        self
    }
}

#[cfg(test)]
mod test {}
