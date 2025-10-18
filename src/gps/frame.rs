use crate::{
    gps::{
        GpsError, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrameId, GpsQzssHow,
        GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
    },
    Buffer, BufferingError, Message,
};

use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadBuffer, BitReadStream, BitWrite, BitWriteStream,
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

impl BitRead<'_, BigEndian> for GpsQzssFrame {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let telemetry = stream.read::<GpsQzssTelemetry>()?;
        let how = stream.read::<GpsQzssHow>()?;

        let subframe = match how.frame_id {
            GpsQzssFrameId::Ephemeris1 => {
                let eph = stream.read::<GpsQzssFrame1>()?;
                GpsQzssSubframe::Ephemeris1(eph)
            },
            GpsQzssFrameId::Ephemeris2 => {
                let eph = stream.read::<GpsQzssFrame2>()?;
                GpsQzssSubframe::Ephemeris2(eph)
            },
            GpsQzssFrameId::Ephemeris3 => {
                let eph = stream.read::<GpsQzssFrame3>()?;
                GpsQzssSubframe::Ephemeris3(eph)
            },
            _ => unimplemented!("almanach"),
        };

        Ok(Self {
            telemetry,
            how,
            subframe,
        })
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write(&self.telemetry)?;
        stream.write(&self.how)?;
        stream.write(&self.subframe)?;
        Ok(())
    }
}

impl Message for GpsQzssFrame {
    type Err = GpsError;

    fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }

    fn encoding_bitsize(&self) -> usize {
        GPS_FRAME_BITS
    }

    fn encode(&self, buffer: &mut [u8]) -> Result<usize, Self::Err> {
        let avail = buffer.len();

        if avail < GPS_FRAME_BYTES {
            return Err(GpsError::Buffering(BufferingError::StorageFull));
        }

        let mut stream = BitWriteStream::from_slice(buffer, BigEndian);
        stream.write(&self.telemetry)?;
        stream.write(&self.how)?;

        match self.subframe {
            GpsQzssSubframe::Ephemeris1(eph) => stream.write(&eph)?,
            GpsQzssSubframe::Ephemeris2(eph) => stream.write(&eph)?,
            GpsQzssSubframe::Ephemeris3(eph) => stream.write(&eph)?,
        }

        Ok(GPS_FRAME_BYTES)
    }

    fn decode(buffer: &[u8]) -> Result<Self, Self::Err> {
        let buffer = BitReadBuffer::new(buffer, BigEndian);
        let mut reader = BitReadStream::new(buffer);
        Ok(reader.read::<Self>()?)
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
            GpsQzssSubframe::Ephemeris2(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris2,
            GpsQzssSubframe::Ephemeris3(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris3,
        }

        self
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{GpsBuffer, GpsQzssFrame},
        Buffering, Message,
    };

    #[test]
    fn default_reciprocal() {
        let default = GpsQzssFrame::default();

        let mut buffer = GpsBuffer::default();
        let mut writer = buffer.bit_write_stream();

        assert!(writer.write(&default).is_ok(), "failed to encode frame");

        let mut reader = buffer.bit_read_stream();

        let decoded = reader.read::<GpsQzssFrame>().unwrap_or_else(|e| {
            panic!("failed to decode frame: {}", e);
        });

        assert_eq!(decoded, default);
    }
}
