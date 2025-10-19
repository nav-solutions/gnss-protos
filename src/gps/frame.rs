use crate::{
    gps::{
        GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
        GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
    },
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

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

impl Message<BigEndian> for GpsQzssFrame {
    fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }
    fn encoding_bits(&self) -> usize {
        GPS_FRAME_BITS
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let telemetry = stream.read::<GpsQzssTelemetry>()?;
        let how = stream.read::<GpsQzssHow>()?;

        panic!("HOW: {:?}", how);

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
            GpsQzssFrameId::Almanach5 => {
                let alm = stream.read::<GpsQzssFrame5>()?;
                GpsQzssSubframe::Almaach5(alm)
            },
            _ => unimplemented!("almanach-4"),
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

impl GpsQzssFrame {
    #[cfg(test)]
    fn model(frame_id: GpsQzssFrameId) -> Self {
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
            GpsQzssSubframe::Almanach5(_) => self.how.frame_id = GpsQzssFrameId::Almanach5,
        }

        self
    }
}

#[cfg(test)]
mod test {
    use crate::{gps::GpsQzssFrame, Buffer, Message, StaticBuffer};

    use bitbuffer::{BigEndian, BitReadStream};

    #[test]
    fn default_reciprocal() {
        let default = GpsQzssFrame::default();

        let mut buffer = [0; 1024];
        assert!(
            default.encode(&mut buffer).is_ok(),
            "failed to encode frame"
        );

        let decoded = GpsQzssFrame::decode(&buffer).unwrap_or_else(|e| {
            panic!("failed to decode frame: {}", e);
        });

        assert_eq!(decoded, default);
    }
}
