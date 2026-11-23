//! GPS 30" message rotation
use crate::{
    gps::lnav::{
        GpsError, GpsQzssLnavSubframe, GpsQzssLnavSubframeId, GPS_LNAV_FRAME_BITS,
        GPS_LNAV_FRAME_BYTES, GPS_LNAV_FRAME_ITEMS,
    },
    BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream, Message,
};

mod builder;

pub use builder::GpsQzssLnavFrameBuilder;

/// [GpsQzssLnavFrame] represents a 30s rotation broadcasted by every satellite
/// which consists in a rotation of EPH-1, 2, 3 then ALM4 and ALM5 subframe sequence.
///
/// This object is dedicated to people interested in satellite message rotations
/// and working at the scale to 30s (not the almanac/super frame scale).
/// It conveniently supports both encoding & decoding once again.
///
/// A valid 30s rotation always starts with an EPH-1 subframe and always
/// concludes with an ALM-5 subframe. Currently, this structure
/// verifies the correctness of each frame class, but does not verify
/// correct temporal progression between items. Therefore,
/// it is possible to deal with partly incorrect 30s bursts here,
#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct GpsQzssLnavFrame([GpsQzssLnavSubframe; GPS_LNAV_FRAME_ITEMS]);

impl Message<BigEndian> for GpsQzssLnavFrame {
    fn encoding_size(&self) -> usize {
        GPS_LNAV_FRAME_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GPS_LNAV_FRAME_BITS
    }
}

impl GpsQzssLnavFrame {
    /// Returns the first subframe of this 30s burst, representing
    /// the first 6s of the burst, which is an EPH-1 [GpsQzssLnavSubframe].
    pub fn eph_1(&self) -> &GpsQzssLnavSubframe {
        &self.0[0]
    }

    /// Returns the second subframe of this 30s burst,
    /// as [GpsQzssLnavFrame] following the EPH-1 subframe.
    pub fn eph_2(&self) -> &GpsQzssLnavSubframe {
        &self.0[1]
    }

    /// Returns the third subframe of this 30s burst,
    /// as [GpsQzssLnavFrame] following the EPH-2 subframe
    /// and conluding the EPH subframes (first 3/5 18s of the burst).
    pub fn eph_3(&self) -> &GpsQzssLnavSubframe {
        &self.0[2]
    }

    /// Returns the fourth subframe of this 30s burst,
    /// as [GpsQzssLnavFrame] following the EPH-3 subframe
    /// and marking the first almanach subframe.
    pub fn alm_4(&self) -> &GpsQzssLnavSubframe {
        &self.0[3]
    }

    /// Returns last subframe of this 30s burst,
    /// as [GpsQzssLnavFrame] following ALM-4 subframe
    /// and concluding this 30s cycle.
    pub fn alm_5(&self) -> &GpsQzssLnavSubframe {
        &self.0[4]
    }

    /// Builds a [GpsQzssLnavFrame] correctly, if the burst sequence is correctly
    /// respected, from a slice of [GpsQzssLnavSubframe]s.
    ///
    /// Currently, this method does not check the actual timeline progression,
    /// only frame class progression.
    pub fn from_slice(
        slice: [GpsQzssLnavSubframe; GPS_LNAV_FRAME_ITEMS],
    ) -> Result<Self, GpsError> {
        if slice[0].how.sbf_id != GpsQzssLnavSubframeId::Ephemeris1 {
            return Err(GpsError::InvalidSubframesRotation);
        }
        if slice[1].how.sbf_id != GpsQzssLnavSubframeId::Ephemeris2 {
            return Err(GpsError::InvalidSubframesRotation);
        }
        if slice[2].how.sbf_id != GpsQzssLnavSubframeId::Ephemeris3 {
            return Err(GpsError::InvalidSubframesRotation);
        }
        if slice[3].how.sbf_id != GpsQzssLnavSubframeId::Almanach4 {
            return Err(GpsError::InvalidSubframesRotation);
        }
        if slice[4].how.sbf_id != GpsQzssLnavSubframeId::Almanach5 {
            return Err(GpsError::InvalidSubframesRotation);
        }
        Ok(Self(slice))
    }

    /// Builds a [GpsQzssLnavFrame] from individual parts, which must be correctly defined.
    /// Currently, this method does not check the actual timeline progression,
    /// only frame class progression.
    pub fn from_parts(
        eph_1: GpsQzssLnavSubframe,
        eph_2: GpsQzssLnavSubframe,
        eph_3: GpsQzssLnavSubframe,
        alm_4: GpsQzssLnavSubframe,
        alm_5: GpsQzssLnavSubframe,
    ) -> Result<Self, GpsError> {
        Self::from_slice([eph_1, eph_2, eph_3, alm_4, alm_5])
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavFrame {
    /// Encodes this [GpsQzssMessage] (completely: 125 contiguous subframes) into any [BitWrite] buffer.
    /// [GpsQzssMessage::encoding_size] must fit completely in provided buffer
    /// for this method to work.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        for i in 0..GPS_LNAV_FRAME_ITEMS {
            stream.write(&self.0[i])?;
        }
        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavFrame {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        // parse each parts
        let eph_1 = GpsQzssLnavSubframe::read(stream)?;
        let eph_2 = GpsQzssLnavSubframe::read(stream)?;
        let eph_3 = GpsQzssLnavSubframe::read(stream)?;
        let alm_4 = GpsQzssLnavSubframe::read(stream)?;
        let alm_5 = GpsQzssLnavSubframe::read(stream)?;

        // attempt to build: sequence must be correct
        let parsed = Self::from_slice([eph_1, eph_2, eph_3, alm_4, alm_5]).map_err(|_| {
            BitError::UnmatchedDiscriminant {
                discriminant: 0,
                enum_name: "invalid GPS 30s rotation".to_string(),
            }
        })?;

        Ok(parsed)
    }
}

#[cfg(test)]
mod test {

    use crate::gps::lnav::{
        GpsQzssLnavFrame, GpsQzssLnavHow, GpsQzssLnavSubframe, GpsQzssLnavSubframeEnum,
        GpsQzssLnavSubframeId,
    };

    // TODO test all SF4 and SF5 variants

    #[test]
    fn from_default_slice() {
        let eph_1 = GpsQzssLnavSubframe {
            how: GpsQzssLnavHow::default().with_subframe_id(GpsQzssLnavSubframeId::Ephemeris1),
            telemetry: Default::default(),
            subframe: GpsQzssLnavSubframeEnum::Ephemeris1(Default::default()),
        };
        let eph_2 = GpsQzssLnavSubframe {
            how: GpsQzssLnavHow::default().with_subframe_id(GpsQzssLnavSubframeId::Ephemeris2),
            telemetry: Default::default(),
            subframe: GpsQzssLnavSubframeEnum::Ephemeris2(Default::default()),
        };
        let eph_3 = GpsQzssLnavSubframe {
            how: GpsQzssLnavHow::default().with_subframe_id(GpsQzssLnavSubframeId::Ephemeris3),
            telemetry: Default::default(),
            subframe: GpsQzssLnavSubframeEnum::Ephemeris3(Default::default()),
        };
        let alm_4 = GpsQzssLnavSubframe {
            how: GpsQzssLnavHow::default().with_subframe_id(GpsQzssLnavSubframeId::Almanach4),
            telemetry: Default::default(),
            subframe: GpsQzssLnavSubframeEnum::Almanach4(Default::default()),
        };
        let alm_5 = GpsQzssLnavSubframe {
            how: GpsQzssLnavHow::default().with_subframe_id(GpsQzssLnavSubframeId::Almanach5),
            telemetry: Default::default(),
            subframe: GpsQzssLnavSubframeEnum::Almanach5(Default::default()),
        };

        let frame = GpsQzssLnavFrame::from_slice([eph_1, eph_2, eph_3, alm_4, alm_5]).unwrap();

        assert_eq!(*frame.eph_1(), eph_1);
        assert_eq!(*frame.eph_2(), eph_2);
        assert_eq!(*frame.eph_3(), eph_3);
        assert_eq!(*frame.alm_4(), alm_4);
        assert_eq!(*frame.alm_5(), alm_5);
    }
}
