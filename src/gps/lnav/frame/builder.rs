//! GPS 30" message rotation
use crate::gps::lnav::{
    GpsQzssLnavFrame, GpsQzssLnavSubframe, GpsQzssLnavSubframeId, GPS_LNAV_FRAME_ITEMS,
};

#[cfg(feature = "log")]
use log::error;

/// [GpsQzssLnavFrameBuilder] may be used to collect (record)
/// [GpsQzssLnavFrame]s, which are composed of [GPS_LNAV_FRAME_ITEMS] subframes.
///
/// The frame builder will align itself to the first EPH-1 received
/// and will return once a valid 30s burst is received.
/// Which is convenient for real-world receivers.
///
/// It supports discontinuities in the data stream: when locked
/// once EPH-1 has been received, if the sequence is broken, it will
/// reset to EPH-1 locking. Which makes it tolerant
/// to discontinuity in the bitstreams, for real-world receivers.
#[derive(Default, Copy, Clone, PartialEq)]
pub struct GpsQzssLnavFrameBuilder {
    /// last frame_id received
    frame_id: Option<GpsQzssLnavSubframeId>,

    /// Internal buffer
    buffer: [GpsQzssLnavSubframe; GPS_LNAV_FRAME_ITEMS],
}

impl GpsQzssLnavFrameBuilder {
    /// Reset mutable self. You may use this method in a real-world continuous application,
    ///
    /// Note that you don't have to reset upon [GpsQzssLnavFrame] collection: the object does it for you
    /// at return time.
    pub fn reset(&mut self) {
        self.frame_id = None;
    }

    /// Push a new [GpsQzssLnavSubframe] to the [GpsQzssLnavFrameBuilder].
    /// If provided subframe breaks the 30s cycle, the builder will reset
    /// and wait for a new EPH-1 (or this one, if that's an EPH-1).
    /// If provided subframe terminates a 30s cycle, we return an [GpsQzssLnavFrame].
    ///
    /// Currently this API verifies the frame class progression,
    /// it does not verify the temporal progression.
    ///
    /// Any time a 30s is concluded, the builder automatically resets itself
    /// and you can continue providing subframes, to collect a new 30s frame.
    pub fn push_sbf(&mut self, subframe: GpsQzssLnavSubframe) -> Option<GpsQzssLnavFrame> {
        let new_id = subframe.how.sbf_id;

        match self.frame_id {
            None => {
                if new_id == GpsQzssLnavSubframeId::Ephemeris1 {
                    self.frame_id = Some(new_id);
                    self.buffer[0] = subframe;
                }
                None
            },
            Some(frame_id) => match frame_id {
                GpsQzssLnavSubframeId::Ephemeris1 => {
                    if new_id == GpsQzssLnavSubframeId::Ephemeris2 {
                        self.frame_id = Some(new_id);
                        self.buffer[1] = subframe;
                    } else {
                        self.reset();
                    }
                    None
                },
                GpsQzssLnavSubframeId::Ephemeris2 => {
                    if new_id == GpsQzssLnavSubframeId::Ephemeris3 {
                        self.frame_id = Some(new_id);
                        self.buffer[2] = subframe;
                    } else {
                        self.reset();
                    }
                    None
                },
                GpsQzssLnavSubframeId::Ephemeris3 => {
                    if new_id == GpsQzssLnavSubframeId::Almanach4 {
                        self.frame_id = Some(new_id);
                        self.buffer[3] = subframe;
                    } else {
                        self.reset();
                    }
                    None
                },
                GpsQzssLnavSubframeId::Almanach4 => {
                    if new_id == GpsQzssLnavSubframeId::Almanach5 {
                        self.frame_id = Some(new_id);
                        self.buffer[4] = subframe;
                        self.reset();
                        match GpsQzssLnavFrame::from_slice(self.buffer.clone()) {
                            Ok(frame) => Some(frame),
                            Err(_e) => {
                                #[cfg(feature = "log")]
                                {
                                    error!("{} - resetting subframe collection", _e);
                                }
                                None
                            },
                        }
                    } else {
                        // reset
                        self.frame_id = None;
                        None
                    }
                },
                _ => unreachable!("internal errror"),
            },
        }
    }
}

#[cfg(test)]
mod test {

    use crate::{
        gps::lnav::{
            subframe::GpsQzssLnavSubfDecoder,
            test_vectors::{on_air, UBLOX_G25_STREAM},
            GpsQzssLnavFrame, GpsQzssLnavFrameBuilder, GpsQzssLnavHow, GpsQzssLnavSubframe,
            GpsQzssLnavSubframeId, GPS_LNAV_FRAME_ITEMS, GPS_LNAV_WORD_BITS,
        },
        tests::init_logger,
        BigEndian, BitWriteStream,
    };

    // fn id_to_usize(id: GpsQzssLnavSubframeId) -> usize {
    //     match id {
    //         GpsQzssLnavSubframeId::Ephemeris1 => 0,
    //         GpsQzssLnavSubframeId::Ephemeris2 => 1,
    //         GpsQzssLnavSubframeId::Ephemeris3 => 2,
    //         GpsQzssLnavSubframeId::Almanach4 => 3,
    //         GpsQzssLnavSubframeId::Almanach5 => 4,
    //     }
    // }

    fn usize_to_id(val: usize) -> GpsQzssLnavSubframeId {
        match val {
            1 => GpsQzssLnavSubframeId::Ephemeris2,
            2 => GpsQzssLnavSubframeId::Ephemeris3,
            3 => GpsQzssLnavSubframeId::Almanach4,
            4 => GpsQzssLnavSubframeId::Almanach5,
            _ => GpsQzssLnavSubframeId::Ephemeris1,
        }
    }

    #[test]
    fn test_natural_progression() {
        let mut builder = GpsQzssLnavFrameBuilder::default();

        for _ in 0..1 {
            for i in 0..=4 {
                let sbf = GpsQzssLnavSubframe {
                    how: GpsQzssLnavHow::default().with_subframe_id(usize_to_id(i)),
                    telemetry: Default::default(),
                    subframe: Default::default(),
                };

                let ret = builder.push_sbf(sbf);

                if i < 4 {
                    assert!(ret.is_none(), "builder has completed too early");
                } else {
                    let frame = ret.unwrap();

                    let expected = GpsQzssLnavFrame::from_slice([
                        GpsQzssLnavSubframe {
                            how: GpsQzssLnavHow::default()
                                .with_subframe_id(GpsQzssLnavSubframeId::Ephemeris1),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssLnavSubframe {
                            how: GpsQzssLnavHow::default()
                                .with_subframe_id(GpsQzssLnavSubframeId::Ephemeris2),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssLnavSubframe {
                            how: GpsQzssLnavHow::default()
                                .with_subframe_id(GpsQzssLnavSubframeId::Ephemeris3),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssLnavSubframe {
                            how: GpsQzssLnavHow::default()
                                .with_subframe_id(GpsQzssLnavSubframeId::Almanach4),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssLnavSubframe {
                            how: GpsQzssLnavHow::default()
                                .with_subframe_id(GpsQzssLnavSubframeId::Almanach5),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                    ])
                    .unwrap();

                    assert_eq!(frame, expected);
                    builder.reset();
                }
            }
        }
    }

    #[test]
    fn test_initial_offset() {
        let mut builder = GpsQzssLnavFrameBuilder::default();

        for _ in 0..1 {
            for offset in 1..=4 {
                for i in 0..=4 {
                    let frame_id = usize_to_id((i + offset) % 4);
                    let sbf = GpsQzssLnavSubframe {
                        how: GpsQzssLnavHow::default().with_subframe_id(frame_id),
                        telemetry: Default::default(),
                        subframe: Default::default(),
                    };

                    let ret = builder.push_sbf(sbf);

                    if frame_id != GpsQzssLnavSubframeId::Almanach5 {
                        assert!(ret.is_none(), "builder has completed too early");
                    } else {
                        let frame = ret.unwrap();

                        let expected = GpsQzssLnavFrame::from_slice([
                            GpsQzssLnavSubframe {
                                how: GpsQzssLnavHow::default()
                                    .with_subframe_id(GpsQzssLnavSubframeId::Ephemeris1),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssLnavSubframe {
                                how: GpsQzssLnavHow::default()
                                    .with_subframe_id(GpsQzssLnavSubframeId::Ephemeris2),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssLnavSubframe {
                                how: GpsQzssLnavHow::default()
                                    .with_subframe_id(GpsQzssLnavSubframeId::Ephemeris3),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssLnavSubframe {
                                how: GpsQzssLnavHow::default()
                                    .with_subframe_id(GpsQzssLnavSubframeId::Almanach4),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssLnavSubframe {
                                how: GpsQzssLnavHow::default()
                                    .with_subframe_id(GpsQzssLnavSubframeId::Almanach5),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                        ])
                        .unwrap();

                        assert_eq!(frame, expected);
                        builder.reset();
                    }
                }
            }
        }
    }

    #[test]
    fn test_g25_ubx() {
        init_logger();

        // the transmitted bit stream
        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            writer.write_int(0u8, 5).unwrap();

            for words in UBLOX_G25_STREAM.iter() {
                for word in on_air(*words).iter() {
                    writer.write_int(*word, GPS_LNAV_WORD_BITS).unwrap();
                }
            }

            writer.write_int(0u8, 8).unwrap();
        }

        let mut position = 0;
        let mut decoder = GpsQzssLnavSubfDecoder::default();

        let mut builder = GpsQzssLnavFrameBuilder::default();

        // consume whole stream:
        // verifies frame builder is imune to out-of-sync start
        //
        // EPH-1 has a 4 page offset in this stream
        let mut count = 0;

        loop {
            let (consumed, sbf) = decoder.decode(&bytes, position);
            position += consumed;

            match sbf {
                Some(sbf) => {
                    if builder.push_sbf(sbf).is_some() {
                        count += 1;
                        builder.reset();
                    }
                },
                _ => break,
            }
        }

        assert_eq!(count, (UBLOX_G25_STREAM.len() - 4) / GPS_LNAV_FRAME_ITEMS);
    }
}
