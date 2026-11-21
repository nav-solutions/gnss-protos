//! GPS 30" message rotation
use crate::gps::{GpsQzssFrame, GpsQzssSubframe, GpsQzssSubframeId, GPS_FRAME_ITEMS};

#[cfg(feature = "log")]
use log::error;

/// [GpsQzssFrameBuilder] may be used to collect (record)
/// [GpsQzssFrame]s, which are composed of [GPS_FRAME_ITEMS] subframes.
///
/// The frame builder will align itself to the first EPH-1 received
/// and will return once a valid 30s burst is received.
/// Which is convenient for real-world receivers.
///
/// It supports discontinuities in the data stream: when locked
/// once EPH-1 has been received, if the sequence is broken, it will
/// reset to EPH-1 locking. Which makes it tolerant
/// to discontinuity in the bitstreams, for real-world receivers.
#[derive(Default, PartialEq)]
pub struct GpsQzssFrameBuilder {
    /// last frame_id received
    frame_id: Option<GpsQzssSubframeId>,

    /// Internal buffer
    buffer: [GpsQzssSubframe; GPS_FRAME_ITEMS],
}

impl GpsQzssFrameBuilder {
    /// Reset mutable self. You may use this method in a real-world continuous application,
    /// once a new [GpsQzssFrame] has been correctly received, and you want to move-on to the
    /// next frame without new memory allocation.
    pub fn reset(&mut self) {
        self.frame_id = None;
    }

    /// Push a new [GpsQzssSubframe] to the [GpsQzssFrameBuilder].
    /// If this frame completes a 30s correctly
    ///
    /// Remember that this API only verifies the frame class progression,
    /// not the temporal progression.
    ///
    /// If this subframe completes a 30s cycle, you may [Self::reset] and move-on
    /// to the following frames without new memory allocation.
    pub fn push(&mut self, subframe: GpsQzssSubframe) -> Option<GpsQzssFrame> {
        let new_id = subframe.how.frame_id;

        match self.frame_id {
            None => {
                if new_id == GpsQzssSubframeId::Ephemeris1 {
                    self.frame_id = Some(new_id);
                    self.buffer[0] = subframe;
                }
                None
            },
            Some(frame_id) => match frame_id {
                GpsQzssSubframeId::Ephemeris1 => {
                    if new_id == GpsQzssSubframeId::Ephemeris2 {
                        self.frame_id = Some(new_id);
                        self.buffer[1] = subframe;
                    } else {
                        // reset
                        self.frame_id = None;
                    }
                    None
                },
                GpsQzssSubframeId::Ephemeris2 => {
                    if new_id == GpsQzssSubframeId::Ephemeris3 {
                        self.frame_id = Some(new_id);
                        self.buffer[2] = subframe;
                    } else {
                        // reset
                        self.frame_id = None;
                    }
                    None
                },
                GpsQzssSubframeId::Ephemeris3 => {
                    if new_id == GpsQzssSubframeId::Almanach4 {
                        self.frame_id = Some(new_id);
                        self.buffer[3] = subframe;
                    } else {
                        // reset
                        self.frame_id = None;
                    }
                    None
                },
                GpsQzssSubframeId::Almanach4 => {
                    if new_id == GpsQzssSubframeId::Almanach5 {
                        self.frame_id = Some(new_id);
                        self.buffer[4] = subframe;
                        match GpsQzssFrame::from_slice(self.buffer.clone()) {
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
        gps::{
            test_vectors::{on_air, UBLOX_G25_STREAM},
            GpsQzssDecoder, GpsQzssFrame, GpsQzssFrameBuilder, GpsQzssHow, GpsQzssSubframe,
            GpsQzssSubframeId, GPS_FRAME_ITEMS, GPS_WORD_BITS,
        },
        tests::init_logger,
        BigEndian, BitWriteStream,
    };

    // fn id_to_usize(id: GpsQzssSubframeId) -> usize {
    //     match id {
    //         GpsQzssSubframeId::Ephemeris1 => 0,
    //         GpsQzssSubframeId::Ephemeris2 => 1,
    //         GpsQzssSubframeId::Ephemeris3 => 2,
    //         GpsQzssSubframeId::Almanach4 => 3,
    //         GpsQzssSubframeId::Almanach5 => 4,
    //     }
    // }

    fn usize_to_id(val: usize) -> GpsQzssSubframeId {
        match val {
            1 => GpsQzssSubframeId::Ephemeris2,
            2 => GpsQzssSubframeId::Ephemeris3,
            3 => GpsQzssSubframeId::Almanach4,
            4 => GpsQzssSubframeId::Almanach5,
            _ => GpsQzssSubframeId::Ephemeris1,
        }
    }

    #[test]
    fn test_natural_progression() {
        let mut builder = GpsQzssFrameBuilder::default();

        for _ in 0..1 {
            for i in 0..=4 {
                let sbf = GpsQzssSubframe {
                    how: GpsQzssHow::default().with_subframe_id(usize_to_id(i)),
                    telemetry: Default::default(),
                    subframe: Default::default(),
                };

                let ret = builder.push(sbf);

                if i < 4 {
                    assert!(ret.is_none(), "builder has completed too early");
                } else {
                    let frame = ret.unwrap();

                    let expected = GpsQzssFrame::from_slice([
                        GpsQzssSubframe {
                            how: GpsQzssHow::default()
                                .with_subframe_id(GpsQzssSubframeId::Ephemeris1),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssSubframe {
                            how: GpsQzssHow::default()
                                .with_subframe_id(GpsQzssSubframeId::Ephemeris2),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssSubframe {
                            how: GpsQzssHow::default()
                                .with_subframe_id(GpsQzssSubframeId::Ephemeris3),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssSubframe {
                            how: GpsQzssHow::default()
                                .with_subframe_id(GpsQzssSubframeId::Almanach4),
                            telemetry: Default::default(),
                            subframe: Default::default(),
                        },
                        GpsQzssSubframe {
                            how: GpsQzssHow::default()
                                .with_subframe_id(GpsQzssSubframeId::Almanach5),
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
        let mut builder = GpsQzssFrameBuilder::default();

        for _ in 0..1 {
            for offset in 1..=4 {
                for i in 0..=4 {
                    let frame_id = usize_to_id((i + offset) % 4);
                    let sbf = GpsQzssSubframe {
                        how: GpsQzssHow::default().with_subframe_id(frame_id),
                        telemetry: Default::default(),
                        subframe: Default::default(),
                    };

                    let ret = builder.push(sbf);

                    if frame_id != GpsQzssSubframeId::Almanach5 {
                        assert!(ret.is_none(), "builder has completed too early");
                    } else {
                        let frame = ret.unwrap();

                        let expected = GpsQzssFrame::from_slice([
                            GpsQzssSubframe {
                                how: GpsQzssHow::default()
                                    .with_subframe_id(GpsQzssSubframeId::Ephemeris1),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssSubframe {
                                how: GpsQzssHow::default()
                                    .with_subframe_id(GpsQzssSubframeId::Ephemeris2),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssSubframe {
                                how: GpsQzssHow::default()
                                    .with_subframe_id(GpsQzssSubframeId::Ephemeris3),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssSubframe {
                                how: GpsQzssHow::default()
                                    .with_subframe_id(GpsQzssSubframeId::Almanach4),
                                telemetry: Default::default(),
                                subframe: Default::default(),
                            },
                            GpsQzssSubframe {
                                how: GpsQzssHow::default()
                                    .with_subframe_id(GpsQzssSubframeId::Almanach5),
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
                    writer.write_int(*word, GPS_WORD_BITS).unwrap();
                }
            }

            writer.write_int(0u8, 8).unwrap();
        }

        let mut position = 0;
        let mut decoder = GpsQzssDecoder::default();

        let mut builder = GpsQzssFrameBuilder::default();

        // consume whole stream:
        // verifies frame builder is imune to out-of-sync start
        //
        // EPH-1 has a 4 page offset in this stream
        let mut count = 0;

        loop {
            let (consumed, frame) = decoder.decode(&bytes, position);
            position += consumed;

            match frame {
                Some(frame) => {
                    if builder.push(frame).is_some() {
                        count += 1;
                        builder.reset();
                    }
                },
                _ => break,
            }
        }

        assert_eq!(count, (UBLOX_G25_STREAM.len() - 4) / GPS_FRAME_ITEMS);
    }
}
