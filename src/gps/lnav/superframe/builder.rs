//! GPS 12'30" message (25 pages of 5 subframes)

use crate::gps::lnav::{
    GpsQzssLnavFrame, GpsQzssLnavSubframe, GpsQzssLnavSubframeId, GpsQzssLnavSuperframe,
    GPS_LNAV_FRAME_ITEMS, GPS_LNAV_SUBFRAME_DURATION_S, GPS_LNAV_SUPERFRAME_DURATION_S,
    GPS_LNAV_SUPERFRAME_ITEMS,
};

// #[cfg(feature = "log")]
// use log::error;

const TOTAL_SUBFRAMES: usize = GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS;

/// [GpsQzssLnavSuperframeBuilder] collects a [GpsQzssFrame]s (30s cycles) consecutively,
/// and aligns itself to the Page-1 in the frame pagination.
/// Therefore, the resulting super frame is correctly aligned and starts
/// with Page-1 EPH-1.
///
/// ```
/// use gnss_protos::gps::{
///     GpsQzssSubframe,
///     GpsQzssHow,
///     GpsQzssLnavSuperframeBuilder,
/// };
///
/// // reads one of our real-world data stream
/// // each file contains several 12'30s
/// // this example demonstrates GpsQzssLnavSuperframeBuilder did
/// // synchronize correctly to EPH-1 page-1
///
/// todo
///
/// let mut builder = GpsQzssMessageBuilder::default();
///
/// // EPH-1 at the beginning of the week: page 1
/// let frame = GpsQzssFrame::default()
///     .with_hand_over_word(GpsQzssHow::default().with_tow_seconds(6));
///
/// assert!(frame.starts_rotation());
/// assert!(builder.push(frame).is_none());
/// assert_eq!(builder.len(), 1);
/// ```
#[derive(Clone, Copy, PartialEq)]
pub struct GpsQzssLnavSuperframeBuilder {
    // number of frames currently collected
    sbf: usize,

    // last SBF kind collected
    last_id: Option<GpsQzssLnavSubframeId>,

    // subframe buffer
    buffer: [GpsQzssLnavSubframe; TOTAL_SUBFRAMES],
}

impl Default for GpsQzssLnavSuperframeBuilder {
    fn default() -> Self {
        Self {
            sbf: 0,
            last_id: None,
            buffer: [GpsQzssLnavSubframe::default(); TOTAL_SUBFRAMES],
        }
    }
}

impl GpsQzssLnavSuperframeBuilder {
    // Use this method to collected several [GpsQzssLnavSuperframe]s in a row,
    // using the same builder without reallocation.
    pub fn reset(&mut self) {
        self.sbf = 0;
        self.last_id = None;
    }

    // /// Returns true if `next` immediately follows `previous` in the GPS/QZSS stream.
    // fn follows(previous: &GpsQzssFrame, next: &GpsQzssFrame) -> bool {

    //     let expected_id = self.last_id.encode() % GPS_FRAME_ITEMS as u8 + 1;

    //     next.subframe.frame_id().encode() == expected_id
    //         && next.transmission_time_s()
    //             == (previous.transmission_time_s() + GPS_SUBFRAME_DURATION_S) % SECONDS_PER_WEEK
    // }

    /// Returns the total of subframes (6s messages) collected so far.
    pub fn total_subframes(&self) -> usize {
        self.sbf
    }

    /// Returns the total of frames (30s messages) collected so far.
    pub fn total_frames(&self) -> usize {
        self.sbf / GPS_LNAV_FRAME_ITEMS
    }

    /// Returns true if no subframe has been collected yet.
    pub fn is_empty(&self) -> bool {
        self.sbf == 0
    }

    /// Returns total number of seconds remaining before the super frame completes
    /// (recording of ALM-5 page 25).
    pub fn seconds_to_end_of_period(&self) -> Option<u32> {
        if self.sbf == 0 {
            None
        } else {
            Some(GPS_LNAV_SUPERFRAME_DURATION_S - self.sbf as u32 * GPS_LNAV_SUBFRAME_DURATION_S)
        }
    }

    /// Use this method when working at the [GpsQzssSubframe] (GPS/QZSS LNAV 6s core element).
    /// Method returns [GpsQzssLnavSuperframe] if the builder has been locked for 124 subframes
    /// and this last subframe completes a 12'30" period.
    ///
    /// This is most convenient when working with our [GpsQzssDecoder] and your application
    /// intends to collect 12'30s periods.
    pub fn push_sbf(&mut self, sbf: GpsQzssLnavSubframe) -> Option<GpsQzssLnavSuperframe> {
        if self.sbf == 0 {
            // waits for EPH-1 Page-1
            if sbf.starts_superframe_period() {
                self.buffer[self.sbf] = sbf;
                self.last_id = Some(sbf.how.sbf_id);
                self.sbf = 1;
            }
            None
        } else {
            let last_id = self.last_id.unwrap();

            let expected_id =
                GpsQzssLnavSubframeId::decode((last_id.encode() + 1) % GPS_LNAV_FRAME_ITEMS as u8)
                    .unwrap();

            if sbf.how.sbf_id != expected_id {
                // breaks sequence
                self.reset();
                None
            } else {
                // valid sequence

                // guards against corrupted streams
                self.sbf = (self.sbf + 1) % TOTAL_SUBFRAMES;

                if sbf.ends_superframe_period() {
                    // last ALM5 in 12'30"
                    // -> terminates sequence
                    self.reset();
                    None
                } else {
                    None
                }
            }
        }
    }

    /// Use this method if your decoding process works at the 30s cycle scale,
    /// otherwise prefer [Self::push_subframe], especially when working with our
    /// [GpsQzssDecoder].
    pub fn push_frame(&mut self, frame: GpsQzssLnavFrame) -> Option<GpsQzssLnavSuperframe> {
        let _ = self.push_sbf(*frame.eph_1())?;
        let _ = self.push_sbf(*frame.eph_2())?;
        let _ = self.push_sbf(*frame.eph_3())?;
        let _ = self.push_sbf(*frame.alm_4())?;
        self.push_sbf(*frame.alm_5())
    }
}
