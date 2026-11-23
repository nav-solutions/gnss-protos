//! GPS 12'30" message (25 pages of 5 subframes)

use crate::{
    constants::SECONDS_PER_WEEK,
    gps::lnav::{
        GpsQzssLnavFrame, GpsQzssLnavSubframe, GpsQzssLnavSuperframe, GPS_LNAV_FRAME_ITEMS,
        GPS_LNAV_SUBFRAME_DURATION_S, GPS_LNAV_SUPERFRAME_DURATION_S, GPS_LNAV_SUPERFRAME_ITEMS,
    },
};

#[cfg(feature = "log")]
use log::error;

/// Total number of subframes in a 12'30" period
pub(crate) const TOTAL_SUBFRAMES: usize = GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS;

/// [GpsQzssLnavSuperframeBuilder] collects [GpsQzssLnavSubframe]s (6s) or
/// [GpsQzssLnavFrame]s (30s) consecutively, and aligns itself to EPH-1 of Page-1
/// of the almanach pagination. Therefore, the resulting [GpsQzssLnavSuperframe]
/// is correctly aligned and starts with Page-1 EPH-1.
///
/// The builder verifies both the subframe class progression (EPH-1 to ALM-5)
/// and the temporal progression (continuous time of week). Any discontinuity
/// drops the current collection and the builder waits for the next Page-1 EPH-1.
///
/// ```
/// use gnss_protos::gps::lnav::{
///     GpsQzssLnavHow, GpsQzssLnavSubframe, GpsQzssLnavSuperframeBuilder,
/// };
///
/// let mut builder = GpsQzssLnavSuperframeBuilder::default();
///
/// // EPH-1 at the beginning of the week: page 1
/// let sbf = GpsQzssLnavSubframe::default()
///     .with_hand_over_word(GpsQzssLnavHow::ephemeris1().with_tow_seconds(6));
///
/// assert!(sbf.starts_superframe_period());
/// assert!(builder.push_sbf(sbf).is_none());
/// assert_eq!(builder.total_subframes(), 1);
/// assert_eq!(builder.seconds_to_end_of_period(), Some(744));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsQzssLnavSuperframeBuilder {
    // number of subframes currently collected
    sbf: usize,

    // subframe buffer
    buffer: [GpsQzssLnavSubframe; TOTAL_SUBFRAMES],
}

impl Default for GpsQzssLnavSuperframeBuilder {
    fn default() -> Self {
        Self {
            sbf: 0,
            buffer: [GpsQzssLnavSubframe::default(); TOTAL_SUBFRAMES],
        }
    }
}

impl GpsQzssLnavSuperframeBuilder {
    /// Drops the current collection: the builder will wait for the next Page-1 EPH-1.
    /// You don't have to call this once a [GpsQzssLnavSuperframe] has been returned,
    /// the builder resets itself.
    pub fn reset(&mut self) {
        self.sbf = 0;
    }

    /// Returns true if `next` immediately follows `previous` in a GPS/QZSS stream:
    /// next subframe ID (ALM-5 is followed by EPH-1) and time of week incremented by 6s
    /// (rolling over at the end of the week).
    pub fn follows(previous: &GpsQzssLnavSubframe, next: &GpsQzssLnavSubframe) -> bool {
        let expected_id = previous.how.sbf_id.encode() % GPS_LNAV_FRAME_ITEMS as u8 + 1;

        next.how.sbf_id.encode() == expected_id
            && next.week_transmission_time_s()
                == (previous.week_transmission_time_s() + GPS_LNAV_SUBFRAME_DURATION_S)
                    % SECONDS_PER_WEEK
    }

    /// Returns the total of subframes (6s messages) collected so far.
    pub fn total_subframes(&self) -> usize {
        self.sbf
    }

    /// Returns the total of complete frames (30s messages) collected so far.
    pub fn total_frames(&self) -> usize {
        self.sbf / GPS_LNAV_FRAME_ITEMS
    }

    /// Returns true if no subframe has been collected yet.
    pub fn is_empty(&self) -> bool {
        self.sbf == 0
    }

    /// Returns total number of seconds remaining before the super frame completes
    /// (end of the recording of ALM-5 page 25), or None if not locked yet.
    pub fn seconds_to_end_of_period(&self) -> Option<u32> {
        if self.sbf == 0 {
            None
        } else {
            Some(GPS_LNAV_SUPERFRAME_DURATION_S - self.sbf as u32 * GPS_LNAV_SUBFRAME_DURATION_S)
        }
    }

    /// Use this method when working at the [GpsQzssLnavSubframe] level (GPS/QZSS LNAV 6s core element).
    /// Returns a [GpsQzssLnavSuperframe] when this subframe completes a 12'30" period
    /// (ALM-5 of page 25), that started with EPH-1 of page 1, without discontinuity.
    ///
    /// This is most convenient when working with our [crate::gps::lnav::GpsQzssLnavSubfDecoder]
    /// and your application intends to collect 12'30s periods.
    pub fn push_sbf(&mut self, sbf: GpsQzssLnavSubframe) -> Option<GpsQzssLnavSuperframe> {
        if sbf.starts_superframe_period() {
            // (re)locks to Page-1 EPH-1. This also handles the last period of the week,
            // which is interrupted after 10 pages (604800 is not a multiple of 750).
            self.buffer[0] = sbf;
            self.sbf = 1;
            return None;
        }

        if self.sbf == 0 {
            // waiting for Page-1 EPH-1
            return None;
        }

        if !Self::follows(&self.buffer[self.sbf - 1], &sbf) {
            // sequence broken
            self.reset();
            return None;
        }

        self.buffer[self.sbf] = sbf;
        self.sbf += 1;

        if self.sbf < TOTAL_SUBFRAMES {
            return None;
        }

        // last ALM-5 of the 12'30" period
        self.reset();

        let mut frames = [GpsQzssLnavFrame::default(); GPS_LNAV_SUPERFRAME_ITEMS];

        for (page, frame) in frames.iter_mut().enumerate() {
            let mut slice = [GpsQzssLnavSubframe::default(); GPS_LNAV_FRAME_ITEMS];
            slice.copy_from_slice(
                &self.buffer[page * GPS_LNAV_FRAME_ITEMS..(page + 1) * GPS_LNAV_FRAME_ITEMS],
            );

            // cannot fail: the class progression has been verified
            *frame = GpsQzssLnavFrame::from_slice(slice).ok()?;
        }

        match GpsQzssLnavSuperframe::from_frame_slice(frames) {
            Ok(spf) => Some(spf),
            Err(_e) => {
                #[cfg(feature = "log")]
                error!("(GPS/QZSS) invalid 12'30\" period: {}", _e);
                None
            },
        }
    }

    /// Use this method if your decoding process works at the 30s cycle scale,
    /// otherwise prefer [Self::push_sbf], especially when working with our
    /// [crate::gps::lnav::GpsQzssLnavSubfDecoder].
    pub fn push_frame(&mut self, frame: GpsQzssLnavFrame) -> Option<GpsQzssLnavSuperframe> {
        // only ALM-5 may complete a period
        let _ = self.push_sbf(*frame.eph_1());
        let _ = self.push_sbf(*frame.eph_2());
        let _ = self.push_sbf(*frame.eph_3());
        let _ = self.push_sbf(*frame.alm_4());
        self.push_sbf(*frame.alm_5())
    }
}
