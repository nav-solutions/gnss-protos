//! GPS/QZSS LNAV period helpers

use thiserror::Error;

use hifitime::{
    prelude::{Duration, Epoch, TimeScale},
    NANOSECONDS_PER_SECOND,
};

use crate::{
    gps::{GPS_FRAME_DURATION_S, GPS_QZSS_ALMANACH_PAGES, GPS_SUBFRAME_DURATION_S, GPS_WEEK_ROLLOVER},
    constants::SECONDS_PER_WEEK,
};

#[derive(Error, Debug)]
pub enum Error {
    #[error("time before 1980/01 is invalid")]
    InvalidEpoch,
}

/// Half of the GPS Week Rollover, used in week counter unwrapping process
const GPS_HALF_WEEK_ROLLOVER: i64 = GPS_WEEK_ROLLOVER as i64 / 2;

const SUBFRAME_NS: u128 = GPS_SUBFRAME_DURATION_S as u128 * NANOSECONDS_PER_SECOND as u128; // 300 bits à 50 bit/s
const FRAME_NS: u128 = GPS_FRAME_DURATION_S as u128 * SUBFRAME_NS; // 5 sous-trames = 1 page
const SUPERFRAME_NS: u128 = GPS_QZSS_ALMANACH_PAGES as u128 * FRAME_NS; // 750 s = 12'30"
const WEEK_NS: u128 = SECONDS_PER_WEEK as u128 * SUBFRAME_NS;

/// [GpsQzssLnavTimePrecision] defines the precision of the
/// temporal arc we can design. This depends on the use case
/// and application. 
#[derive(Debug, Clone, Copy)]
pub enum GpsQzssLnavTimePrecision {
    /// [GpsQzssLnavTimePrecision::SuperFrame] gives a 12'30s precision,
    /// which is best suited when working at the GPS/QZSS sideral period,
    /// for example in GPS/QZSS constellation Almanach analysis and synthesis.
    #[default]
    SuperFrame,

    /// [GpsQzssLnavTimePrecision::Frame] gives a 30s precision,
    /// which is needed if you intend to work on a GPS/QZSS 
    /// message EPH1,2,3 (18s) + ALM4-5 (12s) message rotation.
    Frame,

    /// [GpsQzssLnavTimePrecision::SubFrame] gives a 6s precision,
    /// which is needed if you intend to work on a GPS/QZSS 
    /// message basis (whatever their kind)
    SubFrame,

    /// [GpsQzssLnavTimePrecision::Byte] gives a 160ms precision (1 GPS/QZSS byte),
    /// which is best suited for GPS/QZSS modulators.
    Byte,

    /// [GpsQzssLnavTimePrecision::Bit] gives a 20ms precision (1 GPS/QZSS bit),
    /// which is best suited for GPS/QZSS modulators. 
    Bit,
}

impl From<Duration> for GpsQzssLnavTimePrecision {
    /// Convenient method to define [GpsQzssLnavTimePrecision] from the _nearest_
    /// [Duration].
    fn from(duration: Duration) -> Self {
        Self::default()        
    }
}

impl Into<Duration> for GpsQzssLnavTimePrecision {
    fn into(&self) -> Duration {
        match self {
            Self::SuperFrame => {
                Duration::from_total_nanoseconds(
                    (GPS_FRAME_DURATION_S as i128 * GPS_QZSS_ALMANACH_PAGES as i128) * NANOSECONDS_PER_SECOND
                )
            },
            Self::Frame => {
                Duration::from_total_nanoseconds(
                    GPS_FRAME_DURATION_S as i128 * NANOSECONDS_PER_SECOND
                )
            },
            Self::SubFrame => {
                Duration::from_total_nanoseconds(
                    GPS_SUBFRAME_DURATION_S as i128 * NANOSECONDS_PER_SECOND
                )
            },
            Self::Bit => {
                Duration::from_total_nanoseconds(
                    GPS_SUBFRAME_DURATION_S as i128 / 300
                )
            },
            Self::Byte => {
                (Self::Bit as Duration) * 8.0
            },
        }
    }
}

// /// [GpsQzssLnavTimeSync] gives the placement in [TimeScale::GPST] Timescale,
// /// of every single GPS/QZSS navigation message data words, according to
// /// the broadcast scheduling table. It conveniently builds from
// /// an Hifitime [Epoch] and from any [TimeScale].
// /// It may be used to transmit GPS/QZSS navigation messages correctly.
// /// It can also be used to determine the almanach pagination at any point in time.
// #[derive(Debug, Clone, Copy)]
// pub struct GpsQzssLnavTimeSync {
//     /// GPS or QZSS reference point in time
//     /// this object is defined from. This is very broad,
//     /// it can be the instant of GPS/QZSS bytes decoding,
//     /// it can be the intended instant of the next GPS/QZSS 
//     /// byte or word or superframe transmission, depending on the use case.
//     ///
//     /// From this GPS/QZSS [Epoch], we propose many helper functions.
//     epoch: Epoch,
// 
//     /// Début de la super-trame (cycle de 25 pages) en cours.
//     pub superframe_start: Epoch,
// 
//     /// Durée réelle de cette super-trame : 750 s, sauf la dernière de la semaine (300 s).
//     pub superframe_duration: Duration,
// 
//     /// Temps écoulé depuis le début de la super-trame.
//     pub elapsed_in_superframe: Duration,
// 
//     /// Page des sous-trames 4 et 5 (1..=25).
//     pub page: u8,
// 
//     /// Sous-trame en cours (1..=5).
//     pub subframe: u8,
// 
//     /// Bit en cours dans la sous-trame (0..=299).
//     pub bit_in_subframe: u16,
// 
//     /// Temps écoulé dans la sous-trame en cours (< 6 s).
//     pub elapsed_in_subframe: Duration,
// }
// 
// impl GpsQzssLnavTimeSync {
//     /// Conveniently builds a [GpsQzssLnavTimeSync] from a wrapped
//     /// GPS/QZSS 10-bit week counter, as transported by the GPS/QZSS protocol.
//     /// To unwrap the week number correctly, we need a reference point in time which lies
//     /// in the same 10-bit rollover period (more than 19 years). If you use the decoding
//     /// instant or intended transmission time, you are more then fine.
//     ///
//     /// ## Input
//     ///
//     /// - t_ref: reference point in time expressed as any [Epoch], not necessarily
//     /// GPST or QZSST [Epoch]s
//     ///
//     /// - week_10b: wrapped 10b week counter as transported by GPS/QZSS protocol
//     ///
//     /// - seconds: elapsed seconds in said week
//     ///
//     /// ## Output
//     ///
//     /// - [GpsQzssLnavTemporalPosition]
//     pub fn from_wrapped_gps_time_of_week(t_ref: Epoch, week_10b: u32, seconds: u32) -> Self {
//         let (week, seconds) = Self::week_10b_unwrapping(t_ref, week_10b, seconds);
//         let t_gpst = Epoch::from_time_of_week(week, seconds as u64 * NANOSECONDS_PER_SECOND, TimeScale::GPST);
//         Self::from(t_gpst)
//     }
//     
//     /// See [Self::from_wrapped_gps_time_of_week].
//     pub fn from_wrapped_qzss_time_of_week(t_ref: Epoch, week_10b: u32, seconds: u32) -> Self {
//         let (week, seconds) = Self::week_10b_unwrapping(t_ref, week_10b, seconds);
//         let t_qzss = Epoch::from_time_of_week(week, seconds as u64 * NANOSECONDS_PER_SECOND, TimeScale::QZSS);
//         Self::from(t_qzss)
//     }
// 
//     /// This is a convenient method to build a temporal arc (sequence of discrete instants) described
//     /// as [TimeSeries], which is correctly aligned to the GPS broadcast table.
//     ///
//     /// We propose [Self::next_qzss_sidecal_arc] which is strictly identical, expect that each
//     /// point in time is expressed in [TimeScale::QZSST] timescale.
//     ///
//     /// ## Input
//     ///
//     /// - [GpsQzssLnavTimeSync] built from any [TimeScale], internally converted to
//     /// [TimeScale::GPST] timescale.
//     ///
//     /// ## Output
//     ///
//     /// - temporal arc, expressed as [TimeSeries] 
//     ///
//     pub fn next_gps_sideral_arc(num_superframes: u32) -> TimeSeries {
//         // round to nearest superframe t0
// 
//         // arc designing
//     }
// 
//     /// [Self::next_gps_sideral_arc] except that each output point is expressed
//     /// in [TimeScale::QZSST] timescale.
//     pub fn next_qzss_sidecar_arc(t_start: Epoch, num_superframes: u32) -> TimeSeries {
//         let t_qzsst = t_start.to_time_scale(TimeScale::QZSST);
//         
//         // round to nearest superframe t0
// 
//         // arc designing
//     }
// 
//     /// 10-bit GPS/QZSS week count unwrapping algorithm.
//     /// Unwraps the 10-bit week count, as transported by
//     /// the GPS/QZSS navigation messages, to be further used in required calculations.
//     /// To unwrap the week number correctly, we need a reference point in time which lies
//     /// in the same 10-bit rollover period, which lasts for more than 19 years.
//     /// If you use the decoding instant or intended transmission time, you are more then fine.
//     /// 
//     /// ## Input
//     ///
//     /// - t_ref: Reference point in time, expressed as [Epoch], any [TimeScale] accepted here,
//     /// this method is not limited to GPST or QZSST.
//     ///
//     /// - week_10b: 10b wrapped week counter
//     /// - seconds: elapsed in current GPS/QZSS week
//     ///
//     /// ## Output
//     ///
//     /// - (unrapped week, seconds): as (u32, u32)
//     pub fn week_10b_unwrapping(t_ref: Epoch, week_10b: u32, seconds: u32) -> (u32, u32) {
// 
//         let t_gpst = t_ref.to_time_scale(TimeScale::GPST);
// 
//         let reference_week = t_gpst.to_time_of_week().0 as i64;
//         let wn = (wn as i64) % GPS_WEEK_ROLLOVER as i64;
// 
//         // candidate within the same 1024 week cycle as the reference
//         let mut week = reference_week - reference_week.rem_euclid(GPS_WEEK_ROLLOVER as i64) + wn;
// 
//         // pick closest candidate
//         let delta = week - reference_week;
// 
//         if delta > GPS_HALF_WEEK_ROLLOVER {
//             week -= GPS_WEEK_ROLLOVER as i64;
//         } else if delta < -GPS_HALF_WEEK_ROLLOVER {
//             week += GPS_WEEK_ROLLOVER as i64;
//         }
// 
//         // weeks prior GPST origin do not exist
//         if week < 0 {
//             week += GPS_WEEK_ROLLOVER as i64;
//         }
// 
//         week as u32
//         (unwrapped_week, seconds)
//     }
// }
// 
// impl From<Epoch> for GpsQzssLnavTimeSync {
//     /// Calculates [GpsQzssLnavTemporalPosition] from any transmission time,
//     /// expressed as [Epoch] in any [TimeScale],
//     /// not limited to [TimeScale::GPST].
//     /// You may use [TimeScale::UTC] and it is also possible to create
//     /// an [Epoch] from the system-time when stdlib is supported.
//     fn from(epoch: Epoch) -> Self {
// 
//         let t_gpst = epoch.to_time_scale(TimeScale::GPST);
// 
//         let (week, tow_ns) = t_gpst.to_time_of_week();
// 
//         let week = week as u128;
// 
//         // Le cycle des 25 pages redémarre à chaque début de semaine (IS-GPS-200).
//         let superframe_index = tow_ns / SUPERFRAME_NS as i128;
//         let sf_start_tow_nanos = superframe_index * SUPERFRAME_NS as i128;
//         let elapsed_sf_nanos = tow_ns - sf_start_tow_nanos;
//         let sf_len = SUPERFRAME_NS.min(WEEK_NS - sf_start_tow_nanos);
// 
//         let frame_in_sf = elapsed_sf_nanos / FRAME_NS; // 0..=24
//         let in_frame = elapsed_sf_nanos % FRAME_NS;
// 
//         let subframe = in_frame / SUBFRAME_NS; // 0..=4
//         let in_subframe = in_frame % SUBFRAME_NS;
//         let bit = in_subframe / (SUBFRAME_NS / 50); // 20 ms par bit
// 
//         let start_ns = week as i128 * WEEK_NS as i128 + sf_start_tow_nanos as i128;
// 
//         let superframe_start = Epoch::from_time_of_week(week as u32, 
//                                                         sf_start_tow_nanos as u64, TimeScale::GPST);
// 
//         // debug_assert_eq!(superframe_start.duration.total_nanoseconds(), start_ns);
// 
//         Self {
//             week: week as u32,
//             superframe_index: superframe_index as u32,
//             superframe_start,
//             superframe_duration: Duration::from_total_nanoseconds(sf_len as i128),
//             elapsed_in_superframe: Duration::from_total_nanoseconds(elapsed_sf_nanos as i128),
//             page: frame_in_sf as u8 + 1,
//             subframe: subframe as u8 + 1,
//             bit_in_subframe: bit as u16,
//             elapsed_in_subframe: Duration::from_total_nanoseconds(in_subframe as i128),
//         }
//     }
// }
// 
// #[cfg(test)]
// mod test {
//     use crate::gps::GpsQzssLnavTemporalPosition;
//     use core::str::FromStr;
//     use hifitime::prelude::{Epoch, TimeScale};
// 
//     #[test]
//     fn test_lnav_temporal_reciprocal() {
//         for (time_str, week, page_num, subframe_num, superframe_start_str) in [
//             // ("1980-01-26T00:00:00 GPST", 0, 1, 1, "1980-01-26T00:00:00 GPST"),
//             ("2026-11-20T10:04:17 UTC", 2445, 10, 1, "2026-11-20T10:00:00 GPST"),
//             // ("2026-11-21T23:59:59 GPST",
//             // ("2026-11-22T00:00:00 GPST",
//             // ("1980-01-05T12:00:00 GPST",
//             // ("1980-01-06T00:00:00 GPST",
//             // ("1980-01-06T12:00:00 GPST",
//         ] {
//             let time = Epoch::from_str(time_str)
//                 .unwrap();
// 
//             let superframe_start = Epoch::from_str(superframe_start_str)
//                 .unwrap();
// 
//             let gps_time = GpsQzssLnavTemporalPosition::from(time);
// 
//             assert_eq!(gps_time.week, week & 0x3ff, "invalid week number");
//             assert_eq!(gps_time.superframe_start, superframe_start, "invalid week number");
// 
//         }
//     }
// }
