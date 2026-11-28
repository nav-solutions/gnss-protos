use hifitime::{
    prelude::{Duration, Epoch, TimeScale, TimeSeries},
    NANOSECONDS_PER_SECOND,
};

use crate::gps::lnav::{GPS_LNAV_PERIOD_DURATION, GPS_LNAV_SUPERFRAME_DURATION_S};

/// Helper function to generate a temporal arc, expressed as [TimeSeries],
/// which is synchronous and aligned to the GPS publication period (12'30").
///
/// ## Input
///
/// - epoch: nearest epoch from starting point. We will round
/// this [Epoch], expressed in _any_ supported [TimeScale], to the nearest
/// GPST period starting point.
///
/// - duration: [Duration] of the output arc
pub fn gps_temporal_arc(start: Epoch, duration: Duration) -> TimeSeries {
    let start = start.to_time_scale(TimeScale::GPST);
    synchronous_temporal_arc(start, duration)
}

/// Helper function to generate a temporal arc, expressed as [TimeSeries],
/// which is synchronous and aligned to the QZSS publication period (12'30").
/// Considering QZSST is stirred to GPST, this will produce the same instants,
/// but this method expresses all instants in QZSST instead.
/// Read [gps_temporal_arc] for more information.
pub fn qzss_temporal_arc(start: Epoch, duration: Duration) -> TimeSeries {
    let start = start.to_time_scale(TimeScale::QZSST);
    synchronous_temporal_arc(start, duration)
}

/// Generates correct 12'30" temporal arc starting point
/// which must be expressed in correct [TimeScale]
fn synchronous_temporal_arc(start: Epoch, duration: Duration) -> TimeSeries {
    let end = start + duration;
    let start = start.round(GPS_LNAV_PERIOD_DURATION);
    let end = end.round(GPS_LNAV_PERIOD_DURATION);
    TimeSeries::inclusive(start, end, GPS_LNAV_PERIOD_DURATION)
}

#[cfg(test)]
mod test {
    use super::*;

    use std::str::FromStr;

    use hifitime::prelude::{Epoch, TimeScale, Unit};

    #[test]
    fn test_gps_periods() {
        for (start_utc_str, expected_t0_str) in
            [("1980-01-06T00:00:00 UTC", "1980-01-06T00:00:00 UTC")]
        {
            let start_utc = Epoch::from_str(start_utc_str).unwrap();
            let expected_t0 = Epoch::from_str(expected_t0_str).unwrap();
            let expected_t0_gpst = expected_t0.to_time_scale(TimeScale::GPST);

            let arc = gps_temporal_arc(start_utc, GPS_LNAV_PERIOD_DURATION - 1.0 * Unit::Second);

            assert_eq!(arc.first_epoch(), expected_t0_gpst);
        }
    }
}
