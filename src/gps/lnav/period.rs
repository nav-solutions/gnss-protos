use hifitime::prelude::{Duration, Epoch, TimeScale, TimeSeries};

use crate::gps::lnav::GPS_LNAV_PERIOD_DURATION;

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
        for (start_utc_str, expected_t0_str, expected_last_str) in [
            (
                "1980-01-06T00:00:00 UTC",
                "1980-01-06T00:00:00 UTC",
                "1980-01-06T00:12:30 UTC",
            ),
            (
                "1980-01-06T00:05:00 UTC",
                "1980-01-06T00:00:00 UTC",
                "1980-01-06T00:12:30 UTC",
            ),
            (
                "1980-01-06T00:06:00 UTC",
                "1980-01-06T00:00:00 UTC",
                "1980-01-06T00:12:30 UTC",
            ),
            (
                "1980-01-06T00:06:14 UTC",
                "1980-01-06T00:00:00 UTC",
                "1980-01-06T00:12:30 UTC",
            ),
            (
                "1980-01-06T00:06:15 UTC",
                "1980-01-06T00:12:30 UTC",
                "1980-01-06T00:12:30 UTC",
            ),
            (
                "1980-01-06T00:12:29 UTC",
                "1980-01-06T00:12:30 UTC",
                "1980-01-06T00:25:00 UTC",
            ),
            (
                "1980-01-06T00:12:16 UTC",
                "1980-01-06T00:12:30 UTC",
                "1980-01-06T00:25:00 UTC",
            ),
            (
                "1985-12-13T12:34:55 UTC",
                "1985-12-13T12:34:56 UTC",
                "1985-12-13T12:47:26 UTC",
            ),
            (
                "1995-12-09T01:10:33 UTC",
                "1995-12-09T01:12:20 UTC",
                "1995-12-09T01:24:50 UTC",
            ),
            (
                "1999-08-21T23:52:33 UTC",
                "1999-08-21T23:52:17 UTC",
                "1999-08-22T00:04:47 UTC",
            ),
            (
                "2000-01-01T00:00:00 UTC",
                "1999-12-31T23:59:47 UTC",
                "2000-01-01T00:12:17 UTC",
            ),
            (
                "2005-01-22T00:38:00 UTC",
                "2005-01-22T00:42:17 UTC",
                "2005-01-22T00:54:47 UTC",
            ),
            (
                "2019-04-07T00:01:12 UTC",
                "2019-04-06T23:57:12 UTC",
                "2019-04-07T00:09:42 UTC",
            ),
            (
                "2026-11-28T12:28:12 UTC",
                "2026-11-28T12:22:12 UTC",
                "2026-11-28T12:34:42 UTC",
            ),
        ] {
            let start_utc = Epoch::from_str(start_utc_str).unwrap();
            let expected_t0 = Epoch::from_str(expected_t0_str).unwrap();
            let expected_last = Epoch::from_str(expected_last_str).unwrap();

            let expected_t0_gpst = expected_t0.to_time_scale(TimeScale::GPST);
            let expected_last_gpst = expected_last.to_time_scale(TimeScale::GPST);

            let arc = gps_temporal_arc(start_utc, GPS_LNAV_PERIOD_DURATION - 1.0 * Unit::Second);

            let (first, last) = (arc.first_epoch(), arc.last_epoch());

            assert_eq!(
                first.time_scale,
                TimeScale::GPST,
                "expressed in invalid timescale"
            );
            assert_eq!(first, expected_t0_gpst);

            assert_eq!(
                last.time_scale,
                TimeScale::GPST,
                "expressed in invalid timescale"
            );
            assert_eq!(last, expected_last_gpst);

            // verifies QZSS aswell, same instants because both timescale are "identical"
            let arc = qzss_temporal_arc(start_utc, GPS_LNAV_PERIOD_DURATION - 1.0 * Unit::Second);

            let (first, last) = (arc.first_epoch(), arc.last_epoch());

            assert_eq!(
                first.time_scale,
                TimeScale::QZSST,
                "expressed in invalid timescale"
            );
            assert_eq!(first, expected_t0_gpst);

            assert_eq!(
                last.time_scale,
                TimeScale::QZSST,
                "expressed in invalid timescale"
            );
            assert_eq!(last, expected_last_gpst);
        }
    }
}
