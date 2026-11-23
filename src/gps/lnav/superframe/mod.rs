//! GPS 12'30" message (25 pages of 5 subframes)

use crate::{
    constants::SECONDS_PER_WEEK,
    gps::lnav::{
        GpsError, GpsQzssLnavAlmanach, GpsQzssLnavAlmanachHealth, GpsQzssLnavConfigHealth,
        GpsQzssLnavFrame, GpsQzssLnavHow, GpsQzssLnavIonosphereUtc, GpsQzssLnavNmct,
        GpsQzssLnavSatelliteHealth, GpsQzssLnavSpecialMessage, GpsQzssLnavSubframe,
        GpsQzssLnavSubframe1, GpsQzssLnavSubframe2, GpsQzssLnavSubframe3, GpsQzssLnavSubframe4,
        GpsQzssLnavSubframe5, GpsQzssLnavSubframeEnum, GpsQzssLnavSubframeId, GpsQzssLnavTelemetry,
        GPS_LNAV_FRAME_DURATION_S, GPS_LNAV_FRAME_ITEMS, GPS_LNAV_SUBFRAME_BITS,
        GPS_LNAV_SUBFRAME_DURATION_S, GPS_LNAV_SUPERFRAME_BITS, GPS_LNAV_SUPERFRAME_BYTES,
        GPS_LNAV_SUPERFRAME_ITEMS,
    },
    BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream, Message,
};

mod builder;
mod decoder;

pub use builder::GpsQzssLnavSuperframeBuilder;
pub use decoder::GpsQzssLnavSuperframeDecoder;

/// [GpsQzssLnavSuperframe] is a convenient structure to work at the
/// GPS/QZSS Almanach level, which is described by 25 [GpsQzssLnavFrame]s collected in 12'30s.
/// Like any other structures, it supports both encoding & decoding.
///
/// The rotation always starts with EPH-1 of page 1, at a multiple of 12'30" since the
/// beginning of the week (the page number is defined by the time of transmission, see
/// [GpsQzssLnavSubframe::page_number]), so the page of each subframe is defined by its position.
/// The first period of each week is aligned to sunday midnight. Because 604800 s is not a multiple
/// of 750 s, the last period of the week is interrupted after 10 pages and never completes.
///
/// To understand Almanach pages content, refer to [GpsQzssLnavSubframe4] and [GpsQzssLnavSubframe5]):
/// - SF.5 p1-24 (SVID=1-24) gives almanach per satellite ([GpsQzssLnavAlmanach])
/// - SF.5 p25 (SVID=51) gives ToA, WNa (8bits), health for SV1-24 ([GpsQzssLnavAlmanachHealth])
/// - SF.4 p2-5, p7-10 (SVID=25-32) gives almanach per satellite ([GpsQzssLnavAlmanach])
/// - SF.4 p18 (SVID=56) Ionosphere Kb alpha,beta and UTC (a0, a1, ToT, WNt..) ([GpsQzssLnavIonosphereUtc])
/// - SF.4 p25 (SVID=63) 4 bits per SV: anti-spoofing + signal block type, health for SV25-32 ([GpsQzssLnavConfigHealth]).
/// - SF.4 p13 (SVID=52) NMCT: ERD distance correction ([GpsQzssLnavNmct])
/// - SF.4 p17 (SVID=55) Special message (22 ASCII bytes) ([GpsQzssLnavSpecialMessage])
/// - SF.4 p1, 6, 11, 12, 14, 15, 16, 19-24 (SVID=53,54,57-62) reserved ([crate::gps::lnav::GpsQzssLnavReservedSubframe])
///
/// ```
/// use gnss_protos::gps::lnav::{
///     GpsQzssLnavAlmanach, GpsQzssLnavAlmanachHealth, GpsQzssLnavSubframe1, GpsQzssLnavSubframe2,
///     GpsQzssLnavSubframe3, GpsQzssLnavSubframe4, GpsQzssLnavSubframe5, GpsQzssLnavHow,
///     GpsQzssLnavSuperframe, GpsQzssLnavReservedSubframe, GpsQzssLnavTelemetry,
/// };
/// use gnss_protos::Message;
///
/// // the content of the 25 pages of subframes 4 and 5 (IS-GPS-200 pagination)
/// let frame4: [GpsQzssLnavSubframe4; 25] = core::array::from_fn(|p| {
///     let page = p as u8 + 1;
///     let sv_id = GpsQzssLnavSubframe4::page_sv_id(page).unwrap();
///     if GpsQzssLnavSubframe4::is_almanach_page(page) {
///         GpsQzssLnavSubframe4::Almanach(
///             GpsQzssLnavAlmanach::default()
///                 .with_sv_id(sv_id)
///                 .with_keplerian_parameters(5153.6, 0.01, 0.1 * page as f64, 0.2, 0.3),
///         )
///     } else {
///         GpsQzssLnavSubframe4::Reserved(GpsQzssLnavReservedSubframe::default().with_sv_id(sv_id))
///     }
/// });
///
/// let frame5: [GpsQzssLnavSubframe5; 25] = core::array::from_fn(|p| match p {
///     24 => GpsQzssLnavSubframe5::Health(GpsQzssLnavAlmanachHealth::default()),
///     _ => GpsQzssLnavSubframe5::Almanach(GpsQzssLnavAlmanach::default().with_sv_id(p as u8 + 1)),
/// });
///
/// // a rotation that starts with the week
/// let message = GpsQzssLnavSuperframe::from_pages(
///     0,
///     (GpsQzssLnavTelemetry::default(), GpsQzssLnavHow::default()),
///     (GpsQzssLnavSubframe1::default(), GpsQzssLnavSubframe2::default(), GpsQzssLnavSubframe3::default()),
///     (frame4, frame5),
/// );
///
/// assert!(message.validate().is_ok());
///
/// // 125 subframes: 37500 bits
/// let mut bytes = vec![0u8; message.encoding_size()];
/// message.encode(&mut bytes).unwrap();
///
/// let decoded = GpsQzssLnavSuperframe::decode(&bytes).unwrap();
/// assert_eq!(decoded.week_transmission_time_s(), 0);
///
/// // bit exact
/// let mut reencoded = vec![0u8; decoded.encoding_size()];
/// decoded.encode(&mut reencoded).unwrap();
/// assert_eq!(reencoded, bytes);
///
/// // the almanac of the 32 satellites
/// assert_eq!(decoded.almanachs().count(), 32);
///
/// let almanach = decoded.almanach(27).unwrap();
/// let (x, y, z) = almanach.position_ecef_m(almanach.time_from_reference_s(3600.0));
/// let radius = (x * x + y * y + z * z).sqrt();
/// assert!(radius > 26_000e3 && radius < 27_000e3);
///
/// assert!(decoded.satellite_health_by_prn(3).unwrap().is_healthy());
/// ```
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsQzssLnavSuperframe([GpsQzssLnavFrame; GPS_LNAV_SUPERFRAME_ITEMS]);

impl Default for GpsQzssLnavSuperframe {
    fn default() -> Self {
        Self([GpsQzssLnavFrame::default(); GPS_LNAV_SUPERFRAME_ITEMS])
    }
}

impl GpsQzssLnavSuperframe {
    /// Builds a [GpsQzssLnavSuperframe] from a sequence of 25 [GpsQzssLnavFrame]s (30s recordings),
    /// which must form a valid 12'30" period (see [Self::validate]).
    ///
    /// To facilitate real world-applications, we propose [GpsQzssLnavSuperframeBuilder]
    /// to collect the subframe sequence: a [GpsQzssLnavSuperframe] returned by the builder
    /// is garanteed to be valid.
    pub fn from_frame_slice(
        slice: [GpsQzssLnavFrame; GPS_LNAV_SUPERFRAME_ITEMS],
    ) -> Result<Self, GpsError> {
        let s = Self(slice);
        s.validate()?;
        Ok(s)
    }

    /// Returns the [GpsQzssLnavFrame] at given index.
    ///
    /// ## Input
    ///
    /// - index: must be < GPS_LNAV_SUPERFRAME_ITEMS, otherwise this will panic.
    pub fn nth_frame(&self, index: usize) -> &GpsQzssLnavFrame {
        &self.0[index]
    }

    /// Builds a coherent [GpsQzssLnavSuperframe] from its content: the rotation starts at
    /// `start_tow_seconds` (a multiple of 750 s, see [GpsQzssLnavSubframe::starts_superframe_period]),
    /// every subframe uses the `header` [GpsQzssLnavTelemetry] and [GpsQzssLnavHow] (alert and
    /// anti-spoofing flags), with its own time of week and subframe ID,
    /// the `ephemeris` are repeated on every page, and `almanachs` gives
    /// the content of the 25 pages of subframes 4 and 5.
    ///
    /// The resulting message is valid as long as the start time
    /// is aligned and the pages contents match the IS-GPS-200 pagination
    /// ([GpsQzssLnavSubframe4::matches_page], [GpsQzssLnavSubframe5::matches_page]).
    pub fn from_pages(
        start_tow_seconds: u32,
        header: (GpsQzssLnavTelemetry, GpsQzssLnavHow),
        ephemeris: (
            GpsQzssLnavSubframe1,
            GpsQzssLnavSubframe2,
            GpsQzssLnavSubframe3,
        ),
        almanachs: (
            [GpsQzssLnavSubframe4; GPS_LNAV_SUPERFRAME_ITEMS],
            [GpsQzssLnavSubframe5; GPS_LNAV_SUPERFRAME_ITEMS],
        ),
    ) -> Self {
        let (telemetry, how) = header;
        let (eph1, eph2, eph3) = ephemeris;

        let mut pages = [GpsQzssLnavFrame::default(); GPS_LNAV_SUPERFRAME_ITEMS];

        for (p, page) in pages.iter_mut().enumerate() {
            let contents = [
                (
                    GpsQzssLnavSubframeId::Ephemeris1,
                    GpsQzssLnavSubframeEnum::Ephemeris1(eph1),
                ),
                (
                    GpsQzssLnavSubframeId::Ephemeris2,
                    GpsQzssLnavSubframeEnum::Ephemeris2(eph2),
                ),
                (
                    GpsQzssLnavSubframeId::Ephemeris3,
                    GpsQzssLnavSubframeEnum::Ephemeris3(eph3),
                ),
                (
                    GpsQzssLnavSubframeId::Almanach4,
                    GpsQzssLnavSubframeEnum::Almanach4(almanachs.0[p]),
                ),
                (
                    GpsQzssLnavSubframeId::Almanach5,
                    GpsQzssLnavSubframeEnum::Almanach5(almanachs.1[p]),
                ),
            ];

            let subframes: [GpsQzssLnavSubframe; GPS_LNAV_FRAME_ITEMS] =
                core::array::from_fn(|k| {
                    let (sbf_id, subframe) = contents[k];

                    // the HOW carries the TOW of the next subframe
                    let tow = (start_tow_seconds
                        + p as u32 * GPS_LNAV_FRAME_DURATION_S
                        + (k as u32 + 1) * GPS_LNAV_SUBFRAME_DURATION_S)
                        % SECONDS_PER_WEEK;

                    GpsQzssLnavSubframe {
                        telemetry,
                        how: how.with_subframe_id(sbf_id).with_tow_seconds(tow),
                        subframe,
                    }
                });

            // cannot fail: the subframe IDs are in order
            *page = GpsQzssLnavFrame::from_slice(subframes).unwrap();
        }

        Self(pages)
    }

    /// Returns transmission start time of this [GpsQzssLnavSuperframe],
    /// as number of elapsed seconds of that week.
    pub fn week_transmission_time_s(&self) -> u32 {
        self.0[0].eph_1().week_transmission_time_s()
    }

    /// Returns true if this [GpsQzssLnavSuperframe] marks the beginning of a new GPS or QZSS week
    pub fn start_of_week(&self) -> bool {
        self.week_transmission_time_s() == 0
    }

    /// Iterates over the 25 [GpsQzssLnavFrame]s, in transmission order.
    pub fn frames(&self) -> impl Iterator<Item = &GpsQzssLnavFrame> {
        self.0.iter()
    }

    /// Iterates over the 125 [GpsQzssLnavSubframe]s, in transmission order.
    pub fn subframes(&self) -> impl Iterator<Item = &GpsQzssLnavSubframe> {
        self.0.iter().flat_map(|frame| {
            [
                frame.eph_1(),
                frame.eph_2(),
                frame.eph_3(),
                frame.alm_4(),
                frame.alm_5(),
            ]
        })
    }

    /// Returns the 5 [GpsQzssLnavSubframe]s of given page (1..=25).
    ///
    /// ## Input
    ///
    /// - page number: must be within 1..=GPS_LNAV_SUPERFRAME_ITEMS otherwise
    ///   this will panic
    pub fn page(&self, page: usize) -> &GpsQzssLnavFrame {
        &self.0[page - 1]
    }

    /// Iterates over the 25 pages, with their page number (1..=25).
    pub fn pages(&self) -> impl Iterator<Item = (u8, &GpsQzssLnavFrame)> {
        self.0
            .iter()
            .enumerate()
            .map(|(p, frame)| (p as u8 + 1, frame))
    }

    /// Iterates over the 25 subframe 4 pages, with their page number.
    pub fn alm4_pages(&self) -> impl Iterator<Item = (u8, &GpsQzssLnavSubframe4)> {
        self.pages()
            .filter_map(|(page, frame)| Some((page, frame.alm_4().subframe.as_alm4()?)))
    }

    /// Returns the subframe 4 content of given page (1..=25).
    pub fn alm4_by_page(&self, page: u8) -> Option<&GpsQzssLnavSubframe4> {
        self.alm4_pages()
            .find_map(|(p, alm4)| if p == page { Some(alm4) } else { None })
    }

    /// Iterates over the 25 subframe 5 pages, with their page number.
    pub fn alm5_pages(&self) -> impl Iterator<Item = (u8, &GpsQzssLnavSubframe5)> {
        self.pages()
            .filter_map(|(page, frame)| Some((page, frame.alm_5().subframe.as_alm5()?)))
    }

    /// Returns the subframe 5 content of given page (1..=25).
    pub fn alm5_by_page(&self, page: u8) -> Option<&GpsQzssLnavSubframe5> {
        self.alm5_pages()
            .find_map(|(p, alm5)| if p == page { Some(alm5) } else { None })
    }

    /// Returns the [GpsQzssLnavAlmanach] of satellite `prn`, if transmitted in this rotation.
    pub fn almanach(&self, prn: u8) -> Option<&GpsQzssLnavAlmanach> {
        self.almanachs().find(|alm| alm.sv_id == prn)
    }

    /// Iterates over all the [GpsQzssLnavAlmanach]s of this rotation
    /// (subframe 5 pages 1 to 24, then subframe 4 pages 2 to 5 and 7 to 10).
    /// Dummy pages are skipped.
    pub fn almanachs(&self) -> impl Iterator<Item = &GpsQzssLnavAlmanach> {
        self.alm5_pages()
            .filter_map(|(_, f)| f.as_almanach())
            .chain(self.alm4_pages().filter_map(|(_, f)| f.as_almanach()))
            .filter(|alm| alm.sv_id != 0)
    }

    /// Returns the [GpsQzssLnavAlmanachHealth] (subframe 5 page 25).
    pub fn almanach_health_page(&self) -> Option<&GpsQzssLnavAlmanachHealth> {
        self.alm5_pages().find_map(|(_, f)| f.as_health())
    }

    /// Returns the [GpsQzssLnavConfigHealth] (subframe 4 page 25).
    pub fn config_health_page(&self) -> Option<&GpsQzssLnavConfigHealth> {
        self.alm4_pages().find_map(|(_, f)| f.as_config_health())
    }

    /// Returns the [GpsQzssLnavIonosphereUtc] parameters (subframe 4 page 18).
    pub fn ionosphere_utc_page(&self) -> Option<&GpsQzssLnavIonosphereUtc> {
        self.alm4_pages().find_map(|(_, f)| f.as_ionosphere_utc())
    }

    /// Returns the [GpsQzssLnavNmct] (subframe 4 page 13).
    pub fn nmct_page(&self) -> Option<&GpsQzssLnavNmct> {
        self.alm4_pages().find_map(|(_, f)| f.as_nmct())
    }

    /// Returns the [GpsQzssLnavSpecialMessage] (subframe 4 page 17).
    pub fn special_message_page(&self) -> Option<&GpsQzssLnavSpecialMessage> {
        self.alm4_pages().find_map(|(_, f)| f.as_special_message())
    }

    /// Returns the [GpsQzssLnavSatelliteHealth] of satellite `prn` (1..=32), as
    /// transmitted in subframe 5 page 25 (1 to 24) or subframe 4 page 25 (25 to 32).
    pub fn satellite_health_by_prn(&self, prn: u8) -> Option<GpsQzssLnavSatelliteHealth> {
        match prn {
            1..=24 => self.almanach_health_page()?.satellite_health(prn),
            25..=32 => self.config_health_page()?.satellite_health(prn),
            _ => None,
        }
    }

    /// Returns the ephemeris (EPH-1, 2, 3) of the last page of this rotation,
    /// which is the most recent one.
    pub fn ephemeris(
        &self,
    ) -> Option<(
        &GpsQzssLnavSubframe1,
        &GpsQzssLnavSubframe2,
        &GpsQzssLnavSubframe3,
    )> {
        let page = &self.0[GPS_LNAV_SUPERFRAME_ITEMS - 1];
        Some((
            page.eph_1().subframe.as_eph1()?,
            page.eph_2().subframe.as_eph2()?,
            page.eph_3().subframe.as_eph3()?,
        ))
    }

    /// Verifies that this [GpsQzssLnavSuperframe] is a valid 12'30" period:
    /// - it starts with EPH-1 of page 1 ([GpsQzssLnavSubframe::starts_superframe_period]),
    ///   otherwise [GpsError::MessageAlignment] is returned.
    /// - the 125 subframes are in sequence: subframe IDs 1 to 5 and continuous time of week
    ///   ([GpsQzssLnavSuperframeBuilder::follows]), otherwise [GpsError::MessageSequence]
    ///   is returned, with the index (0..125) of the first subframe that breaks the sequence.
    /// - the content of subframes 4 and 5 matches the IS-GPS-200 pagination
    ///   ([GpsQzssLnavSubframe4::matches_page], [GpsQzssLnavSubframe5::matches_page]),
    ///   otherwise [GpsError::UnexpectedPage] is returned (page, subframe).
    ///
    /// This is always true for [GpsQzssLnavSuperframe]s returned by our
    /// [GpsQzssLnavSuperframeBuilder] and [GpsQzssLnavSuperframeDecoder].
    pub fn validate(&self) -> Result<(), GpsError> {
        if !self.0[0].eph_1().starts_superframe_period() {
            return Err(GpsError::MessageAlignment);
        }

        let mut previous: Option<&GpsQzssLnavSubframe> = None;

        for (index, sbf) in self.subframes().enumerate() {
            if let Some(previous) = previous {
                if !GpsQzssLnavSuperframeBuilder::follows(previous, sbf) {
                    return Err(GpsError::MessageSequence(index));
                }
            }
            previous = Some(sbf);
        }

        for (page, frame4) in self.alm4_pages() {
            if !frame4.matches_page(page) {
                return Err(GpsError::UnexpectedPage(page, 4));
            }
        }

        for (page, frame5) in self.alm5_pages() {
            if !frame5.matches_page(page) {
                return Err(GpsError::UnexpectedPage(page, 5));
            }
        }

        Ok(())
    }

    /// Generates a realistic and valid 12'30" period, for testing purposes.
    #[cfg(test)]
    pub(crate) fn model(start_tow_seconds: u32) -> Self {
        let s = Self::from_pages(
            start_tow_seconds,
            (GpsQzssLnavTelemetry::model(), GpsQzssLnavHow::default()),
            (
                GpsQzssLnavSubframe1::model(),
                GpsQzssLnavSubframe2::model(),
                GpsQzssLnavSubframe3::model(),
            ),
            (
                core::array::from_fn(|p| GpsQzssLnavSubframe4::model(p as u8 + 1)),
                core::array::from_fn(|p| GpsQzssLnavSubframe5::model(p as u8 + 1)),
            ),
        );

        s.validate().unwrap_or_else(|e| {
            panic!("super frame model is not a valid 12'30'' period: {}", e);
        });

        s
    }
}

impl Message<BigEndian> for GpsQzssLnavSuperframe {
    fn encoding_size(&self) -> usize {
        GPS_LNAV_SUPERFRAME_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GPS_LNAV_SUPERFRAME_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssLnavSuperframe {
    /// Encodes this [GpsQzssLnavSuperframe] (completely: 125 contiguous subframes)
    /// into any [BitWrite] buffer.
    /// [GpsQzssLnavSuperframe::encoding_size] must fit completely in provided buffer
    /// for this method to work.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        for frame in self.0.iter() {
            stream.write(frame)?;
        }
        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavSuperframe {
    /// Reads the first complete 12'30" period that exists in provided buffer.
    /// This method automatically locks itself to the first EPH-1 of page 1
    /// encountered (any bit alignment, normal or inverted stream), skips
    /// noise and gaps between subframes, and restarts on any discontinuity,
    /// which is compatible with real-word recordings.
    ///
    /// On success, the stream is positioned right after the last subframe.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let mut decoder = GpsQzssLnavSuperframeDecoder::default();

        loop {
            if stream.bits_left() == 0 {
                let collected = decoder.builder().total_subframes();
                return Err(BitError::NotEnoughData {
                    requested: GPS_LNAV_SUPERFRAME_BITS - collected * GPS_LNAV_SUBFRAME_BITS,
                    bits_left: 0,
                });
            }

            if let Some(spf) = decoder.push_bit(stream.read_bool()?) {
                return Ok(spf);
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use crate::{
        bits::read_bits,
        gps::lnav::{
            test_vectors::bytes_to_words, GpsQzssLnavReservedSubframe, GPS_LNAV_FRAME_BITS,
            GPS_LNAV_FRAME_BYTES, GPS_LNAV_SUBFRAME_BITS, GPS_LNAV_SUPERFRAME_DURATION_S,
        },
        BitReadBuffer,
    };

    /// Pseudo random bit generator
    struct Noise(u16);

    impl Noise {
        fn bit(&mut self) -> bool {
            self.0 = (self.0 >> 1) ^ (-((self.0 & 1) as i16) as u16 & 0xB400);
            self.0 & 0x1 > 0
        }
    }

    fn encode(message: &GpsQzssLnavSuperframe) -> Vec<u8> {
        let mut bytes = vec![0u8; GPS_LNAV_SUPERFRAME_BYTES];
        assert_eq!(
            message.encode(&mut bytes).unwrap(),
            GPS_LNAV_SUPERFRAME_BYTES
        );
        bytes
    }

    /// Rebuilds a [GpsQzssLnavSuperframe] from 125 subframes, without validation.
    fn from_subframes(subframes: &[GpsQzssLnavSubframe]) -> GpsQzssLnavSuperframe {
        assert_eq!(
            subframes.len(),
            GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS
        );
        GpsQzssLnavSuperframe(core::array::from_fn(|p| {
            let mut slice = [GpsQzssLnavSubframe::default(); GPS_LNAV_FRAME_ITEMS];
            slice.copy_from_slice(
                &subframes[p * GPS_LNAV_FRAME_ITEMS..(p + 1) * GPS_LNAV_FRAME_ITEMS],
            );
            GpsQzssLnavFrame::from_slice(slice).unwrap()
        }))
    }

    fn subframes(message: &GpsQzssLnavSuperframe) -> Vec<GpsQzssLnavSubframe> {
        message.subframes().copied().collect()
    }

    #[test]
    fn spf_model_validity() {
        for start in [0, 750, 2 * 750, 16 * 750, 603_750] {
            let spf = GpsQzssLnavSuperframe::model(start);

            assert_eq!(spf.week_transmission_time_s(), start);
            assert_eq!(spf.start_of_week(), start == 0);
            assert_eq!(spf.frames().count(), GPS_LNAV_SUPERFRAME_ITEMS);
            assert_eq!(
                spf.subframes().count(),
                GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS
            );

            for (page, frame) in spf.pages() {
                // subframes know their pagination
                for (k, sbf) in [
                    frame.eph_1(),
                    frame.eph_2(),
                    frame.eph_3(),
                    frame.alm_4(),
                    frame.alm_5(),
                ]
                .iter()
                .enumerate()
                {
                    assert_eq!(sbf.page_number(), page, "start={} page={}", start, page);
                    assert_eq!(sbf.how.sbf_id.encode() as usize, k + 1);
                    assert_eq!(sbf.subframe.frame_id().encode() as usize, k + 1);
                    assert_eq!(
                        sbf.week_transmission_time_s(),
                        start + 30 * (page as u32 - 1) + 6 * k as u32
                    );
                }
            }

            let subframes = subframes(&spf);
            assert!(subframes[0].starts_superframe_period());
            assert!(subframes[124].ends_superframe_period());
        }
    }

    #[test]
    fn reciprocal() {
        for tow_s in [0, 750, 750 * 7, 750 * 9] {
            let message = GpsQzssLnavSuperframe::model(tow_s);
            let bytes = encode(&message);

            // 125 contiguous subframes, each starting with the preamble
            for k in 0..GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS {
                assert_eq!(read_bits(&bytes, k * GPS_LNAV_SUBFRAME_BITS, 8), Some(0x8B));
            }

            assert_eq!(bytes.len(), GPS_LNAV_SUPERFRAME_BYTES);

            let decoded = GpsQzssLnavSuperframe::decode(&bytes).unwrap();
            assert_eq!(decoded, message);

            // bit exact
            assert_eq!(encode(&decoded), bytes);
        }
    }

    #[test]
    fn frames_are_bit_exact() {
        // the message is the concatenation of its frames
        let message = GpsQzssLnavSuperframe::model(0);
        let bytes = encode(&message);

        for (k, frame) in message.frames().enumerate() {
            let mut expected = [0u8; GPS_LNAV_FRAME_BYTES];
            frame.encode(&mut expected).unwrap();

            for bit in 0..GPS_LNAV_FRAME_BITS {
                assert_eq!(
                    read_bits(&bytes, k * GPS_LNAV_FRAME_BITS + bit, 1),
                    read_bits(&expected, bit, 1),
                    "frame #{} bit #{}",
                    k,
                    bit
                );
            }
        }

        for sbf in message.subframes() {
            let mut expected = [0u8; 40];
            sbf.encode(&mut expected).unwrap();
            let words = bytes_to_words(&expected);
            assert!(GpsQzssLnavSubframe::from_words(&words, true).is_ok());
        }
    }

    #[test]
    fn buffer_too_small() {
        let message = GpsQzssLnavSuperframe::model(0);

        let mut bytes = vec![0u8; GPS_LNAV_SUPERFRAME_BYTES - 1];
        assert!(message.encode(&mut bytes).is_err());

        let bytes = encode(&message);
        assert!(GpsQzssLnavSuperframe::decode(&bytes[..GPS_LNAV_SUPERFRAME_BYTES - 2]).is_err());
        assert!(GpsQzssLnavSuperframe::decode(&[]).is_err());
    }

    #[test]
    fn gaps_and_noise() {
        let message = GpsQzssLnavSuperframe::model(750 * 3);

        for (leading, gap) in [(0, 0), (5, 0), (0, 3), (17, 1), (123, 9)] {
            let mut noise = Noise(0xACE1);
            let mut bytes = Vec::new();

            {
                let mut writer = BitWriteStream::new(&mut bytes, BigEndian);

                for _ in 0..leading {
                    writer.write_bool(noise.bit()).unwrap();
                }

                for sbf in message.subframes() {
                    writer.write(sbf).unwrap();
                    for _ in 0..gap {
                        writer.write_bool(noise.bit()).unwrap();
                    }
                }

                writer.write_int(0u8, 8).unwrap();
            }

            let mut reader = BitReadStream::new(BitReadBuffer::new(&bytes, BigEndian));
            let decoded: GpsQzssLnavSuperframe = reader.read().unwrap_or_else(|e| {
                panic!("leading={} gap={}: {}", leading, gap, e);
            });

            assert_eq!(decoded, message, "leading={} gap={}", leading, gap);
        }
    }

    #[test]
    fn reading_starts_at_page_1() {
        // the stream starts in the middle of a rotation: the end of the previous rotation
        // is skipped, the next complete rotation is returned
        let previous = GpsQzssLnavSuperframe::model(0);
        let message = GpsQzssLnavSuperframe::model(750);

        for (skip, offset) in [(1, 0), (37, 3), (124, 11)] {
            let mut bytes = Vec::new();

            {
                let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
                writer.write_int(0u16, offset).unwrap();

                for sbf in previous.subframes().skip(skip) {
                    writer.write(sbf).unwrap();
                }

                writer.write(&message).unwrap();
            }

            let mut reader = BitReadStream::new(BitReadBuffer::new(&bytes, BigEndian));
            let decoded: GpsQzssLnavSuperframe = reader.read().unwrap();

            assert_eq!(decoded, message, "skip={}", skip);
            assert_eq!(decoded.week_transmission_time_s(), 750);

            // stream positioned right after the last subframe
            assert_eq!(reader.bits_left(), bytes.len() * 8 - reader.pos());
            assert_eq!(
                reader.pos(),
                offset
                    + (GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS - skip)
                        * GPS_LNAV_SUBFRAME_BITS
                    + GPS_LNAV_SUPERFRAME_BITS
            );
        }
    }

    #[test]
    fn reading_restarts_on_discontinuity() {
        // a subframe is missing in the first rotation: it is discarded
        let first = GpsQzssLnavSuperframe::model(0);
        let second = GpsQzssLnavSuperframe::model(750);

        let write_first = |writer: &mut BitWriteStream<'_, BigEndian>| {
            for (k, sbf) in first.subframes().enumerate() {
                if k != 60 {
                    writer.write(sbf).unwrap();
                }
            }
        };

        let mut bytes = Vec::new();
        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            write_first(&mut writer);
            writer.write(&second).unwrap();
        }

        let decoded = GpsQzssLnavSuperframe::decode(&bytes).unwrap();
        assert_eq!(decoded, second);

        // the first rotation alone is not enough
        let mut bytes = Vec::new();
        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            write_first(&mut writer);
        }

        assert!(GpsQzssLnavSuperframe::decode(&bytes).is_err());
    }

    #[test]
    fn corrupted_subframe_restarts() {
        let first = GpsQzssLnavSuperframe::model(0);
        let second = GpsQzssLnavSuperframe::model(750);

        let mut bytes = encode(&first);

        // corrupt one bit of subframe #99 of the first rotation: parity error
        let bit = 99 * GPS_LNAV_SUBFRAME_BITS + 100;
        bytes[bit / 8] ^= 0x80 >> (bit % 8);
        assert!(GpsQzssLnavSuperframe::decode(&bytes).is_err());

        let mut stream = Vec::new();
        {
            let mut writer = BitWriteStream::new(&mut stream, BigEndian);
            writer
                .write_bytes(&bytes[..GPS_LNAV_SUPERFRAME_BYTES - 1])
                .unwrap();
            writer.write(&second).unwrap();
        }

        assert_eq!(GpsQzssLnavSuperframe::decode(&stream).unwrap(), second);
    }

    #[test]
    fn week_end() {
        // 604800 is not a multiple of 750: the last rotation of the week
        // (starting at 604500) is interrupted after 10 pages.
        let last_complete = GpsQzssLnavSuperframe::model(603_750);
        last_complete.validate().unwrap();

        let last_sbf = subframes(&last_complete)[124];
        assert_eq!(last_sbf.week_transmission_time_s(), 604_494);
        assert_eq!(last_sbf.how.tow, 604_500);

        // the interrupted rotation, followed by the first rotation of the next week
        let interrupted = GpsQzssLnavSuperframe::from_pages(
            604_500,
            (GpsQzssLnavTelemetry::model(), GpsQzssLnavHow::default()),
            (
                GpsQzssLnavSubframe1::model(),
                GpsQzssLnavSubframe2::model(),
                GpsQzssLnavSubframe3::model(),
            ),
            (
                core::array::from_fn(|p| GpsQzssLnavSubframe4::model(p as u8 + 1)),
                core::array::from_fn(|p| GpsQzssLnavSubframe5::model(p as u8 + 1)),
            ),
        );

        let interrupted: Vec<_> = subframes(&interrupted).into_iter().take(50).collect();
        assert!(interrupted[0].starts_superframe_period());
        assert_eq!(interrupted[49].week_transmission_time_s(), 604_794);
        assert_eq!(interrupted[49].how.tow, 0);

        let next_week = GpsQzssLnavSuperframe::model(0);
        let first = subframes(&next_week)[0];
        assert!(first.starts_superframe_period());
        assert_eq!(first.week_transmission_time_s(), 0);

        // continuity across the end of the week
        assert!(GpsQzssLnavSuperframeBuilder::follows(
            &interrupted[49],
            &first
        ));

        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        let mut output = Vec::new();

        for sbf in interrupted.iter().chain(next_week.subframes()) {
            if let Some(spf) = builder.push_sbf(*sbf) {
                output.push(spf);
            }
        }

        assert_eq!(output, vec![next_week]);
    }

    #[test]
    fn validation() {
        let message = GpsQzssLnavSuperframe::model(1500);
        message.validate().unwrap();

        let frames: [GpsQzssLnavFrame; GPS_LNAV_SUPERFRAME_ITEMS] =
            core::array::from_fn(|p| *message.nth_frame(p));
        assert_eq!(
            GpsQzssLnavSuperframe::from_frame_slice(frames).unwrap(),
            message
        );

        // misaligned
        let misaligned = GpsQzssLnavSuperframe::from_pages(
            1530,
            (GpsQzssLnavTelemetry::model(), GpsQzssLnavHow::default()),
            (
                GpsQzssLnavSubframe1::model(),
                GpsQzssLnavSubframe2::model(),
                GpsQzssLnavSubframe3::model(),
            ),
            (
                core::array::from_fn(|p| GpsQzssLnavSubframe4::model(p as u8 + 1)),
                core::array::from_fn(|p| GpsQzssLnavSubframe5::model(p as u8 + 1)),
            ),
        );
        assert!(matches!(
            misaligned.validate(),
            Err(GpsError::MessageAlignment)
        ));

        // time discontinuity
        let mut broken = subframes(&message);
        broken[17].how.tow += 6;
        assert!(matches!(
            from_subframes(&broken).validate(),
            Err(GpsError::MessageSequence(17))
        ));

        // repeated subframe (same ID and time)
        let mut broken = subframes(&message);
        broken[17] = broken[16];
        broken[17].how.sbf_id = GpsQzssLnavSubframeId::Ephemeris3;
        assert!(matches!(
            from_subframes(&broken).validate(),
            Err(GpsError::MessageSequence(17))
        ));

        // a configuration page on page 18 of subframe 4
        let mut broken = subframes(&message);
        broken[17 * 5 + 3] =
            broken[17 * 5 + 3].with_subframe_enum(GpsQzssLnavSubframeEnum::Almanach4(
                GpsQzssLnavSubframe4::ConfigHealth(Default::default()),
            ));
        assert!(matches!(
            from_subframes(&broken).validate(),
            Err(GpsError::UnexpectedPage(18, 4))
        ));

        // page 25 of subframe 5 must carry the health page
        let mut broken = subframes(&message);
        broken[24 * 5 + 4] =
            broken[24 * 5 + 4].with_subframe_enum(GpsQzssLnavSubframeEnum::Almanach5(
                GpsQzssLnavSubframe5::Reserved(GpsQzssLnavReservedSubframe::model(57)),
            ));
        assert!(matches!(
            from_subframes(&broken).validate(),
            Err(GpsError::UnexpectedPage(25, 5))
        ));

        // a dummy page instead of an almanac is fine
        let mut dummy = subframes(&message);
        dummy[6 * 5 + 4] = dummy[6 * 5 + 4].with_subframe_enum(GpsQzssLnavSubframeEnum::Almanach5(
            GpsQzssLnavSubframe5::default(),
        ));
        let dummy = from_subframes(&dummy);
        dummy.validate().unwrap();
        assert!(dummy.almanach(7).is_none());
        assert_eq!(dummy.almanachs().count(), 31);
    }

    #[test]
    fn accessors() {
        let message = GpsQzssLnavSuperframe::model(0);

        // 24 + 8 almanacs, for PRN 1 to 32
        assert_eq!(message.almanachs().count(), 32);

        for prn in 1..=32 {
            let almanach = message.almanach(prn).unwrap();
            assert_eq!(almanach.sv_id, prn);
        }

        assert!(message.almanach(33).is_none());

        // subframe 4 and 5 pages
        assert_eq!(message.alm4_by_page(18).unwrap().sv_id(), 56);
        assert_eq!(message.alm5_by_page(25).unwrap().sv_id(), 51);
        assert_eq!(message.alm4_by_page(0), None);
        assert_eq!(message.alm5_by_page(26), None);
        assert_eq!(message.alm4_pages().count(), 25);
        assert_eq!(message.alm5_pages().count(), 25);

        assert_eq!(
            message.ionosphere_utc_page(),
            Some(&GpsQzssLnavIonosphereUtc::model())
        );
        assert_eq!(message.nmct_page(), Some(&GpsQzssLnavNmct::model()));
        assert_eq!(
            message.special_message_page().unwrap().text(),
            Some("GPS IS-GPS-200 NAV MSG")
        );
        assert_eq!(
            message.config_health_page(),
            Some(&GpsQzssLnavConfigHealth::model())
        );
        assert_eq!(
            message.almanach_health_page(),
            Some(&GpsQzssLnavAlmanachHealth::model())
        );

        // health of the whole constellation
        for prn in 1..=32 {
            let health = message.satellite_health_by_prn(prn).unwrap();
            assert_eq!(health.is_healthy(), prn != 21 && prn != 27, "prn {}", prn);
        }

        assert!(message.satellite_health_by_prn(0).is_none());
        assert!(message.satellite_health_by_prn(33).is_none());

        let (eph1, eph2, eph3) = message.ephemeris().unwrap();
        assert_eq!(*eph1, GpsQzssLnavSubframe1::model());
        assert_eq!(*eph2, GpsQzssLnavSubframe2::model());
        assert_eq!(*eph3, GpsQzssLnavSubframe3::model());

        assert_eq!(
            message.page(1).eph_1().subframe.frame_id(),
            GpsQzssLnavSubframeId::Ephemeris1
        );
        assert_eq!(message.page(25).alm_5().page_number(), 25);
        assert_eq!(GPS_LNAV_SUPERFRAME_DURATION_S, 750);
    }

    #[test]
    fn builder() {
        let previous = GpsQzssLnavSuperframe::model(0);
        let message = GpsQzssLnavSuperframe::model(750);

        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        assert!(builder.is_empty());
        assert_eq!(builder.seconds_to_end_of_period(), None);

        // the end of the previous rotation is ignored
        for sbf in previous.subframes().skip(50) {
            assert!(builder.push_sbf(*sbf).is_none());
            assert!(builder.is_empty());
        }

        let mut output = None;
        let total = GPS_LNAV_SUPERFRAME_ITEMS * GPS_LNAV_FRAME_ITEMS;

        for (k, sbf) in message.subframes().enumerate() {
            output = builder.push_sbf(*sbf);

            if k < total - 1 {
                assert!(output.is_none());
                assert_eq!(builder.total_subframes(), k + 1);
                assert_eq!(builder.total_frames(), (k + 1) / 5);
                assert_eq!(
                    builder.seconds_to_end_of_period(),
                    Some(750 - 6 * (k as u32 + 1))
                );
            }
        }

        assert_eq!(output, Some(message));
        assert!(builder.is_empty());

        // a missing subframe restarts the collection
        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        for sbf in message.subframes().take(30) {
            builder.push_sbf(*sbf);
        }
        assert_eq!(builder.total_subframes(), 30);
        builder.push_sbf(*message.subframes().nth(31).unwrap());
        assert!(builder.is_empty());

        builder.push_sbf(*message.subframes().next().unwrap());
        assert_eq!(builder.total_subframes(), 1);
        builder.reset();
        assert!(builder.is_empty());

        // 30s scale
        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        let mut output = Vec::new();

        for frame in previous.frames().chain(message.frames()) {
            if let Some(spf) = builder.push_frame(*frame) {
                output.push(spf);
            }
        }

        assert_eq!(output, vec![previous, message]);
    }

    #[test]
    fn builder_with_decoder() {
        use crate::gps::lnav::GpsQzssLnavSubfDecoder;

        // a raw bit stream of 2 rotations, decoded subframe by subframe
        let first = GpsQzssLnavSuperframe::model(750 * 10);
        let second = GpsQzssLnavSuperframe::model(750 * 11);

        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            writer.write_int(0u8, 3).unwrap();
            writer.write(&first).unwrap();
            writer.write(&second).unwrap();
            writer.write_int(0u8, 8).unwrap();
        }

        let mut decoder = GpsQzssLnavSubfDecoder::default();
        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        let mut messages = Vec::new();
        let mut position = 0;

        loop {
            let (consumed, sbf) = decoder.decode(&bytes, position);
            position += consumed;

            match sbf {
                Some(sbf) => {
                    if let Some(message) = builder.push_sbf(sbf) {
                        messages.push(message);
                    }
                },
                None => break,
            }
        }

        assert_eq!(messages, vec![first, second]);
    }
}
