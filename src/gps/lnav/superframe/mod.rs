//! GPS 12'30" message (25 pages of 5 subframes)

use crate::{
    constants::SECONDS_PER_WEEK,
    gps::lnav::{
        GpsError, GpsQzssLnavAlmanach, GpsQzssLnavAlmanachHealth, GpsQzssLnavConfigHealth, GpsQzssLnavSubframe,
        GpsQzssLnavHow, GpsQzssLnavIonosphereUtc, GpsQzssLnavNmct, GpsQzssLnavSatelliteHealth,
        GpsQzssLnavSpecialMessage, GpsQzssLnavTelemetry, GPS_LNAV_FRAME_BITS, GPS_LNAV_FRAME_ITEMS,
        GPS_LNAV_SUBFRAME_DURATION_S, GPS_LNAV_SUPERFRAME_BITS, GPS_LNAV_SUPERFRAME_BYTES, GPS_LNAV_SUPERFRAME_ITEMS,
        GpsQzssLnavSubframe4,
        GpsQzssLnavSubframe1,
        GpsQzssLnavSubframe2,
        GpsQzssLnavSubframe3,
        GpsQzssLnavSubframe5,
        GpsQzssLnavSubframeEnum,
        GpsQzssLnavFrame,
        GPS_LNAV_SUPERFRAME_DURATION_S,
    },
    BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream, Message,
};

// mod builder;

// pub use builder::*;

/// [GpsQzssLnavSuperFrame] is a convenient structures to work at the
/// GPS/QZSS Alamanach level, which is described by 25 [GpsQzssFrame]s collected in 12'30s.
/// Like any other structures, it supports both encoding & decoding.
///
/// The rotation always starts with EPH-1 of page 1, at a multiple of 12'30" since the
/// beginning of the week (the page number is defined by the time of transmission, see
/// [GpsQzssLnavSubframe::page_number]), so the page of each subframe is defined by its position.
/// The first period of each day is aligned to sunday midnight GPS week.
///
/// To understand Almanach pages content, refer to [GpsQzssLnavSubframe4] and [GpsQzssLnavSubframe5]):
/// - SF.5 p1-24 (SVID=1-24) gives almanach per satellite ([GpsQzssLnavAlmanach])
/// - SF.5 p25 (SVID=51) gives ToA, WNa (8bits), health for SV1-24 ([GpsQzssLnavAlmanachHealth])
/// - SF.4 p2-5, p7-10 (SVID=25-32) gives almanach per satellite ([GpsQzssLnavAlmanach])
/// - SF.4 p18 (SVID=56) Ionosphere Kb alpha,beta and UTC (a0, a1, ToT, WNt..) ([GpsQzssLnavIonosphereUtc])
/// - SF.4 p25 (SVID=63) 4 bits per SV: anti-spoofing + signal block type, health for SV25-32 ([GpsQzssLnavConfigHealth]).
/// - SF.4 p13 (SVID=52) NMCT: ERD distance correction ([GpsQzssLnavNmct])
/// - SF.4 p17 (SVID=55) Special message (22 ASCII bytes) ([GpsQzssLnavSpecialMessage])
/// - SF.4 p1, 6, 11, 12, 14, 15, 16, 19-24 (SVID=53,54,57-62) reserved ([crate::gps::GpsQzssLnavReservedPage])
///
/// ```
/// use gnss_protos::gps::lnav::{
///     GpsQzssLnavAlmanach, GpsQzssLnavAlmanachHealth, GpsQzssLnavSubframe1, GpsQzssLnavSubframe2, GpsQzssLnavSubframe3,
///     GpsQzssLnavSubframe4, GpsQzssLnavSubframe5, GpsQzssLnavHow, GpsQzssLnavSuperframe, GpsQzssLnavReservedPage,
///     GpsQzssLnavTelemetry, Message,
/// };
///
/// // the content of the 25 pages of subframes 4 and 5 (IS-GPS-200 pagination)
/// let frame4: [GpsQzssLnavSubframe4; 25] = core::array::from_fn(|p| {
///     let page = p as u8 + 1;
///     let sv_id = GpsQzssLnavSubframe4::page_sv_id(page).unwrap();
///     if GpsQzssLnavSubframe4::is_almanach_page(page) {
///         GpsQzssLnavSubframe4::Almanach(
///             GpsQzssLnavAlmanach::default()
///                 .with_sv_id(sv_id)
///                 .with_orbit(5153.6, 0.01, 0.1 * page as f64, 0.2, 0.3),
///         )
///     } else {
///         GpsQzssLnavSubframe4::Reserved(GpsQzssLnavReservedPage::default().with_sv_id(sv_id))
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
/// assert_eq!(decoded, message);
///
/// // the almanac of the 32 satellites
/// assert_eq!(decoded.almanachs().count(), 32);
///
/// let almanach = decoded.almanach(27).unwrap();
/// let (x, y, z) = almanach.position_ecef_m(almanach.time_from_reference_s(3600.0));
/// let radius = (x * x + y * y + z * z).sqrt();
/// assert!(radius > 26_000e3 && radius < 27_000e3);
///
/// assert!(decoded.satellite_health(30).unwrap().is_healthy());
/// ```
#[derive(Debug, Copy, Clone, PartialEq, Default)]
pub struct GpsQzssLnavSuperframe([GpsQzssLnavframe; GPS_LNAV_SUPERFRAME_ITEMS]);

impl GpsQzssLnavSuperframe {
    /// Builds a [GpsQzssLnavSuperframe] from a sequence of [GpsQzssLnavSubframe]s (30s recordings)
    /// which must follow the correct almanach pagination.
    /// Currently, this framework only verifies the subframe classes progression,
    /// not the temporal progression.
    ///
    /// To facilitate real world-applications,
    /// we propose [GpsQzssLnavSubframeBuilder] to collect the subframe sequence.
    /// If any component of the proposed slice here were collected using
    /// our [GpsQzssLnavSubframeBuilder], the proposed slice is garanteed to be correct
    /// and this method will never return an error.
    pub fn from_frame_slice(slice: [GpsQzssLnavSubframe; GPS_LNAV_SUPERFRAME_ITEMS]) -> Result<Self, GpsError> {
        let s = Self(slice);
        s.validate_period()?;
        Ok(s)
    }

    /// Returns the underlying [GpsQzssLnavSubframe].
    ///
    /// ## Input
    ///
    /// - page: must be < GPS_LNAV_SUPERFRAME_ITEMS, otherwise this will panic.
    pub fn nth_frame(&self, index: usize) -> &GpsQzssLnavSubframe {
        &self.0[index]
    }

    // /// Builds a coherent [GpsQzssLnavSuperframe] from its content: the rotation starts at
    // /// `start_tow_seconds` (a multiple of 750 s, see [GpsQzssLnavSubframe::starts_rotation]),
    // /// every subframe uses the `header` [GpsQzssLnavTelemetry] and [GpsQzssLnavHow] (alert and
    // /// anti-spoofing flags), with its own time of week and frame ID,
    // /// the `ephemeris` are repeated on every page, and `almanach` gives
    // /// the content of the 25 pages of subframes 4 and 5.
    // ///
    // /// The resulting message is valid ([Self::validate]) as long as the start time
    // /// is aligned and the pages contents match the IS-GPS-200 pagination
    // /// ([GpsQzssLnavSubframe4::matches_page], [GpsQzssLnavSubframe5::matches_page]).
    // pub fn from_pages(
    //     start_tow_seconds: u32,
    //     header: (GpsQzssLnavTelemetry, GpsQzssLnavHow),
    //     ephemeris: (GpsQzssLnavSubframe1, GpsQzssLnavSubframe2, GpsQzssLnavSubframe3),
    //     almanach: (
    //         [GpsQzssLnavSubframe4; GPS_QZSS_ALMANACH_PAGES],
    //         [GpsQzssLnavSubframe5; GPS_QZSS_ALMANACH_PAGES],
    //     ),
    // ) -> Self {
    //     let (telemetry, how) = header;
    //     let (eph1, eph2, eph3) = ephemeris;
    //     let (frame4, frame5) = almanach;

    //     let mut pages =
    //         [[GpsQzssLnavSubframe::default(); GPS_LNAV_SUPERFRAME_ITEMS]; GPS_QZSS_ALMANACH_PAGES];

    //     for (p, page) in pages.iter_mut().enumerate() {
    //         let subframes = [
    //             GpsQzssLnavSubframe::Ephemeris1(eph1),
    //             GpsQzssLnavSubframe::Ephemeris2(eph2),
    //             GpsQzssLnavSubframe::Ephemeris3(eph3),
    //             GpsQzssLnavSubframe::Almanach4(frame4[p]),
    //             GpsQzssLnavSubframe::Almanach5(frame5[p]),
    //         ];

    //         for (k, (frame, subframe)) in page.iter_mut().zip(subframes.iter()).enumerate() {
    //             let index = (p * GPS_LNAV_SUPERFRAME_ITEMS + k + 1) as u32;
    //             let tow = (start_tow_seconds + index * GPS_LNAV_SUBFRAME_DURATION_S) % SECONDS_PER_WEEK;

    //             *frame = GpsQzssLnavSubframe::default()
    //                 .with_telemetry(telemetry)
    //                 .with_hand_over_word(how.with_tow_seconds(tow))
    //                 .with_subframe(*subframe);
    //         }
    //     }

    //     Self(pages)
    // }

    /// Returns the 5 [GpsQzssLnavSubframe]s of given page (1..=25).
    ///
    /// ## Input
    ///
    /// - page number: must be <= GPS_LNAV_SUPERFRAME_ITEMS otherwise
    /// this will panic
    pub fn page(&self, page: usize) -> &GpsQzssLnavSubframe {
        &self.0[page]
    }

    /// Returns transmission start time in seconds of current week.
    pub fn week_transmission_time_s(&self) -> u32 {
        self.0[0].week_transmission_time_s()
    }

    /// Returns true if this [GpsQzssLnavSuperframe] marks the beginning of a new GPS or QZSS week
    pub fn start_of_week(&self) -> bool {
        self.week_transmission_time_s() == 0
    }

    /// Returns the subframe 4 content of given page (1..=25).
    pub fn frame4(&self, page: u8) -> Option<&GpsQzssLnavSubframe4> {
        match &self.page(page)?[3].subframe {
            GpsQzssLnavSubframeEnum::Almanach4(frame4) => Some(frame4),
            _ => None,
        }
    }

    /// Returns the subframe 5 content of given page (1..=25).
    pub fn frame5(&self, page: u8) -> Option<&GpsQzssLnavSubframe5> {
        match &self.page(page)?[4].subframe {
            GpsQzssLnavSubframeEnum::Almanach5(frame5) => Some(frame5),
            _ => None,
        }
    }

    /// Iterates over the 25 subframe 4 pages, with their page number.
    pub fn frames4(&self) -> impl Iterator<Item = (u8, &GpsQzssLnavSubframe4)> {
        (1..=GPS_LNAV_SUPERFRAME_ITEMS as u8).filter_map(move |p| Some((p, self.frame4(p)?)))
    }

    /// Iterates over the 25 subframe 5 pages, with their page number.
    pub fn frames5(&self) -> impl Iterator<Item = (u8, &GpsQzssLnavSubframe5)> {
        (1..=GPS_LNAV_SUPERFRAME_ITEMS as u8).filter_map(move |p| Some((p, self.frame5(p)?)))
    }

    /// Returns the [GpsQzssLnavAlmanach] of satellite `prn`, if transmitted in this rotation.
    pub fn almanach(&self, prn: u8) -> Option<&GpsQzssLnavAlmanach> {
        self.almanachs().find(|alm| alm.sv_id == prn)
    }

    /// Iterates over all the [GpsQzssLnavAlmanach]s of this rotation
    /// (subframe 5 pages 1 to 24, then subframe 4 pages 2 to 5 and 7 to 10).
    /// Dummy pages are skipped.
    pub fn almanachs(&self) -> impl Iterator<Item = &GpsQzssLnavAlmanach> {
        self.frames5()
            .filter_map(|(_, f)| f.as_almanach())
            .chain(self.frames4().filter_map(|(_, f)| f.as_almanach()))
    }

    /// Returns the [GpsQzssLnavAlmanachHealth] (subframe 5 page 25).
    pub fn almanach_health(&self) -> Option<&GpsQzssLnavAlmanachHealth> {
        self.frames5().find_map(|(_, f)| f.as_health())
    }

    /// Returns the [GpsQzssLnavConfigHealth] (subframe 4 page 25).
    pub fn config_health(&self) -> Option<&GpsQzssLnavConfigHealth> {
        self.frames4().find_map(|(_, f)| f.as_config_health())
    }

    /// Returns the [GpsQzssLnavIonosphereUtc] parameters (subframe 4 page 18).
    pub fn ionosphere_utc(&self) -> Option<&GpsQzssLnavIonosphereUtc> {
        self.frames4().find_map(|(_, f)| f.as_ionosphere_utc())
    }

    /// Returns the [GpsQzssLnavNmct] (subframe 4 page 13).
    pub fn nmct(&self) -> Option<&GpsQzssLnavNmct> {
        self.frames4().find_map(|(_, f)| f.as_nmct())
    }

    /// Returns the [GpsQzssLnavSpecialMessage] (subframe 4 page 17).
    pub fn special_message(&self) -> Option<&GpsQzssLnavSpecialMessage> {
        self.frames4().find_map(|(_, f)| f.as_special_message())
    }

    /// Returns the [GpsQzssLnavSatelliteHealth] of satellite `prn` (1..=32), as
    /// transmitted in subframe 5 page 25 (1 to 24) or subframe 4 page 25 (25 to 32).
    pub fn satellite_health(&self, prn: u8) -> Option<GpsQzssLnavSatelliteHealth> {
        match prn {
            1..=24 => self.almanach_health()?.satellite_health(prn),
            25..=32 => self.config_health()?.satellite_health(prn),
            _ => None,
        }
    }

    // /// Returns the ephemeris (EPH-1, 2, 3) of the last page of this rotation.
    // pub fn ephemeris(&self) -> Option<(GpsQzssLnavSubframe1, GpsQzssLnavSubframe2, GpsQzssLnavSubframe3)> {
    //     let page = &self.0[GPS_LNAV_SUPERFRAME_ITEMS- 1];
    //     Some((
    //         page[0].subframe.as_eph1()?,
    //         page[1].subframe.as_eph2()?,
    //         page[2].subframe.as_eph3()?,
    //     ))
    // }

    // /// This method is used when constructed from a raw data slice (raw parts)
    // /// collected by the user itself, and verifies the slice forms a valid GPS period.
    // ///
    // /// Use this method Verifies that this [GpsQzssLnavSuperframe] is a valid 12'30" period:
    // /// - it starts with EPH-1 of page 1 ([GpsQzssLnavSubframe::starts_rotation])
    // /// - subframes are in sequence: subframe IDs 1 to 5 and continuous time of week
    // /// - the content of subframes 4 and 5 matches the IS-GPS-200 pagination
    // ///   ([GpsQzssLnavSubframe4::matches_page], [GpsQzssLnavSubframe5::matches_page])
    // ///
    // /// This can only be true for [GpsQzssLnavSuperframe]s built using our 
    // /// [GpsQzssLnavSuperframeBuilder].
    // fn validate_period(&self) -> Result<(), GpsError> {

    //     if self.0[0].page_number() != 0 {
    //         return Err(GpsError::MessageAlignment);
    //     }

    //     for (index, frame) in self.frames().enumerate() {
    //         if let Some(previous) = previous {
    //             if !GpsQzssLnavSuperframeBuilder::follows(previous, frame) {
    //                 return Err(GpsError::MessageSequence(index));
    //             }
    //         }
    //         previous = Some(frame);
    //     }

    //     for (page, frame4) in self.frames4() {
    //         if !frame4.matches_page(page) {
    //             return Err(GpsError::UnexpectedPage(page, 4));
    //         }
    //     }

    //     for (page, frame5) in self.frames5() {
    //         if !frame5.matches_page(page) {
    //             return Err(GpsError::UnexpectedPage(page, 5));
    //         }
    //     }

    //     Ok(())
    // }

    /// Generates a realistic and valid 12'30" period, for testing purposes.
    #[cfg(test)]
    pub(crate) fn model(start_tow_seconds: u32) -> Self {
        let s = Self::from_pages(
            start_tow_seconds,
            (
                GpsQzssLnavTelemetry::model(),
                GpsQzssLnavHow::model(crate::gps::GpsQzssLnavSubframeId::Ephemeris1),
            ),
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

        s.validate_period()
            .unwrap_or_else(|e| {
                panic!("super frame simulation model did not create a valid 12'30'' period");
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
    /// Encodes this [GpsQzssLnavSuperframe] (completely) into any [BitWrite] buffer.
    /// [GpsQzssLnavSuperframe::encoding_size] must fit completely in provided buffer
    /// for this method to work.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        for i in 0..GPS_LNAV_SUPERFRAME_ITEMS {
            stream.write(&self.0[i])?;
        }
        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssLnavSuperframe {
    /// Reads the 12'30" rotation that must exist in provided buffer.
    /// This method will automatically lock itself to the first GPS period start
    /// encountered, and supports gaps between each subframe, which is compatible
    /// with real-word applications.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        let mut pos = stream.pos();

        loop {
            let missing_bits = (GPS_LNAV_SUPERFRAME_BITS - pos);
            let bits_left = stream.bits_left();

            if bits_left < missing_bits {
                return Err(BitError::NotEnoughData {
                    requested: missing_bits,
                    bits_left,
                });
            }

            match GpsQzssLnavSubframe::read(stream) {
                Ok(frame) => {
                    pos = stream.pos();

                    if let Some(message) = builder.push_frame(frame) {
                        return Ok(message);
                    }
                },
                Err(_) => {
                    // advance by one bit
                    pos += 1;
                    stream.set_pos(pos)?;
                },
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
            test_vectors::bytes_to_words, GpsQzssLnavReservedPage, GPS_LNAV_FRAME_BYTES,
            GPS_LNAV_SUPERFRAME_ITEMS,
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
        assert_eq!(message.encode(&mut bytes).unwrap(), GPS_LNAV_SUPERFRAME_BYTES);
        bytes
    }

    #[test]
    fn model_is_valid() {
        for start in [0, 750, 302_250, 603_750] {
            let message = GpsQzssLnavSuperframe::model(start);
            message.validate().unwrap_or_else(|e| {
                panic!("start={}: {}", start, e);
            });

            assert_eq!(message.start_tow_s(), start);
            assert_eq!(message.frames().count(), GPS_QZSS_MESSAGE_FRAMES);

            // every subframe knows its page
            for (p, page) in message.pages().iter().enumerate() {
                for (k, frame) in page.iter().enumerate() {
                    assert_eq!(frame.page_number() as usize, p + 1);
                    assert_eq!(frame.subframe.frame_id().encode() as usize, k + 1);
                    assert_eq!(
                        frame.transmission_time_s(),
                        start + 30 * p as u32 + 6 * k as u32
                    );
                }
            }
        }
    }

    #[test]
    fn reciprocal() {
        let message = GpsQzssLnavSuperframe::model(750 * 7);
        let bytes = encode(&message);

        // 125 contiguous subframes, each starting with the preamble
        for k in 0..GPS_QZSS_MESSAGE_FRAMES {
            assert_eq!(read_bits(&bytes, k * GPS_LNAV_FRAME_BITS, 8), Some(0x8B));
        }

        let decoded = GpsQzssLnavSuperframe::decode(&bytes).unwrap();
        assert_eq!(decoded, message);
        decoded.validate().unwrap();

        // bit exact
        assert_eq!(encode(&decoded), bytes);
    }

    #[test]
    fn subframes_are_bit_exact() {
        // the message is the concatenation of its subframes
        let message = GpsQzssLnavSuperframe::model(0);
        let bytes = encode(&message);

        for (k, frame) in message.frames().enumerate() {
            let mut expected = [0u8; GPS_LNAV_FRAME_BYTES];
            frame.encode(&mut expected).unwrap();

            for bit in 0..GPS_LNAV_FRAME_BITS {
                assert_eq!(
                    read_bits(&bytes, k * GPS_LNAV_FRAME_BITS + bit, 1),
                    read_bits(&expected, bit, 1),
                    "subframe #{} bit #{}",
                    k,
                    bit
                );
            }

            let words = bytes_to_words(&expected);
            assert!(GpsQzssLnavSubframe::from_words(&words, true).is_ok());
        }
    }

    #[test]
    fn buffer_too_small() {
        let message = GpsQzssLnavSuperframe::model(0);

        let mut bytes = vec![0u8; GPS_QZSS_MESSAGE_BYTES - 1];
        assert!(message.encode(&mut bytes).is_err());

        let bytes = encode(&message);
        assert!(GpsQzssLnavSuperframe::decode(&bytes[..GPS_QZSS_MESSAGE_BYTES - 1]).is_err());
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

                for frame in message.frames() {
                    writer.write(frame).unwrap();
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

                for frame in previous.frames().skip(skip) {
                    writer.write(frame).unwrap();
                }

                writer.write(&message).unwrap();
            }

            let decoded = GpsQzssLnavSuperframe::decode(&bytes).unwrap();
            assert_eq!(decoded, message, "skip={}", skip);
            assert_eq!(decoded.start_tow_s(), 750);
        }
    }

    #[test]
    fn reading_restarts_on_discontinuity() {
        // a subframe is missing in the first rotation: it is discarded
        let first = GpsQzssLnavSuperframe::model(0);
        let second = GpsQzssLnavSuperframe::model(750);

        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);

            for (k, frame) in first.frames().enumerate() {
                if k != 60 {
                    writer.write(frame).unwrap();
                }
            }

            writer.write(&second).unwrap();
        }

        let decoded = GpsQzssLnavSuperframe::decode(&bytes).unwrap();
        assert_eq!(decoded, second);

        // the first rotation alone is not enough
        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);

            for (k, frame) in first.frames().enumerate() {
                if k != 60 {
                    writer.write(frame).unwrap();
                }
            }
        }

        assert!(GpsQzssLnavSuperframe::decode(&bytes).is_err());
    }

    #[test]
    fn corrupted_subframe_restarts() {
        let first = GpsQzssLnavSuperframe::model(0);
        let second = GpsQzssLnavSuperframe::model(750);

        let mut bytes = encode(&first);
        bytes.extend_from_slice(&[0u8; 8]);

        // corrupt one bit of subframe #99 of the first rotation: parity error
        let bit = 99 * GPS_LNAV_FRAME_BITS + 100;
        bytes[bit / 8] ^= 0x80 >> (bit % 8);
        assert!(GpsQzssLnavSuperframe::decode(&bytes).is_err());

        let mut stream = Vec::new();
        {
            let mut writer = BitWriteStream::new(&mut stream, BigEndian);
            writer.write_bytes(&bytes).unwrap();
            writer.write(&second).unwrap();
        }

        assert_eq!(GpsQzssLnavSuperframe::decode(&stream).unwrap(), second);
    }

    #[test]
    fn week_end() {
        // 604800 is not a multiple of 750: the last rotation of the week
        // (starting at 604500) is interrupted after 10 pages.
        let last = GpsQzssLnavSuperframe::model(603_750);
        last.validate().unwrap();

        let frame = GpsQzssLnavSubframe::default()
            .with_hand_over_word(GpsQzssLnavHow::default().with_tow_seconds(604_506));
        assert_eq!(frame.page_number(), 1);
        assert!(!frame.starts_rotation());

        // the last subframe of this rotation
        let last_frame = last.frames().last().unwrap();
        assert_eq!(last_frame.how.tow, 604_500);

        // the first subframe of the week starts a rotation
        let first =
            GpsQzssLnavSubframe::default().with_hand_over_word(GpsQzssLnavHow::default().with_tow_seconds(6));
        assert!(first.starts_rotation());
        assert_eq!(first.transmission_time_s(), 0);

        // continuity across the end of the week
        let before =
            GpsQzssLnavSubframe::default().with_hand_over_word(GpsQzssLnavHow::default().with_tow_seconds(0));
        let before = before.with_subframe(GpsQzssLnavSubframe::Almanach5(Default::default()));
        assert_eq!(before.transmission_time_s(), 604_794);
        assert!(GpsQzssLnavSuperframeBuilder::follows(&before, &first));
    }

    #[test]
    fn validation() {
        let message = GpsQzssLnavSuperframe::model(1500);
        message.validate().unwrap();

        // misaligned
        let misaligned = GpsQzssLnavSuperframe::model(1530);
        assert!(matches!(
            misaligned.validate(),
            Err(GpsError::MessageAlignment)
        ));

        // sequence
        let mut broken = message;
        broken.0[3][2].how.tow += 6;
        assert!(matches!(
            broken.validate(),
            Err(GpsError::MessageSequence(17))
        ));

        let mut broken = message;
        broken.0[3][2] = broken.0[3][1];
        assert!(matches!(
            broken.validate(),
            Err(GpsError::MessageSequence(17))
        ));

        // a configuration page on page 18 of subframe 4
        let mut broken = message;
        broken.0[17][3] = broken.0[17][3].with_subframe(GpsQzssLnavSubframe::Almanach4(
            GpsQzssLnavSubframe4::ConfigHealth(Default::default()),
        ));
        assert!(matches!(
            broken.validate(),
            Err(GpsError::UnexpectedPage(18, 4))
        ));

        // page 25 of subframe 5 must carry the health page
        let mut broken = message;
        broken.0[24][4] = broken.0[24][4].with_subframe(GpsQzssLnavSubframe::Almanach5(
            GpsQzssLnavSubframe5::Reserved(GpsQzssLnavReservedPage::model(57)),
        ));
        assert!(matches!(
            broken.validate(),
            Err(GpsError::UnexpectedPage(25, 5))
        ));

        // a dummy page instead of an almanac is fine
        let mut dummy = message;
        dummy.0[6][4] =
            dummy.0[6][4].with_subframe(GpsQzssLnavSubframe::Almanach5(GpsQzssLnavSubframe5::default()));
        dummy.validate().unwrap();
        assert!(dummy.almanach(7).is_none());
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
        assert_eq!(message.frame4(18).unwrap().sv_id(), 56);
        assert_eq!(message.frame5(25).unwrap().sv_id(), 51);
        assert_eq!(message.frame4(0), None);
        assert_eq!(message.frame5(26), None);
        assert_eq!(message.frames4().count(), 25);
        assert_eq!(message.frames5().count(), 25);

        assert_eq!(
            message.ionosphere_utc(),
            Some(&GpsQzssLnavIonosphereUtc::model())
        );
        assert_eq!(message.nmct(), Some(&GpsQzssLnavNmct::model()));
        assert_eq!(
            message.special_message().unwrap().text(),
            Some("GPS IS-GPS-200 NAV MSG")
        );
        assert_eq!(message.config_health(), Some(&GpsQzssLnavConfigHealth::model()));
        assert_eq!(
            message.almanach_health(),
            Some(&GpsQzssLnavAlmanachHealth::model())
        );

        // health of the whole constellation
        for prn in 1..=32 {
            let health = message.satellite_health(prn).unwrap();
            assert_eq!(health.is_healthy(), prn != 21 && prn != 27, "prn {}", prn);
        }

        assert!(message.satellite_health(0).is_none());
        assert!(message.satellite_health(33).is_none());

        let (eph1, eph2, eph3) = message.ephemeris().unwrap();
        assert_eq!(eph1, GpsQzssLnavSubframe1::model());
        assert_eq!(eph2, GpsQzssLnavSubframe2::model());
        assert_eq!(eph3, GpsQzssLnavSubframe3::model());

        assert_eq!(
            message.page(1).unwrap()[0].subframe.frame_id(),
            crate::gps::GpsQzssLnavSubframeId::Ephemeris1
        );
        assert!(message.page(0).is_none());
        assert!(message.page(26).is_none());
        assert_eq!(GPS_QZSS_MESSAGE_DURATION_S, 750);
    }

    #[test]
    fn builder() {
        let previous = GpsQzssLnavSuperframe::model(0);
        let message = GpsQzssLnavSuperframe::model(750);

        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        assert!(builder.is_empty());
        assert_eq!(builder.remaining_s(), None);

        // the end of the previous rotation is ignored
        for frame in previous.frames().skip(50) {
            assert!(builder.push(*frame).is_none());
            assert!(builder.is_empty());
        }

        let mut output = None;

        for (k, frame) in message.frames().enumerate() {
            output = builder.push(*frame);

            if k < GPS_QZSS_MESSAGE_FRAMES - 1 {
                assert!(output.is_none());
                assert_eq!(builder.len(), k + 1);
                assert_eq!(builder.remaining_s(), Some(750 - 6 * (k as u32 + 1)));
            }
        }

        assert_eq!(output, Some(message));
        assert!(builder.is_empty());

        // a missing subframe restarts the collection
        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        for frame in message.frames().take(30) {
            builder.push(*frame);
        }
        assert_eq!(builder.len(), 30);
        builder.push(*message.frames().nth(31).unwrap());
        assert!(builder.is_empty());

        builder.push(*message.frames().next().unwrap());
        assert_eq!(builder.len(), 1);
        builder.reset();
        assert!(builder.is_empty());
    }

    #[test]
    fn builder_with_decoder() {
        use crate::gps::GpsQzssDecoder;

        // a raw bit stream of 2 rotations, decoded frame by frame
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

        let mut decoder = GpsQzssDecoder::default();
        let mut builder = GpsQzssLnavSuperframeBuilder::default();
        let mut messages = Vec::new();
        let mut position = 0;

        loop {
            let (consumed, frame) = decoder.decode(&bytes, position);
            position += consumed;

            match frame {
                Some(frame) => {
                    if let Some(message) = builder.push(frame) {
                        messages.push(message);
                    }
                },
                None => break,
            }
        }

        assert_eq!(messages, vec![first, second]);
    }
}
