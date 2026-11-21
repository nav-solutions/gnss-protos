//! GPS 12'30" message (25 pages of 5 subframes)

use crate::{
    gps::{
        GpsError, GpsQzssAlmanach, GpsQzssAlmanachHealth, GpsQzssConfigHealth, GpsQzssFrame,
        GpsQzssHow,
        GpsQzssIonosphereUtc, GpsQzssNmct, GpsQzssSatelliteHealth, GpsQzssSpecialMessage,
        GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS, 
        GPS_SUPERFRAME_ITEMS,
        GPS_SUBFRAME_DURATION_S,
        GPS_SUPERFRAME_BITS,
        GPS_SUPERFRAME_BYTES,
        GPS_FRAME_ITEMS,
        GPS_SUPERFRAME_ITEMS,
        GpsQzssFrame,
        GPS_SUBFRAME_DURATION_S,
        GPS_FRAME_DURATION_S,
    },
    constants::SECONDS_PER_WEEK,
    BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream, Message,
};

#[cfg(feature = "log")]
use log::error;

/// [GpsQzssSuperframeBuilder] collects a [GpsQzssFrame]s (30s cycles) consecutively,
/// and aligns itself to the Page-1 in the frame pagination.
/// Therefore, the resulting super frame is correctly aligned and starts
/// with Page-1 EPH-1.
///
/// ```
/// use gnss_protos::gps::{
///     GpsQzssSubframe, 
///     GpsQzssHow, 
///     GpsQzssSuperframeBuilder,
/// };
///
/// // reads one of our real-world data stream
/// // each file contains several 12'30s 
/// // this example demonstrates GpsQzssSuperframeBuilder did
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
#[derive(Debug, Clone, Default)]
pub struct GpsQzssSuperframeBuilder {
    // number of subframes collected
    sbf: usize,

    // latest frame type
    latest_id: Option<GpsQzssSubframeId>,

    // latest frame collected
    buffer: [GpsQzssFrame; GPS_SUPERFRAME_ITEMS],
}

impl GpsQzssMessageBuilder {
    // Use this method to collected several [GpsQzssSuperframe]s in a row,
    // using the same builder without reallocation.
    pub fn reset(&mut self) {
        self.sbf = 0;
        self.latest_id = None;
    }

    /// Returns true if `next` immediately follows `previous` in the GPS/QZSS stream.
    fn follows(previous: &GpsQzssFrame, next: &GpsQzssFrame) -> bool {


        let expected_id = self.latest_id.encode() % GPS_FRAME_ITEMS as u8 + 1;

        next.subframe.frame_id().encode() == expected_id
            && next.transmission_time_s()
                == (previous.transmission_time_s() + GPS_SUBFRAME_DURATION_S) % SECONDS_PER_WEEK
    }

    /// Returns the total of subframes (6s messages) collected so far.
    pub fn total_subframes(&self) -> usize {
        self.sbf
    }
    
    /// Returns the total of frames (30s messages) collected so far.
    pub fn total_frames(&self) -> usize {
        self.sbf / GPS_FRAME_ITEMS
    }

    /// Returns true if no subframe has been collected yet.
    pub fn is_empty(&self) -> bool {
        self.sbf.len == 0
    }

    /// Returns total number of seconds remaining before the super frame completes
    /// (recording of ALM-5 page 25).
    pub fn seconds_to_end_of_period(&self) -> Option<u32> {
        if self.len == 0 {
            None
        } else {
            Some(GPS_SUPERFRAME_DURATION_S - self.sbf as u32 * GPS_SUBFRAME_DURATION_S)
        }
    }

    /// Use this method when working at the [GpsQzssSubframe] (GPS/QZSS LNAV 6s core element).
    /// Method returns [GpsQzssSuperframe] if the builder has been locked for 124 subframes
    /// and this last subframe completes a 12'30" period.
    ///
    /// This is most convenient when working with our [GpsQzssDecoder] and your application
    /// intends to collect 12'30s periods.
    pub fn push_subframe(&mut self, subf: GpsQzssSubframe) -> Option<GpsQzssSuperframe> {
        if self.sbf == 0 {
            // waits for EPH-1 Page-1
            if subf.starts_superframe_period() {
                self.buffer[self.sbf] = subf;
                self.latest_id = Some(subf.how.sbf_id);
                self.sbf = 1;
            }
            None
        } else {

            let latest_sbfid = self.latest_id.unwrap();
            let current_frame = self.buffer[self.sbf / GPS_FRAME_ITEMS];

            let latest_sbf = match latest_sbfid {
                GpsQzssSubframeId::Ephemeris1 => current_frame.eph_1(),
                GpsQzssSubframeId::Ephemeris2 => current_frame.eph_2(),
                GpsQzssSubframeId::Ephemeris3 => current_frame.eph_3(),
                GpsQzssSubframeId::Almanach4 => current_frame.alm_4(),
                GpsQzssSubframeId::Almanach5 => current_frame.alm_5(),
            };

            let expected_id = GpsQzssSubframeId::decode((latest_sbf.how.frame_id.encode() +1) % GPS_FRAME_ITEMS)
                .unwrap();

            if subf.how.frame_id == expected_id {

                if subf.how.frame_id == GpsQzssSubframeId::Almanach5 {
                    // completes a new frame
                    self.buffer[self.sbf / GPS_FRAME_ITEMS] = GpsQzssFrame::from_parts(
                        current_frame.eph_1(), 
                        current_frame.eph_2(), 
                        current_frame.eph_3(), 
                        current_frame.alm_4(), 
                        current_frame.alm_5(), 
                    )
                        .unwrap_or_else(|e| {
                            panic!("internal error");
                        });
                }

                // guards against corrupted streams
                self.sbf = (self.sbf +1) % GPS_SUPERFRAME_ITEMS; 

                if subf.ends_superframe_period() {
                    // completes the frame period
                    self.reset();
                    Some(GpsQzssSuperframe(self.buffer))
                } else {
                    None
                }
                
            } else {
                self.reset();
                None
            }
        }
    }

    /// Use this method if your decoding process works at the 30s cycle scale,
    /// otherwise prefer [Self::push_subframe], especially when working with our
    /// [GpsQzssDecoder].
    pub fn push_frame(&mut self, frame: GpsQzssFrame) -> Option<GpsQzssSuperframe> {
        let _ = self.push_subframe(frame.0[0])?;  
        let _ = self.push_subframe(frame.0[1])?;  
        let _ = self.push_subframe(frame.0[2])?;  
        let _ = self.push_subframe(frame.0[3])?;  
        self.push_subframe(frame.0[4])
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use crate::{
        bits::read_bits,
        gps::{
            test_vectors::bytes_to_words, GpsQzssReservedPage, GPS_FRAME_BYTES,
            GPS_QZSS_MESSAGE_DURATION_S,
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

    fn encode(message: &GpsQzssMessage) -> Vec<u8> {
        let mut bytes = vec![0u8; GPS_QZSS_MESSAGE_BYTES];
        assert_eq!(message.encode(&mut bytes).unwrap(), GPS_QZSS_MESSAGE_BYTES);
        bytes
    }

    #[test]
    fn model_is_valid() {
        for start in [0, 750, 302_250, 603_750] {
            let message = GpsQzssMessage::model(start);
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
        let message = GpsQzssMessage::model(750 * 7);
        let bytes = encode(&message);

        // 125 contiguous subframes, each starting with the preamble
        for k in 0..GPS_QZSS_MESSAGE_FRAMES {
            assert_eq!(read_bits(&bytes, k * GPS_FRAME_BITS, 8), Some(0x8B));
        }

        let decoded = GpsQzssMessage::decode(&bytes).unwrap();
        assert_eq!(decoded, message);
        decoded.validate().unwrap();

        // bit exact
        assert_eq!(encode(&decoded), bytes);
    }

    #[test]
    fn subframes_are_bit_exact() {
        // the message is the concatenation of its subframes
        let message = GpsQzssMessage::model(0);
        let bytes = encode(&message);

        for (k, frame) in message.frames().enumerate() {
            let mut expected = [0u8; GPS_FRAME_BYTES];
            frame.encode(&mut expected).unwrap();

            for bit in 0..GPS_FRAME_BITS {
                assert_eq!(
                    read_bits(&bytes, k * GPS_FRAME_BITS + bit, 1),
                    read_bits(&expected, bit, 1),
                    "subframe #{} bit #{}",
                    k,
                    bit
                );
            }

            let words = bytes_to_words(&expected);
            assert!(GpsQzssFrame::from_words(&words, true).is_ok());
        }
    }

    #[test]
    fn buffer_too_small() {
        let message = GpsQzssMessage::model(0);

        let mut bytes = vec![0u8; GPS_QZSS_MESSAGE_BYTES - 1];
        assert!(message.encode(&mut bytes).is_err());

        let bytes = encode(&message);
        assert!(GpsQzssMessage::decode(&bytes[..GPS_QZSS_MESSAGE_BYTES - 1]).is_err());
        assert!(GpsQzssMessage::decode(&[]).is_err());
    }

    #[test]
    fn gaps_and_noise() {
        let message = GpsQzssMessage::model(750 * 3);

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
            let decoded: GpsQzssMessage = reader.read().unwrap_or_else(|e| {
                panic!("leading={} gap={}: {}", leading, gap, e);
            });

            assert_eq!(decoded, message, "leading={} gap={}", leading, gap);
        }
    }

    #[test]
    fn reading_starts_at_page_1() {
        // the stream starts in the middle of a rotation: the end of the previous rotation
        // is skipped, the next complete rotation is returned
        let previous = GpsQzssMessage::model(0);
        let message = GpsQzssMessage::model(750);

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

            let decoded = GpsQzssMessage::decode(&bytes).unwrap();
            assert_eq!(decoded, message, "skip={}", skip);
            assert_eq!(decoded.start_tow_s(), 750);
        }
    }

    #[test]
    fn reading_restarts_on_discontinuity() {
        // a subframe is missing in the first rotation: it is discarded
        let first = GpsQzssMessage::model(0);
        let second = GpsQzssMessage::model(750);

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

        let decoded = GpsQzssMessage::decode(&bytes).unwrap();
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

        assert!(GpsQzssMessage::decode(&bytes).is_err());
    }

    #[test]
    fn corrupted_subframe_restarts() {
        let first = GpsQzssMessage::model(0);
        let second = GpsQzssMessage::model(750);

        let mut bytes = encode(&first);
        bytes.extend_from_slice(&[0u8; 8]);

        // corrupt one bit of subframe #99 of the first rotation: parity error
        let bit = 99 * GPS_FRAME_BITS + 100;
        bytes[bit / 8] ^= 0x80 >> (bit % 8);
        assert!(GpsQzssMessage::decode(&bytes).is_err());

        let mut stream = Vec::new();
        {
            let mut writer = BitWriteStream::new(&mut stream, BigEndian);
            writer.write_bytes(&bytes).unwrap();
            writer.write(&second).unwrap();
        }

        assert_eq!(GpsQzssMessage::decode(&stream).unwrap(), second);
    }

    #[test]
    fn week_end() {
        // 604800 is not a multiple of 750: the last rotation of the week
        // (starting at 604500) is interrupted after 10 pages.
        let last = GpsQzssMessage::model(603_750);
        last.validate().unwrap();

        let frame = GpsQzssFrame::default()
            .with_hand_over_word(GpsQzssHow::default().with_tow_seconds(604_506));
        assert_eq!(frame.page_number(), 1);
        assert!(!frame.starts_rotation());

        // the last subframe of this rotation
        let last_frame = last.frames().last().unwrap();
        assert_eq!(last_frame.how.tow, 604_500);

        // the first subframe of the week starts a rotation
        let first =
            GpsQzssFrame::default().with_hand_over_word(GpsQzssHow::default().with_tow_seconds(6));
        assert!(first.starts_rotation());
        assert_eq!(first.transmission_time_s(), 0);

        // continuity across the end of the week
        let before =
            GpsQzssFrame::default().with_hand_over_word(GpsQzssHow::default().with_tow_seconds(0));
        let before = before.with_subframe(GpsQzssSubframe::Almanach5(Default::default()));
        assert_eq!(before.transmission_time_s(), 604_794);
        assert!(GpsQzssMessageBuilder::follows(&before, &first));
    }

    #[test]
    fn validation() {
        let message = GpsQzssMessage::model(1500);
        message.validate().unwrap();

        // misaligned
        let misaligned = GpsQzssMessage::model(1530);
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
        broken.0[17][3] = broken.0[17][3].with_subframe(GpsQzssSubframe::Almanach4(
            GpsQzssFrame4::ConfigHealth(Default::default()),
        ));
        assert!(matches!(
            broken.validate(),
            Err(GpsError::UnexpectedPage(18, 4))
        ));

        // page 25 of subframe 5 must carry the health page
        let mut broken = message;
        broken.0[24][4] = broken.0[24][4].with_subframe(GpsQzssSubframe::Almanach5(
            GpsQzssFrame5::Reserved(GpsQzssReservedPage::model(57)),
        ));
        assert!(matches!(
            broken.validate(),
            Err(GpsError::UnexpectedPage(25, 5))
        ));

        // a dummy page instead of an almanac is fine
        let mut dummy = message;
        dummy.0[6][4] =
            dummy.0[6][4].with_subframe(GpsQzssSubframe::Almanach5(GpsQzssFrame5::default()));
        dummy.validate().unwrap();
        assert!(dummy.almanach(7).is_none());
    }

    #[test]
    fn accessors() {
        let message = GpsQzssMessage::model(0);

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
            Some(&GpsQzssIonosphereUtc::model())
        );
        assert_eq!(message.nmct(), Some(&GpsQzssNmct::model()));
        assert_eq!(
            message.special_message().unwrap().text(),
            Some("GPS IS-GPS-200 NAV MSG")
        );
        assert_eq!(message.config_health(), Some(&GpsQzssConfigHealth::model()));
        assert_eq!(
            message.almanach_health(),
            Some(&GpsQzssAlmanachHealth::model())
        );

        // health of the whole constellation
        for prn in 1..=32 {
            let health = message.satellite_health(prn).unwrap();
            assert_eq!(health.is_healthy(), prn != 21 && prn != 27, "prn {}", prn);
        }

        assert!(message.satellite_health(0).is_none());
        assert!(message.satellite_health(33).is_none());

        let (eph1, eph2, eph3) = message.ephemeris().unwrap();
        assert_eq!(eph1, GpsQzssFrame1::model());
        assert_eq!(eph2, GpsQzssFrame2::model());
        assert_eq!(eph3, GpsQzssFrame3::model());

        assert_eq!(
            message.page(1).unwrap()[0].subframe.frame_id(),
            crate::gps::GpsQzssFrameId::Ephemeris1
        );
        assert!(message.page(0).is_none());
        assert!(message.page(26).is_none());
        assert_eq!(GPS_QZSS_MESSAGE_DURATION_S, 750);
    }

    #[test]
    fn builder() {
        let previous = GpsQzssMessage::model(0);
        let message = GpsQzssMessage::model(750);

        let mut builder = GpsQzssMessageBuilder::default();
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
        let mut builder = GpsQzssMessageBuilder::default();
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
        let first = GpsQzssMessage::model(750 * 10);
        let second = GpsQzssMessage::model(750 * 11);

        let mut bytes = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            writer.write_int(0u8, 3).unwrap();
            writer.write(&first).unwrap();
            writer.write(&second).unwrap();
            writer.write_int(0u8, 8).unwrap();
        }

        let mut decoder = GpsQzssDecoder::default();
        let mut builder = GpsQzssMessageBuilder::default();
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
        loop {
            let (consumed, frame) = decoder.decode(&bytes, position);
            position += consumed;

            match frame {
                Some(frame) => {
                    assert!(builder.push(frame).is_none());
                }
            }
        }

        // builder waited for Page1 (4 pages offset) and collected the whole stream
        assert_eq!(builder.len(), UBLOX_G25_STREAM.len() -4 *5);
    }
}
