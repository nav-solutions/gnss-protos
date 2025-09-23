use crate::gps::{
    GpsDataWord, GpsQzssFrame, GpsQzssFrameId, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE,
    GPS_WORDS_PER_FRAME,
};

impl GpsQzssFrame {
    /// Returns total number of bytes needed to encode this [GpsQzssFrame] to binary
    /// aligned to [u8]
    pub const fn encoding_size() -> usize {
        GPS_FRAME_BYTES
    }

    /// Returns exact number of bits needed to encode this [GpsQzssFrame]
    pub const fn encoding_bits() -> usize {
        GPS_FRAME_BITS
    }

    /// Encodes this [GpsQzssFrame] as serie of 10 [GpsDataWord]s, aligned to 32 bits
    /// and parity bits correctly encoded.  
    /// When working with a real-time transmitter, you should prefer
    /// [Self::encode_raw] which is not aligned.
    pub fn encode(&self) -> [GpsDataWord; GPS_WORDS_PER_FRAME] {
        let subwords = self.subframe.to_words();

        let mut words = [
            self.telemetry.to_word(),
            self.how.to_word(),
            subwords[0],
            subwords[1],
            subwords[2],
            subwords[3],
            subwords[4],
            subwords[5],
            subwords[6],
            subwords[7],
        ];

        // calculate parity for each word
        for (ith, word) in words.iter_mut().enumerate() {
            let parity = word.parity(&Default::default(), false);

            *word |= 0x00u8;
        }

        words
    }

    /// Encodes this [GpsQzssFrame] as a 300 bit burst (38 bytes).
    /// Because [GpsQzssFrame] is not aligned to [u8], the very last byte contains 4 MSB padding bits, set to zeros
    /// (unsigned). If you leave it to that, any streaming/transmitter looses a little bit of efficiency
    /// any time a [GpsQzssFrame] is encoded/transmitted. The only solution then, is to manually remove this padding
    /// an truly concatenate your frames, but one can't expect easy processes when working with poorly designed and very old protocols.
    /// A true synchronous [GpsQzssFrame] emitter is supposed to transmit one frame every 6 seconds,
    /// that is 50 bits per second.
    /// NB: this [GpsQzssFrame] is not ready to transmit as-is and must be CDMA encoded
    /// using [GpsQzssFrame::cdma_encoding].
    pub fn encode_raw(&self) -> [u8; GPS_FRAME_BYTES] {
        let mut encoded = [0; GPS_FRAME_BYTES];

        encoded[0] = GPS_PREAMBLE_BYTE;
        encoded[1] = ((self.telemetry.message & 0x3fc0) >> 6) as u8;
        encoded[2] = (self.telemetry.message & 0x3f) as u8;
        encoded[2] <<= 2;

        if self.telemetry.integrity {
            encoded[2] |= 0x02;
        }

        if self.telemetry.reserved_bit {
            encoded[2] |= 0x01;
        }

        encoded[3] <<= 2; // TODO (PAR)

        let tow = self.how.tow * 2 / 3;

        encoded[3] |= ((tow & 0x1_8000) >> 15) as u8;
        encoded[4] = ((tow & 0x0_7f80) >> 7) as u8;
        encoded[5] = (tow & 0x0_007f) as u8;
        encoded[5] <<= 1;

        if self.how.alert {
            encoded[5] |= 0x01;
        }

        if self.how.anti_spoofing {
            encoded[6] |= 0x80;
        }

        encoded[6] |= self.how.frame_id.encode() << 4;

        encoded[7] <<= 4; // TODO (PAR)

        match self.how.frame_id {
            GpsQzssFrameId::Ephemeris1 => {
                let subf = self.subframe.as_eph1().unwrap_or_default();

                encoded[7] |= ((subf.week & 0x3c0) >> 6) as u8;
                encoded[8] = (subf.week & 0x03f) as u8;
                encoded[8] <<= 2;
                encoded[8] |= (subf.ca_or_p_l2) & 0x03;

                encoded[9] = subf.ura & 0x0f;
                encoded[9] <<= 4;
                encoded[9] |= (subf.health & 0x3c) >> 2;

                encoded[10] = subf.health & 0x03;
                encoded[10] <<= 6;
                encoded[10] |= (((subf.iodc & 0x300) >> 8) as u8) << 4;

                encoded[11] <<= 6; // TODO (PAR)

                if subf.l2_p_data_flag {
                    encoded[11] |= 0x20;
                }

                encoded[11] |= ((subf.reserved_word4 & 0x7c_0000) >> 18) as u8;
                encoded[12] = ((subf.reserved_word4 & 0x03_fc00) >> 10) as u8;
                encoded[13] = ((subf.reserved_word4 & 0x00_03fc) >> 2) as u8;

                encoded[14] = (subf.reserved_word4 & 0x3) as u8;
                encoded[14] <<= 6; // TODO

                encoded[15] = ((subf.reserved_word5 & 0xff_0000) >> 16) as u8;
                encoded[16] = ((subf.reserved_word5 & 0x00_ff00) >> 8) as u8;
                encoded[17] = ((subf.reserved_word5 & 0x00_00ff) >> 0) as u8;
                encoded[18] <<= 2; // TODO

                encoded[18] |= ((subf.reserved_word6 & 0xc0_0000) >> 22) as u8;
                encoded[19] = ((subf.reserved_word6 & 0x3f_c000) >> 14) as u8;
                encoded[20] = ((subf.reserved_word6 & 0x00_3fc0) >> 6) as u8;
                encoded[21] = ((subf.reserved_word6 & 0x00_003f) >> 0) as u8;
                encoded[21] <<= 2;
                encoded[22] <<= 4; // TODO

                encoded[22] |= ((subf.reserved_word7 & 0xf000) >> 12) as u8;
                encoded[23] = ((subf.reserved_word7 & 0x0ff0) >> 4) as u8;
                encoded[24] = ((subf.reserved_word7 & 0x000f) >> 0) as u8;
                encoded[24] <<= 4;

                let mut tgd = (subf.tgd * 2.0_f64.powi(31)).round() as u8;

                if subf.tgd < 0.0 {
                    tgd += 128;
                    tgd -= 1;
                }

                encoded[24] |= (tgd as u8 & 0xf0) >> 4;

                encoded[25] = tgd as u8 & 0x0f;
                encoded[25] <<= 4; // TODO
                encoded[26] <<= 6; // TODO
                encoded[26] |= ((subf.iodc & 0x0fc) >> 2) as u8;

                encoded[27] = (subf.iodc & 0x03) as u8;
                encoded[27] <<= 6;

                let toc = subf.toc / 16;
                encoded[27] |= ((toc & 0xfc00) >> 10) as u8;
                encoded[28] = ((toc & 0x03fc) >> 2) as u8;
                encoded[29] = (toc & 0x0003) as u8;
                encoded[29] <<= 6; // TODO

                let af2 = (subf.af2 * 2.0_f64.powi(55)).round() as u8;
                let af1 = (subf.af1 * 2.0_f64.powi(43)).round() as u16;
                let af0 = (subf.af0 * 2.0_f64.powi(31)).round() as u32;

                encoded[30] = af2;
                encoded[31] = ((af1 & 0xff00) >> 8) as u8;
                encoded[32] = ((af1 & 0x00ff) >> 0) as u8;
                encoded[33] <<= 2; // TODO

                encoded[33] |= ((af0 & 0x30_0000) >> 20) as u8;
                encoded[34] = ((af0 & 0x0f_f000) >> 12) as u8;
                encoded[35] = ((af0 & 0x00_0ff0) >> 4) as u8;
                encoded[36] = ((af0 & 0x00_000f) >> 0) as u8;
                encoded[36] <<= 4; // TODO

                encoded[37] <<= 4;
            },
            GpsQzssFrameId::Ephemeris2 => {
                // let subf = self.subframe.as_eph2().unwrap_or_default();

                // encoded[7] |= (subf.iode & 0xf0) >> 4;
                // encoded[8] |= subf.iode & 0x0f;
                // encoded[8] <<= 4;

                // let crs = (subf.crs * 2.0_f64.powi(5)).round() as u16;
                // encoded[8] |= ((crs & 0xf000) >> 12) as u8;
                // encoded[9] |= ((crs & 0x0ff0) >> 4) as u8;
                // encoded[10] |= (crs & 0x000f) as u8;
                // encoded[10] <<= 4; // TODO

                // let dn = (subf.dn * 2.0_f64.powi(43)).round() as u16;
                // encoded[11] |= ((dn & 0xfc00) >> 10) as u8;

                // encoded[12] |= ((dn & 0x03fc) >> 2) as u8;
                // encoded[13] |= (dn & 0x0003) as u8;
                // encoded[13] <<= 6;

                // let m0 = (subf.m0 * 2.0_f64.powi(31)).round() as u32;

                // encoded[13] |= ((m0 & 0xfc000000) >> 26) as u8;
                // encoded[14] |= ((m0 & 0x03000000) >> 24) as u8;
                // encoded[14] <<= 6; //TODO

                // encoded[15] |= ((m0 & 0x00ff0000) >> 16) as u8;
                // encoded[16] |= ((m0 & 0x0000ff00) >> 8) as u8;
                // encoded[17] |= (m0 & 0x000000ff) as u8;

                // encoded[18] <<= 2; // TODO

                // let cuc = (subf.cuc * 2.0_f64.powi(29)).round() as u16;
                // encoded[18] |= ((cuc & 0xc000) >> 14) as u8;
                // encoded[19] |= ((cuc & 0x3fc0) >> 6) as u8;
                // encoded[20] |= (cuc & 0x003f) as u8;
                // encoded[20] <<= 2;

                // let e = (subf.e * 2.0_f64.powi(33)).round() as u32;

                // encoded[20] |= ((e & 0xc0000000) >> 30) as u8;
                // encoded[21] |= ((e & 0x3f000000) >> 24) as u8;
                // encoded[21] <<= 2; // TODO
                // encoded[22] <<= 4; // TODO

                // encoded[22] |= ((e & 0x00f00000) >> 20) as u8;
                // encoded[23] |= ((e & 0x000ff000) >> 12) as u8;
                // encoded[24] |= ((e & 0x00000ff0) >> 4) as u8;
                // encoded[25] |= (e & 0x0000000f) as u8;
                // encoded[25] <<= 4; // TODO
                // encoded[26] <<= 2; // TODO

                // let cus = (subf.cus * 2.0_f64.powi(29)).round() as u16;
                // encoded[26] |= ((cus & 0xfc00) >> 10) as u8;
                // encoded[27] |= ((cus & 0x03fc) >> 2) as u8;
                // encoded[28] |= (cus & 0x3) as u8;
                // encoded[28] <<= 6;

                // let sqrt_a = (subf.sqrt_a * 2.0_f64.powi(19)).round() as u32;
                // encoded[28] |= ((sqrt_a & 0xfc000000) >> 26) as u8;
                // encoded[29] |= ((sqrt_a & 0x03000000) >> 24) as u8;
                // encoded[29] <<= 6; // TODO

                // encoded[30] |= ((sqrt_a & 0x00ff0000) >> 16) as u8;
                // encoded[31] |= ((sqrt_a & 0x0000ff00) >> 8) as u8;
                // encoded[32] |= (sqrt_a & 0x000000ff) as u8;

                // let toe = (subf.toe * 16) as u16;

                // encoded[33] <<= 2; // TODO
                // encoded[33] |= ((toe & 0xc000) >> 14) as u8;
                // encoded[34] |= ((toe & 0x3fc0) >> 6) as u8;
                // encoded[35] |= (toe & 0x003f) as u8;
                // encoded[35] <<= 2;

                // if subf.fit_int_flag {
                //     encoded[35] |= 0x02;
                // }

                // encoded[35] |= (subf.aodo & 0x10) >> 4;

                // encoded[36] |= subf.aodo & 0x0f;
                // encoded[36] <<= 4;

                // encoded[36] |= 0x00; // two non-information bits for parity calculations
                // encoded[37] |= 0x00;
                // encoded[37] <<= 4; // TODO
            },
            GpsQzssFrameId::Ephemeris3 => {
                // let subf = self.subframe.as_eph3().unwrap_or_default();

                // let cic = (subf.cic * 2.0_f64.powi(29)).round() as u16;

                // encoded[7] |= ((cic & 0xf000) >> 12) as u8;
                // encoded[8] |= (cic & 0x0ff0) as u8;
                // encoded[8] <<= 4;

                // encoded[9] |= (cic & 0x000f) as u8;
                // encoded[9] <<= 4;
            },
        }

        encoded
    }

    /// Encodes this [GpsQzssFrame] into mutable [u8] buffer directly.
    /// Returns the total number of _bits_ (not bytes) that were encoded.
    /// Note that [GpsQzssFrame] is not aligned to [u8] so the final bytes is padded.
    /// You can use the returned size here, to create a truly contiguous stream,
    /// otherwise, you slighthly degraed the transmission efficiency.
    ///
    /// This method is particularly suited when working with a real-time GPS decoder.
    /// Otherwise, you might want to use our [GpsQzssDecoder] structure.
    ///
    /// This is [] mirror operation.
    ///
    /// ## Inputs
    /// - buffer: mutable [u8] buffer
    ///
    /// ## Output
    /// - Err if buffer can't accept this [GpsQzssFrame] entirely
    /// - total number of bits that were encoded
    pub fn encode_to_buffer(&self, buffer: &mut [u8]) -> std::io::Result<usize> {
        if buffer.len() < GPS_FRAME_BYTES {
            return Err(std::io::Error::new(
                std::io::ErrorKind::StorageFull,
                "would not fit",
            ));
        }

        buffer[..GPS_FRAME_BYTES].copy_from_slice(&self.encode_raw());

        Ok(GPS_FRAME_BYTES)
    }
}

#[cfg(test)]
mod encoding {
    use std::fs::File;
    use std::io::{Read, Write};

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    use log::info;

    use crate::gps::{
        GpsQzssDecoder,
        GpsQzssFrame,
        GpsQzssFrame1,
        // GpsQzssFrame2, GpsQzssFrame3,
        GpsQzssFrameId,
        GpsQzssHow,
        GpsQzssSubframe,
        GpsQzssTelemetry,
        GPS_FRAME_BITS,
        GPS_FRAME_BYTES,
    };

    #[test]
    fn encoding_size() {
        assert_eq!(GpsQzssFrame::encoding_size(), 300 / 8 + 1);
    }

    #[test]
    fn encoding_bits() {
        assert_eq!(GpsQzssFrame::encoding_bits(), 300);
    }

    #[test]
    fn default_frame() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let default = GpsQzssFrame::default();
        let encoded = default.encode_raw();
        let encoded_size = encoded.len();

        assert_eq!(GpsQzssFrame::encoding_size(), 300 / 8 + 1);
        assert_eq!(GpsQzssFrame::encoding_bits(), 300);

        assert_eq!(encoded_size, GPS_FRAME_BYTES, "encoded invalid size!");

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x00);
        assert_eq!(encoded[2], 0x00);
        assert_eq!(encoded[3], 0x00);
        assert_eq!(encoded[4], 0x00);
        assert_eq!(encoded[5], 0x00);
        assert_eq!(encoded[6], 0x10);
        assert_eq!(encoded[7], 0x00);
        assert_eq!(encoded[8], 0x00);
        assert_eq!(encoded[9], 0x00);
        assert_eq!(encoded[10], 0x00);
        assert_eq!(encoded[11], 0x00);
        assert_eq!(encoded[12], 0x00);
        assert_eq!(encoded[13], 0x00);
        assert_eq!(encoded[14], 0x00);
        assert_eq!(encoded[15], 0x00);
        assert_eq!(encoded[16], 0x00);
        assert_eq!(encoded[17], 0x00);
        assert_eq!(encoded[18], 0x00);
        assert_eq!(encoded[19], 0x00);
        assert_eq!(encoded[20], 0x00);
        assert_eq!(encoded[21], 0x00);
        assert_eq!(encoded[22], 0x00);
        assert_eq!(encoded[23], 0x00);
        assert_eq!(encoded[24], 0x00);
        assert_eq!(encoded[25], 0x00);
        assert_eq!(encoded[26], 0x00);
        assert_eq!(encoded[27], 0x00);
        assert_eq!(encoded[28], 0x00);
        assert_eq!(encoded[29], 0x00);
        assert_eq!(encoded[30], 0x00);
        assert_eq!(encoded[31], 0x00);
        assert_eq!(encoded[32], 0x00);
        assert_eq!(encoded[33], 0x00);
        assert_eq!(encoded[34], 0x00);
        assert_eq!(encoded[35], 0x00);
        assert_eq!(encoded[36], 0x00);
        assert_eq!(encoded[37], 0x00);

        // reciprocal
        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&encoded, encoded_size);

        assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
        assert_eq!(decoded, Some(default), "reciprocal failed");
    }

    #[test]
    fn ephemeris1_raw() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut decoder = GpsQzssDecoder::default();

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x1234)
                    .with_integrity()
                    .with_reserved_bit(),
            )
            .with_hand_over_word(
                GpsQzssHow::default()
                    .with_tow_seconds(18_510)
                    .with_alert_bit()
                    .with_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris1(
                GpsQzssFrame1::default()
                    .with_week(0x123)
                    .with_iodc(0x123)
                    .with_all_signals_ok()
                    .with_time_of_clock_seconds(12_000)
                    .with_l2p_flag()
                    .with_clock_offset_nanoseconds(1.0)
                    .with_clock_drift_seconds_s(1E-12)
                    .with_clock_drift_rate_seconds_s2(1E-15)
                    .with_reserved23_word(0x12_3456)
                    .with_reserved24_word1(0x34_5678)
                    .with_reserved24_word2(0x98_7654)
                    .with_reserved16_word(0x1234)
                    .with_total_group_delay_nanos(1.0)
                    .with_ca_or_p_l2_mask(0x3)
                    .with_user_range_accuracy_m(4.0),
            ));

        let encoded = frame.encode_raw();
        let encoded_size = encoded.len();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x48);
        assert_eq!(encoded[2], 0xD0 | 0x02 | 0x01);
        assert_eq!(encoded[3], 0x00);

        assert_eq!(encoded[4], 0x60);
        assert_eq!(encoded[5], 0x69);
        assert_eq!(encoded[6], 0x90);
        assert_eq!(encoded[7], 0x04);

        assert_eq!(encoded[8], 0x8f);
        assert_eq!(encoded[9], 0x20);
        assert_eq!(encoded[10], 0x10);
        assert_eq!(encoded[11], 0x24);
        assert_eq!(encoded[12], 0x8D);
        assert_eq!(encoded[13], 0x15);
        assert_eq!(encoded[14], 0x80);
        assert_eq!(encoded[15], 0x34);
        assert_eq!(encoded[16], 0x56);
        assert_eq!(encoded[17], 0x78);
        assert_eq!(encoded[18], 0x02);
        assert_eq!(encoded[19], 0x61);
        assert_eq!(encoded[20], 0xD9);
        assert_eq!(encoded[21], 0x50);
        assert_eq!(encoded[22], 0x01);
        assert_eq!(encoded[23], 0x23);
        assert_eq!(encoded[24], 0x40);
        assert_eq!(encoded[25], 0x20);
        assert_eq!(encoded[26], 0x08);
        assert_eq!(encoded[27], 0xC0);
        assert_eq!(encoded[28], 0xBB);
        assert_eq!(encoded[29], 0x80);
        assert_eq!(encoded[30], 0x24);
        assert_eq!(encoded[31], 0x00);
        assert_eq!(encoded[32], 0x09);
        assert_eq!(encoded[33], 0x00);
        assert_eq!(encoded[34], 0x00);
        assert_eq!(encoded[35], 0x00);
        assert_eq!(encoded[36], 0x20);
        assert_eq!(encoded[37], 0x00);

        // reciprocal
        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&encoded, encoded_size);

        assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");

        assert_eq!(
            decoded,
            Some(frame),
            "reciprocal failed, got {:#?}",
            decoded
        );

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x1234)
                    .without_integrity()
                    .with_reserved_bit(),
            )
            .with_hand_over_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x4_6789)
                    .with_alert_bit()
                    .without_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris1(
                GpsQzssFrame1::default()
                    .with_week(0x123)
                    .with_iodc(0x345)
                    .with_all_signals_ok()
                    .with_ca_or_p_l2_mask(0x1)
                    .with_user_range_accuracy_m(24.0)
                    .with_clock_offset_nanoseconds(2.0)
                    .with_clock_drift_seconds_s(2E-12)
                    .with_clock_drift_rate_seconds_s2(2E-15),
            ));

        let encoded = frame.encode_raw();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x48);
        assert_eq!(encoded[2], 0x34 << 2 | 0x01);
        assert_eq!(encoded[3], 0x01);

        assert_eq!(encoded[4], 0xDF);
        assert_eq!(encoded[5], 0x61);
        assert_eq!(encoded[6], 0x10);
        assert_eq!(encoded[7], 0x04);

        assert_eq!(encoded[8], 0x8D);
        assert_eq!(encoded[9], 0x60);
        assert_eq!(encoded[10], 0x30);
        assert_eq!(encoded[11], 0x00);
        assert_eq!(encoded[12], 0x00);
        assert_eq!(encoded[13], 0x00);
        assert_eq!(encoded[14], 0x00);
        assert_eq!(encoded[15], 0x00);
        assert_eq!(encoded[16], 0x00);
        assert_eq!(encoded[17], 0x00);
        assert_eq!(encoded[18], 0x00);
        assert_eq!(encoded[19], 0x00);
        assert_eq!(encoded[20], 0x00);
        assert_eq!(encoded[21], 0x00);
        assert_eq!(encoded[22], 0x00);
        assert_eq!(encoded[23], 0x00);
        assert_eq!(encoded[24], 0x00);
        assert_eq!(encoded[25], 0x00);
        assert_eq!(encoded[26], 0x11);
        assert_eq!(encoded[27], 0x40);
        assert_eq!(encoded[28], 0x00);
        assert_eq!(encoded[29], 0x00);
        assert_eq!(encoded[30], 0x48);
        assert_eq!(encoded[31], 0x00);
        assert_eq!(encoded[32], 0x12);
        assert_eq!(encoded[33], 0x00);
        assert_eq!(encoded[34], 0x00);
        assert_eq!(encoded[35], 0x00);
        assert_eq!(encoded[36], 0x40);
        assert_eq!(encoded[37], 0x00);

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x0123)
                    .without_integrity()
                    .without_reserved_bit(),
            )
            .with_hand_over_word(
                GpsQzssHow::default()
                    .with_tow_seconds(15_000)
                    .without_alert_bit()
                    .without_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris1(
                GpsQzssFrame1::default()
                    .with_week(0x321)
                    .with_iodc(0x321)
                    .with_all_signals_ok()
                    .with_time_of_clock_seconds(24_000)
                    .with_clock_offset_nanoseconds(2.0)
                    .with_clock_drift_seconds_s(2E-12)
                    .with_clock_drift_rate_seconds_s2(2E-15)
                    .with_reserved23_word(0x12_3456)
                    .with_reserved24_word1(0x34_5678)
                    .with_reserved24_word2(0x98_7654)
                    .with_reserved16_word(0x1234)
                    .with_total_group_delay_nanos(3.0)
                    .with_ca_or_p_l2_mask(0x2)
                    .with_user_range_accuracy_m(8.0),
            ));

        let encoded = frame.encode_raw();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x04);
        assert_eq!(encoded[2], 0x23 << 2);
        assert_eq!(encoded[3], 0x00);

        assert_eq!(encoded[4], 0x4E);
        assert_eq!(encoded[5], 0x20);
        assert_eq!(encoded[6], 0x10);
        assert_eq!(encoded[7], 0x0C);

        assert_eq!(encoded[8], 0x86);
        // TODO
        // assert_eq!(encoded[9], 0x00);
        // assert_eq!(encoded[10], 0x00);
        // assert_eq!(encoded[11], 0x00);
        // assert_eq!(encoded[12], 0x00);
        // assert_eq!(encoded[13], 0x00);
        // assert_eq!(encoded[14], 0x00);
        // assert_eq!(encoded[15], 0x00);
        // assert_eq!(encoded[16], 0x00);
        // assert_eq!(encoded[17], 0x00);
        // assert_eq!(encoded[18], 0x00);
        // assert_eq!(encoded[19], 0x00);
        // assert_eq!(encoded[20], 0x00);
        // assert_eq!(encoded[21], 0x00);
        // assert_eq!(encoded[22], 0x00);
        // assert_eq!(encoded[23], 0x00);
        // assert_eq!(encoded[24], 0x00);
        // assert_eq!(encoded[25], 0x00);
        // assert_eq!(encoded[26], 0x00);
        // assert_eq!(encoded[27], 0x00);
        // assert_eq!(encoded[28], 0x00);
        // assert_eq!(encoded[29], 0x00);
        // assert_eq!(encoded[30], 0x00);
        // assert_eq!(encoded[31], 0x00);
        // assert_eq!(encoded[32], 0x00);
        // assert_eq!(encoded[33], 0x00);
        // assert_eq!(encoded[34], 0x00);
        // assert_eq!(encoded[35], 0x00);
        // assert_eq!(encoded[36], 0x00);
        // assert_eq!(encoded[37], 0x00);

        // reciprocal
        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&encoded, encoded_size);

        assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");

        assert_eq!(
            decoded,
            Some(frame),
            "reciprocal failed, got {:#?}",
            decoded
        );
    }

    #[test]
    fn ephemeris1_reciprocal() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut decoder = GpsQzssDecoder::default();

        for (
            test_num,
            (
                tow,
                alert,
                anti_spoofing,
                frame_id,
                message,
                integrity,
                tlm_reserved_bit,
                week,
                ca_or_p_l2,
                ura,
                health,
                iodc,
                toc,
                tgd,
                af0,
                af1,
                af2,
            ),
        ) in [
            (
                15_000,
                false,
                false,
                GpsQzssFrameId::Ephemeris1,
                0x13E,
                false,
                false,
                10,
                0x0,
                0,
                0,
                0x10,
                10_000,
                1.0,
                1.0E-9,
                1.0E-12,
                1.0E-14,
            ),
            (
                15_000,
                true,
                false,
                GpsQzssFrameId::Ephemeris1,
                0x13F,
                false,
                true,
                100,
                0x1,
                1,
                1,
                0x0,
                16_000,
                2.0,
                2.0E-9,
                2.0E-12,
                2.0E-14,
            ),
            (
                15_000,
                true,
                false,
                GpsQzssFrameId::Ephemeris1,
                0x13F,
                false,
                true,
                1024,
                0x02,
                2,
                2,
                0x20,
                16_160,
                3.0,
                3.0E-9,
                3.0E-12,
                3.0E-14,
            ),
            // TODO: signed
            // (
            //     15_000,
            //     true,
            //     false,
            //     GpsQzssFrameId::Ephemeris1,
            //     0x13F,
            //     false,
            //     true,
            //     1024,
            //     0x03,
            //     3,
            //     3,
            //     0x123,
            //     16_432,
            //     -4.0,
            //     4.0E-9,
            //     4.0E-12,
            //     4.0E-14,
            // ),
            // TODO: signed
            // (
            //     15_000,
            //     true,
            //     false,
            //     GpsQzssFrameId::Ephemeris1,
            //     0x13F,
            //     false,
            //     true,
            //     1024,
            //     0x03,
            //     3,
            //     3,
            //     0x123,
            //     16_432,
            //     5.0,
            //     -4.0E-9,
            //     4.0E-12,
            //     4.0E-14,
            // ),
            // TODO: signed
            // (
            //     15_000,
            //     true,
            //     false,
            //     GpsQzssFrameId::Ephemeris1,
            //     0x13F,
            //     false,
            //     true,
            //     1024,
            //     0x03,
            //     3,
            //     3,
            //     0x123,
            //     16_432,
            //     5.0,
            //     6.0E-9,
            //     -4.0E-12,
            //     4.0E-14,
            // ),
            // TODO: signed
            // (
            //     15_000,
            //     true,
            //     false,
            //     GpsQzssFrameId::Ephemeris1,
            //     0x13F,
            //     false,
            //     true,
            //     1024,
            //     0x03,
            //     3,
            //     3,
            //     0x123,
            //     16_432,
            //     5.0,
            //     6.0E-9,
            //     4.0E-12,
            //     -4.0E-14,
            // ),
        ]
        .iter()
        .enumerate()
        {
            let mut how = GpsQzssHow::default().with_tow_seconds(*tow);

            let mut telemetry = GpsQzssTelemetry::default().with_message(*message);

            let mut subframe = GpsQzssFrame1::default()
                .with_week(*week)
                .with_iodc(*iodc)
                .with_health_mask(*health)
                .with_time_of_clock_seconds(*toc)
                .with_total_group_delay_nanos(*tgd)
                .with_ca_or_p_l2_mask(*ca_or_p_l2)
                .with_clock_offset_seconds(*af0)
                .with_clock_drift_seconds_s(*af1)
                .with_clock_drift_rate_seconds_s2(*af2);

            if *alert {
                how = how.with_alert_bit();
            }

            if *anti_spoofing {
                how = how.with_anti_spoofing();
            }

            if *integrity {
                telemetry = telemetry.with_integrity();
            }

            if *tlm_reserved_bit {
                telemetry = telemetry.with_reserved_bit();
            }

            let frame = GpsQzssFrame::default()
                .with_telemetry(telemetry)
                .with_hand_over_word(how)
                .with_subframe(GpsQzssSubframe::Ephemeris1(subframe));

            let encoded = frame.encode_raw();
            let encoded_size = encoded.len();
            assert_eq!(encoded.len(), GPS_FRAME_BYTES, "encoded invalid size!");

            let (size, decoded) = decoder.decode(&encoded, encoded.len());
            assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
            assert_eq!(decoded, Some(frame), "reciprocal failed");

            info!("test ({}): {:?}", test_num, frame);
        }
    }

    // #[test]
    // fn ephemeris2_0() {
    //     #[cfg(all(feature = "std", feature = "log"))]
    //     init_logger();

    //     let mut decoder = GpsQzssDecoder::default();

    //     let frame = GpsQzssFrame::default()
    //         .with_telemetry(
    //             GpsQzssTelemetry::default()
    //                 .with_message(0x9999)
    //                 .with_integrity()
    //                 .with_reserved_bit(),
    //         )
    //         .with_hand_over_word(
    //             GpsQzssHow::default()
    //                 .with_tow_seconds(0x9_9999)
    //                 .with_alert_bit()
    //                 .with_anti_spoofing(),
    //         )
    //         .with_subframe(GpsQzssSubframe::Ephemeris2(
    //             GpsQzssFrame2::default()
    //                 .with_iode(0x12)
    //                 .with_crs_meters(1.8)
    //                 .with_mean_motion_difference_semi_circles(100.0)
    //                 .with_mean_anomaly_semi_circles(9.768415465951e-001)
    //                 .with_toe(266_400)
    //                 .with_square_root_semi_major_axis(5.153602432251e+003)
    //                 .with_fit_interval_flag()
    //                 .with_aodo(0x15),
    //         ));

    //     let encoded = frame.encode();

    //     assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    //     assert_eq!(encoded[1], 0x19);
    //     assert_eq!(encoded[2], 0x99 << 2 | 0x02 | 0x01);
    //     assert_eq!(encoded[3], 0x03);

    //     assert_eq!(encoded[4], 0x33);
    //     assert_eq!(encoded[5], 0x33);
    //     assert_eq!(encoded[6], 0xA0);
    //     assert_eq!(encoded[7], 0x01);

    //     assert_eq!(encoded[8], 0x20);
    //     assert_eq!(encoded[9], 0x03);
    //     assert_eq!(encoded[10], 0xA0);
    //     assert_eq!(encoded[11], 0x3F);

    //     assert_eq!(encoded[12], 0xFF);
    //     assert_eq!(encoded[13], 0xDF);
    //     assert_eq!(encoded[14], 0x40);
    //     assert_eq!(encoded[15], 0x09);
    //     assert_eq!(encoded[16], 0x24);
    //     assert_eq!(encoded[17], 0xD0);
    //     assert_eq!(encoded[18], 0x00);
    //     assert_eq!(encoded[19], 0x00);
    //     assert_eq!(encoded[20], 0x00);
    //     assert_eq!(encoded[21], 0x00);
    //     assert_eq!(encoded[22], 0x00);
    //     assert_eq!(encoded[23], 0x00);
    //     assert_eq!(encoded[24], 0x00);
    //     assert_eq!(encoded[25], 0x00);
    //     assert_eq!(encoded[26], 0x00);
    //     assert_eq!(encoded[27], 0x00);
    //     assert_eq!(encoded[28], 40);
    //     assert_eq!(encoded[29], 64);
    //     assert_eq!(encoded[30], 12);
    //     assert_eq!(encoded[31], 209);
    //     assert_eq!(encoded[32], 200);
    //     assert_eq!(encoded[33], 0x00);
    //     assert_eq!(encoded[34], 40);
    //     assert_eq!(encoded[35], 0x03);
    //     assert_eq!(encoded[36], 0x50);
    //     assert_eq!(encoded[37], 0x00);

    //     let frame = GpsQzssFrame::default()
    //         .with_telemetry(
    //             GpsQzssTelemetry::default()
    //                 .with_message(0x9999)
    //                 .with_integrity()
    //                 .with_reserved_bit(),
    //         )
    //         .with_hand_over_word(
    //             GpsQzssHow::default()
    //                 .with_tow_seconds(0x9_9999)
    //                 .without_alert_bit()
    //                 .with_anti_spoofing(),
    //         )
    //         .with_subframe(GpsQzssSubframe::Ephemeris2(
    //             GpsQzssFrame2::default().with_iode(0x34),
    //         ));

    //     let encoded = frame.encode();

    //     assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
    //     assert_eq!(encoded[1], 0x19);
    //     assert_eq!(encoded[2], 0x99 << 2 | 0x02 | 0x01);
    //     assert_eq!(encoded[3], 0x03);

    //     assert_eq!(encoded[4], 0x33);
    //     assert_eq!(encoded[5], 0x32);
    //     assert_eq!(encoded[6], 0xA0);
    //     assert_eq!(encoded[7], 0x03);

    //     assert_eq!(encoded[8], 0x40);
    //     assert_eq!(encoded[9], 0x00);
    //     assert_eq!(encoded[10], 0x00);
    //     assert_eq!(encoded[11], 0x00);

    //     assert_eq!(encoded[12], 0x00);
    //     assert_eq!(encoded[13], 0x00);
    //     assert_eq!(encoded[14], 0x00);
    //     assert_eq!(encoded[15], 0x00);

    //     assert_eq!(encoded[16], 0x00);
    //     assert_eq!(encoded[17], 0x00);
    //     assert_eq!(encoded[18], 0x00);
    //     assert_eq!(encoded[19], 0x00);

    //     assert_eq!(encoded[20], 0x00);
    //     assert_eq!(encoded[21], 0x00);
    //     assert_eq!(encoded[22], 0x00);
    //     assert_eq!(encoded[23], 0x00);
    //     assert_eq!(encoded[24], 0x00);
    //     assert_eq!(encoded[25], 0x00);
    //     assert_eq!(encoded[26], 0x00);
    //     assert_eq!(encoded[27], 0x00);
    //     assert_eq!(encoded[28], 0x00);
    //     assert_eq!(encoded[29], 0x00);
    //     assert_eq!(encoded[30], 0x00);
    //     assert_eq!(encoded[31], 0x00);
    //     assert_eq!(encoded[32], 0x00);
    //     assert_eq!(encoded[33], 0x00);
    //     assert_eq!(encoded[34], 0x00);
    //     assert_eq!(encoded[35], 0x00);
    //     assert_eq!(encoded[36], 0x00);
    //     assert_eq!(encoded[37], 0x00);
    // }

    #[test]
    fn generate_eph1_bin() {
        let mut fd = File::create("data/GPS/eph1.bin").unwrap_or_else(|e| {
            panic!("Failed to create file: {}", e);
        });

        let mut frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x1234)
                    .with_integrity()
                    .with_reserved_bit(),
            )
            .with_hand_over_word(
                GpsQzssHow::default()
                    .with_tow_seconds(15_000)
                    .with_alert_bit()
                    .with_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris1(
                GpsQzssFrame1::default()
                    .with_week(0x123)
                    .with_iodc(0x1)
                    .with_all_signals_ok()
                    .with_l2p_flag()
                    .with_clock_offset_nanoseconds(1.0)
                    .with_clock_drift_seconds_s(1E-12)
                    .with_clock_drift_rate_seconds_s2(1E-15)
                    .with_reserved23_word(0x12_3456)
                    .with_reserved24_word1(0x34_5678)
                    .with_reserved24_word2(0x98_7654)
                    .with_total_group_delay_nanos(5.0)
                    .with_ca_or_p_l2_mask(0x3)
                    .with_user_range_accuracy_m(4.0),
            ));

        let encoded = frame.encode_raw();

        for i in 0..128 {
            let encoded = frame.encode_raw();

            fd.write(&encoded).unwrap_or_else(|e| {
                panic!("Failed to write encoded frame #{}: {}", i, e);
            });

            frame.telemetry.message += 1;
            frame.telemetry.integrity = !frame.telemetry.integrity;
            frame.telemetry.reserved_bit = !frame.telemetry.reserved_bit;

            frame.how.tow += 1;
            frame.how.alert = !frame.how.alert;
            frame.how.anti_spoofing = !frame.how.anti_spoofing;

            let subframe = frame.subframe.as_mut_eph1().unwrap();

            subframe.week += 1;
            subframe.ura += 1;
            subframe.ca_or_p_l2 = subframe.ca_or_p_l2 ^ 0x3;
            subframe.iodc += 1;
            subframe.health += 1;
            subframe.toc += 1;

            subframe.af0 += 1.0E-9;
            subframe.af1 += 1.0E-12;
            subframe.af2 += 1.0E-15;

            subframe.tgd += 1.0E-9;

            subframe.reserved_word4 += 1;
            subframe.reserved_word5 += 1;
            subframe.reserved_word6 += 1;
            subframe.reserved_word7 += 1;

            subframe.l2_p_data_flag = !subframe.l2_p_data_flag;
        }
    }

    #[test]
    fn eph1_bin_test() {
        let mut rd_ptr = 0;
        let mut buffer = [0; 1024];

        let mut fd = File::open("data/GPS/eph1.bin").unwrap();

        let mut size = fd.read(&mut buffer).unwrap();

        let mut decoder = GpsQzssDecoder::default();

        // grab first frame
        let (processed_size, decoded) = decoder.decode(&buffer[rd_ptr..], size);

        assert_eq!(processed_size, GPS_FRAME_BITS); // bits!

        let decoded = decoded.unwrap(); // success (we have 128 frames)

        assert_eq!(decoded.telemetry.message, 0x1234);
        assert_eq!(decoded.telemetry.integrity, true);
        assert_eq!(decoded.telemetry.reserved_bit, true);

        assert_eq!(decoded.how.tow, 15_000);
        assert_eq!(decoded.how.alert, true);
        assert_eq!(decoded.how.anti_spoofing, true);

        // bits->byte lazy conversion.
        // Frames are contiguous (no dead time)
        // Each frame is round(37.5)=37 byte long
        // Last bits are processed twice (lazily),
        // but the decoder correctly synchronizes itself.
        rd_ptr += processed_size / 8 - 1; // bytes!
        size -= processed_size / 8 - 1; // bytes!

        for i in 1..128 {
            let (processed_size, decoded) = decoder.decode(&buffer[rd_ptr..], size);

            // TODO residues
            // assert_eq!(processed_size, GPS_FRAME_BITS); // bits!

            // let decoded = decoded.unwrap(); // success (we have 128 frames)

            if i % 2 == 0 {
            } else {
            }
        }
    }

    // #[test]
    // fn generate_eph2_bin() {
    //     let mut fd = File::create("data/GPS/eph2.bin").unwrap_or_else(|e| {
    //         panic!("Failed to create file: {}", e);
    //     });

    //     let mut frame = GpsQzssFrame::default()
    //         .with_telemetry(
    //             GpsQzssTelemetry::default()
    //                 .with_message(0x3456)
    //                 .with_integrity()
    //                 .with_reserved_bit(),
    //         )
    //         .with_hand_over_word(
    //             GpsQzssHow::default()
    //                 .with_tow_seconds(0x9_8765)
    //                 .with_alert_bit()
    //                 .with_anti_spoofing(),
    //         )
    //         .with_subframe(GpsQzssSubframe::Ephemeris2(
    //             GpsQzssFrame2::default()
    //                 .with_toe(54_321)
    //                 .with_iode(0x01)
    //                 .with_mean_anomaly_semi_circles(0.1)
    //                 .with_mean_motion_difference_semi_circles(0.1)
    //                 .with_square_root_semi_major_axis(0.1)
    //                 .with_eccentricity(0.1)
    //                 .with_aodo(0x12)
    //                 .with_cuc_radians(0.1)
    //                 .with_cus_radians(0.1)
    //                 .with_fit_interval_flag(),
    //         ));

    //     let encoded = frame.encode();

    //     for i in 0..128 {
    //         let encoded = frame.encode();

    //         fd.write(&encoded).unwrap_or_else(|e| {
    //             panic!("Failed to write encoded frame #{}: {}", i, e);
    //         });

    //         frame.telemetry.message += 1;
    //         frame.telemetry.integrity = !frame.telemetry.integrity;
    //         frame.telemetry.reserved_bit = !frame.telemetry.reserved_bit;

    //         frame.how.tow += 1;
    //         frame.how.alert = !frame.how.alert;
    //         frame.how.anti_spoofing = !frame.how.anti_spoofing;

    //         let subframe = frame.subframe.as_mut_eph2().unwrap();

    //         subframe.toe += 1;
    //         subframe.aodo += 1;
    //         subframe.iode += 1;
    //         subframe.fit_int_flag = !subframe.fit_int_flag;
    //     }
    // }

    #[test]
    fn generate_burst_bin() {
        let mut fd = File::create("data/GPS/burst.bin").unwrap_or_else(|e| {
            panic!("Failed to create file: {}", e);
        });

        let mut eph_1 = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x3456)
                    .with_integrity()
                    .with_reserved_bit(),
            )
            .with_hand_over_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x9_8765)
                    .with_alert_bit()
                    .with_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris1(
                GpsQzssFrame1::default()
                    .with_week(0x123)
                    .with_iodc(0x1)
                    .with_all_signals_ok()
                    .with_l2p_flag()
                    .with_reserved23_word(0x12_3456)
                    .with_reserved24_word1(0x34_5678)
                    .with_reserved24_word2(0x98_7654)
                    .with_total_group_delay_nanos(5.0)
                    .with_ca_or_p_l2_mask(0x3)
                    .with_user_range_accuracy_m(4.0),
            ));

        // let mut eph_2 = GpsQzssFrame::default()
        //     .with_telemetry(
        //         GpsQzssTelemetry::default()
        //             .with_message(0x3457)
        //             .with_integrity()
        //             .with_reserved_bit(),
        //     )
        //     .with_hand_over_word(
        //         GpsQzssHow::default()
        //             .with_tow_seconds(0x9_8765)
        //             .with_alert_bit()
        //             .with_anti_spoofing(),
        //     )
        //     .with_subframe(GpsQzssSubframe::Ephemeris2(
        //         GpsQzssFrame2::default()
        //             .with_toe(54_326)
        //             .with_iode(0x01)
        //             .with_mean_anomaly_semi_circles(0.1)
        //             .with_mean_motion_difference_semi_circles(0.1)
        //             .with_square_root_semi_major_axis(0.1)
        //             .with_eccentricity(0.1)
        //             .with_aodo(0x12)
        //             .with_cuc_radians(0.1)
        //             .with_cus_radians(0.1)
        //             .with_fit_interval_flag(),
        //     ));

        for i in 0..128 {
            let encoded = eph_1.encode_raw();

            fd.write(&encoded).unwrap_or_else(|e| {
                panic!("Failed to write encoded frame #{}: {}", i, e);
            });

            eph_1.telemetry.message += 1;
            eph_1.telemetry.integrity = !eph_1.telemetry.integrity;
            eph_1.telemetry.reserved_bit = !eph_1.telemetry.reserved_bit;

            eph_1.how.tow += 1;
            eph_1.how.alert = !eph_1.how.alert;
            eph_1.how.anti_spoofing = !eph_1.how.anti_spoofing;

            let subframe = eph_1.subframe.as_mut_eph1().unwrap();

            // let encoded = eph_2.encode();

            // fd.write(&encoded).unwrap_or_else(|e| {
            //     panic!("Failed to write encoded frame #{}: {}", i, e);
            // });

            // eph_2.telemetry.message += 1;
            // eph_2.telemetry.integrity = !eph_2.telemetry.integrity;
            // eph_2.telemetry.reserved_bit = !eph_2.telemetry.reserved_bit;

            // eph_2.how.tow += 1;
            // eph_2.how.alert = !eph_2.how.alert;
            // eph_2.how.anti_spoofing = !eph_2.how.anti_spoofing;

            // let subframe = eph_2.subframe.as_mut_eph2().unwrap();

            // subframe.toe += 1;
            // subframe.aodo += 1;
            // subframe.iode += 1;
            // subframe.fit_int_flag = !subframe.fit_int_flag;
        }
    }
}
