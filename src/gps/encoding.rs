use crate::gps::{
    GpsDataByte, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
    GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME, GPS_WORD_BITS,
};

#[cfg(feature = "log")]
use log::{error, trace};

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

    /// Encodes this [GpsQzssFrame] as a 300 bit burst (38 bytes).
    /// Because [GpsQzssFrame] is not aligned to [u8], the very last byte contains 4 MSB padding bits, set to zeros
    /// (unsigned). If you leave it to that, any streaming/transmitter looses a little bit of efficiency
    /// any time a [GpsQzssFrame] is encoded/transmitted. The only solution then, is to manually remove this padding
    /// an truly concatenate your frames, but one can't expect easy processes when working with poorly designed and very old protocols.
    pub fn encode(&self) -> [u8; GPS_FRAME_BYTES] {
        let mut encoded = [0; GPS_FRAME_BYTES];

        encoded[0] = GPS_PREAMBLE_BYTE;
        encoded[1] = ((self.telemetry.message & 0xff00) >> 8) as u8;

        encoded[2] = (self.telemetry.message & 0xff) as u8;
        encoded[2] <<= 2;

        if self.telemetry.integrity {
            encoded[2] |= 0x02;
        }

        if self.telemetry.reserved_bit {
            encoded[2] |= 0x01;
        }

        encoded[3] <<= 2; // TODO
        encoded[3] |= ((self.how.tow & 0x1_8000) >> 15) as u8;

        encoded[4] = ((self.how.tow & 0x0_7f80) >> 7) as u8;

        encoded[5] = (self.how.tow & 0x0_007f) as u8;
        encoded[5] <<= 1;

        if self.how.alert {
            encoded[5] |= 0x01;
        }

        if self.how.anti_spoofing {
            encoded[6] |= 0x80;
        }

        encoded[6] |= self.how.frame_id.encode() << 4;

        encoded[7] <<= 4; // TODO

        match self.how.frame_id {
            GpsQzssFrameId::Ephemeris1 => {
                let subf = self.subframe.as_eph1().unwrap_or_default();

                encoded[7] |= ((subf.week & 0x3c0) >> 6) as u8;

                encoded[8] = (subf.week & 0x03f) as u8;
                encoded[8] <<= 2;
            },
            GpsQzssFrameId::Ephemeris2 => {
                let subf = self.subframe.as_eph2().unwrap_or_default();

                encoded[7] |= (subf.iode & 0xf0) >> 4;
                encoded[8] |= subf.iode & 0x0f;
                encoded[8] <<= 4;

                let crs = (subf.crs * 2.0_f64.powi(5)).round() as u16;
                encoded[8] |= ((crs & 0xf000) >> 12) as u8;
                encoded[9] |= ((crs & 0x0ff0) >> 4) as u8;
                encoded[10] |= (crs & 0x000f) as u8;
                encoded[10] <<= 4; // TODO

                let dn = (subf.dn * 2.0_f64.powi(43)).round() as u16;
                encoded[11] |= ((dn & 0xfc00) >> 10) as u8;

                encoded[12] |= ((dn & 0x03fc) >> 2) as u8;
                encoded[13] |= (dn & 0x0003) as u8;
                encoded[13] <<= 6;

                let m0 = (subf.m0 * 2.0_f64.powi(31)).round() as u32;

                encoded[13] |= ((m0 & 0xfc000000) >> 26) as u8;
                encoded[14] |= ((m0 & 0x03000000) >> 24) as u8;
                encoded[14] <<= 6; //TODO

                encoded[15] |= ((m0 & 0x00ff0000) >> 16) as u8;
                encoded[16] |= ((m0 & 0x0000ff00) >> 8) as u8;
                encoded[17] |= (m0 & 0x000000ff) as u8;

                encoded[18] <<= 2; // TODO

                let cuc = (subf.cuc * 2.0_f64.powi(29)).round() as u16;
                encoded[18] |= ((cuc & 0xc000) >> 14) as u8;
                encoded[19] |= ((cuc & 0x3fc0) >> 6) as u8;
                encoded[20] |= (cuc & 0x003f) as u8;
                encoded[20] <<= 2;

                let e = (subf.e * 2.0_f64.powi(33)).round() as u32;

                encoded[20] |= ((e & 0xc0000000) >> 30) as u8;
                encoded[21] |= ((e & 0x3f000000) >> 24) as u8;
                encoded[21] <<= 2; // TODO
                encoded[22] <<= 4; // TODO

                encoded[22] |= ((e & 0x00f00000) >> 20) as u8;
                encoded[23] |= ((e & 0x000ff000) >> 12) as u8;
                encoded[24] |= ((e & 0x00000ff0) >> 4) as u8;
                encoded[25] |= (e & 0x0000000f) as u8;
                encoded[25] <<= 4; // TODO
                encoded[26] <<= 2; // TODO

                let cus = (subf.cus * 2.0_f64.powi(29)).round() as u16;
                encoded[26] |= ((cus & 0xfc00) >> 10) as u8;
                encoded[27] |= ((cus & 0x03fc) >> 2) as u8;
                encoded[28] |= (cus & 0x3) as u8;
                encoded[28] <<= 6;

                let sqrt_a = (subf.sqrt_a * 2.0_f64.powi(19)).round() as u32;
                encoded[28] |= ((sqrt_a & 0xfc000000) >> 26) as u8;
                encoded[29] |= ((sqrt_a & 0x03000000) >> 24) as u8;
                encoded[29] <<= 6; // TODO

                encoded[30] |= ((sqrt_a & 0x00ff0000) >> 16) as u8;
                encoded[31] |= ((sqrt_a & 0x0000ff00) >> 8) as u8;
                encoded[32] |= (sqrt_a & 0x000000ff) as u8;

                let toe = (subf.toe * 16) as u16;

                encoded[33] <<= 2; // TODO
                encoded[33] |= ((toe & 0xc000) >> 14) as u8;
                encoded[34] |= ((toe & 0x3fc0) >> 6) as u8;
                encoded[35] |= (toe & 0x003f) as u8;
                encoded[35] <<= 2;

                if subf.fit_int_flag {
                    encoded[35] |= 0x02;
                }

                encoded[35] |= (subf.aodo & 0x10) >> 4;

                encoded[36] |= subf.aodo & 0x0f;
                encoded[36] <<= 4;

                encoded[36] |= 0x00; // two non-information bits for parity calculations
                encoded[37] |= 0x00;
                encoded[37] <<= 4; // TODO
            },
            GpsQzssFrameId::Ephemeris3 => {
                let subf = self.subframe.as_eph3().unwrap_or_default();

                let cic = (subf.cic * 2.0_f64.powi(29)).round() as u16;

                encoded[7] |= ((cic & 0xf000) >> 12) as u8;
                encoded[8] |= (cic & 0x0ff0) as u8;
                encoded[8] <<= 4;

                encoded[9] |= (cic & 0x000f) as u8;
                encoded[9] <<= 4;
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

        let encoded = self.encode();
        buffer[..GPS_FRAME_BYTES].copy_from_slice(&self.encode());
        Ok(GPS_FRAME_BYTES)
    }
}

#[cfg(test)]
mod encoding {
    use std::fs::File;
    use std::io::Read;

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    use crate::gps::{
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrameId,
        GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
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
        let default = GpsQzssFrame::default();
        let encoded = default.encode();
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
        assert_eq!(encoded[6], 0x00);
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

        // let mut decoder = GpsQzssDecoder::default();

        // let (size, decoded) = decoder.decode(&encoded, encoded_size);
        // assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
        // assert_eq!(decoded, Some(default), "reciprocal failed");
    }

    #[test]
    fn ephemeris1_0() {
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
            .with_how_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x5_6789)
                    .with_alert_bit()
                    .with_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris1(
                GpsQzssFrame1::default().with_week(0x123),
            ));

        let encoded = frame.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x12);
        assert_eq!(encoded[2], 0x34 << 2 | 0x02 | 0x01);
        assert_eq!(encoded[3], 0x02);

        assert_eq!(encoded[4], 0xCF);
        assert_eq!(encoded[5], 0x13);
        assert_eq!(encoded[6], 0x90);
        assert_eq!(encoded[7], 0x04);

        assert_eq!(encoded[8], 0x8c);
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

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x1234)
                    .without_integrity()
                    .with_reserved_bit(),
            )
            .with_how_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x4_6789)
                    .with_alert_bit()
                    .without_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::default());

        let encoded = frame.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x12);
        assert_eq!(encoded[2], 0x34 << 2 | 0x01);
        assert_eq!(encoded[3], 0x00);

        assert_eq!(encoded[4], 0xCF);
        assert_eq!(encoded[5], 0x13);
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

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x0123)
                    .without_integrity()
                    .without_reserved_bit(),
            )
            .with_how_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x0_1234)
                    .without_alert_bit()
                    .without_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::default());

        let encoded = frame.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x01);
        assert_eq!(encoded[2], 0x23 << 2);
        assert_eq!(encoded[3], 0x00);

        assert_eq!(encoded[4], 0x24);
        assert_eq!(encoded[5], 0x68);
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
    }

    #[test]
    fn ephemeris1_1() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut decoder = GpsQzssDecoder::default();

        for (ith, (tow, alert, anti_spoofing, frame_id, message, integrity, tlm_reserved_bit)) in
            [(
                259956,
                false,
                false,
                GpsQzssFrameId::Ephemeris1,
                0x13E,
                false,
                false,
            )]
            .iter()
            .enumerate()
        {
            let frame = GpsQzssFrame {
                how: GpsQzssHow {
                    tow: *tow,
                    alert: *alert,
                    frame_id: *frame_id,
                    anti_spoofing: *anti_spoofing,
                },
                telemetry: GpsQzssTelemetry {
                    message: *message,
                    integrity: *integrity,
                    reserved_bit: *tlm_reserved_bit,
                },
                subframe: Default::default(),
            };

            let encoded = frame.encode();
            let encoded_size = encoded.len();
            assert_eq!(encoded_size, GPS_FRAME_BYTES, "encoded invalid size!");

            // let (size, decoded) = decoder.decode(&encoded, encoded_size);
            // assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
            // assert_eq!(decoded, Some(frame), "reciprocal failed");
        }
    }

    #[test]
    fn ephemeris2_0() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut decoder = GpsQzssDecoder::default();

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x9999)
                    .with_integrity()
                    .with_reserved_bit(),
            )
            .with_how_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x9_9999)
                    .with_alert_bit()
                    .with_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris2(
                GpsQzssFrame2::default()
                    .with_iode(0x12)
                    .with_crs_meters(1.8)
                    .with_mean_motion_difference_semi_circles(100.0)
                    .with_mean_anomaly_semi_circles(9.768415465951e-001)
                    .with_toe(266_400)
                    .with_square_root_semi_major_axis(5.153602432251e+003)
                    .with_fit_interval_flag()
                    .with_aodo(0x15),
            ));

        let encoded = frame.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x19);
        assert_eq!(encoded[2], 0x99 << 2 | 0x02 | 0x01);
        assert_eq!(encoded[3], 0x03);

        assert_eq!(encoded[4], 0x33);
        assert_eq!(encoded[5], 0x33);
        assert_eq!(encoded[6], 0xA0);
        assert_eq!(encoded[7], 0x01);

        assert_eq!(encoded[8], 0x20);
        assert_eq!(encoded[9], 0x03);
        assert_eq!(encoded[10], 0xA0);
        assert_eq!(encoded[11], 0x3F);

        assert_eq!(encoded[12], 0xFF);
        assert_eq!(encoded[13], 0xDF);
        assert_eq!(encoded[14], 0x40);
        assert_eq!(encoded[15], 0x09);
        assert_eq!(encoded[16], 0x24);
        assert_eq!(encoded[17], 0xD0);
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
        assert_eq!(encoded[28], 40);
        assert_eq!(encoded[29], 64);
        assert_eq!(encoded[30], 12);
        assert_eq!(encoded[31], 209);
        assert_eq!(encoded[32], 200);
        assert_eq!(encoded[33], 0x00);
        assert_eq!(encoded[34], 40);
        assert_eq!(encoded[35], 0x03);
        assert_eq!(encoded[36], 0x50);
        assert_eq!(encoded[37], 0x00);

        let frame = GpsQzssFrame::default()
            .with_telemetry(
                GpsQzssTelemetry::default()
                    .with_message(0x9999)
                    .with_integrity()
                    .with_reserved_bit(),
            )
            .with_how_word(
                GpsQzssHow::default()
                    .with_tow_seconds(0x9_9999)
                    .without_alert_bit()
                    .with_anti_spoofing(),
            )
            .with_subframe(GpsQzssSubframe::Ephemeris2(
                GpsQzssFrame2::default().with_iode(0x34),
            ));

        let encoded = frame.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x19);
        assert_eq!(encoded[2], 0x99 << 2 | 0x02 | 0x01);
        assert_eq!(encoded[3], 0x03);

        assert_eq!(encoded[4], 0x33);
        assert_eq!(encoded[5], 0x32);
        assert_eq!(encoded[6], 0xA0);
        assert_eq!(encoded[7], 0x03);

        assert_eq!(encoded[8], 0x40);
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
    }
}
