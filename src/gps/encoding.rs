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

    /// Encodes this [GpsQzssFrame] as a 300 bit burst (10 data words).
    /// Because [GpsQzssFrame] is not aligned to [u8], the very last byte contains 3 padding bits
    /// in the most-significant position (all set to zeros by this method, unsigned data words).
    /// If you leave it to that, in a streaming/transmitter application, you loose a little bit of efficiency
    /// for every [GpsQzssFrame] in your message bursts. The only solution then is to manually remove the padding bits
    /// in the buffer, but working with old and poorly designed protocols can only have downsides.
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

        encoded[3] = ((self.how.tow & 0x1_8000) >> 15) as u8;
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

        // encoded[7] = (how & 0xff) as u8;
        // encoded[6] = ((how & 0xff00) >> 8) as u8;
        // encoded[5] = ((how & 0xff0000) >> 16) as u8;
        // encoded[4] = ((how & 0xff000000) >> 24) as u8;

        // encoded[0] = self.how.encode();
        // encoded[1] = self.telemetry.encode();
        // encoded[2..].copy_from_slice(&self.subframe.encode());

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
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
        GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
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
            .with_subframe(GpsQzssSubframe::default());

        let encoded = frame.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x12);
        assert_eq!(encoded[2], 0x34 << 2 | 0x02 | 0x01);
        assert_eq!(encoded[3], 0x02);

        assert_eq!(encoded[4], 0xCF);
        assert_eq!(encoded[5], 0x13);
        assert_eq!(encoded[6], 0x90);
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
}
