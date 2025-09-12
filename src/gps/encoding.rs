use crate::gps::{
    BitReader, GpsDataByte, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
    GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_WORDS_PER_FRAME, GPS_WORD_BITS,
};

#[cfg(feature = "log")]
use log::{error, trace};

impl GpsQzssFrame {
    /// Returns total number of bytes needed to encode this [GpsQzssFrame] to binary
    /// aligned to [u8]
    pub const fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }

    /// Returns exact number of bits needed to encode this [GpsQzssFrame]
    pub const fn encoding_bits(&self) -> usize {
        GPS_FRAME_BITS
    }

    /// Encodes this [GpsQzssFrame] as a 10 MSBF [u32] word burst.
    /// GPS is 30-bit aligned which is not [u8] aligned, the final word contains padding.
    /// If you can accept a little bit of inefficiency, simply do not mind and stream the padding bits.
    /// Otherwise, discard them at the end of this frame and concatenate the next frame.
    pub fn encode(&self) -> [u8; GPS_FRAME_BYTES] {
        let mut encoded = [0; GPS_FRAME_BYTES];

        let telemetry = self.telemetry.encode();
        let how = self.how.encode();
        let subframe = self.subframe.encode();

        // encoded[3] = ((telemetry & 0xff) << 2) as u8;
        // encoded[2] = ((telemetry & 0xff00) >> 8 ) as u8;
        // encoded[1] = ((telemetry & 0xff0000) >> 16) as u8;
        encoded[0] = (((telemetry & 0xfff00000) << 2) >> 24) as u8;
        encoded[1] = (((telemetry & 0x00fff000) << 2) >> 16) as u8;
        encoded[2] = (((telemetry & 0x0000fff0) << 2) >> 8) as u8;
        encoded[3] = (((telemetry & 0x0000000f) << 2) >> 0) as u8;

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

    use crate::gps::{GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssTelemetry, GPS_FRAME_BITS};

    #[test]
    fn encoding_size() {
        assert_eq!(GpsQzssFrame::default().encoding_size(), 300 / 8 + 1);
    }

    #[test]
    fn encoding_bits() {
        assert_eq!(GpsQzssFrame::default().encoding_bits(), 300);
    }

    #[test]
    fn default_frame() {
        let default = GpsQzssFrame::default();
        let encoded = default.encode();

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x00);
        assert_eq!(encoded[2], 0x00);
        assert_eq!(encoded[3], 0x00);

        let (decoded, size) = GpsQzssFrame::decode(&encoded);

        assert_eq!(decoded, Some(default), "reciprocal failed");
    }

    #[test]
    #[ignore]
    fn two_frames_decoding_bin() {
        let mut buffer = [0; 1024];

        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut file = File::open("two_frames.bin").unwrap();

        let content = file.read(&mut buffer).unwrap();
    }

    #[test]
    #[ignore]
    fn test_eph1_buffer_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let buffer = [
            0x22, 0xC1, 0x3E, 0x1B, // HOW
            0x15, 0x27, 0xC9, 0x73, // WORD3
            0x13, 0xE4, 0x00, 0x04, //WORD4
            0x10, 0x4F, 0x5D, 0x31, //WORD5
            0x97, 0x44, 0xE6, 0xD7, // WORD6
            0x07, 0x75, 0x57, 0x83, //WORD7
            0x33, 0x0C, 0x80, 0xB5, // WORD8
            0x92, 0x50, 0x42, 0xA1, // WORD9
            0x80, 0x00, 0x16, 0x84, //WORD10
            0x31, 0x2C, 0x30, 0x33,
        ];
    }

    #[test]
    fn ephemeris1() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        for (
            ith,
            (tow, alert, anti_spoofing, frame_id, message, integrity, tlm_reserved_bits, bytes),
        ) in [(
            259956,
            false,
            false,
            GpsQzssFrameId::Ephemeris1,
            0x13E,
            false,
            false,
            [
                0x8B, 0x04, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
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
                    reserved_bits: *tlm_reserved_bits,
                },
                subframe: Default::default(),
            };

            let encoded = frame.encode();

            for (i, expected) in bytes.iter().enumerate() {
                assert_eq!(
                    encoded[i], *expected,
                    "TEST #{} BYTE({}) expected 0x{:02X}, got 0x{:02X}",
                    ith, i, bytes[i], encoded[i]
                );
            }

            let (decoded, size) = GpsQzssFrame::decode(&encoded);
            assert_eq!(size, GPS_FRAME_BITS);

            assert_eq!(decoded, Some(frame), "reciprocal failed");
        }
    }
}
