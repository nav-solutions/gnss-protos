use crate::gps::{
    BitReader, GpsDataByte, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
    GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_WORDS_PER_FRAME, GPS_WORD_BITS,
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

    /// Encodes this [GpsQzssFrame] as a 10 MSBF [u32] word burst.
    /// GPS is 30-bit aligned which is not [u8] aligned, the final word contains padding.
    /// If you can accept a little bit of inefficiency, simply do not mind and stream the padding bits.
    /// Otherwise, discard them at the end of this frame and concatenate the next frame.
    pub fn encode(&self) -> [u8; GPS_FRAME_BYTES] {
        let mut encoded = [0; GPS_FRAME_BYTES];

        let how = self.how.encode();
        let telemetry = self.telemetry.encode();
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

    use crate::gps::{
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssTelemetry, GPS_FRAME_BITS,
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
        let default = GpsQzssFrame::default();
        let encoded = default.encode();
        let encoded_size = encoded.len();
        assert_eq!(encoded_size, GPS_FRAME_BYTES, "encoded invalid size!");

        assert_eq!(encoded[0], 0x8B, "does not start with preamble bits");
        assert_eq!(encoded[1], 0x00);
        assert_eq!(encoded[2], 0x00);
        assert_eq!(encoded[3], 0x00);

        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&encoded, encoded_size);
        assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
        assert_eq!(decoded, Some(default), "reciprocal failed");
    }

    #[test]
    fn ephemeris1() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut decoder = GpsQzssDecoder::default();

        for (ith, (tow, alert, anti_spoofing, frame_id, message, integrity, tlm_reserved_bits)) in
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
                    reserved_bits: *tlm_reserved_bits,
                },
                subframe: Default::default(),
            };

            let encoded = frame.encode();
            let encoded_size = encoded.len();
            assert_eq!(encoded_size, GPS_FRAME_BYTES, "encoded invalid size!");

            let (size, decoded) = decoder.decode(&encoded, encoded_size);
            assert_eq!(size, GPS_FRAME_BITS, "invalid size processed!");
            assert_eq!(decoded, Some(frame), "reciprocal failed");
        }
    }
}
