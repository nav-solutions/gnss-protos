use crate::gps::{
    GpsDataWord, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
    GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME,
    GPS_WORD_BITS,
};

#[cfg(feature = "log")]
use log::{debug, error, trace};

impl GpsQzssFrame {
    /// Decodes a burst of [GPS_WORDS_PER_FRAME] [GpsDataWord]s, with possible
    /// parity verification.
    ///
    /// This method is not capable to synchronize itself anywhere other than
    /// the very first byte. When working with raw stream of bits,
    /// you will prefer working with the [GpsQzssDecoder].
    ///
    /// ## Input
    /// - array of [GPS_WORDS_PER_FRAME] [GpsDataWord]s.
    /// - check_parity: true if parity verification is required.
    /// In this case, the method will return None on parity errors.
    ///
    /// ## Output
    /// - Decoded [GpsQzssFrame]
    pub fn decode(
        words: &[GpsDataWord; GPS_WORDS_PER_FRAME],
        check_parity: bool,
    ) -> Option<GpsQzssFrame> {
        // TLM
        let telemetry = match GpsQzssTelemetry::from_word(words[0]) {
            Ok(tlm) => tlm,
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return None;
            },
            #[cfg(feature = "log")]
            Err(_) => {
                error!("invalid/unknown frame: {:?}", words[1]);
                return None;
            },
        };

        // HOW
        let how = match GpsQzssHow::from_word(words[1]) {
            Ok(how) => how,
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return None;
            },
            #[cfg(feature = "log")]
            Err(_) => {
                error!("invalid/unknown frame: {:?}", words[1]);
                return None;
            },
        };

        Some(GpsQzssFrame {
            subframe: GpsQzssSubframe::decode(how.frame_id, &words[2..]),
            telemetry,
            how,
        })
    }
}

#[cfg(test)]
mod test {
    use std::{fs::File, io::Read};

    use crate::{
        gps::{GpsQzssFrame, GpsQzssFrameId, GPS_FRAME_BYTES},
        tests::{from_ublox_bytes, insert_zeros},
    };

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    // #[test]
    // fn eph1_bytes_decoding_noparity() {
    //     #[cfg(all(feature = "std", feature = "log"))]
    //     init_logger();

    //     let mut found = false;

    //     let bytes = from_ublox_be_bytes(&[
    //         // TLM
    //         0x22, 0xC1, 0x3E, 0x1B, // HOW
    //         0x15, 0x27, 0xC9, 0x73, // WORD3
    //         0x13, 0xE4, 0x00, 0x04, // WORD4
    //         0x10, 0x4F, 0x5D, 0x31, // WORD5
    //         0x97, 0x44, 0xE6, 0xD7, // WORD6
    //         0x07, 0x75, 0x57, 0x83, // WORD7
    //         0x33, 0x0C, 0x80, 0xB5, // WORD8
    //         0x92, 0x50, 0x42, 0xA1, // WORD9
    //         0x80, 0x00, 0x16, 0x84, // WORD10
    //         0x31, 0x2C, 0x30, 0x33,
    //     ]);

    //     let decoded = GpsQzssFrame::decode(&bytes, 40, false).unwrap_or_else(|| {
    //         panic!("Failed to decode valid message");
    //     });

    //     assert_eq!(decoded.telemetry.message, 0x13E);
    //     assert_eq!(decoded.telemetry.integrity, false);
    //     assert_eq!(decoded.telemetry.reserved_bit, false);

    //     assert_eq!(decoded.how.alert, false);
    //     assert_eq!(decoded.how.anti_spoofing, true);
    //     assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris1);

    //     let frame1 = decoded.subframe.as_eph1().unwrap_or_else(|| {
    //         panic!("Decoded invalid subframe");
    //     });

    //     assert!((frame1.af1 - 1.023181539495E-11).abs() < 1e-14);
    //     assert!((frame1.af0 - -4.524961113930E-04).abs() < 1.0e-11);
    //     assert_eq!(frame1.af2, 0.0);

    //     assert_eq!(frame1.week, 318);
    //     assert_eq!(frame1.toc, 266_400);
    //     assert_eq!(frame1.health, 0);
    // }

    // #[test]
    // fn eph2_bytes_decoding_noparity() {
    //     #[cfg(all(feature = "std", feature = "log"))]
    //     init_logger();

    //     let mut found = false;

    //     let bytes = from_ublox_be_bytes(&[
    //         // TLM
    //         0x22, 0xC1, 0x3E, 0x1B, // HOW
    //         0x15, 0x27, 0xEA, 0x1B, // WORD3
    //         0x12, 0x7F, 0xF1, 0x65, // WORD4
    //         0x8C, 0x68, 0x1F, 0x7C, // WORD5
    //         0x02, 0x49, 0x34, 0x15, // WORD6
    //         0xBF, 0xF8, 0x81, 0x1E, // WORD7
    //         0x99, 0x1B, 0x81, 0x14, // W0RD8
    //         0x04, 0x3E, 0x68, 0x6E, // WORD9
    //         0x83, 0x34, 0x72, 0x21, // WORD10
    //         0x90, 0x42, 0x9F, 0x7B,
    //     ]);

    //     let decoded = GpsQzssFrame::decode(&bytes, 40, false).unwrap_or_else(|| {
    //         panic!("Failed to decode valid message");
    //     });

    //     assert_eq!(decoded.telemetry.message, 0x13E);
    //     assert_eq!(decoded.telemetry.integrity, false);
    //     assert_eq!(decoded.telemetry.reserved_bit, false);

    //     assert_eq!(decoded.how.alert, false);
    //     assert_eq!(decoded.how.anti_spoofing, true);
    //     assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris2);

    //     let frame2 = decoded.subframe.as_eph2().unwrap_or_else(|| {
    //         panic!("Decoded invalid subframe");
    //     });

    //     assert_eq!(frame2.toe, 266_400);
    //     assert_eq!(frame2.crs, -1.843750000000e+000);
    //     assert!((frame2.sqrt_a - 5.153602432251e+003).abs() < 1e-9);
    //     assert!((frame2.m0 - 9.768415465951e-001).abs() < 1e-9);
    //     assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-9);
    //     assert!((frame2.e - 8.578718174249e-003).abs() < 1e-9);
    //     assert!((frame2.cus - 8.093193173409e-006).abs() < 1e-9);
    //     assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-6);
    //     assert!((frame2.dn - 1.444277586415e-009).abs() < 1e-9);
    //     assert_eq!(frame2.fit_int_flag, false);
    // }

    // #[test]
    // fn eph3_bytes_decoding_noparity() {
    //     #[cfg(all(feature = "std", feature = "log"))]
    //     init_logger();

    //     let mut found = false;

    //     let bytes = from_ublox_be_bytes(&[
    //         // TLM
    //         0x22, 0xC1, 0x3E, 0x1B, // HOW
    //         0x15, 0x28, 0x0B, 0xDB, // WORD3
    //         0x00, 0x0A, 0xEA, 0x34, // WORD4
    //         0x03, 0x3C, 0xFF, 0xEE, // WORD5
    //         0xBF, 0xE5, 0xC9, 0xEB, // WORD6
    //         0x13, 0x6F, 0xB6, 0x4E, // WORD7
    //         0x86, 0xF4, 0xAB, 0x2C, // WORD8
    //         0x06, 0x71, 0xEB, 0x44, // WORD9
    //         0x3F, 0xEA, 0xF6, 0x02, // WORD10
    //         0x92, 0x45, 0x52, 0x13,
    //     ]);

    //     let decoded = GpsQzssFrame::decode(&bytes, 40, false).unwrap_or_else(|| {
    //         panic!("Failed to decode valid message");
    //     });

    //     assert_eq!(decoded.telemetry.message, 0x13E);
    //     assert_eq!(decoded.telemetry.integrity, false);
    //     assert_eq!(decoded.telemetry.reserved_bit, false);

    //     assert_eq!(decoded.how.alert, false);
    //     assert_eq!(decoded.how.anti_spoofing, true);
    //     assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris3);

    //     let frame3 = decoded.subframe.as_eph3().unwrap_or_else(|| {
    //         panic!("Decoded invalid subframe");
    //     });

    //     assert!((frame3.cic - 8.009374141693e-008).abs() < 1e-9);
    //     assert!((frame3.cis - -1.955777406693e-007).abs() < 1E-9);
    //     assert!((frame3.crc - 2.225625000000e+002).abs() < 1E-9);
    //     assert!((frame3.i0 - 3.070601043291e-001).abs() < 1e-9);
    //     assert!((frame3.idot - 1.548414729768e-010).abs() < 1E-9);
    //     assert!((frame3.omega0 - -6.871047024615e-001).abs() < 1e-9);
    //     assert!((frame3.omega_dot - -2.449269231874e-009).abs() < 1e-9);
    //     assert!((frame3.omega - -6.554632573389e-001).abs() < 1e-9);
    // }
}
