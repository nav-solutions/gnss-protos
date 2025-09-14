use crate::gps::{
    bytes::{ByteArray, GpsDataByte},
    BitReader, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe,
    GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE, GPS_WORDS_PER_FRAME,
    GPS_WORD_BITS,
};

#[cfg(feature = "log")]
use log::{error, trace};

/// Packs 38 bytes (10x 30-bit + 4bit padding) aligned correcty to [u8], ready to decode
///
/// ## Input
/// - slice: &[u8], will panic if not [GPS_FRAME_BYTES] byte long!
/// - preamble_offset in bits!
fn pack_align_slice<'a>(slice: &'a [u8], preamble_offset_bit: usize) -> &'a [u8] {
    // byte index
    let byte_index = preamble_offset_bit / 8;

    // bit index within byte
    let bit_index = preamble_offset_bit % 8;

    if bit_index == 0 {
        &slice[byte_index..byte_index + GPS_FRAME_BYTES]
    } else {
        panic!("not supported yet");
    }
}

/// Locates the preamble bit marker (sync byte) within a buffer
///
/// ## Input
/// - slice: slice of bytes, must be [GPS_FRAME_BYTES] byte long
/// - size: total number of bytes
///
/// ## Returns
/// - offset in bits !
fn find_preamble(slice: &[u8], size: usize) -> Option<usize> {
    for i in 0..size - GPS_FRAME_BYTES {
        if slice[i] == GPS_PREAMBLE_BYTE {
            return Some(i * 8);
        }

        // intra byte test
        let mut byte1_mask = 0x7F;
        let mut byte2_mask = 0x80;

        for j in 1..8 {
            let mut value = slice[i + 1];
            value >>= 8 - j;
            value |= (slice[i] & byte1_mask) << j;

            byte1_mask >>= 1;
            byte2_mask |= 0x1 << 8 - j;

            if value == GPS_PREAMBLE_BYTE {
                return Some(i * 8 + j);
            }
        }
    }

    None
}

impl GpsQzssFrame {
    /// Decodes a slice of [GpsDataByte]s collected, that must start
    /// correctly on the first synchronization byte. All bytes may have some
    /// padding, as a valid [GpsQzssFrame] is not aligned to [GPS_FRAME_BYTES]
    /// and will contain padding at some point (usually on frame termination).
    ///
    /// This method is not capable to synchronize itself anywhere other than
    /// the very first bit. Prefer the [Self::decode] from raw bytes method instead,
    /// especially when working with real-time streams.
    ///
    /// Warning: this method only supports padding on each word ending or termination,
    /// not intra word padding!
    ///
    /// ## Input
    /// - array of [GPS_FRAME_BYTES] [GpsDataByte]s
    /// - check_parity: true if parity verification is required
    ///
    /// ## Output
    /// - Decoded [GpsQzssFrame]
    pub fn decode_bytes(bytes: &[GpsDataByte], check_parity: bool) -> Option<GpsQzssFrame> {
        if bytes.len() < GPS_FRAME_BYTES {
            return None;
        }

        // TLM
        let telemetry = ByteArray::new(&bytes[0..4]).value_u32();

        let telemetry = match GpsQzssTelemetry::decode(telemetry) {
            Ok(tlm) => tlm,
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return None;
            },
            #[cfg(feature = "log")]
            Err(e) => {
                error!("invalid telemetry: 0x{:08X}", telemetry);
                return None;
            },
        };

        // HOW
        let how = ByteArray::new(&bytes[4..8]).value_u32();

        let how = match GpsQzssHow::decode(how) {
            Ok(how) => how,
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return None;
            },
            #[cfg(feature = "log")]
            Err(e) => {
                error!("invalid frame-id: 0x{:08X}", how);
                return None;
            },
        };

        Some(GpsQzssFrame {
            subframe: GpsQzssSubframe::decode(how.frame_id, &bytes[8..]),
            telemetry,
            how,
        })
    }

    /// Decodes the first valid [GpsQzssFrame] found in this read-only [u8] buffer.
    /// We expect raw bytes, unaligned to [u8] because raw GPS/QZSS stream
    /// is not aligned. Buffer must contain at least [GPS_FRAME_BYTES] bytes for this to work.
    ///
    /// This is [Self::encode] mirror operation.
    ///
    /// ## Input
    /// - buffer: read-only [u8] buffer
    ///
    /// ## Ouput
    /// - Optional [GpsQzssFrame] correctly decoded. First in order of appearance in the buffer.
    /// - Total number of _bits_ that were consumed (not bytes!).
    /// You are expected to discard all processed _bits_ not to decode the same frame twice.
    pub fn decode(buffer: &[u8], check_parity: bool) -> (Option<Self>, usize) {
        let mut size = 0;
        let mut byte_offset = 0;
        let mut bit_offset = 0;
        let mut preamble_offset = 0;

        let available = buffer.len();
        let available_bits = available * 8;

        if available < GPS_FRAME_BYTES {
            return (None, 0);
        }

        // locates preamble
        let preamble_offset_bin = find_preamble(buffer, available);

        if preamble_offset_bin.is_none() {
            return (None, available_bits);
        }

        // // Gathers & aligns 38 bytes (10x30 bit + 4bit padding)
        // println!("PREAMBLE : byte_offset={} bit_offset={}", byte_offset, bit_offset);

        // let mut bytes : [GpsDataByte; GPS_FRAME_BYTES] = [Default::default(); GPS_FRAME_BYTES];

        // for i in 0..GPS_FRAME_BYTES {
        //     if i % 4 == 0 {
        //         bytes[i] = GpsDataByte::MsbPadded(buffer[byte_offset +i]);
        //     } else {
        //         bytes[i] = GpsDataByte::Byte(buffer[byte_offset +i]);
        //     }
        // }

        // println!("BYTES: {:?}", bytes);

        // let telemetry = ByteArray::new(&bytes[0..4])
        //     .value_u32();

        // let telemetry = match GpsQzssTelemetry::decode(telemetry) {
        //     Ok(tlm) => tlm,
        //     #[cfg(not(feature = "log"))]
        //     Err(_) => {
        //         return (None, byte_offset * 8 + bit_offset);
        //     },
        //     #[cfg(feature = "log")]
        //     Err(e) => {
        //         error!("invalid TLM: {}", telemetry);
        //         return (None, byte_offset * 8 + bit_offset);
        //     },
        // };

        // let how = ByteArray::new(&bytes[4..8])
        //     .value_u32();

        // let how = match GpsQzssHow::decode(how) {
        //     Ok(how) => how,
        //     #[cfg(not(feature = "log"))]
        //     Err(_) => {
        //         return (None, byte_offset * 8 + bit_offset +32);
        //     },
        //     #[cfg(feature = "log")]
        //     Err(e) => {
        //         error!("invalid frame-id: {}", how);
        //         return (None, byte_offset * 8 + bit_offset +32);
        //     },
        // };

        // let processed_size = preamble_offset + Self::encoding_bits();

        // let frame = GpsQzssFrame {
        //     how,
        //     telemetry,
        //     subframe: Default::default(),
        // };

        // (Some(frame), processed_size)
        (None, available_bits)
    }
}

#[cfg(test)]
mod test {
    use std::{fs::File, io::Read};

    use crate::{
        gps::{GpsQzssFrame, GpsQzssFrameId, GPS_FRAME_BYTES},
        tests::{from_ublox_be_bytes, insert_zeros},
    };

    use super::{find_preamble, pack_align_slice};

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    #[test]
    fn preamble() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut buffer = [0; 1024];
        let mut file = File::open("data/GPS/two_frames.bin").unwrap();
        file.read(&mut buffer).unwrap();

        assert_eq!(find_preamble(&buffer, 1024), Some(0));

        // test delay < 1 byte
        for i in 1..7 {
            let delayed = insert_zeros(&buffer, i);
            assert_eq!(
                find_preamble(&delayed, 1024),
                Some(i),
                "failed for bit position {}",
                i
            );
        }

        // test 1 byte offset
        let delayed = insert_zeros(&buffer, 8);
        assert_eq!(
            find_preamble(&delayed, 1024),
            Some(8),
            "failed for bit position 8"
        );

        // test 1 byte + bits
        for i in 1..7 {
            let delayed = insert_zeros(&buffer, i + 8);
            assert_eq!(
                find_preamble(&delayed, 1024),
                Some(8 + i),
                "failed for bit position {}",
                8 + i
            );
        }

        // test other values
        for i in 2..9 {
            let delayed = insert_zeros(&buffer, i * 8);
            assert_eq!(
                find_preamble(&delayed, 1024),
                Some(i * 8),
                "failed for bit position {}",
                i * 8
            );

            for j in 1..7 {
                let delayed = insert_zeros(&buffer, i * 8 + j);
                assert_eq!(
                    find_preamble(&delayed, 1024),
                    Some(i * 8 + j),
                    "failed for bit position {}",
                    i * 8 + j
                );
            }
        }
    }

    #[test]
    fn slice_alignmnt_packing() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        assert_eq!(GPS_FRAME_BYTES, 38);

        let mut buffer = [0; 1024];
        let mut file = File::open("data/GPS/two_frames.bin").unwrap();
        file.read(&mut buffer).unwrap();

        let packed_aligned = pack_align_slice(&buffer, 0);

        assert_eq!(
            packed_aligned.len(),
            GPS_FRAME_BYTES,
            "did not extract correct amount of bytes!"
        );
    }

    #[test]
    fn eph1_bytes_decoding_noparity() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            // TLM
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
        ]);

        let decoded = GpsQzssFrame::decode_bytes(&bytes, false).unwrap_or_else(|| {
            panic!("Failed to decode valid message");
        });

        assert_eq!(decoded.telemetry.message, 0x13E);
        assert_eq!(decoded.telemetry.integrity, false);
        assert_eq!(decoded.telemetry.reserved_bits, false);

        assert_eq!(decoded.how.alert, false);
        assert_eq!(decoded.how.anti_spoofing, true);
        assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris1);

        let frame1 = decoded.subframe.as_eph1().unwrap_or_else(|| {
            panic!("Decoded invalid subframe");
        });

        assert!((frame1.af1 - 1.023181539495E-11).abs() < 1e-14);
        assert!((frame1.af0 - -4.524961113930E-04).abs() < 1.0e-11);
        assert_eq!(frame1.af2, 0.0);

        assert_eq!(frame1.week, 318);
        assert_eq!(frame1.toc, 266_400);
        assert_eq!(frame1.health, 0);
    }

    #[test]
    fn eph1_buffer_decoding_noparity() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut buffer = [0; 1024];
        let mut file = File::open("data/GPS/two_frames.bin").unwrap();
        file.read(&mut buffer).unwrap();

        let (frame, size) = GpsQzssFrame::decode(&buffer, false);

        assert_eq!(
            size,
            GpsQzssFrame::encoding_bits(),
            "returned invalid size!"
        );
        assert_eq!(frame, Some(Default::default()));
    }

    #[test]
    fn eph2_bytes_decoding_noparity() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            // TLM
            0x22, 0xC1, 0x3E, 0x1B, // HOW
            0x15, 0x27, 0xEA, 0x1B, // WORD3
            0x12, 0x7F, 0xF1, 0x65, //WORD4
            0x8C, 0x68, 0x1F, 0x7C, //WORD5
            0x02, 0x49, 0x34, 0x15, // WORD6
            0xBF, 0xF8, 0x81, 0x1E, //WORD7
            0x99, 0x1B, 0x81, 0x14, // W0RD8
            0x04, 0x3E, 0x68, 0x6E, // WORD9
            0x83, 0x34, 0x72, 0x21, // WORD10
            0x90, 0x42, 0x9F, 0x7B,
        ]);

        let decoded = GpsQzssFrame::decode_bytes(&bytes, false).unwrap_or_else(|| {
            panic!("Failed to decode valid message");
        });

        assert_eq!(decoded.telemetry.message, 0x13E);
        assert_eq!(decoded.telemetry.integrity, false);
        assert_eq!(decoded.telemetry.reserved_bits, false);

        assert_eq!(decoded.how.alert, false);
        assert_eq!(decoded.how.anti_spoofing, true);
        assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris2);

        let frame2 = decoded.subframe.as_eph2().unwrap_or_else(|| {
            panic!("Decoded invalid subframe");
        });

        assert_eq!(frame2.toe, 266_400);
        assert_eq!(frame2.crs, -1.843750000000e+000);
        assert!((frame2.sqrt_a - 5.153602432251e+003).abs() < 1e-9);
        assert!((frame2.m0 - 9.768415465951e-001).abs() < 1e-9);
        assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-9);
        assert!((frame2.e - 8.578718174249e-003).abs() < 1e-9);
        assert!((frame2.cus - 8.093193173409e-006).abs() < 1e-9);
        assert!((frame2.cuc - -5.587935447693e-008).abs() < 1e-6);
        assert!((frame2.dn - 1.444277586415e-009).abs() < 1e-9);
        assert_eq!(frame2.fit_int_flag, false);
    }

    #[test]
    fn eph3_bytes_decoding_noparity() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = from_ublox_be_bytes(&[
            0x22, 0xC1, 0x3E, 0x1B, 0x15, 0x28, 0x0B, 0xDB, 0x00, 0x0A, 0xEA, 0x34, 0x03, 0x3C,
            0xFF, 0xEE, 0xBF, 0xE5, 0xC9, 0xEB, 0x13, 0x6F, 0xB6, 0x4E, 0x86, 0xF4, 0xAB, 0x2C,
            0x06, 0x71, 0xEB, 0x44, 0x3F, 0xEA, 0xF6, 0x02, 0x92, 0x45, 0x52, 0x13,
        ]);

        let decoded = GpsQzssFrame::decode_bytes(&bytes, false).unwrap_or_else(|| {
            panic!("Failed to decode valid message");
        });

        assert_eq!(decoded.telemetry.message, 0x13E);
        assert_eq!(decoded.telemetry.integrity, false);
        assert_eq!(decoded.telemetry.reserved_bits, false);

        assert_eq!(decoded.how.alert, false);
        assert_eq!(decoded.how.anti_spoofing, true);
        assert_eq!(decoded.how.frame_id, GpsQzssFrameId::Ephemeris3);

        let frame3 = decoded.subframe.as_eph3().unwrap_or_else(|| {
            panic!("Decoded invalid subframe");
        });

        assert!((frame3.cic - 8.009374141693e-008).abs() < 1e-9);
        assert!((frame3.cis - -1.955777406693e-007).abs() < 1E-9);
        assert!((frame3.crc - 2.225625000000e+002).abs() < 1E-9);
        assert!((frame3.i0 - 3.070601043291e-001).abs() < 1e-9);
        assert!((frame3.idot - 1.548414729768e-010).abs() < 1E-9);
        assert!((frame3.omega0 - -6.871047024615e-001).abs() < 1e-9);
        assert!((frame3.omega_dot - -2.449269231874e-009).abs() < 1e-9);
        assert!((frame3.omega - -6.554632573389e-001).abs() < 1e-9);
    }
}
