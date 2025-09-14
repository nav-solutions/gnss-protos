use crate::gps::{
    GpsQzssFrame, GpsQzssHow, GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE,
};

#[cfg(feature = "log")]
use log::{debug, error, trace};

/// [GpsQzssDecoder] can decode GPS (or QZSS) protocol.
/// By [Default], our [GpsQzssDecoder] does not verify parity.
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssDecoder {
    /// Aligned bytes
    buffer: [u8; GPS_FRAME_BYTES],

    /// True when parity verification is requested
    parity_verification: bool,
}

impl Default for GpsQzssDecoder {
    /// Creates a default [GpsQzssDecoder] that does not verify parity.
    fn default() -> Self {
        Self {
            buffer: [0; GPS_FRAME_BYTES],
            parity_verification: false,
        }
    }
}

impl GpsQzssDecoder {
    /// Creates a new [GpsQzssDecoder] with parity verification.
    /// Our [Default] implementation does not verify the parity bits, and
    /// tolerates parity error. When switching to this implementation,
    /// the parity bits must be correct for the decoder to accept a frame.
    pub fn with_parity_verification(mut self) -> Self {
        self.parity_verification = true;
        self
    }

    /// Packs 38 bytes (10x 30-bit + 4bit padding) correcty aligned to [u8], ready to process.
    ///
    /// ## Input
    /// - slice: &[u8], will panic if not [GPS_FRAME_BYTES] byte long!
    /// - preamble_offset in bits!
    pub(crate) fn resync_align(&mut self, slice: &[u8], preamble_offset_bit: usize) {
        // byte index
        let byte_index = preamble_offset_bit / 8;

        // bit index within byte
        let bit_index = preamble_offset_bit % 8;

        if bit_index == 0 {
            self.buffer
                .copy_from_slice(&slice[byte_index..byte_index + GPS_FRAME_BYTES]);
        } else {
            panic!("not supported yet");
            // slice[byte_index..byte_index + GPS_FRAME_BYTES]
            //     .into_iter()
            //     .map(|b| {
            //         *b
            //     })
            //     .collect()
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
    pub(crate) fn find_preamble(slice: &[u8], size: usize) -> Option<usize> {
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

    /// Decodes the first valid [GpsQzssFrame] found in this read-only [u8] buffer.
    /// [GpsQzssDecoder] will align itself to the Sync byte, which is not aligned to [u8],
    /// because GPS/QZSS is made of 30 bit data words.
    ///
    /// ## Input
    /// - buffer: read-only [u8] buffer
    /// - size: buffer size (in bytes)
    ///
    /// ## Ouput
    /// - Total number of _bits_ that were consumed (not bytes!).
    /// You are expected to discard all processed _bits_ not to decode the same frame twice.
    /// - Optional [GpsQzssFrame] correctly decoded. First in order of appearance in the buffer.
    pub fn decode(&mut self, buffer: &[u8], size: usize) -> (usize, Option<GpsQzssFrame>) {
        // locate preamble
        let preamble_offset_bit = Self::find_preamble(buffer, size);

        if preamble_offset_bit.is_none() {
            return (size * 8 - GPS_FRAME_BITS, None);
        }

        // realign and pack as [u8]
        let preamble_offset_bit = preamble_offset_bit.unwrap();

        #[cfg(feature = "log")]
        trace!("(GPS/QZSS)  [preamble]: pos={}", preamble_offset_bit);

        self.resync_align(buffer, preamble_offset_bit);

        let telemetry_dword = u32::from_be_bytes([
            self.buffer[0],
            self.buffer[1],
            self.buffer[2],
            self.buffer[3],
        ]);

        let shifted_dword = telemetry_dword >> 2;
        let parity = (telemetry_dword & 0xfc) >> 2;

        #[cfg(feature = "log")]
        trace!("(GPS/QZSS) [telemetry]: par={}", parity);

        let telemetry = match GpsQzssTelemetry::decode(shifted_dword) {
            Ok(telemetry) => {
                #[cfg(feature = "log")]
                debug!("(GPS/QZSS) [telemetry]: {}", telemetry);
                telemetry
            },
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
            #[cfg(feature = "log")]
            Err(e) => {
                error!("(GPS/QZSS) [telemetry]: {}", e);
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
        };

        let how_dword = u32::from_be_bytes([
            self.buffer[4],
            self.buffer[5],
            self.buffer[6],
            self.buffer[7],
        ]);

        let mut shifted_dword = (telemetry_dword & 0x3) << 30;
        shifted_dword |= how_dword >> 6;

        let parity = (how_dword & 0xfc) >> 2;
        trace!("(GPS/QZSS)       [how]:    ={:08X}", shifted_dword);

        #[cfg(feature = "log")]
        trace!("(GPS/QZSS)       [how]: par={}", parity);

        let how = match GpsQzssHow::decode(shifted_dword) {
            Ok(how) => {
                #[cfg(feature = "log")]
                debug!("(GPS/QZSS) [how]: {}", how);
                how
            },
            #[cfg(not(feature = "log"))]
            Err(_) => {
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
            #[cfg(feature = "log")]
            Err(e) => {
                error!("(GPS/QZSS) [how]: {}", e);
                return (preamble_offset_bit + GPS_FRAME_BITS, None);
            },
        };

        // now process the 8 data words
        for i in 0..8 {}

        let frame = GpsQzssFrame {
            telemetry,
            how,
            subframe: Default::default(),
        };

        return (preamble_offset_bit + GPS_FRAME_BITS, Some(frame));
    }
}

#[cfg(test)]
mod decoder {
    use std::{fs::File, io::Read};

    use crate::{
        gps::{GpsQzssDecoder, GPS_FRAME_BITS, GPS_FRAME_BYTES},
        tests::{insert_zeros, GPS_EPH1_DATA},
    };

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    #[test]
    fn preamble() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut buffer = [0; 1024];
        let mut file = File::open("data/GPS/eph-1.bin").unwrap();
        file.read(&mut buffer).unwrap();

        assert_eq!(GpsQzssDecoder::find_preamble(&buffer, 1024), Some(0));

        // test delay < 1 byte
        for i in 1..7 {
            let delayed = insert_zeros(&buffer, i);
            assert_eq!(
                GpsQzssDecoder::find_preamble(&delayed, 1024),
                Some(i),
                "failed for bit position {}",
                i
            );
        }

        // test 1 byte offset
        let delayed = insert_zeros(&buffer, 8);
        assert_eq!(
            GpsQzssDecoder::find_preamble(&delayed, 1024),
            Some(8),
            "failed for bit position 8"
        );

        // test 1 byte + bits
        for i in 1..7 {
            let delayed = insert_zeros(&buffer, i + 8);
            assert_eq!(
                GpsQzssDecoder::find_preamble(&delayed, 1024),
                Some(8 + i),
                "failed for bit position {}",
                8 + i
            );
        }

        // test other values
        for i in 2..9 {
            let delayed = insert_zeros(&buffer, i * 8);
            assert_eq!(
                GpsQzssDecoder::find_preamble(&delayed, 1024),
                Some(i * 8),
                "failed for bit position {}",
                i * 8
            );

            for j in 1..7 {
                let delayed = insert_zeros(&buffer, i * 8 + j);
                assert_eq!(
                    GpsQzssDecoder::find_preamble(&delayed, 1024),
                    Some(i * 8 + j),
                    "failed for bit position {}",
                    i * 8 + j
                );
            }
        }
    }

    #[test]
    fn decoder_test_data() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&GPS_EPH1_DATA, GPS_EPH1_DATA.len());

        assert_eq!(size, GPS_FRAME_BITS, "returned invalid size");

        let decoded = decoded.unwrap_or_else(|| {
            panic!("did not decoded GPS frame!");
        });

        assert_eq!(decoded.telemetry.message, 0);
        assert_eq!(decoded.telemetry.integrity, false);
        assert_eq!(decoded.telemetry.reserved_bit, false);
    }

    #[test]
    fn decoder_raw_data() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        assert_eq!(GPS_FRAME_BYTES, 38);

        let mut buffer = [0; 1024];
        let mut file = File::open("data/GPS/eph-1.bin").unwrap();
        file.read(&mut buffer).unwrap();

        let mut decoder = GpsQzssDecoder::default();

        let (size, decoded) = decoder.decode(&buffer, 1024);

        assert_eq!(size, GPS_FRAME_BITS, "returned invalid size");

        let decoded = decoded.unwrap_or_else(|| {
            panic!("did not decoded GPS frame!");
        });

        assert_eq!(decoded.telemetry.message, 0);
        assert_eq!(decoded.telemetry.integrity, false);
        assert_eq!(decoded.telemetry.reserved_bit, false);
    }
}
