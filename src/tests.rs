use std::{fs::File, io::Read, sync::Once};

use log::LevelFilter;

use crate::gps::{GpsDataWord, GpsQzssFrameId, GpsQzssHow, GpsQzssTelemetry, GPS_WORDS_PER_FRAME};

static INIT: Once = Once::new();

pub fn init_logger() {
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Trace)
            .init();
    });
}

/// Inserts desired number of zero (bits) at the begginning of a frame
pub fn insert_zeros(slice: &[u8], shift_bits: usize) -> Vec<u8> {
    let size = slice.len();
    let mut out = vec![0u8; size];

    let byte_shift = shift_bits / 8;
    let bit_shift = shift_bits % 8;

    for (i, &byte) in slice.iter().enumerate() {
        let out_idx = i + byte_shift;

        if out_idx >= size {
            break;
        }

        out[out_idx] |= byte >> bit_shift;

        if bit_shift != 0 && out_idx + 1 < size {
            out[out_idx + 1] |= byte << (8 - bit_shift);
        }
    }

    out
}

/// A custom [FileReader] with possible offset (delay)
/// in the stream.
pub struct FileReader<const N: usize> {
    // FD
    fd: File,

    // RD pointer
    rd_ptr: usize,

    // WR pointer
    wr_ptr: usize,

    // Initial offset in bits
    initial_offset_bits: usize,

    // internal buufer
    buffer: [u8; N],
}

impl<const N: usize> std::io::Read for FileReader<N> {
    fn read(&mut self, buffer: &mut [u8]) -> std::io::Result<usize> {
        let mut capacity = buffer.len();

        let avail_in_buffer = self.rd_ptr;

        if self.rd_ptr > 0 {
            // provide existing content
            if capacity > self.rd_ptr {
                buffer[..avail_in_buffer].copy_from_slice(&self.buffer[self.rd_ptr..]);

                self.rd_ptr = 0;
                capacity -= self.rd_ptr;
            } else {
                // provide capacity and return
                self.rd_ptr += capacity;
                return Ok(capacity);
            }
        }

        // read new content
        let new_size = self.fd.read(&mut self.buffer[self.rd_ptr..])?;

        if new_size == 0 {
            if avail_in_buffer == 0 {
                // end of stream
                return Ok(0);
            } else {
                // already copied
                self.rd_ptr += new_size;
                return Ok(avail_in_buffer);
            }
        }

        if capacity >= new_size {
            // can absorb new content entirely
            buffer[avail_in_buffer..].copy_from_slice(&self.buffer[self.rd_ptr..]);

            self.rd_ptr += new_size;

            return Ok(avail_in_buffer + new_size);
        }

        // can only absorb a part of it

        Ok(0)
    }
}

impl<const N: usize> FileReader<N> {
    /// Creates new [FileReader] with possible fake offset in bits
    pub fn new(fp: &str, initial_offset_bits: usize) -> Self {
        let initial_offset_bytes = initial_offset_bits / 8;

        let mut buffer = [0; N];

        let mut fd = File::open(fp).unwrap_or_else(|e| {
            panic!("Failed to open file {}: {}", fp, e);
        });

        Self {
            fd,
            buffer,
            initial_offset_bits,
            wr_ptr: 0,
            rd_ptr: 0,
        }
    }
}

#[test]
fn test_zeros_padder() {
    let mut buffer = [0; 1024];
    let mut file = FileReader::<1024>::new("data/GPS/eph1.bin", 0);

    file.read(&mut buffer).unwrap_or_else(|e| {
        panic!("Failed to read test data: {}", e);
    });

    let size = buffer.len();

    // test offset by bytes
    for i in 1..16 {
        // insert 1 byte
        let delayed = insert_zeros(&buffer, i * 8);

        assert_eq!(delayed.len(), size); // size should never change
    }

    // test offset by bits
    for i in 1..7 {
        let delayed = insert_zeros(&buffer, i);

        assert_eq!(delayed[0], 0x8B >> i);
        assert_eq!(delayed.len(), size); // size should never change
    }
}

#[test]
fn test_file_reader() {
    let mut buffer = [0; 1024];
    let mut file = FileReader::<1024>::new("data/GPS/eph1.bin", 0);

    file.read(&mut buffer).unwrap_or_else(|e| {
        panic!("Failed to read test data: {}", e);
    });

    assert_eq!(buffer[0], 0x8B);
    assert_eq!(buffer[1], 0x48);
    assert_eq!(buffer[2], 0xD3);
    assert_eq!(buffer[3], 0x03);
}

#[cfg(feature = "gps")]
pub fn from_ublox_bytes<const N: usize>(bytes: &[u8; N]) -> [GpsDataWord; GPS_WORDS_PER_FRAME] {
    let mut ret: [GpsDataWord; GPS_WORDS_PER_FRAME] = Default::default();

    let mut count = 0;
    let mut value = 0u32;

    for i in 0..N {
        match i % 4 {
            0 => value |= (bytes[i % 4 + count * 4] as u32) << 26,
            1 => value |= (bytes[i % 4 + count * 4] as u32) << 18,
            2 => value |= (bytes[i % 4 + count * 4] as u32) << 10,
            3 => {
                value |= (bytes[i % 4 + count * 4] as u32) << 2;
                ret[count] = GpsDataWord::from(value);
                count += 1;
                value = 0;

                if count == GPS_WORDS_PER_FRAME {
                    return ret;
                }
            },
            _ => unreachable!("compiler issue"),
        }
    }

    ret
}

#[test]
fn test_from_ublox_bytes() {
    let data = [
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
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris1);

    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x27, 0xEA, 0x1B, // WORD3
        0x12, 0x7F, 0xF1, 0x65, // WORD4
        0x8C, 0x68, 0x1F, 0x7C, // WORD5
        0x02, 0x49, 0x34, 0x15, // WORD6
        0xBF, 0xF8, 0x81, 0x1E, // WORD7
        0x99, 0x1B, 0x81, 0x14, // W0RD8
        0x04, 0x3E, 0x68, 0x6E, // WORD9
        0x83, 0x34, 0x72, 0x21, // WORD10
        0x90, 0x42, 0x9F, 0x7B,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris2);

    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x28, 0x0B, 0xDB, // WORD3
        0x00, 0x0A, 0xEA, 0x34, // WORD4
        0x03, 0x3C, 0xFF, 0xEE, // WORD5
        0xBF, 0xE5, 0xC9, 0xEB, // WORD6
        0x13, 0x6F, 0xB6, 0x4E, // WORD7
        0x86, 0xF4, 0xAB, 0x2C, // WORD8
        0x06, 0x71, 0xEB, 0x44, // WORD9
        0x3F, 0xEA, 0xF6, 0x02, // WORD10
        0x92, 0x45, 0x52, 0x13,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris3);

    let data = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x15, 0x27, 0xC9, 0x73, // WORD3
        0x00, 0x0A, 0xEA, 0x34, // WORD4
        0x03, 0x3C, 0xFF, 0xEE, // WORD5
        0xBF, 0xE5, 0xC9, 0xEB, // WORD6
        0x13, 0x6F, 0xB6, 0x4E, // WORD7
        0x86, 0xF4, 0xAB, 0x2C, // WORD8
        0x06, 0x71, 0xEB, 0x44, // WORD9
        0x3F, 0xEA, 0xF6, 0x02, // WORD10
        0x92, 0x45, 0x52, 0x13,
    ];

    let words = from_ublox_bytes(&data);

    let tlm = GpsQzssTelemetry::from_word(words[0]).unwrap_or_else(|e| {
        panic!("failed to decode telemetry: {}", e);
    });

    assert_eq!(tlm.message, 0x13E);
    assert_eq!(tlm.integrity, false);
    assert_eq!(tlm.reserved_bit, false);

    let how = GpsQzssHow::from_word(words[1]).unwrap_or_else(|e| {
        panic!("failed to decode how: {}", e);
    });

    assert_eq!(how.alert, false);
    assert_eq!(how.anti_spoofing, true);
    assert_eq!(how.frame_id, GpsQzssFrameId::Ephemeris1);
}
