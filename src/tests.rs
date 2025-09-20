use std::{fs::File, io::Read};

#[cfg(feature = "std")]
use std::sync::Once;

#[cfg(feature = "std")]
use log::LevelFilter;

#[cfg(feature = "gps")]
use crate::gps::GpsDataByte;

#[cfg(feature = "std")]
static INIT: Once = Once::new();

pub fn init_logger() {
    #[cfg(feature = "std")]
    INIT.call_once(|| {
        env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Trace)
            .init();
    });
}

// TLM : 10001011 0000000000000000 101010   (préambule + padding + parité fictive)
// HOW : 00000000000000000 0001  111000   (Z-count fictif + SubframeID=1 + parité)
// W3  : 010101010101010101010101 101010
// W4  : 001100110011001100110011 110011
// W5  : 111100001111000011110000 000111
// W6  : 000111000111000111000111 111000
// W7  : 101010101010101010101010 010101
// W8  : 011001100110011001100110 101101
// W9  : 110000110000110000110000 111100
// W10 : 111111000000111111000000 000111
pub const GPS_EPH1_DATA: [u8; 40] = [
    0x8B, // PREAMBLE
    0x00, 0x00, // TLM-MSG+I+R
    0x17, // PAR(TLM=5) + 2b HOW=3
    0x00, 0x00, // HOW.TOW + A
    0x90, // A/S + FID=1
    0x00, // PAR(HOW)
    0x00, // P1, P2, PAR(MSBx2)
    0x00, // PAR(LSBx4), BIT29=0, BIT30=0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

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
    let mut file = FileReader::<1024>::new("data/GPS/eph-1.bin", 0);

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
    assert_eq!(buffer[2], 0x10);
    assert_eq!(buffer[3], 0x04);
}

#[cfg(feature = "gps")]
pub fn from_ublox_be_bytes<const N: usize>(bytes: &[u8; N]) -> Vec<GpsDataByte> {
    bytes
        .iter()
        .enumerate()
        .map(|(index, byte)| {
            if index % 4 == 0 {
                GpsDataByte::padded(*byte)
            } else {
                GpsDataByte::Byte(*byte)
            }
        })
        .collect()
}

#[test]
#[cfg(feature = "gps")]
fn test_from_ublox_be_bytes() {
    let ubx_bytes = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B,
    ];

    let bytes = from_ublox_be_bytes(&ubx_bytes);
    assert_eq!(bytes.len(), 4);

    assert_eq!(
        bytes,
        [
            GpsDataByte::LsbPadded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B)
        ]
    );

    let ubx_bytes = [
        // TLM
        0x22, 0xC1, 0x3E, 0x1B, // HOW
        0x73, 0xC9, 0x27, 0x15, // WORD3
        0x13, 0xE4, 0x00, 0x04, //WORD4
        0x10, 0x4F, 0x5D, 0x31, //WORD5
        0x97, 0x44, 0xE6, 0xD7, // WORD6
        0x07, 0x75, 0x57, 0x83, //WORD7
        0x33, 0x0C, 0x80, 0xB5, // WORD8
        0x92, 0x50, 0x42, 0xA1, // WORD9
        0x80, 0x00, 0x16, 0x84, //WORD10
        0x31, 0x2C, 0x30, 0x33,
    ];

    let bytes = from_ublox_be_bytes(&ubx_bytes);

    assert_eq!(bytes.len(), 40);

    assert_eq!(bytes[0], GpsDataByte::LsbPadded(0x22));
    assert_eq!(bytes[1], GpsDataByte::Byte(0xC1));
    assert_eq!(bytes[2], GpsDataByte::Byte(0x3E));
    assert_eq!(bytes[3], GpsDataByte::Byte(0x1B));

    assert_eq!(bytes[4], GpsDataByte::LsbPadded(0x33));
    assert_eq!(bytes[5], GpsDataByte::Byte(0xC9));
    assert_eq!(bytes[6], GpsDataByte::Byte(0x27));
    assert_eq!(bytes[7], GpsDataByte::Byte(0x15));

    assert_eq!(bytes[8], GpsDataByte::LsbPadded(0x13));
    assert_eq!(bytes[9], GpsDataByte::Byte(0xE4));
    assert_eq!(bytes[10], GpsDataByte::Byte(0x00));
    assert_eq!(bytes[11], GpsDataByte::Byte(0x04));

    assert_eq!(bytes[12], GpsDataByte::LsbPadded(0x10));
    assert_eq!(bytes[13], GpsDataByte::Byte(0x4F));
    assert_eq!(bytes[14], GpsDataByte::Byte(0x5D));
    assert_eq!(bytes[15], GpsDataByte::Byte(0x31));

    assert_eq!(bytes[16], GpsDataByte::LsbPadded(0x17));
    assert_eq!(bytes[17], GpsDataByte::Byte(0x44));
    assert_eq!(bytes[18], GpsDataByte::Byte(0xE6));
    assert_eq!(bytes[19], GpsDataByte::Byte(0xD7));
}
