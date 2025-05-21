#[derive(Debug)]
/// [GpsDataByte] aligned to 32 bits
pub enum GpsDataByte {
    /// 2-bit MSB padding.
    /// Usually used at the beginning or stream termination by computers.
    MsbPadded(u8),

    /// 2-bit LSB padding.
    /// Usually used at the beginning or stream termination by computers.
    LsbPadded(u8),

    /// Plain byte
    Byte(u8),
}

impl core::fmt::LowerExp for GpsDataByte {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MsbPadded(value) => write!(f, "0x{:02}X", value),
            Self::LsbPadded(value) => write!(f, "0x{:02}X", (value << 2)),
            Self::Byte(value) => write!(f, "0x{:02}X", value),
        }
    }
}

impl GpsDataByte {
    /// Unwraps and left align this value as [u8]
    pub(crate) fn unwrap(&self) -> u8 {
        match self {
            Self::LsbPadded(value) => (value & 0xfc) >> 2,
            Self::MsbPadded(value) => value & 0x3f,
            Self::Byte(value) => *value,
        }
    }

    /// Stores provided byte as 2-bit MSB padded [u8]
    pub fn msb_padded(byte: u8) -> Self {
        Self::MsbPadded(byte & 0x3f)
    }

    /// Stores provided byte as 2-bit LSB padded [u8]
    pub fn lsb_padded(byte: u8) -> Self {
        Self::LsbPadded((byte & 0xfc) >> 2)
    }
}

// /// BitStream collecter, is used to collect MSBF binary streams.
// /// If we take the GPS stream as an example:
// /// - the stream is aligned to 30 bit which is not convenient for computers.
// /// We use the [Byte] wrapper to allow the user to describe whenever they introduce
// /// a padding.
// /// - the stream is MSBF so which bytes are collected in that order
// /// - each byte is least significant bit first
// #[derive(Copy, Clone)]
// pub struct BitStream {
//     /// True of MSBF streams
//     msbf: bool,

//     /// Bitwise!
//     pub needs: usize,

//     pub dword: u32,

//     /// Bitwise!
//     pub collected: usize,

//     /// Size of the next shift to the left (buffer is right aligned)
//     pub next_shift: usize,
// }

// impl BitStream {
//     /// Creates a new MSBF stream collecter
//     pub fn msbf() -> Self {
//         Self {
//             msbf: true,
//             needs: 8,
//             dword: 0,
//             collected: 0,
//             next_shift: 0,
//         }
//     }

//     /// Updates this stream collecter with desired collection size
//     pub fn with_collection_size(&self, size: usize) -> Self {
//         let mut s = self.clone();
//         s.needs = size;
//         s
//     }

//     pub fn set_size_to_collect(&mut self, size: usize) {
//         self.needs = size;
//     }

//     fn binmask(&self) -> u32 {
//         (2u32).pow(self.needs as u32) - 1
//     }

//     /// MSBF bytes collection
//     pub fn collect(&mut self, byte: Byte) -> Option<u32> {
//         self.dword <<= self.next_shift;

//         let (bits_left_aligned, new_size) = match byte {
//             Byte::Byte(value) => (value, 8),
//             Byte::LsbPadded((_, _)) => panic!("LSB padding not supported yet!"),
//             Byte::MsbPadded((_, _)) => panic!("MSB padding not supported yet!"),
//         };

//         self.dword |= bits_left_aligned as u32;

//         self.collected += new_size;

//         if self.collected > self.needs {
//             let preserved = self.collected - self.needs;
//             let mask = 2u32.pow(self.needs as u32) - 1;
//             let ret = (self.dword & mask) >> preserved;

//             self.dword &= mask;
//             self.collected -= self.needs;
//             self.next_shift = preserved;

//             if self.collected == 0 {
//                 self.dword &= 0;
//             }

//             #[cfg(feature = "log")]
//             debug!(
//                 "dword={} | collected={} | needs={}",
//                 self.dword, self.collected, self.needs
//             );

//             Some(ret)
//         } else if self.collected == self.needs {
//             #[cfg(feature = "log")]
//             debug!(
//                 "dword={} | collected={} | needs={}",
//                 self.dword, self.collected, self.needs
//             );

//             let mask = 2u32.pow(self.needs as u32) - 1;
//             let ret = self.dword & mask;

//             self.dword = 0;
//             self.next_shift = 0;
//             self.collected = 0;

//             Some(ret)
//         } else {
//             None
//         }
//     }
// }

// #[cfg(test)]
// mod test {

//     use super::{BitStream, Byte};

//     #[cfg(feature = "log")]
//     use crate::tests::init_logger;

//     #[test]
//     fn test_bit_stream_byte_collecter() {
//         let mut bitstream = BitStream::msbf().with_collection_size(8);

//         // collect N bytes, verify we obtain the correct value
//         for byte in 0..1024 {
//             let value = (byte & 0xff) as u8;
//             let collected = bitstream.collect(Byte::byte(value)).unwrap();

//             assert_eq!(collected, value as u32);
//         }
//     }

//     #[test]
//     fn test_raw_stream_collecter() {
//         #[cfg(feature = "log")]
//         init_logger();

//         let mut bitstream = BitStream::msbf().with_collection_size(8);

//         let collected = bitstream.collect(Byte::byte(1)).unwrap();

//         assert_eq!(collected, 1);
//         assert_eq!(bitstream.next_shift, 0);
//         assert_eq!(bitstream.collected, 0);
//         assert_eq!(bitstream.dword, 0);

//         bitstream.set_size_to_collect(6);

//         let collected = bitstream.collect(Byte::byte(0x02)).unwrap();

//         assert_eq!(collected, 0x02);
//         assert_eq!(bitstream.next_shift, 2);
//         assert_eq!(bitstream.collected, 2);
//         assert_eq!(bitstream.dword, 0x2);

//         bitstream.set_size_to_collect(4);

//         let collected = bitstream.collect(Byte::byte(0x03)).unwrap();

//         assert_eq!(bitstream.next_shift, 6);
//         assert_eq!(bitstream.collected, 6);
//         assert_eq!(bitstream.dword, 0x0B);
//     }
// }
