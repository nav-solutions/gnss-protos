#[cfg(feature = "log")]
use log::debug;

pub enum Byte {
    Byte(u8),
    LsbPadded((usize, u8)),
    MsbPadded((usize, u8)),
}

impl Byte {
    pub(crate) fn binmask(&self) -> u8 {
        match self {
            Self::Byte(_) => 0xff,
            Self::LsbPadded((padding, _)) => 0,
            Self::MsbPadded((padding, _)) => 0,
        }
    }

    pub(crate) fn unwrap(&self) -> u8 {
        match self {
            Self::Byte(value) => *value,
            Self::LsbPadded((padding, value)) => (value & 0xfc) >> padding,
            Self::MsbPadded((padding, value)) => (value & 0x3f),
        }
    }

    pub fn byte(byte: u8) -> Self {
        Self::Byte(byte)
    }

    /// 2-bit MSB padding
    pub fn msb2_padded(byte: u8) -> Self {
        Self::MsbPadded((2, byte & 0x3f))
    }

    /// 2-bit LSB padding
    pub fn lsb2_padded(byte: u8) -> Self {
        Self::LsbPadded((2, byte & 0xfc))
    }
}

/// BitStream collecter, is used to collect MSBF binary streams.
/// If we take the GPS stream as an example:
/// - the stream is aligned to 30 bit which is not convenient for computers.
/// We use the [Byte] wrapper to allow the user to describe whenever they introduce
/// a padding.
/// - the stream is MSBF so which bytes are collected in that order
/// - each byte is least significant bit first
#[derive(Copy, Clone)]
pub struct BitStream {
    /// True of MSBF streams
    msbf: bool,

    /// Bitwise!
    pub needs: usize,

    pub dword: u32,

    /// Bitwise!
    pub collected: usize,

    /// Size of the next shift to the left (buffer is right aligned)
    pub next_shift: usize,
}

impl BitStream {
    /// Creates a new MSBF stream collecter
    pub fn msbf() -> Self {
        Self {
            msbf: true,
            needs: 8,
            dword: 0,
            collected: 0,
            next_shift: 0,
        }
    }

    /// Updates this stream collecter with desired collection size
    pub fn with_collection_size(&self, size: usize) -> Self {
        let mut s = self.clone();
        s.needs = size;
        s
    }

    pub fn set_size_to_collect(&mut self, size: usize) {
        self.needs = size;
    }

    fn binmask(&self) -> u32 {
        (2u32).pow(self.needs as u32) - 1
    }

    pub fn collect(&mut self, byte: Byte) -> Option<u32> {
        self.dword <<= self.next_shift;

        let (bits_left_aligned, new_size) = match byte {
            Byte::Byte(value) => (value, 8),
            Byte::LsbPadded((_, _)) => panic!("LSB padding not supported yet!"),
            Byte::MsbPadded((_, _)) => panic!("MSB padding not supported yet!"),
        };

        self.dword |= bits_left_aligned as u32;

        self.next_shift = new_size;
        self.collected += new_size;

        if self.collected >= self.needs {
            let discarded_saved = self.collected - self.needs;
            let mask = 2u32.pow((self.needs + discarded_saved) as u32) - 1;
            let ret = (self.dword & mask) >> discarded_saved;

            self.dword &= mask;
            self.collected -= self.needs;
            self.next_shift = discarded_saved;

            if self.collected == 0 {
                self.dword &= 0;
            }

            #[cfg(feature = "log")]
            debug!(
                "dword={} | collected={} | needs={}",
                self.dword, self.collected, self.needs
            );

            Some(ret)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {

    use super::{BitStream, Byte};

    #[cfg(feature = "log")]
    use crate::tests::init_logger;

    #[test]
    fn test_bit_stream_byte_collecter() {
        let mut bitstream = BitStream::msbf().with_collection_size(8);

        // collect N bytes, verify we obtain the correct value
        for byte in 0..1024 {
            let value = (byte & 0xff) as u8;
            let byte = Byte::byte(value);

            let collected = bitstream.collect(byte).unwrap();

            assert_eq!(collected, value as u32);
        }
    }

    #[test]
    fn test_raw_stream_collecter() {
        #[cfg(feature = "log")]
        init_logger();

        let mut bitstream = BitStream::msbf().with_collection_size(8);

        let collected = bitstream.collect(Byte::byte(1)).unwrap();

        assert_eq!(collected, 1);
        assert_eq!(bitstream.next_shift, 0);
        assert_eq!(bitstream.collected, 0);
        assert_eq!(bitstream.dword, 0);

        bitstream.set_size_to_collect(6);

        let collected = bitstream.collect(Byte::byte(0x02)).unwrap();

        assert_eq!(bitstream.next_shift, 2);
        assert_eq!(bitstream.collected, 2);
        assert_eq!(bitstream.dword, 0x2);

        bitstream.set_size_to_collect(4);

        let collected = bitstream.collect(Byte::byte(0x03)).unwrap();

        assert_eq!(bitstream.next_shift, 6);
        assert_eq!(bitstream.collected, 6);
        assert_eq!(bitstream.dword, 0x0B);
    }
}
