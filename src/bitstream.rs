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
    needs: usize,

    dword: u32,

    /// BItwise!
    collected: usize,

    /// Size of the next shift to the left (buffer is right aligned)
    next_shift: usize,
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
            Byte::LsbPadded((padding, value)) => ((value & 0xfc) >> 2, 6),
            Byte::MsbPadded((padding, value)) => (value & 0x3f, 6),
        };

        self.dword |= bits_left_aligned as u32;
        self.next_shift = new_size;
        self.collected += new_size;

        let ret = if self.collected >= self.needs {
            // mask
            let mask = 2u32.pow(self.needs as u32) - 1;
            let ret = self.dword & mask;

            // publish
            let shift_size = 0;

            self.dword >>= self.needs;
            self.collected -= self.needs;

            Some(ret)
        } else {
            None
        };

        ret
    }
}

#[cfg(test)]
mod test {

    use super::{BitStream, Byte};

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

    // #[test]
    // fn test_bit_stream_msb_byte_collecter() {

    //     let mut bitstream = BitStream::msbf()
    //         .with_collection_size(8);

    //     // collect N bytes, verify we obtain the correct value
    //     let byte = Byte::msb_padded(0x01);

    //     assert!(bitstream.collect(byte).is_none());

    //     let byte = Byte::msb_padded(0x02);

    //     let collected = bitstream.collect(byte).unwrap();
    //     assert_eq!(collected,
    // }
}
