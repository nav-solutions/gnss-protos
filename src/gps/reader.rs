pub(crate) struct BitReader<'a> {
    buffer: &'a [u8],
    byte_index: usize,
    bit_index: u8, // 0..7
}

impl<'a> BitReader<'a> {
    pub fn new(buffer: &'a [u8], start_bit: usize) -> Self {
        Self {
            buffer,
            byte_index: start_bit / 8,
            bit_index: (start_bit % 8) as u8,
        }
    }

    /// Grabs a new [u32] with two MSB padding from the input stream
    pub fn read_u32(&mut self) -> Option<u32> {
        let mut acc = 0u32;

        // grab 4 bytes
        let mut bytes = [0; 4];

        bytes[0] = *self.buffer.get(self.byte_index)?;
        bytes[1] = *self.buffer.get(self.byte_index + 1)?;
        bytes[2] = *self.buffer.get(self.byte_index + 1)?;
        bytes[3] = *self.buffer.get(self.byte_index + 1)?;

        Some(u32::from_be_bytes(bytes))
    }
}

#[cfg(test)]
mod bit_reader {
    use super::*;

    use crate::tests::FileReader;

    use std::fs::File;
    use std::io::Read;

    #[test]
    #[ignore]
    fn test_bit_reader() {
        let mut file = FileReader::<1024>::new("two_frames.bin", 0);

        // let mut reader = BitReader::new(&file.buffer, 0);
        // assert_eq!(reader.read_u32(), Some(0x8B000000));
    }
}
