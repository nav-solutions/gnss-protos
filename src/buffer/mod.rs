mod view;
pub use view::*;

#[cfg(feature = "std")]
mod std;

#[derive(Debug, Copy, Clone)]
pub enum BufferingError {
    /// std-io WouldBlock equivalent
    WouldBlock,

    /// Storage is full, target will not fit entirely (std-io equivalent).
    StorageFull,
}

/// [Buffer] storage that helps capture and decode
/// a real-time binary stream. You should adapt the total pre-allocated
/// size to the protocol being received.
///
/// [Buffer] implements [std::io::Read] and [std::io::Write]
/// for convenient and easy system interfacing. It also provides
/// bytewise or bitwise operations.
///
/// [Buffer] does not provide bitwise filling,
/// [std::io::Write] is the only option to feed new data, and it operates bytewise.
/// In otherwords, zero padding and slight inefficiency will have to be introduced
/// inevitably at some point, when interfacing unaligned hardware streams to a decoder in software.
/// But [Buffer] proposes methods like [Self::read_available_bits] which, when correctly implement,
/// allows you to drop padding bits, making the decoding process fully efficient.
#[derive(Clone, Copy, PartialEq)]
pub(crate) struct Buffer<const M: usize> {
    /// Bytes storage
    inner: [u8; M],

    /// Byte wise RD ptr
    rd_ptr: usize,

    /// offset withint current RD ptr (<8)
    rd_offset: usize,

    /// bytewise WR ptr
    wr_ptr: usize,

    /// offset withint current WR ptr (<8)
    wr_offset: usize,
}

impl<const M: usize> Default for Buffer<M> {
    fn default() -> Self {
        Self {
            rd_ptr: 0,
            wr_ptr: 0,
            rd_offset: 0,
            wr_offset: 0,
            inner: [0; M],
        }
    }
}

impl<const M: usize> Buffer<M> {
    /// Returns the total storage capacity of this [Decoder].
    pub const fn capacity(&self) -> usize {
        M
    }

    /// Returns total number of bytes that can be returned by a read operation.
    /// Not all bits will be significant (we use zero termination), you need to check [Self::read_available_bits].
    pub fn read_available(&self) -> usize {
        let mut size = self.wr_ptr;

        size -= self.read_available_bits() / 8; // bytes

        if self.rd_offset > 0 {
            size += 1;
        }

        size
    }

    /// Returns total number of effective bits in this [Buffer].
    pub fn read_available_bits(&self) -> usize {
        self.rd_ptr * 8 + self.rd_offset
    }

    /// Returns total number of bytes that can be written.
    pub fn write_available(&self) -> usize {
        let mut size = M - self.wr_ptr;

        if self.wr_offset > 0 {
            size -= 1;
        }

        size
    }
    
    /// Returns total number of bits that can be written to this [Buffer].
    pub fn write_available_bits(&self) -> usize {
        let mut size = (M - self.wr_ptr) * 8;
        size -= self.wr_offset;
        size
    }

    pub fn read(&mut self, dest: &mut [u8]) -> Result<usize, BufferingError> {
        let dest_size = dest.len();
        let avail = self.read_available();

        if avail == 0 {
            return Ok(0);
        }

        let ret = if dest_size < avail {
            dest.copy_from_slice(&self.inner[self.rd_ptr..self.rd_ptr + dest_size]);

            // internal swap:
            // shift internal buffer, preserving remaining data while accepting new writes.
            self.inner
                .copy_within(self.rd_ptr + dest_size..self.wr_ptr, 0);

            self.wr_ptr -= self.rd_ptr;
            self.wr_ptr -= dest_size;

            Ok(dest_size)
        } else {
            dest[..avail].copy_from_slice(&self.inner[self.rd_ptr..self.rd_ptr + avail]);

            self.wr_ptr = 0;

            Ok(avail)
        };

        self.rd_ptr = 0;
        ret
    }

    /// Fill this mutable [Buffer] with new data.
    pub fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError> {
        let src_size = src.len();

        let avail = self.write_available();

        if src_size == 0 {
            return Ok(0);
        }

        if avail == 0 {
            return Err(BufferingError::WouldBlock);
        }

        if self.wr_offset == 0 {
            if src_size > avail {
                self.inner[self.wr_ptr..].copy_from_slice(&src[..avail]);
                self.wr_ptr = M;
                Ok(avail)
            } else {
                self.inner[self.wr_ptr..self.wr_ptr + src_size].copy_from_slice(&src);
                self.wr_ptr += src_size;
                Ok(src_size)
            }
        } else {
            panic!("not yet");
        }
    }

    /// Returns a slice view of the internal bytes.
    /// Prefer the [BufferView] which supports bitwise rotation without further allocation.
    pub fn slice(&self) -> &[u8; M] {
        &self.inner
    }

    /// Obtain a [BufferView] (buffer snapshot) at the current state.
    ///
    /// To offset and adjust the [BufferView], you can either
    /// - adapt the initial state first, assuming you already determined
    /// that irrelevant bits were being stored, either using
    ///    - [Self::discard_bytes_mut] for bytewise pre adaptation
    ///    - [Self::discard_bits_mut] for bitwise pre adaptation
    /// All prediscarded bytes are trashed and will not be viewable
    /// at the time the [BufferView] is obtained.
    ///
    /// - or use the proposed [BufferView] byte or bit iteration methods.
    pub fn view<'a>(&'a self) -> BufferView<'a, M> {
        BufferView::from_slice(&self.inner)
    }

    /// Discard the selected number of bytes, like they were consumed
    /// by a read operation. The bytes are trashed and will no longer be viewable
    /// (not proposed to following read operations).
    pub fn discard_bytes_mut(&mut self, bytes: usize) {
        let avail = self.read_available();
        let size = if bytes > avail { avail } else { bytes };

        // internal swap:
        // shift internal buffer, preserving remaining data while accepting new writes.
        self.inner.copy_within(self.rd_ptr + size..self.wr_ptr, 0);

        self.wr_ptr -= size;

        if bytes > avail {
            self.rd_ptr -= size;
        } else {
            self.rd_ptr = 0;
        }
    }

    /// Discard the selected number of bits, like they were consumed
    /// by a read operation. The bits are trashed and will no longer be viewable
    /// (not proposed to following read operations).
    pub fn discard_bits_mut(&mut self, bits: usize) {
        let bytes_avail = self.read_available();
        let (bytes, bits) = (bits / 8, bits % 8);

        // internal bytewise swap:
        // shift internal buffer, preserving remaining data while accepting new writes.
        self.inner.copy_within(self.rd_ptr + bytes..self.wr_ptr, 0);
        
        self.wr_ptr -= bytes;

        if bits > 0 {
            let mask = 2u8.pow(bits as u32) - 1;

            // shift & rotate all effective bytes
            for i in 0..self.read_available() {
                self.inner[i] <<= bits;
                
                if i < M -1 {
                    self.inner[i] |= (self.inner[i +1] >> (8 - bits)) & mask;
                }
            }
        }

        if bytes > bytes_avail {
            self.rd_ptr -= bytes;
        } else {
            self.rd_ptr = 0;
        }

    }

    /// Shfits internal buffer to the right.
    pub fn shift_right_mut(&mut self, shift: usize) {
        for i in self.rd_ptr..self.wr_ptr - 1 {
            self.inner[i + 1] = self.inner[i] << (8 - shift);
            self.inner[i] >>= shift;
        }
    }
}

#[cfg(test)]
mod test {
    use super::{Buffer, BufferView};
    use std::io::{Read, Write};

    #[test]
    fn buffer_8_16() {
        let source = [1u8, 2u8, 3u8, 4u8, 5u8, 6u8, 7u8, 8u8];
        let mut dest = source.clone();

        let mut buffer = Buffer::<16>::default();

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0],
        );

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8],
        );

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        let written = buffer.write(&source);
        assert!(written.is_err()); // full at this point

        // read half
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 8);
        assert_eq!(dest, [1, 2, 3, 4, 5, 6, 7, 8]);

        // half full at this point
        assert_eq!(buffer.write_available(), 8);
        assert_eq!(buffer.read_available(), 8);

        // read half
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 8);
        assert_eq!(dest, [1, 2, 3, 4, 5, 6, 7, 8]);

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);
    }

    #[test]
    fn buffer_7_16() {
        let source = [1u8, 2u8, 3u8, 4u8, 5u8, 6u8, 7u8];
        let mut dest = source.clone();

        let mut buffer = Buffer::<16>::default();

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 0);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 7); // should all fit
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 7);
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        );

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 7); // should all fit
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 14);
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );

        // still not full at this point
        assert_eq!(buffer.write_available(), 2);
        assert_eq!(buffer.read_available(), 14);

        // read half
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 7);
        assert_eq!(dest, [1, 2, 3, 4, 5, 6, 7]);

        assert_eq!(buffer.write_available(), 9);
        assert_eq!(buffer.read_available(), 7);

        // read half
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 7);
        assert_eq!(dest, [1, 2, 3, 4, 5, 6, 7]);

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 7); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );

        assert_eq!(buffer.read_available(), 7);
        assert_eq!(buffer.write_available(), 9);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 7); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );

        assert_eq!(buffer.read_available(), 14);
        assert_eq!(buffer.write_available(), 2);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 2); // should not fit entirely
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2]
        );

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 16);

        let written = buffer.write(&source);
        assert!(written.is_err()); // should not accept at this point

        // Read 7 bytes
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 7);
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 9);
        assert_eq!(buffer.read_available(), 9);
        assert_eq!(buffer.write_available(), 7);

        // Read 7 bytes
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 7);
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 2);
        assert_eq!(buffer.read_available(), 2);
        assert_eq!(buffer.write_available(), 14);

        // Read 2 bytes
        let read = buffer.read(&mut dest);
        assert_eq!(read.unwrap(), 2);
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);
    }

    #[test]
    fn buffer_8_16_discard_bytes() {
        let source = [1u8, 2u8, 3u8, 4u8, 5u8, 6u8, 7u8, 8u8];
        let mut dest = source.clone();

        let mut buffer = Buffer::<16>::default();

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0],
        );

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        // discard
        buffer.discard_bytes_mut(2);
        assert_eq!(buffer.wr_ptr, 14);
        assert_eq!(buffer.write_available(), 2);
        assert_eq!(buffer.read_available(), 14);
        assert_eq!(
            buffer.slice(),
            &[3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8]
        );

        // discard
        buffer.discard_bytes_mut(2);
        assert_eq!(buffer.wr_ptr, 12);
        assert_eq!(buffer.write_available(), 4);
        assert_eq!(buffer.read_available(), 12);
        assert_eq!(
            buffer.slice(),
            &[5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8, 7, 8]
        );

        // discard
        buffer.discard_bytes_mut(1);
        assert_eq!(buffer.wr_ptr, 11);
        assert_eq!(buffer.write_available(), 5);
        assert_eq!(buffer.read_available(), 11);
        // TODO verify internal

        buffer.discard_bytes_mut(1);
        assert_eq!(buffer.wr_ptr, 10);
        assert_eq!(buffer.write_available(), 6);
        assert_eq!(buffer.read_available(), 10);

        buffer.discard_bytes_mut(2);
        assert_eq!(buffer.write_available(), 8);
        assert_eq!(buffer.read_available(), 8);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        // discard all but one
        buffer.discard_bytes_mut(15);
        assert_eq!(buffer.write_available(), 15);
        assert_eq!(buffer.read_available(), 1);

        // emptied
        buffer.discard_bytes_mut(1);
        assert_eq!(buffer.write_available(), 16);
        assert_eq!(buffer.read_available(), 0);

        // fill
        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        // discard entirely
        buffer.discard_bytes_mut(16);
        assert_eq!(buffer.write_available(), 16);
        assert_eq!(buffer.read_available(), 0);
    }

    #[test]
    fn buffer_8_16_discard_8bits() {
        let source = [1u8, 2u8, 3u8, 4u8, 5u8, 6u8, 7u8, 8u8];
        let mut dest = source.clone();

        let mut buffer = Buffer::<16>::default();

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0],
        );

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        // discard
        buffer.discard_bits_mut(16);
        assert_eq!(buffer.wr_ptr, 14);
        assert_eq!(buffer.write_available(), 2);
        assert_eq!(buffer.read_available(), 14);
        assert_eq!(
            buffer.slice(),
            &[3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8]
        );

        // discard
        buffer.discard_bits_mut(16);
        assert_eq!(buffer.wr_ptr, 12);
        assert_eq!(buffer.write_available(), 4);
        assert_eq!(buffer.read_available(), 12);
        assert_eq!(
            buffer.slice(),
            &[5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8, 7, 8, 7, 8]
        );

        // discard
        buffer.discard_bits_mut(8);
        assert_eq!(buffer.wr_ptr, 11);
        assert_eq!(buffer.write_available(), 5);
        assert_eq!(buffer.read_available(), 11);
        // TODO verify internal

        buffer.discard_bits_mut(8);
        assert_eq!(buffer.wr_ptr, 10);
        assert_eq!(buffer.write_available(), 6);
        assert_eq!(buffer.read_available(), 10);

        buffer.discard_bits_mut(16);
        assert_eq!(buffer.write_available(), 8);
        assert_eq!(buffer.read_available(), 8);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        // discard all but one
        buffer.discard_bits_mut(15 * 8);
        assert_eq!(buffer.write_available(), 15);
        assert_eq!(buffer.read_available(), 1);

        // emptied
        buffer.discard_bits_mut(8);
        assert_eq!(buffer.write_available(), 16);
        assert_eq!(buffer.read_available(), 0);
    }

    #[test]
    fn buffer_8_16_discard_bits() {
        let source = [1u8, 2u8, 3u8, 4u8, 5u8, 6u8, 7u8, 8u8];
        let mut dest = source.clone();

        let mut buffer = Buffer::<16>::default();

        // empty at this point
        assert_eq!(buffer.read_available(), 0);
        assert_eq!(buffer.write_available(), 16);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0],
        );

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
        );

        // full at this point
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.read_available(), 16);

        // discard
        buffer.discard_bits_mut(1);
        assert_eq!(buffer.write_available(), 0);
        assert_eq!(buffer.write_available_bits(), 1);
        assert_eq!(buffer.read_available(), 15);

        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0e, 0x10, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10]
        );
        
        buffer.discard_bits_mut(2);
        assert_eq!(buffer.write_available(), 2);
        assert_eq!(buffer.read_available(), 14);
        
        assert_eq!(
            buffer.view().into_iter().collect::<Vec<_>>(),
            vec![0x02<<2, 0x04<<2, 0x06<<2, 0x08<<2, 0x0A<<2, 0x0C<<2, 0x0E<<2, 0x10<<2, 0x02<<2, 0x04<<2, 0x06<<2, 0x08<<2, 0x0A<<2, 0x0C<<2, 0x0E<<2, 0x10<<2],
        );
    }
}
