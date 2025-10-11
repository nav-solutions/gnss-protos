/// [Buffer] storage that helps capture and decode
/// a real-time binary stream. It implements [std::io::Read]
/// for bytewise operation, it provides also a [BufferView] for
/// binary rotation and bitwise decoding made easy.
#[derive(Clone, Copy, PartialEq)]
pub struct Buffer<const M: usize> {
    /// Bytes storage
    inner: [u8; M],

    /// RD ptr
    rd_ptr: usize,

    /// WR ptr
    wr_ptr: usize,
}

impl<const M: usize> Default for Buffer<M> {
    fn default() -> Self {
        Self {
            rd_ptr: 0,
            wr_ptr: 0,
            inner: [0; M],
        }
    }
}

impl<const M: usize> std::io::Read for Buffer<M> {
    /// [Buffer] implements the standard bytewise [std::io::Read] operation,
    /// for convenient system interfacing, usually needed when receiving a real-time stream.
    ///
    /// [Buffer] then proposes other methods to operate at the bit level.
    fn read(&mut self, dest: &mut [u8]) -> std::io::Result<usize> {
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
}

impl<const M: usize> std::io::Write for Buffer<M> {
    /// [Buffer] implements the standard bytewise [std::io::Write] operation,
    /// for convenient system interfacing, usually needed when receiving a real-time stream.
    ///
    /// [Buffer] then proposes other methods to operate at the bit level.
    fn write(&mut self, src: &[u8]) -> std::io::Result<usize> {
        let src_size = src.len();
        let avail = self.write_available();

        if src_size == 0 {
            return Ok(0);
        }

        if avail == 0 {
            return Err(std::io::ErrorKind::WouldBlock.into());
        }

        if src_size > avail {
            self.inner[self.wr_ptr..].copy_from_slice(&src[..avail]);
            self.wr_ptr = M;
            Ok(avail)
        } else {
            self.inner[self.wr_ptr..self.wr_ptr + src_size].copy_from_slice(&src);
            self.wr_ptr += src_size;
            Ok(src_size)
        }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl<const M: usize> Buffer<M> {
    /// Returns total number of bytes available to read.
    pub fn read_available(&self) -> usize {
        self.wr_ptr - self.rd_ptr
    }

    /// Returns total number of bytes that can be written.
    pub fn write_available(&self) -> usize {
        M - self.wr_ptr
    }

    /// Returns a slice view of the internal bytes
    pub fn slice(&self) -> &[u8; M] {
        &self.inner
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
    use super::Buffer;
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

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 8); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8]
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

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 7); // should all fit
        assert_eq!(buffer.rd_ptr, 0);
        assert_eq!(buffer.wr_ptr, 14);
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
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

        assert_eq!(buffer.read_available(), 7);
        assert_eq!(buffer.write_available(), 9);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 7); // should all fit
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 0, 0]
        );

        assert_eq!(buffer.read_available(), 14);
        assert_eq!(buffer.write_available(), 2);

        let written = buffer.write(&source);
        assert_eq!(written.unwrap(), 2); // should not fit entirely
        assert_eq!(
            buffer.slice(),
            &[1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 1, 2]
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
}
