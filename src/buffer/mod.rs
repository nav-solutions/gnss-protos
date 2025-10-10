// mod view;
// pub use view::*;

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
    fn read(&mut self, dest: &mut [u8]) -> std::io::Result<usize> {
        let dest_size = dest.len();
        let avail = self.read_available();

        if avail == 0 {
            return Ok(0);
        }

        if dest_size < avail {
            dest.copy_from_slice(&self.inner[self.rd_ptr..self.rd_ptr + avail]);
            Ok(dest_size)
        } else {
            dest[..avail].copy_from_slice(&self.inner[self.rd_ptr..self.wr_ptr]);     
            self.rd_ptr = 0;
            self.wr_ptr = 0;
            Ok(avail)
        }
    }
}


impl<const M: usize> std::io::Write for Buffer<M> {
    fn write(&mut self, src: &[u8]) -> std::io::Result<usize> {
        let src_size = src.len();
        let avail = self.write_available();

        if src_size == 0 {
            return Ok(0);
        }

        if src_size > avail {
            self.inner[self.wr_ptr..].copy_from_slice(&src[..avail]);
            self.wr_ptr = M -1;
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
    
    // /// Obtain a [BufferView] from internal content,
    // /// that you can then operate bytewise and bitwise.
    // pub fn view<'a>(&'a self) -> BufferView<'a> {
    //     BufferView {
    //         ptr: 0,
    //         offset: 0,
    //         inner: &self.inner,
    //     }
    // }
}

#[cfg(test)]
mod test {
    use super::Buffer;
}   
