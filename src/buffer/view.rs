/// [BufferView] that you can then manipulate
pub struct BufferView<'a, const M: usize> {
    /// RD pointer position, bytewise
    ptr: usize,

    /// Byte offset (bitwise, <= 7)
    offset: usize,

    /// Internal view
    inner: &'a [u8; M],
}

impl<'a, const M: usize> BufferViex<'a, const M: usize> {

    /// Returns number of bytes available for a read, in this [BufferView].
    pub fn len(&self) -> usize {
        M - self.rd_ptr
    }

    /// Consume the following number of bits
    pub fn consume_bits(&mut self, size: usize) {
        
        let num_bytes = size / 8;
        let num_bits = size % 8;

        let offset_remain = 8 - self.offset;
        
        self.offset += num_bits;
        self.ptr += 
    }

    /// Consume the following number of bytes
    pub fn consume_bytes(&mut self, size: usize) {
        self.rd_ptr += size;
        
        if self.rd_ptr > M {
            self.rd_ptr = M;
        }
    }

    /// [std::io::Read] implementation with possible bitwise
    pub fn read_rotated(&mut self, dest: &[u8], right_shift: usize) -> std::io::Result<usize> {

    }
}   
        
impl<'a> std::io::Read for BufferView<'a> {
    /// Bytewise [std::io::Read] operation.
    fn read(&mut self, dest: &[u8]) -> std::io::Result<usize> {
        let avail = src.len();

        if avail < self.size() {
            self.rd_ptr += 
            dest.copy_from_slice(self.inner[..avail]);
        } else {
            dest[..self.size()].copy_from_slice(self.inner);
        }
    }
}
