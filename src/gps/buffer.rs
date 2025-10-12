use crate::{buffer::StreamBuffer, Buffering, BufferingError};

/// internal storage for more than 2 frames (300 bits)
const CAPACITY: usize = 1024;

use bitbuffer::{BigEndian, BitReadBuffer, BitWriteStream, Endianness};

pub struct GpsBuffer {
    pub(crate) inner: StreamBuffer<CAPACITY>,
}

impl Buffering for GpsBuffer {
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError> {
        self.inner.fill(src)
    }

    fn is_full(&self) -> bool {
        self.inner.is_full()
    }

    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    fn read_available(&self) -> usize {
        self.inner.read_available()
    }

    fn write_capacity(&self) -> usize {
        self.inner.write_capacity()
    }

    fn capacity() -> usize {
        CAPACITY
    }

    fn bit_read<'a>(&'a self) -> BitReadBuffer<'a, BigEndian> {
        self.inner.bit_read()
    }

    fn bit_write_stream<'a>(&'a mut self) -> BitWriteStream<'a, BigEndian> {
        self.inner.bit_write_stream()
    }
}
