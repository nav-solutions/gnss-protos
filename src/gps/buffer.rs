use crate::{buffer::StreamBuffer, Buffering, BufferingError, gps::GpsQzssFrame};

use bitbuffer::{BigEndian, BitReadBuffer, BitWriteStream, Endianness};

/// internal storage for more than 2 frames (300 bits)
const CAPACITY: usize = 1024;

#[derive(Default, Copy, Clone)]
pub struct GpsBuffer {
    pub(crate) inner: StreamBuffer<CAPACITY>,
}

impl Buffering for GpsBuffer {
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError> {
        self.inner.fill(src)
    }

    fn to_slice(&self) -> &[u8] {
        self.inner.to_slice()
    }

    fn from_slice(slice: &[u8]) -> Self {
        Self {
            inner: StreamBuffer::<CAPACITY>::from_slice(slice),
        }
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

    fn to_bitread_buffer<'a, E: Endianness>(&'a self, endianness: E) -> BitReadBuffer<'a, E> {
        self.inner.to_bitread_buffer(endianness)
    }
    
    fn write_message<MSG: Message>(&mut self, frame: GpsQzssFrame) -> Result<usize, BufferingError> {
        Ok(0)
    }
}
