use crate::{BufferingError, Message};

#[cfg(doc)]
use crate::Decoder;

use bitbuffer::{
    BigEndian, BitReadBuffer, BitReadStream, BitWriteStream, Endianness, LittleEndian,
};

/// All our protocols buffer implement the [Buffer] trait.
pub trait Buffer: Default {
    /// Creates a new buffered object from a sliced view.
    fn from_slice(slice: &[u8]) -> Self;

    /// Converts this buffered object to a sliced view.
    fn to_slice(&self) -> &[u8];

    /// Feed new bytes into this mutable buffer.
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError>;

    /// Returns true when not a single byte may be accepted, buffer is full.
    /// You need to consume, with [std::io::Read] if feasible for example.
    fn is_full(&self) -> bool;

    /// Returns true when not a single byte is currently buffered.
    fn is_empty(&self) -> bool;

    /// Returns current storage capacity (bytewise)
    fn write_capacity(&self) -> usize;

    /// Returns total number of bytes currently available for read operation.
    fn read_available(&self) -> usize;

    /// Returns total buffering capacity (bytewise)
    fn capacity() -> usize;

    /// Obtain a [BitReadBuffer] from current readable bytes.
    /// Usually used by receivers, possibly in streaming implementations.
    fn to_bitread_buffer<'a, E: Endianness>(&'a self, endianness: E) -> BitReadBuffer<'a, E>;
}

/// Generic [StaticBuffer] used by receivers and protocols decoding,
/// where a fixed size may apply.
#[derive(Copy, Clone)]
pub(crate) struct StaticBuffer<const M: usize> {
    /// RD pointer
    rd_ptr: usize,

    /// WR pointer
    wr_ptr: usize,

    /// Internal storage, for more than two frames.
    inner: [u8; M],
}

impl<const M: usize> Default for StaticBuffer<M> {
    fn default() -> Self {
        Self {
            rd_ptr: 0,
            wr_ptr: 0,
            inner: [0; M],
        }
    }
}

impl<const M: usize> Buffer for StaticBuffer<M> {
    /// Feed new data into the buffer.
    /// Returns total number of bytes that were correctly latched.
    /// Returns [BufferingError] on errors.
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError> {
        let size = src.len();
        let capacity = self.write_capacity();

        if capacity == 0 {
            return Err(BufferingError::StorageFull);
        }

        if capacity > size {
            // copies all provided bytes
            self.inner[self.wr_ptr..self.wr_ptr + size].copy_from_slice(&src);
            self.wr_ptr += size;
            Ok(size)
        } else {
            // copies part of provided bytes
            self.inner[self.wr_ptr..].copy_from_slice(&src[..capacity]);
            self.wr_ptr = M;
            Ok(capacity)
        }
    }

    fn to_slice(&self) -> &[u8] {
        &self.inner
    }

    fn from_slice(slice: &[u8]) -> Self {
        Self {
            rd_ptr: 0,
            wr_ptr: slice.len(),
            inner: {
                let mut values = [0; M];

                let size = slice.len().min(M);

                for i in 0..size {
                    values[i] = slice[i];
                }

                values
            },
        }
    }

    fn is_full(&self) -> bool {
        self.wr_ptr == M
    }

    fn is_empty(&self) -> bool {
        self.rd_ptr == 0
    }

    fn write_capacity(&self) -> usize {
        M - self.wr_ptr
    }

    fn read_available(&self) -> usize {
        self.wr_ptr - self.rd_ptr
    }

    fn capacity() -> usize {
        M
    }

    fn to_bitread_buffer<'a, E: Endianness>(&'a self, endianness: E) -> BitReadBuffer<'a, E> {
        BitReadBuffer::new(&self.inner[self.rd_ptr..self.wr_ptr], endianness)
    }
}

#[cfg(feature = "std")]
impl<const M: usize> std::io::Write for StaticBuffer<M> {
    fn write(&mut self, src: &[u8]) -> std::io::Result<usize> {
        let size = self
            .fill(src)
            .map_err(|e| Into::<std::io::Error>::into(e))?;

        Ok(size)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

#[cfg(feature = "std")]
impl<const M: usize> std::io::Read for StaticBuffer<M> {
    fn read(&mut self, dest: &mut [u8]) -> std::io::Result<usize> {
        Ok(0)
    }
}
