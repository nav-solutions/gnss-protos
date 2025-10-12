use crate::BufferingError;

#[cfg(doc)]
use crate::Message;

use bitbuffer::{BigEndian, BitReadBuffer, BitReadStream, BitWriteStream, Endianness};

/// All our protocols buffer implement the [Buffering] trait.
pub trait Buffering: Default {
    /// Creates a new buffered object from a sliced view.
    /// The first received bits must be stored in most signficant position (Big Endian stream).
    fn from_slice(slice: &[u8]) -> Self;

    /// Converts this buffered object to a sliced view.
    fn to_slice(&self) -> &[u8];

    /// Feed new data into this mutable buffer.
    /// The first received bits must be stored in most signficant position (Big Endian stream).
    fn fill(&mut self, src: &[u8]) -> Result<usize, BufferingError>;

    /// Returns true when not a single byte may be accept, buffer is full.
    fn is_full(&self) -> bool;

    /// Returns true when not a single byte is currently buffered.
    fn is_empty(&self) -> bool;

    /// Returns current storage capacity (bytewise)
    fn write_capacity(&self) -> usize;

    /// Returns total number of bytes currently available for read operation.
    fn read_available(&self) -> usize;

    /// Returns total buffering capacity (bytewise)
    fn capacity() -> usize;

    /// Creates a [BitReadBuffer] view with preset [Endianness], from current buffer state.
    fn bit_read<'a>(&'a self) -> BitReadBuffer<'a, BigEndian>;

    /// Creates a [BitReadStream]er with preset [Endianness], from current buffer state.
    fn bit_read_stream<'a>(&'a self) -> BitReadStream<'a, BigEndian> {
        BitReadStream::new(self.bit_read())
    }

    /// Creates a mutable [BitWriteStream]er with preset [Endianness]
    fn bit_write_stream<'a>(&'a mut self) -> BitWriteStream<'a, BigEndian>;
}

/// Generic [StreamBuffer] used by receiving and decoding proceses.
/// For correct operations, we always recommend a minimal allocation
/// so 2 [Messages] fit in the buffer entirely.
/// The higher the allocation here, the more efficient your I/O operations.
#[derive(Copy, Clone)]
pub(crate) struct StreamBuffer<const M: usize> {
    /// RD pointer
    pub(crate) rd_ptr: usize,

    /// WR pointer
    pub(crate) wr_ptr: usize,

    /// Internal storage, for more than two frames.
    inner: [u8; M],
}

impl<const M: usize> Default for StreamBuffer<M> {
    /// Allocates a new [StreamBuffer].
    fn default() -> Self {
        Self {
            rd_ptr: 0,
            wr_ptr: 0,
            inner: [0; M],
        }
    }
}

impl<const M: usize> Buffering for StreamBuffer<M> {
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

    /// Returns total number of bytes currently available for read operation.
    fn read_available(&self) -> usize {
        self.rd_ptr
    }

    /// Returns total buffering capacity (bytewise)
    fn capacity() -> usize {
        M
    }

    /// Creates a [BitReadBuffer] view with desired [Endianness], from current buffer state.
    fn bit_read<'a>(&'a self) -> BitReadBuffer<'a, BigEndian> {
        BitReadBuffer::new(&self.inner[self.rd_ptr..], BigEndian::endianness())
    }

    fn bit_write_stream<'a>(&'a mut self) -> BitWriteStream<'a, BigEndian> {
        BitWriteStream::from_slice(&mut self.inner[self.wr_ptr..], BigEndian::endianness())
    }
}

#[cfg(feature = "std")]
impl<const M: usize> std::io::Write for StreamBuffer<M> {
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
