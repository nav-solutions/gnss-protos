// provides std::io wrappers conveniently, when available to the system.
use crate::buffer::{Buffer, BufferingError};

impl Into<std::io::Error> for BufferingError {
    fn into(self) -> std::io::Error {
        match self {
            Self::WouldBlock => std::io::ErrorKind::WouldBlock.into(),
            Self::StorageFull => std::io::ErrorKind::StorageFull.into(),
        }
    }
}

impl<const M: usize> std::io::Read for Buffer<M> {
    /// Grabs bytes from this [Read]able [Buffer], this is most useful when
    /// interfacing and receiving a real-time stream.
    ///
    /// Not all returned bytes will be significant (last byte is zero terminated):
    /// you need to check [Self::read_available_bits] ahead of time.
    fn read(&mut self, dest: &mut [u8]) -> std::io::Result<usize> {
        let size = self
            .read(dest)
            .map_err(|e| Into::<std::io::Error>::into(e))?;
        Ok(size)
    }
}

impl<const M: usize> std::io::Write for Buffer<M> {
    /// [Buffer] implements the standard bytewise [std::io::Write] operation,
    /// for convenient system interfacing, usually needed when receiving a real-time stream.
    ///
    /// [Buffer] then proposes other methods to operate at the bit level.
    fn write(&mut self, src: &[u8]) -> std::io::Result<usize> {
        let size = self
            .write(src)
            .map_err(|e| Into::<std::io::Error>::into(e))?;
        Ok(size)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
