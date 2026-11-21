use crate::{Buffer, StaticBuffer};

#[cfg(feature = "std")]
impl<const M: usize> std::io::Write for StaticBuffer<M> {
    fn write(&mut self, src: &[u8]) -> std::io::Result<usize> {
        let size = self.fill(src);

        Ok(size)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
