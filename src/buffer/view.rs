#[cfg(doc)]
use crate::buffer::Buffer;

/// [BufferView] is obtained from a pre-allocated [Buffer]
/// and supports bitwise and bytewise iteration methods,
/// which is particularly helpful to decode unaligned protocols.
///
/// [BufferView] implements [Iterator] for convenient iteration
/// but it is stepped bytes per bytes ([u8] size) at the moment.
#[derive(Copy, Clone)]
pub struct BufferView<'a, const M: usize> {
    /// Snapshot view
    inner: &'a [u8; M],

    /// Pointer
    ptr: usize,
}

impl<'a, const M: usize> BufferView<'a, M> {
    pub(crate) fn from_slice(slice: &'a [u8; M]) -> Self {
        Self {
            ptr: 0,
            inner: slice,
        }
    }

    /// Returns the total size (bytewise) of this snapshot view
    pub const fn len(&self) -> usize {
        M
    }
}

impl<'a, const M: usize> Iterator for BufferView<'a, M> {
    type Item = u8;

    /// Grab the next byte ([u8]) from this [BufferView]
    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr == self.len() {
            None
        } else {
            let ret = self.inner[self.ptr];
            self.ptr += 1;
            Some(ret)
        }
    }
}
