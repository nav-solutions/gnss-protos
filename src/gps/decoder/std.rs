use crate::{
    gps::{GpsQzssDecoder, GPS_FRAME_BYTES},
    Decoder, Message,
};

#[cfg(doc)]
use crate::gps::GpsQzssFrame;

impl std::io::Write for GpsQzssDecoder {
    /// Feed new data conveniently into this [GpsQzssDecoder],
    /// which then becomes available for [GpsQzssDecoder::decode]
    /// or [GpsQzssDecoder::read].
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

impl std::io::Read for GpsQzssDecoder {
    /// Try to decode a complete [GpsQzssFrame] from this [GpsQzssDecoder],
    /// that we directly encode into the provided mutable buffer.
    ///
    /// We do not support partial encoding, a complete [GpsQzssFrame] must fit
    /// at all times, which is [GPS_FRAME_BYTES] long.
    ///
    /// You need to provide data so it's available for decoding, either
    /// using [Write] or [GpsQzssDecoder::fill].
    /// Thanks to this operation, you may use [GpsQzssDecoder] as a FIFO.
    fn read(&mut self, dest: &mut [u8]) -> std::io::Result<usize> {
        let size = dest.len();

        if size < GPS_FRAME_BYTES {
            return Err(std::io::ErrorKind::StorageFull.into());
        }

        match self.decode() {
            Some(frame) => {
                let encoded = frame
                    .encode(dest)
                    .map_err(|e| Into::<std::io::Error>::into(e))?;

                Ok(encoded)
            },
            None => Ok(0),
        }
    }
}
