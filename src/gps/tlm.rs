use crate::{
    gps::{GpsError, GPS_PREAMBLE_BYTE, GPS_WORD_BITS},
    BufferingError, Message,
};

use bitbuffer::{BigEndian, BitRead, BitReadBuffer, BitWrite, BitWriteStream, Endianness};

/// [GpsQzssTelemetry] marks the beginning of each frame
#[derive(Debug, Copy, Clone, PartialEq, BitRead, BitWrite)]
pub struct GpsQzssTelemetry {
    /// First 8 bits serving as synchronization byte.
    preamble: u8,

    /// 14-bit TLM Message
    pub message: u16,

    /// Integrity bit is asserted means the conveying signal is provided
    /// with an enhanced level of integrity assurance.
    pub integrity: bool,

    /// Reserved bit
    pub reserved_bit: bool,

    /// 6 Parity bits
    parity: u8,
}

impl Message for GpsQzssTelemetry {
    type Err = GpsError;

    fn encoding_size(&self) -> usize {
        4
    }

    fn encoding_bitsize(&self) -> usize {
        GPS_WORD_BITS
    }

    /// [GpsQzssTelemetry] decoding attempt from a burst of
    /// GPS bits, where first received bits were stored in MSB position,
    /// so starting on preamble bits _without any padding_.
    fn decode(buf: &[u8]) -> Result<Self, GpsError> {
        let buf = BitReadBuffer::new(buf, BigEndian);

        let preamble = buf.read_int::<u8>(0, 8)?;
        let message = buf.read_int::<u16>(8, 14)?;
        let integrity = buf.read_bool(8 + 14)?;
        let reserved_bit = buf.read_bool(8 + 15)?;
        let parity = buf.read_int::<u8>(8 + 16, 6)?; // TODO

        if preamble == GPS_PREAMBLE_BYTE {
            Ok(Self {
                parity,
                preamble,
                message,
                integrity,
                reserved_bit,
            })
        } else {
            Err(GpsError::InvalidPreamble)
        }
    }

    /// Encodes this [GpsQzssTelemetry] starting with preamble bits
    /// on MSB position (big endian stream), last byte will be padded
    /// because a GPS word is not aligned to [u8].
    fn encode(&self, buf: &mut [u8]) -> Result<usize, GpsError> {
        let len = buf.len();
        let encoding_size = self.encoding_size();

        if len < encoding_size {
            return Err(GpsError::Buffering(BufferingError::StorageFull));
        }

        buf[0] = GPS_PREAMBLE_BYTE;
        buf[1] = ((self.message & 0x3fc0) >> 6) as u8;
        buf[2] = (self.message & 0x003f) as u8;
        buf[2] <<= 2;

        if self.integrity {
            buf[2] |= 0x02;
        }

        if self.reserved_bit {
            buf[2] |= 0x01;
        }

        buf[3] = (self.parity & 0x003f) << 2;

        Ok(encoding_size)
    }
}

impl Default for GpsQzssTelemetry {
    /// Generates a default (null) [GpsQzssTelemetry].
    fn default() -> Self {
        Self {
            preamble: GPS_PREAMBLE_BYTE,
            message: Default::default(),
            integrity: Default::default(),
            reserved_bit: Default::default(),
            parity: Default::default(), // TODO
        }
    }
}

#[cfg(feature = "std")]
impl std::fmt::Display for GpsQzssTelemetry {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "INTEGRITY={} - MSG=0x{:08X} - reserved={}",
            self.integrity, self.message, self.reserved_bit
        )
    }
}

impl GpsQzssTelemetry {
    /// Generates a realistic frame model for testing purposes
    #[cfg(test)]
    pub fn model() -> Self {
        Self::default()
            .with_message(0x1234)
            .with_integrity()
            .with_reserved_bit()
    }

    /// Copies and returns new [GpsQzssTelemetry] with updated 14-bit TLM message
    pub fn with_message(mut self, message_14b: u16) -> Self {
        self.message = message_14b & 0x3fff;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with message integrity asserted
    pub fn with_integrity(mut self) -> Self {
        self.integrity = true;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with message integrity deasserted
    pub fn without_integrity(mut self) -> Self {
        self.integrity = false;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with reserved bit asserted
    pub fn with_reserved_bit(mut self) -> Self {
        self.reserved_bit = true;
        self
    }

    /// Copies and returns new [GpsQzssTelemetry] with reserved bit deasserted
    pub fn without_reserved_bit(mut self) -> Self {
        self.reserved_bit = false;
        self
    }
}

#[cfg(test)]
mod test {
    use crate::{gps::GpsQzssTelemetry, Message};

    use bitbuffer::{BigEndian, BitRead, BitReadBuffer, BitWrite};

    #[test]
    fn test() {
        let mut buffer = [0, 1, 2, 3];

        for (dword, message, integrity, reserved_bit) in [
            (0x8B000000u32, 0x0000, false, false),
            (0x8B04F86Cu32, 0x04F8 >> 2, false, false),
            (0x8B123400u32, 0x1234 >> 2, false, false),
            (0x8B123600u32, 0x1236 >> 2, true, false),
            (0x8B123700u32, 0x048D, true, true),
            (0x8B123500u32, 0x048D, false, true),
        ] {
            let bytes = dword.to_be_bytes();

            let tlm = GpsQzssTelemetry::decode(&bytes).unwrap_or_else(|e| {
                panic!("failed to decode GPS TLM from 0x{:08X} - {}", dword, e);
            });

            assert_eq!(tlm.message, message);
            assert_eq!(tlm.integrity, integrity);
            assert_eq!(tlm.reserved_bit, reserved_bit);

            assert!(tlm.encode(&mut buffer).is_ok(), "failed to encode frame");

            let decoded = GpsQzssTelemetry::decode(&buffer).unwrap_or_else(|e| {
                panic!("GPS TLM reciprocal failed: {}", e);
            });

            assert_eq!(decoded, tlm, "GPS TLM reciprocal failed");
        }
    }
}
