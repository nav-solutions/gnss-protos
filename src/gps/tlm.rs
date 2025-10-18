use crate::gps::{GpsError, GPS_PREAMBLE_BYTE};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// [GpsQzssTelemetry] marks the beginning of each frame
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsQzssTelemetry {
    /// TLM Message
    pub message: u16,

    /// Integrity bit is asserted means the conveying signal is provided
    /// with an enhanced level of integrity assurance.
    pub integrity: bool,

    /// Reserved bit
    pub reserved_bit: bool,
}

impl BitRead<'_, BigEndian> for GpsQzssTelemetry {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let preamble = stream.read_int::<u8>(8)?;
        let message = stream.read_int::<u16>(14)?;
        let integrity = stream.read_bool()?;
        let reserved_bit = stream.read_bool()?;
        let parity = stream.read_int::<u8>(6)?; // TODO (parity)

        if preamble == GPS_PREAMBLE_BYTE {
            Ok(Self {
                message,
                integrity,
                reserved_bit,
            })
        } else {
            Err(BitError::UnmatchedDiscriminant {
                discriminant: preamble as usize,
                enum_name: "preamble".to_string(),
            })
        }
    }
}

impl BitWrite<BigEndian> for GpsQzssTelemetry {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(GPS_PREAMBLE_BYTE, 8)?;
        stream.write_int(self.message & 0x3fff, 14)?;
        stream.write_bool(self.integrity)?;
        stream.write_bool(self.reserved_bit)?;

        stream.write_int(0, 2)?; // TODO (parity)
        stream.write_int(0, 6)?; // TODO (parity)

        Ok(())
    }
}

impl Default for GpsQzssTelemetry {
    /// Generates a default (null) [GpsQzssTelemetry].
    fn default() -> Self {
        Self {
            message: Default::default(),
            integrity: Default::default(),
            reserved_bit: Default::default(),
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
mod telemetry {
    use crate::{
        gps::{GpsBuffer, GpsQzssTelemetry},
        Buffering,
    };

    #[test]
    fn reciprocal() {
        for (dword, message, integrity, reserved_bit) in [
            (0x8B000000u32, 0x0000, false, false),
            (0x8B04F86Cu32, 0x04F8 >> 2, false, false),
            (0x8B123400u32, 0x1234 >> 2, false, false),
            (0x8B123600u32, 0x1236 >> 2, true, false),
            (0x8B123700u32, 0x048D, true, true),
            (0x8B123500u32, 0x048D, false, true),
        ] {
            let rx = GpsBuffer::from_slice(&dword.to_be_bytes());
            let mut reader = rx.bit_read_stream();

            let tlm = reader.read::<GpsQzssTelemetry>().unwrap_or_else(|e| {
                panic!("failed to decode GPS TLM from 0x{:08X} - {}", dword, e);
            });

            assert_eq!(tlm.message, message);
            assert_eq!(tlm.integrity, integrity);
            assert_eq!(tlm.reserved_bit, reserved_bit);

            let mut tx = GpsBuffer::default();
            let mut writer = tx.bit_write_stream();

            assert!(writer.write(&tlm).is_ok(), "failed to encode frame");

            let mut reader = tx.bit_read_stream();
            let decoded = reader.read::<GpsQzssTelemetry>().unwrap_or_else(|e| {
                panic!("GPS TLM reciprocal failed: {}", e);
            });

            assert_eq!(decoded, tlm, "GPS TLM reciprocal failed");
        }
    }
}
