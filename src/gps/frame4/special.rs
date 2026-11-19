//! Special message (subframe 4, page 17, SV ID 55).

use crate::{
    gps::{GPS_QZSS_DATA_ID, GPS_SUBFRAME_DATA_BYTES, GPS_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// Number of characters of a special message.
pub const GPS_SPECIAL_MESSAGE_CHARS: usize = 22;

/// [GpsQzssSpecialMessage] (IS-GPS-200 §20.3.3.5.1.8), transmitted in
/// subframe 4 page 17 (SV ID 55): 22 8-bit ASCII characters.
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssSpecialMessage {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// 22 characters (8-bit ASCII)
    pub message: [u8; GPS_SPECIAL_MESSAGE_CHARS],

    /// Reserved bits (6 bits) of word 10
    pub reserved: u8,
}

crate::bits::impl_eq_by_encoding!(GpsQzssSpecialMessage, GPS_SUBFRAME_DATA_BYTES);

impl Default for GpsQzssSpecialMessage {
    fn default() -> Self {
        Self {
            data_id: GPS_QZSS_DATA_ID,
            message: [b' '; GPS_SPECIAL_MESSAGE_CHARS],
            reserved: 0,
        }
    }
}

impl Message<BigEndian> for GpsQzssSpecialMessage {
    fn encoding_size(&self) -> usize {
        GPS_SUBFRAME_DATA_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssSpecialMessage {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(Self::SV_ID, 6)?;
        stream.write_bytes(&self.message)?;
        stream.write_int(self.reserved & 0x3f, 6)?;
        stream.write_int(0u8, 2) // t-bits: solved when the frame is encoded
    }
}

impl BitRead<'_, BigEndian> for GpsQzssSpecialMessage {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;

        if sv_id != Self::SV_ID {
            return Err(BitError::UnmatchedDiscriminant {
                discriminant: sv_id as usize,
                enum_name: "GpsQzssSpecialMessage".to_string(),
            });
        }

        let mut message = [0u8; GPS_SPECIAL_MESSAGE_CHARS];
        message.copy_from_slice(&stream.read_bytes(GPS_SPECIAL_MESSAGE_CHARS)?);

        let reserved = stream.read_int::<u8>(6)?;
        let _t = stream.read_int::<u8>(2)?;

        Ok(Self {
            data_id,
            message,
            reserved,
        })
    }
}

impl GpsQzssSpecialMessage {
    /// SV (page) ID of the special message page.
    pub const SV_ID: u8 = 55;

    /// Returns the message as text, if it only contains ASCII characters.
    pub fn text(&self) -> Option<&str> {
        if self.message.is_ascii() {
            core::str::from_utf8(&self.message).ok()
        } else {
            None
        }
    }

    /// Copies and returns [GpsQzssSpecialMessage] with updated text.
    /// The text is truncated to 22 characters, and padded with spaces.
    pub fn with_text(mut self, text: &str) -> Self {
        self.message = [b' '; GPS_SPECIAL_MESSAGE_CHARS];

        for (i, byte) in text.bytes().take(GPS_SPECIAL_MESSAGE_CHARS).enumerate() {
            self.message[i] = byte;
        }

        self
    }

    /// Copies and returns [GpsQzssSpecialMessage] with updated reserved bits (6 bits).
    pub fn with_reserved_bits(mut self, reserved: u8) -> Self {
        self.reserved = reserved & 0x3f;
        self
    }

    #[cfg(test)]
    pub(crate) fn model() -> Self {
        Self::default().with_text("GPS IS-GPS-200 NAV MSG")
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bits::read_bits;

    #[test]
    fn reciprocal() {
        for message in [
            GpsQzssSpecialMessage::default(),
            GpsQzssSpecialMessage::model(),
            GpsQzssSpecialMessage::default()
                .with_text("short")
                .with_reserved_bits(0x2a),
        ] {
            let mut bytes = [0u8; 24];
            assert_eq!(message.encode(&mut bytes).unwrap(), 24);

            let decoded = GpsQzssSpecialMessage::decode(&bytes).unwrap();
            assert_eq!(decoded, message);
            assert_eq!(decoded.text(), message.text());
            assert_eq!(decoded.reserved, message.reserved);

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes);
        }
    }

    #[test]
    fn layout() {
        let message = GpsQzssSpecialMessage::default()
            .with_text("ABCDEFGHIJKLMNOPQRSTUV")
            .with_reserved_bits(0x15);

        let mut bytes = [0u8; 24];
        message.encode(&mut bytes).unwrap();

        assert_eq!(read_bits(&bytes, 0, 2), Some(1));
        assert_eq!(read_bits(&bytes, 2, 6), Some(55));

        // 2 characters in word 3, 3 in words 4 to 9, 2 in word 10
        for (i, c) in b"ABCDEFGHIJKLMNOPQRSTUV".iter().enumerate() {
            assert_eq!(read_bits(&bytes, 8 + 8 * i, 8), Some(*c as u32));
        }

        assert_eq!(read_bits(&bytes, 184, 6), Some(0x15));
    }

    #[test]
    fn text() {
        let message = GpsQzssSpecialMessage::default().with_text("HELLO");
        assert_eq!(message.text(), Some("HELLO                 "));

        let message =
            GpsQzssSpecialMessage::default().with_text("THIS MESSAGE IS FAR TOO LONG TO FIT");
        assert_eq!(message.text(), Some("THIS MESSAGE IS FAR TO"));

        let mut message = GpsQzssSpecialMessage::default();
        message.message[3] = 0xff;
        assert_eq!(message.text(), None);
    }
}
