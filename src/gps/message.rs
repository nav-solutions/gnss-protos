//! GPS 12"30" message (25 pages)

use crate::{
    gps::{
        GpsQzssFrame, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_QZSS_MESSAGE_BITS,
        GPS_QZSS_MESSAGE_ITEMS,
    },
    BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream, Message,
};

/// [GpsQzssMessage] represents the complete 25-page rotation
/// of EPH-1,2,3 and ALM-4, 5 frames, represented as [GPS_QZSS_MESSAGE_ITEMS] [GpsQzssFrame]s.
/// It takes 12'30" to acquire a complete message.
///
/// This structure is dedicated to applications and users that are interested
/// in collecting Almanach rotations.
///
/// [GpsQzssMessage] conveniently implements the [Message] trait:
/// - [Message::encode] allows you to dump a complete 25-page rotation very easily.
/// - [Message::read] allows you to collect a 25-page rotation from a single buffer.
///
/// Almanach content:
/// - SF.5 for p1-24 gives almanach per satellite (SVID=1-24)
/// - SF.5 p25 (SVID=51) gives ToA, WNa (8bits), health for SV1-24
/// - SF.4 p2-5, p7-10 gives almanach per satellite (SVID=25-32)
/// - SF.4 p18 (SVID=56) Ionosphere Kb alpha,beta and UTC (a0, a1, ToT, WNt..)
/// - SF.4 p25 (SVID=63) 4 bits per SV: anti-spoofing + signal block type, health for SV25-32.
/// - SF.4 p13 (SVID=52) NMCT: ERD distance correction
/// - SF.4 p17 (SVID=55) Special message (22 ASCII bytes)
/// - SF.4 p1, 6, 11, 12, 14, 15, 16, 19-24 (SVID=53,45,56-62) reserved
#[derive(Debug, Copy, Clone, PartialEq, Default)]
pub struct GpsQzssMessage([GpsQzssFrame; GPS_QZSS_MESSAGE_ITEMS]);

impl Message<BigEndian> for GpsQzssMessage {
    fn encoding_size(&self) -> usize {
        GPS_QZSS_MESSAGE_ITEMS * GPS_FRAME_BYTES
    }
    fn encoding_bits(&self) -> usize {
        GPS_QZSS_MESSAGE_ITEMS * GPS_FRAME_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssMessage {
    /// Encodes this [GpsQzssMessage] (completely, 25-page) into any [BitWrite] buffer.
    /// [GpsQzssmessage.encoding_size] must fit completely in provided buffer
    /// for this method to work.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        for i in 0..GPS_QZSS_MESSAGE_ITEMS {
            stream.write(&self.0[i])?;
        }
        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssMessage {
    /// Reads the 25-pages message rotation that must exist in provided buffer.
    /// When no gap bits exist between frames, the minimum buffer size is
    /// [GpsQzssMessage.encoding_size].
    /// This method allows discontinuity, noise and gap bits between successive
    /// [GpsQzssFrame]s (any number of bits), but the complete message rotation
    /// must exist in the buffer. Because this method will only return when
    /// all frames have been correctly decoded. And it does not support incomplete/partial
    /// messages.
    ///
    /// If the buffer does not describe a complete 25-page rotation, we return
    /// [BitError::NotEnoughData].
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let mut item = 0;
        let mut pos = 0;
        let mut bits_left = stream.bits_left();
        let mut frames = [GpsQzssFrame::default(); GPS_QZSS_MESSAGE_ITEMS];

        while bits_left > GPS_QZSS_MESSAGE_BITS {
            // try to pull a new frame
            match GpsQzssFrame::read(stream) {
                Ok(frame) => {
                    frames[item] = frame;
                    item += 1;
                    if item == GPS_QZSS_MESSAGE_ITEMS {
                        return Ok(Self(frames));
                    }
                },
                Err(_) => {
                    // advanced by one bit
                    pos += 1;
                    stream.set_pos(pos)?;
                },
            }
            bits_left = stream.bits_left();
        }
        Err(BitError::NotEnoughData {
            requested: (GPS_QZSS_MESSAGE_ITEMS - item) * GPS_FRAME_BITS,
            bits_left,
        })
    }
}
