//! Uninterpreted (reserved, spare, classified or dummy) pages of subframes 4 and 5.

use crate::{
    gps::{GPS_QZSS_DATA_ID, GPS_SUBFRAME_DATA_BYTES, GPS_WORD_DATA_BITS},
    Message,
};

use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

/// Number of data bytes following the data ID and SV ID, in a subframe 4 or 5 page
/// (184 bits, the 2 last bits being the t-bits of word 10).
pub const GPS_PAGE_DATA_BYTES: usize = GPS_SUBFRAME_DATA_BYTES - 1;

/// [GpsQzssReservedPage] is a subframe 4 or 5 page that is not interpreted:
/// reserved pages (SV ID 53, 54, 57 to 62 of subframe 4), dummy pages (SV ID 0),
/// or any SV ID that is not expected in this subframe.
///
/// All data bits are preserved, so the page is encoded back bit for bit.
/// The 2 last bits (t-bits of word 10) are solved when the frame is encoded,
/// and decoded as zeros.
#[derive(Debug, Copy, Clone)]
pub struct GpsQzssReservedPage {
    /// Data ID (2 bits)
    pub data_id: u8,

    /// SV (page) ID (6 bits)
    pub sv_id: u8,

    /// The 184 bits that follow the SV ID, MSB first.
    pub data: [u8; GPS_PAGE_DATA_BYTES],
}

crate::bits::impl_eq_by_encoding!(GpsQzssReservedPage, GPS_SUBFRAME_DATA_BYTES);

impl Default for GpsQzssReservedPage {
    fn default() -> Self {
        Self {
            data_id: GPS_QZSS_DATA_ID,
            sv_id: 0,
            data: [0; GPS_PAGE_DATA_BYTES],
        }
    }
}

impl Message<BigEndian> for GpsQzssReservedPage {
    fn encoding_size(&self) -> usize {
        GPS_SUBFRAME_DATA_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_DATA_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssReservedPage {
    /// Writes the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        stream.write_int(self.data_id & 0x3, 2)?;
        stream.write_int(self.sv_id & 0x3f, 6)?;
        stream.write_bytes(&self.data[..GPS_PAGE_DATA_BYTES - 1])?;
        // t-bits: solved when the frame is encoded
        stream.write_int(self.data[GPS_PAGE_DATA_BYTES - 1] >> 2, 6)?;
        stream.write_int(0u8, 2)
    }
}

impl BitRead<'_, BigEndian> for GpsQzssReservedPage {
    /// Reads the 192 data bits of words 3 to 10 (parity free: parity is handled by [crate::gps::GpsQzssFrame]).
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let data_id = stream.read_int::<u8>(2)?;
        let sv_id = stream.read_int::<u8>(6)?;

        let mut data = [0u8; GPS_PAGE_DATA_BYTES];
        data.copy_from_slice(&stream.read_bytes(GPS_PAGE_DATA_BYTES)?);

        // t-bits: no meaning
        data[GPS_PAGE_DATA_BYTES - 1] &= 0xfc;

        Ok(Self {
            data_id,
            sv_id,
            data,
        })
    }
}

impl GpsQzssReservedPage {
    /// Copies and returns [GpsQzssReservedPage] with updated SV (page) ID.
    pub fn with_sv_id(mut self, sv_id: u8) -> Self {
        self.sv_id = sv_id & 0x3f;
        self
    }

    /// Copies and returns [GpsQzssReservedPage] with updated data bits.
    /// The 2 last bits (t-bits) are always decoded as zeros.
    pub fn with_data(mut self, data: [u8; GPS_PAGE_DATA_BYTES]) -> Self {
        self.data = data;
        self
    }

    /// Returns true if this is a dummy page (SV ID 0),
    /// transmitted in place of an almanac that is not available.
    pub fn is_dummy(&self) -> bool {
        self.sv_id == 0
    }

    #[cfg(test)]
    pub(crate) fn model(sv_id: u8) -> Self {
        let mut data = [0xAA; GPS_PAGE_DATA_BYTES];
        data[GPS_PAGE_DATA_BYTES - 1] = 0xA8;
        Self::default().with_sv_id(sv_id).with_data(data)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bits::read_bits;

    #[test]
    fn reciprocal() {
        for sv_id in [0, 53, 54, 57, 58, 62] {
            let page = GpsQzssReservedPage::model(sv_id);

            let mut bytes = [0u8; 24];
            assert_eq!(page.encode(&mut bytes).unwrap(), 24);

            assert_eq!(read_bits(&bytes, 0, 2), Some(1));
            assert_eq!(read_bits(&bytes, 2, 6), Some(sv_id as u32));

            let decoded = GpsQzssReservedPage::decode(&bytes).unwrap();
            assert_eq!(decoded, page);
            assert_eq!(decoded.data, page.data);
            assert_eq!(decoded.is_dummy(), sv_id == 0);

            let mut reencoded = [0u8; 24];
            decoded.encode(&mut reencoded).unwrap();
            assert_eq!(reencoded, bytes);
        }
    }

    #[test]
    fn t_bits_are_ignored() {
        let page = GpsQzssReservedPage::default().with_data([0xff; GPS_PAGE_DATA_BYTES]);

        let mut bytes = [0u8; 24];
        page.encode(&mut bytes).unwrap();
        assert_eq!(bytes[23], 0xfc);

        let decoded = GpsQzssReservedPage::decode(&bytes).unwrap();
        assert_eq!(decoded.data[GPS_PAGE_DATA_BYTES - 1], 0xfc);

        // bit true equality: the t-bits carry no information
        assert_eq!(decoded, page);
    }
}
