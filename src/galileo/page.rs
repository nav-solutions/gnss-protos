use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{get_bit, read_bits, write_bits},
    galileo::{
        link::{
            self, GALILEO_INAV_PART_BYTES, GALILEO_INAV_PART_DATA_BITS,
            GALILEO_INAV_PART_ENCODED_BYTES, GALILEO_INAV_PART_SYMBOLS, GALILEO_INAV_SYNC_PATTERN,
            GALILEO_INAV_SYNC_SYMBOLS, GALILEO_INAV_TAIL_BITS,
        },
        GalileoError, GalileoInavWord, GALILEO_INAV_WORD_BYTES,
    },
    Message,
};

/// Total number of symbols of an I/NAV page (even and odd parts, sync patterns included):
/// 2 seconds at 250 symbols per second.
pub const GALILEO_INAV_PAGE_SYMBOLS: usize = 2 * GALILEO_INAV_PART_SYMBOLS;

/// Number of bytes needed to store the symbols of an I/NAV page.
pub const GALILEO_INAV_PAGE_BYTES: usize = (GALILEO_INAV_PAGE_SYMBOLS + 7) / 8;

/// Number of data bits of the word carried by the even page part.
const EVEN_WORD_BITS: usize = 112;

/// Number of data bits of the word carried by the odd page part.
const ODD_WORD_BITS: usize = 16;

/// Bit positions within the odd page part.
const OSNMA_POS: usize = 2 + ODD_WORD_BITS;
const OSNMA_BITS: usize = 40;
const SAR_POS: usize = OSNMA_POS + OSNMA_BITS;
const SAR_BITS: usize = 22;
const SPARE_POS: usize = SAR_POS + SAR_BITS;
const SPARE_BITS: usize = 2;
const CRC_POS: usize = SPARE_POS + SPARE_BITS;
const CRC_BITS: usize = 24;
const SSP_POS: usize = CRC_POS + CRC_BITS;
const SSP_BITS: usize = 8;

/// Number of bits protected by the CRC: the 114 data bits of the even part,
/// and the 82 first bits of the odd part.
const CRC_COVERED_BITS: usize = GALILEO_INAV_PART_DATA_BITS + CRC_POS;

/// I/NAV page type, transmitted in both page parts.
#[derive(Debug, Default, Copy, Clone, PartialEq, Eq)]
pub enum GalileoInavPageType {
    /// Nominal page, carrying a [GalileoInavWord].
    #[default]
    Nominal,

    /// Alert page: its 128 data bits are reserved (decoded as [GalileoInavWord::Raw]).
    Alert,
}

/// Galileo I/NAV page (OS SIS ICD v2.1, §4.3.2), as transmitted on E1-B
/// (and E5b-I), every 2 seconds: an even page part followed by an odd page part.
///
/// | Part | Even/Odd | Page Type | Data (word) | OSNMA | SAR | Spare | CRC | SSP | Tail |
/// |------|----------|-----------|-------------|-------|-----|-------|-----|-----|------|
/// | Even | 1 (0)    | 1         | 112 (1/2)   |       |     |       |     |     | 6    |
/// | Odd  | 1 (1)    | 1         | 16 (2/2)    | 40    | 22  | 2     | 24  | 8   | 6    |
///
/// On E5b-I, the OSNMA, SAR and Spare fields (64 bits) are "Reserved 1"
/// and the SSP field is "Reserved 2": they are carried as is.
///
/// A page exists under three representations:
/// - the transmitted symbols: 2 x 250 symbols (synchronization pattern,
///   FEC encoding and interleaving) ([Self::decode_at], [Self::encode_at], and the
///   [BitRead] / [BitWrite] implementations);
/// - the two 120-bit page parts ([Self::decode_parts], [Self::encode_parts]), as most receivers
///   report them, for example u-blox UBX-RXM-SFRBX ([Self::decode_ublox_words],
///   [Self::encode_ublox_words]);
/// - this interpreted structure.
///
/// The CRC is always computed when encoding, and verified when decoding.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GalileoInavPage {
    /// [GalileoInavPageType]
    pub page_type: GalileoInavPageType,

    /// [GalileoInavWord] carried by this page.
    pub word: GalileoInavWord,

    /// OSNMA data (40 bits, E1-B), see Galileo OSNMA SIS ICD.
    pub osnma: u64,

    /// Search And Rescue data (22 bits, E1-B).
    pub sar: u32,

    /// Spare bits (2 bits, E1-B).
    pub spare: u8,

    /// Secondary Synchronization Pattern (8 bits, E1-B), not protected by the CRC.
    pub ssp: u8,
}

impl Message<BigEndian> for GalileoInavPage {
    fn encoding_size(&self) -> usize {
        GALILEO_INAV_PAGE_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GALILEO_INAV_PAGE_SYMBOLS
    }
}

/// [BitRead] only allows [BitError]s: express Galileo errors as such.
fn bit_error(e: GalileoError) -> BitError {
    match e {
        GalileoError::BitStream(e) => e,
        e => BitError::UnmatchedDiscriminant {
            discriminant: 0,
            enum_name: e.to_string(),
        },
    }
}

/// Reads a page part (250 symbols) from the stream, at its current position.
fn read_part(
    stream: &mut BitReadStream<'_, BigEndian>,
) -> Result<[u8; GALILEO_INAV_PART_BYTES], BitError> {
    let mut buffer = [0u8; 2 + GALILEO_INAV_PART_ENCODED_BYTES];
    write_bits(
        &mut buffer,
        0,
        stream.read_int::<u32>(GALILEO_INAV_SYNC_SYMBOLS)?,
        GALILEO_INAV_SYNC_SYMBOLS,
    );

    for i in 0..GALILEO_INAV_PART_ENCODED_BYTES {
        let byte = stream.read_int::<u8>(8)?;
        write_bits(
            &mut buffer,
            GALILEO_INAV_SYNC_SYMBOLS + 8 * i,
            byte as u32,
            8,
        );
    }

    let (symbols, _) =
        link::read_part_symbols(&buffer, 0).ok_or(bit_error(GalileoError::InvalidSyncPattern))?;

    Ok(link::decode_part(&symbols).0)
}

/// Writes a page part (250 symbols) to the stream.
fn write_part(
    stream: &mut BitWriteStream<'_, BigEndian>,
    part: &[u8; GALILEO_INAV_PART_BYTES],
) -> Result<(), BitError> {
    stream.write_int(GALILEO_INAV_SYNC_PATTERN, GALILEO_INAV_SYNC_SYMBOLS)?;
    stream.write_bytes(&link::encode_part(part))
}

impl BitRead<'_, BigEndian> for GalileoInavPage {
    /// Reads a complete page (500 symbols) from the stream, at its current position.
    /// Inverted symbols are supported, the CRC is always verified.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let even = read_part(stream)?;
        let odd = read_part(stream)?;
        Self::decode_parts(&even, &odd).map_err(bit_error)
    }
}

impl BitWrite<BigEndian> for GalileoInavPage {
    /// Writes a complete page (500 symbols) to the stream.
    /// Pages written one after the other form a valid I/NAV symbol stream.
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        let (even, odd) = self.encode_parts().map_err(bit_error)?;
        write_part(stream, &even)?;
        write_part(stream, &odd)
    }
}

impl GalileoInavPage {
    #[cfg(test)]
    pub(crate) fn model(word_type: u8) -> Self {
        Self::default()
            .with_word(GalileoInavWord::model(word_type))
            .with_osnma(0x12_3456_789A)
            .with_sar(0x2A_AAAA)
            .with_spare(0x1)
            .with_ssp(0x2F)
    }

    /// Copies and returns [GalileoInavPage] with updated [GalileoInavWord].
    pub fn with_word(mut self, word: GalileoInavWord) -> Self {
        self.word = word;
        self
    }

    /// Copies and returns [GalileoInavPage] with updated [GalileoInavPageType].
    pub fn with_page_type(mut self, page_type: GalileoInavPageType) -> Self {
        self.page_type = page_type;
        self
    }

    /// Copies and returns [GalileoInavPage] with updated OSNMA field (40 bits).
    pub fn with_osnma(mut self, osnma: u64) -> Self {
        self.osnma = osnma & 0xff_ffff_ffff;
        self
    }

    /// Copies and returns [GalileoInavPage] with updated SAR field (22 bits).
    pub fn with_sar(mut self, sar: u32) -> Self {
        self.sar = sar & 0x3f_ffff;
        self
    }

    /// Copies and returns [GalileoInavPage] with updated spare field (2 bits).
    pub fn with_spare(mut self, spare: u8) -> Self {
        self.spare = spare & 0x3;
        self
    }

    /// Copies and returns [GalileoInavPage] with updated Secondary Synchronization Pattern.
    pub fn with_ssp(mut self, ssp: u8) -> Self {
        self.ssp = ssp;
        self
    }

    /// Interprets the two 120-bit page parts (15 bytes each, MSB first, tail bits included),
    /// as obtained after FEC decoding. The CRC is verified.
    pub fn decode_parts(
        even: &[u8; GALILEO_INAV_PART_BYTES],
        odd: &[u8; GALILEO_INAV_PART_BYTES],
    ) -> Result<Self, GalileoError> {
        if get_bit(even, 0) || !get_bit(odd, 0) {
            return Err(GalileoError::EvenOddMismatch);
        }

        if get_bit(even, 1) != get_bit(odd, 1) {
            return Err(GalileoError::PageTypeMismatch);
        }

        for part in [even, odd] {
            if read_bits(part, GALILEO_INAV_PART_DATA_BITS, GALILEO_INAV_TAIL_BITS) != Some(0) {
                return Err(GalileoError::TailBits);
            }
        }

        if Self::crc(even, odd) != read_bits(odd, CRC_POS, CRC_BITS).unwrap_or_default() {
            return Err(GalileoError::Crc);
        }

        let page_type = if get_bit(even, 1) {
            GalileoInavPageType::Alert
        } else {
            GalileoInavPageType::Nominal
        };

        // 112 + 16 bits of the word
        let mut bytes = [0u8; GALILEO_INAV_WORD_BYTES];

        for (i, byte) in bytes.iter_mut().take(EVEN_WORD_BITS / 8).enumerate() {
            *byte = read_bits(even, 2 + 8 * i, 8).unwrap_or_default() as u8;
        }

        bytes[14] = read_bits(odd, 2, 8).unwrap_or_default() as u8;
        bytes[15] = read_bits(odd, 10, 8).unwrap_or_default() as u8;

        let word = match page_type {
            GalileoInavPageType::Nominal => GalileoInavWord::decode_bytes(&bytes)?,
            GalileoInavPageType::Alert => GalileoInavWord::Raw(bytes),
        };

        let osnma = ((read_bits(odd, OSNMA_POS, 8).unwrap_or_default() as u64) << 32)
            | read_bits(odd, OSNMA_POS + 8, 32).unwrap_or_default() as u64;

        Ok(Self {
            page_type,
            word,
            osnma,
            sar: read_bits(odd, SAR_POS, SAR_BITS).unwrap_or_default(),
            spare: read_bits(odd, SPARE_POS, SPARE_BITS).unwrap_or_default() as u8,
            ssp: read_bits(odd, SSP_POS, SSP_BITS).unwrap_or_default() as u8,
        })
    }

    /// Encodes this page as two 120-bit page parts (15 bytes each, MSB first, tail bits included),
    /// ready for FEC encoding. The CRC is computed.
    pub fn encode_parts(
        &self,
    ) -> Result<([u8; GALILEO_INAV_PART_BYTES], [u8; GALILEO_INAV_PART_BYTES]), GalileoError> {
        let mut even = [0u8; GALILEO_INAV_PART_BYTES];
        let mut odd = [0u8; GALILEO_INAV_PART_BYTES];

        let alert = (self.page_type == GalileoInavPageType::Alert) as u32;
        let bytes = self.word.encode_bytes()?;

        write_bits(&mut even, 0, alert, 2);
        write_bits(&mut odd, 0, 0x2 | alert, 2);

        for (i, byte) in bytes.iter().take(EVEN_WORD_BITS / 8).enumerate() {
            write_bits(&mut even, 2 + 8 * i, *byte as u32, 8);
        }

        write_bits(&mut odd, 2, bytes[14] as u32, 8);
        write_bits(&mut odd, 10, bytes[15] as u32, 8);

        write_bits(&mut odd, OSNMA_POS, (self.osnma >> 32) as u32 & 0xff, 8);
        write_bits(&mut odd, OSNMA_POS + 8, self.osnma as u32, 32);
        write_bits(&mut odd, SAR_POS, self.sar & 0x3f_ffff, SAR_BITS);
        write_bits(&mut odd, SPARE_POS, (self.spare & 0x3) as u32, SPARE_BITS);
        let crc = Self::crc(&even, &odd);
        write_bits(&mut odd, CRC_POS, crc, CRC_BITS);
        write_bits(&mut odd, SSP_POS, self.ssp as u32, SSP_BITS);

        Ok((even, odd))
    }

    /// Computes the CRC-24Q of a page: 114 data bits of the even part
    /// followed by the 82 first bits of the odd part (196 bits).
    fn crc(even: &[u8; GALILEO_INAV_PART_BYTES], odd: &[u8; GALILEO_INAV_PART_BYTES]) -> u32 {
        // 4 leading zeros (do not modify the CRC) to align on bytes: 200 bits
        let mut covered = [0u8; (CRC_COVERED_BITS + 4) / 8];

        for i in 0..GALILEO_INAV_PART_DATA_BITS {
            if get_bit(even, i) {
                write_bits(&mut covered, 4 + i, 1, 1);
            }
        }

        for i in 0..CRC_POS {
            if get_bit(odd, i) {
                write_bits(&mut covered, 4 + GALILEO_INAV_PART_DATA_BITS + i, 1, 1);
            }
        }

        link::crc24q(&covered, 0, covered.len() * 8)
    }

    /// Interprets a page as reported by u-blox receivers (UBX-RXM-SFRBX, Galileo E1-B / E5b-I):
    /// 8 words of 32 bits, the even page part (120 bits) in the 4 first words,
    /// the odd page part in the 4 last words (8 padding bits each). The CRC is verified.
    pub fn decode_ublox_words(words: &[u32; 8]) -> Result<Self, GalileoError> {
        let mut bytes = [0u8; 32];

        for (i, word) in words.iter().enumerate() {
            bytes[4 * i..4 * i + 4].copy_from_slice(&word.to_be_bytes());
        }

        let mut even = [0u8; GALILEO_INAV_PART_BYTES];
        let mut odd = [0u8; GALILEO_INAV_PART_BYTES];
        even.copy_from_slice(&bytes[..GALILEO_INAV_PART_BYTES]);
        odd.copy_from_slice(&bytes[16..16 + GALILEO_INAV_PART_BYTES]);

        Self::decode_parts(&even, &odd)
    }

    /// Encodes this page as u-blox receivers report it (UBX-RXM-SFRBX), see
    /// [Self::decode_ublox_words]. Padding bits are set to zero.
    pub fn encode_ublox_words(&self) -> Result<[u32; 8], GalileoError> {
        let (even, odd) = self.encode_parts()?;

        let mut bytes = [0u8; 32];
        bytes[..GALILEO_INAV_PART_BYTES].copy_from_slice(&even);
        bytes[16..16 + GALILEO_INAV_PART_BYTES].copy_from_slice(&odd);

        let mut words = [0u32; 8];

        for (i, word) in words.iter_mut().enumerate() {
            *word = u32::from_be_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ]);
        }

        Ok(words)
    }

    /// Decodes a page whose first symbol (start of the even part synchronization pattern)
    /// is located at any symbol position (`symbol_offset`) of `slice`.
    /// The page must start at that position exactly: use the
    /// [crate::galileo::GalileoInavDecoder] to search for it.
    /// Inverted symbols are supported, and symbol errors are corrected by the Viterbi decoder.
    pub fn decode_at(slice: &[u8], symbol_offset: usize) -> Result<Self, GalileoError> {
        if symbol_offset + GALILEO_INAV_PAGE_SYMBOLS > slice.len() * 8 {
            return Err(GalileoError::WouldNotFit);
        }

        let (even, _) = link::read_part_symbols(slice, symbol_offset)
            .ok_or(GalileoError::InvalidSyncPattern)?;

        let (odd, _) = link::read_part_symbols(slice, symbol_offset + GALILEO_INAV_PART_SYMBOLS)
            .ok_or(GalileoError::InvalidSyncPattern)?;

        Self::decode_parts(&link::decode_part(&even).0, &link::decode_part(&odd).0)
    }

    /// Encodes this page as 500 symbols, at any symbol position (`symbol_offset`) of `slice`.
    /// Surrounding symbols are preserved.
    /// Returns the number of symbols written (always [GALILEO_INAV_PAGE_SYMBOLS]).
    pub fn encode_at(&self, slice: &mut [u8], symbol_offset: usize) -> Result<usize, GalileoError> {
        if symbol_offset + GALILEO_INAV_PAGE_SYMBOLS > slice.len() * 8 {
            return Err(GalileoError::WouldNotFit);
        }

        let (even, odd) = self.encode_parts()?;

        link::write_part_symbols(slice, symbol_offset, &even).ok_or(GalileoError::WouldNotFit)?;

        link::write_part_symbols(slice, symbol_offset + GALILEO_INAV_PART_SYMBOLS, &odd)
            .ok_or(GalileoError::WouldNotFit)?;

        Ok(GALILEO_INAV_PAGE_SYMBOLS)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        bits::{read_bits, set_bit},
        galileo::{
            test_vectors::UBLOX_E08, GalileoError, GalileoInavPage, GalileoInavPageType,
            GalileoInavWord, GALILEO_INAV_PAGE_SYMBOLS,
        },
        Message,
    };

    const ALL_TYPES: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 10, 16, 20, 63];

    #[test]
    fn default_reciprocal() {
        let default = GalileoInavPage::default();

        let mut buffer = [0u8; 64];
        assert_eq!(default.encode(&mut buffer).unwrap(), 63);

        // starts with the synchronization pattern
        assert_eq!(read_bits(&buffer, 0, 10), Some(0x160));
        assert_eq!(read_bits(&buffer, 250, 10), Some(0x160));

        assert_eq!(GalileoInavPage::decode(&buffer).unwrap(), default);
    }

    #[test]
    fn parts_reciprocal() {
        for word_type in ALL_TYPES {
            let page = GalileoInavPage::model(word_type);
            let (even, odd) = page.encode_parts().unwrap();

            // even/odd bits, tails
            assert_eq!(even[0] & 0x80, 0);
            assert_eq!(odd[0] & 0x80, 0x80);
            assert_eq!(even[14] & 0x3f, 0);
            assert_eq!(odd[14] & 0x3f, 0);

            let decoded = GalileoInavPage::decode_parts(&even, &odd).unwrap();
            assert_eq!(decoded, page, "word type {}", word_type);
        }
    }

    #[test]
    fn symbols_reciprocal_any_offset() {
        for word_type in ALL_TYPES {
            let page = GalileoInavPage::model(word_type);

            for offset in 0..16 {
                let mut buffer = [0x5Au8; 72];
                let size = page.encode_at(&mut buffer, offset).unwrap();
                assert_eq!(size, GALILEO_INAV_PAGE_SYMBOLS);

                // surrounding symbols preserved
                let original = [0x5Au8; 72];
                assert_eq!(
                    read_bits(&buffer, 0, offset),
                    read_bits(&original, 0, offset)
                );
                assert_eq!(
                    read_bits(&buffer, offset + 500, 32),
                    read_bits(&original, offset + 500, 32)
                );

                let decoded = GalileoInavPage::decode_at(&buffer, offset).unwrap_or_else(|e| {
                    panic!("word type {} @{}: {}", word_type, offset, e);
                });

                assert_eq!(decoded, page);

                // inverted symbols
                let inverted: Vec<u8> = buffer.iter().map(|b| !b).collect();
                assert_eq!(GalileoInavPage::decode_at(&inverted, offset).unwrap(), page);
            }
        }
    }

    #[test]
    fn alert_page() {
        let mut bytes = [0x33u8; 16];
        bytes[0] = 0x04; // looks like a word type 1: not interpreted on alert pages

        let page = GalileoInavPage::default()
            .with_page_type(GalileoInavPageType::Alert)
            .with_word(GalileoInavWord::Raw(bytes));

        let (even, odd) = page.encode_parts().unwrap();
        assert_eq!(even[0] & 0xC0, 0x40);
        assert_eq!(odd[0] & 0xC0, 0xC0);

        assert_eq!(GalileoInavPage::decode_parts(&even, &odd).unwrap(), page);
    }

    #[test]
    fn errors_are_detected() {
        let page = GalileoInavPage::model(3);
        let (even, odd) = page.encode_parts().unwrap();

        // CRC protects the even data bits and the 82 first odd bits
        for bit in (2..114).chain(120 + 2..120 + 106) {
            let (mut even, mut odd) = (even, odd);
            if bit < 120 {
                even[bit / 8] ^= 0x80 >> (bit % 8);
            } else {
                odd[(bit - 120) / 8] ^= 0x80 >> ((bit - 120) % 8);
            }
            assert!(
                matches!(
                    GalileoInavPage::decode_parts(&even, &odd),
                    Err(GalileoError::Crc)
                ),
                "bit #{}",
                bit
            );
        }

        // the SSP is not protected
        let mut corrupted = odd;
        corrupted[13] ^= 0x10;
        let decoded = GalileoInavPage::decode_parts(&even, &corrupted).unwrap();
        assert_eq!(decoded.word, page.word);
        assert_ne!(decoded.ssp, page.ssp);

        // swapped parts
        assert!(matches!(
            GalileoInavPage::decode_parts(&odd, &even),
            Err(GalileoError::EvenOddMismatch)
        ));

        // page type mismatch
        let mut corrupted = odd;
        corrupted[0] ^= 0x40;
        assert!(matches!(
            GalileoInavPage::decode_parts(&even, &corrupted),
            Err(GalileoError::PageTypeMismatch)
        ));

        // tail bits
        let mut corrupted = even;
        set_bit(&mut corrupted, 119, true);
        assert!(matches!(
            GalileoInavPage::decode_parts(&corrupted, &odd),
            Err(GalileoError::TailBits)
        ));
    }

    #[test]
    fn buffer_too_small() {
        let page = GalileoInavPage::model(1);
        let mut buffer = [0u8; 63];
        assert!(page.encode_at(&mut buffer, 4).is_ok());
        assert!(matches!(
            page.encode_at(&mut buffer, 5),
            Err(GalileoError::WouldNotFit)
        ));
        assert!(matches!(
            GalileoInavPage::decode_at(&buffer, 5),
            Err(GalileoError::WouldNotFit)
        ));
        assert!(matches!(
            GalileoInavPage::decode_at(&[0u8; 63], 0),
            Err(GalileoError::InvalidSyncPattern)
        ));
    }

    #[test]
    fn real_ublox_pages_bit_exact() {
        for (word_type, words) in UBLOX_E08.iter() {
            let page = GalileoInavPage::decode_ublox_words(words).unwrap_or_else(|e| {
                panic!("word type {}: {}", word_type, e);
            });

            assert_eq!(page.page_type, GalileoInavPageType::Nominal);
            assert_eq!(page.word.word_type(), *word_type);

            // bit exact re-encoding
            assert_eq!(
                page.encode_ublox_words().unwrap(),
                *words,
                "type {}",
                word_type
            );

            // and through the symbol layer
            let mut buffer = [0u8; 64];
            page.encode_at(&mut buffer, 3).unwrap();
            assert_eq!(GalileoInavPage::decode_at(&buffer, 3).unwrap(), page);
        }
    }
}
