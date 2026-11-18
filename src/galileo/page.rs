use bitbuffer::{BigEndian, BitError, BitRead, BitReadStream, BitWrite, BitWriteStream};

use crate::{
    bits::{get_bit, read_bits, write_bits},
    galileo::{
        fec::{
            self, GALILEO_INAV_INVERTED_SYNC_PATTERN, GALILEO_INAV_PART_BYTES,
            GALILEO_INAV_PART_DATA_BITS, GALILEO_INAV_PART_ENCODED_BYTES, GALILEO_INAV_TAIL_BITS,
        },
        GalileoError, GalileoInavWord, GALILEO_INAV_PART_SYMBOLS, GALILEO_INAV_SYNC_PATTERN,
        GALILEO_INAV_SYNC_SYMBOLS, GALILEO_INAV_WORD_BYTES,
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
/// [GalileoInavPage] implements [Message]: [Message::encode] and [Message::decode]
/// work on the 500 transmitted symbols (2 x 250 symbols: synchronization pattern,
/// FEC encoding and interleaving), packed MSB first (63 bytes, 4 padding bits).
/// To work at any symbol position, or to stream consecutive pages,
/// use the [BitRead] / [BitWrite] implementations with a [bitbuffer] stream:
///
/// ```
/// use gnss_protos::{
///     BigEndian, BitReadBuffer, BitReadStream, BitWriteStream, GalileoInavPage,
///     GALILEO_INAV_PAGE_SYMBOLS,
/// };
///
/// let page = GalileoInavPage::default()
///     .with_ssp(0x2F);
///
/// // 2 consecutive pages, 3 symbols after the start of the stream
/// let mut symbols = Vec::new();
/// let mut writer = BitWriteStream::new(&mut symbols, BigEndian);
/// writer.write_int(0u8, 3).unwrap();
/// writer.write(&page).unwrap();
/// writer.write(&page).unwrap();
///
/// let mut reader = BitReadStream::new(BitReadBuffer::new(&symbols, BigEndian));
/// reader.set_pos(3).unwrap();
///
/// for _ in 0..2 {
///     let decoded: GalileoInavPage = reader.read().unwrap();
///     assert_eq!(decoded, page);
/// }
///
/// assert_eq!(reader.pos(), 3 + 2 * GALILEO_INAV_PAGE_SYMBOLS);
/// ```
///
/// The CRC is always computed when encoding, and verified when decoding.
/// Inverted symbols (180° carrier phase ambiguity) are decoded transparently,
/// and symbol errors are corrected by the Viterbi decoder.
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

/// Reads the 250 symbols of a page part from the stream, at its current position,
/// and returns its 240 encoded symbols (sync pattern excluded), in normal polarity:
/// an inverted page part (inverted synchronization pattern) is inverted back.
pub(crate) fn read_part_symbols(
    stream: &mut BitReadStream<'_, BigEndian>,
) -> Result<[u8; GALILEO_INAV_PART_ENCODED_BYTES], BitError> {
    let sync = stream.read_int::<u32>(GALILEO_INAV_SYNC_SYMBOLS)?;

    let inverted = if sync == GALILEO_INAV_SYNC_PATTERN {
        false
    } else if sync == GALILEO_INAV_INVERTED_SYNC_PATTERN {
        true
    } else {
        return Err(bit_error(GalileoError::InvalidSyncPattern));
    };

    let mut symbols = [0u8; GALILEO_INAV_PART_ENCODED_BYTES];

    for byte in symbols.iter_mut() {
        *byte = stream.read_int::<u8>(8)?;
    }

    if inverted {
        symbols = fec::invert_symbols(&symbols);
    }

    Ok(symbols)
}

/// Writes a page part (sync pattern and 240 encoded symbols) to the stream.
fn write_part(
    stream: &mut BitWriteStream<'_, BigEndian>,
    part: &[u8; GALILEO_INAV_PART_BYTES],
) -> Result<(), BitError> {
    stream.write_int(GALILEO_INAV_SYNC_PATTERN, GALILEO_INAV_SYNC_SYMBOLS)?;
    stream.write_bytes(&fec::encode_part(part))
}

impl BitRead<'_, BigEndian> for GalileoInavPage {
    /// Reads a complete page (500 symbols) from the stream, at its current position.
    /// Inverted symbols are supported, the CRC is always verified.
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let even = fec::decode_part(&read_part_symbols(stream)?).0;
        let odd = fec::decode_part(&read_part_symbols(stream)?).0;
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
    pub(crate) fn decode_parts(
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
            GalileoInavPageType::Nominal => GalileoInavWord::decode(&bytes)?,
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
    pub(crate) fn encode_parts(
        &self,
    ) -> Result<([u8; GALILEO_INAV_PART_BYTES], [u8; GALILEO_INAV_PART_BYTES]), GalileoError> {
        let mut even = [0u8; GALILEO_INAV_PART_BYTES];
        let mut odd = [0u8; GALILEO_INAV_PART_BYTES];

        let alert = (self.page_type == GalileoInavPageType::Alert) as u32;
        let mut bytes = [0u8; GALILEO_INAV_WORD_BYTES];
        self.word.encode(&mut bytes)?;

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

        crc24q(&covered, 0, covered.len() * 8)
    }
}

/// CRC-24Q generator polynomial, X^24 term omitted:
/// G(X) = (1 + X).P(X) with P(X) = X^23 + X^17 + X^13 + X^12 + X^11 + X^9 + X^8 + X^7 + X^5 + X^3 + 1.
const CRC24Q_POLY: u32 = 0x86_4CFB;

/// Computes the CRC-24Q of `count` bits of `slice`, starting at absolute
/// bit position `bit_pos` (MSB first). The register is initialized to zero.
fn crc24q(slice: &[u8], bit_pos: usize, count: usize) -> u32 {
    let mut crc = 0u32;

    for pos in bit_pos..bit_pos + count {
        let top = ((crc >> 23) & 0x1) > 0;
        crc = (crc << 1) & 0xff_ffff;

        if top ^ get_bit(slice, pos) {
            crc ^= CRC24Q_POLY;
        }
    }

    crc
}

#[cfg(test)]
mod test {
    use super::crc24q;

    use crate::{
        bits::{get_bit, read_bits, set_bit},
        galileo::{
            fec,
            test_vectors::{parts_to_ublox, ublox_to_parts, UBLOX_E08},
            GalileoError, GalileoInavPage, GalileoInavPageType, GalileoInavWord,
            GALILEO_INAV_PAGE_BYTES, GALILEO_INAV_PAGE_SYMBOLS, GALILEO_INAV_PART_SYMBOLS,
            GALILEO_INAV_SYNC_PATTERN,
        },
        BigEndian, Message,
    };

    use bitbuffer::{BitReadBuffer, BitReadStream, BitWriteStream};

    const ALL_TYPES: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 10, 16, 20, 63];

    /// Encodes with the [Message] trait, verifies the encoded size,
    /// and returns the 63 encoded bytes.
    fn encode(page: &GalileoInavPage) -> [u8; GALILEO_INAV_PAGE_BYTES] {
        let mut buffer = [0u8; GALILEO_INAV_PAGE_BYTES];
        assert_eq!(page.encode(&mut buffer).unwrap(), GALILEO_INAV_PAGE_BYTES);
        buffer
    }

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
    fn model_bit_exact_reciprocal() {
        for word_type in ALL_TYPES {
            let page = GalileoInavPage::model(word_type);
            let encoded = encode(&page);

            // 4 padding bits are left null
            assert_eq!(encoded[GALILEO_INAV_PAGE_BYTES - 1] & 0x0f, 0);

            let decoded = GalileoInavPage::decode(&encoded).unwrap();
            assert_eq!(decoded, page, "word type {}", word_type);

            // encode(decode(x)) == x, symbol for symbol
            assert_eq!(encode(&decoded), encoded, "word type {}", word_type);
        }
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

            // the transmitted symbols are the FEC encoded page parts
            let encoded = encode(&page);
            let mut reader = BitReadStream::new(BitReadBuffer::new(&encoded, BigEndian));
            assert_eq!(
                fec::decode_part(&super::read_part_symbols(&mut reader).unwrap()),
                (even, 0)
            );
            assert_eq!(
                fec::decode_part(&super::read_part_symbols(&mut reader).unwrap()),
                (odd, 0)
            );
        }
    }

    #[test]
    fn symbols_reciprocal_any_offset() {
        for word_type in ALL_TYPES {
            let page = GalileoInavPage::model(word_type);
            let reference = encode(&page);

            for offset in 0..16 {
                let mut symbols = Vec::new();

                {
                    let mut writer = BitWriteStream::new(&mut symbols, BigEndian);
                    writer
                        .write_int(0x5A5Au32 >> (16 - offset), offset)
                        .unwrap();
                    writer.write(&page).unwrap();
                    assert_eq!(writer.bit_len(), offset + GALILEO_INAV_PAGE_SYMBOLS);
                    writer.write_int(0x5Au8, 8).unwrap();
                }

                // surrounding symbols
                assert_eq!(
                    read_bits(&symbols, 0, offset),
                    Some(0x5A5Au32 >> (16 - offset))
                );
                assert_eq!(
                    read_bits(&symbols, offset + GALILEO_INAV_PAGE_SYMBOLS, 8),
                    Some(0x5A)
                );

                // the 500 symbols are identical at any offset
                for symbol in 0..GALILEO_INAV_PAGE_SYMBOLS {
                    assert_eq!(
                        get_bit(&symbols, offset + symbol),
                        get_bit(&reference, symbol),
                        "word type {} @{} symbol #{}",
                        word_type,
                        offset,
                        symbol
                    );
                }

                let mut reader = BitReadStream::new(BitReadBuffer::new(&symbols, BigEndian));
                reader.set_pos(offset).unwrap();

                let decoded: GalileoInavPage = reader.read().unwrap_or_else(|e| {
                    panic!("word type {} @{}: {}", word_type, offset, e);
                });

                assert_eq!(decoded, page);
                assert_eq!(reader.pos(), offset + GALILEO_INAV_PAGE_SYMBOLS);

                // inverted symbols
                let inverted: Vec<u8> = symbols.iter().map(|b| !b).collect();
                let mut reader = BitReadStream::new(BitReadBuffer::new(&inverted, BigEndian));
                reader.set_pos(offset).unwrap();
                assert_eq!(reader.read::<GalileoInavPage>().unwrap(), page);
            }
        }
    }

    #[test]
    fn consecutive_pages_stream() {
        let pages: Vec<_> = ALL_TYPES
            .iter()
            .map(|t| GalileoInavPage::model(*t))
            .collect();

        let mut symbols = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut symbols, BigEndian);
            for page in pages.iter() {
                writer.write(page).unwrap();
            }
        }

        assert_eq!(
            symbols.len(),
            (pages.len() * GALILEO_INAV_PAGE_SYMBOLS + 7) / 8
        );

        // a sync pattern every 250 symbols
        for part in 0..2 * pages.len() {
            assert_eq!(
                read_bits(&symbols, part * GALILEO_INAV_PART_SYMBOLS, 10),
                Some(GALILEO_INAV_SYNC_PATTERN)
            );
        }

        let mut reader = BitReadStream::new(BitReadBuffer::new(&symbols, BigEndian));

        for page in pages.iter() {
            assert_eq!(reader.read::<GalileoInavPage>().unwrap(), *page);
        }
    }

    #[test]
    fn symbol_errors_are_corrected() {
        let page = GalileoInavPage::model(4);
        let encoded = encode(&page);
        let mut seed = 0x1357_9BDF;

        for _ in 0..32 {
            let mut corrupted = encoded;

            // 6 symbol errors per page part (sync patterns spared)
            for part in 0..2 {
                for _ in 0..6 {
                    let pos = part * 250 + 10 + (fec::test::lfsr(&mut seed) as usize) % 240;
                    corrupted[pos / 8] ^= 0x80 >> (pos % 8);
                }
            }

            assert_eq!(GalileoInavPage::decode(&corrupted).unwrap(), page);

            // re-encoding removes the errors
            assert_eq!(encode(&page), encoded);
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

        let encoded = encode(&page);
        let decoded = GalileoInavPage::decode(&encoded).unwrap();
        assert_eq!(decoded, page);
        assert_eq!(encode(&decoded), encoded);
    }

    #[test]
    fn field_masks() {
        let page = GalileoInavPage::default()
            .with_osnma(u64::MAX)
            .with_sar(u32::MAX)
            .with_spare(u8::MAX)
            .with_ssp(u8::MAX);

        assert_eq!(page.osnma, 0xff_ffff_ffff);
        assert_eq!(page.sar, 0x3f_ffff);
        assert_eq!(page.spare, 0x3);

        assert_eq!(GalileoInavPage::decode(&encode(&page)).unwrap(), page);
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
    fn invalid_sync_pattern() {
        let encoded = encode(&GalileoInavPage::model(1));

        // even, then odd sync pattern
        for pos in [0, 4, 9, 250, 259] {
            let mut corrupted = encoded;
            corrupted[pos / 8] ^= 0x80 >> (pos % 8);
            assert!(
                GalileoInavPage::decode(&corrupted).is_err(),
                "sync symbol #{}",
                pos
            );
        }

        // swapped parts are rejected
        let mut swapped = [0u8; GALILEO_INAV_PAGE_BYTES];
        {
            let mut writer = BitWriteStream::from_slice(&mut swapped, BigEndian);
            for start in [250, 0] {
                for pos in start..start + 250 {
                    writer.write_bool(get_bit(&encoded, pos)).unwrap();
                }
            }
        }
        assert!(GalileoInavPage::decode(&swapped).is_err());
    }

    #[test]
    fn buffer_too_small() {
        let page = GalileoInavPage::model(1);

        // does not panic
        let mut buffer = [0u8; GALILEO_INAV_PAGE_BYTES - 1];
        assert!(page.encode(&mut buffer).is_err());

        let encoded = encode(&page);
        assert!(GalileoInavPage::decode(&encoded[..GALILEO_INAV_PAGE_BYTES - 1]).is_err());
        assert!(GalileoInavPage::decode(&[]).is_err());
        assert!(GalileoInavPage::decode(&[0u8; 63]).is_err());
    }

    #[test]
    fn crc24q_reference() {
        // CRC-24Q of "123456789" (ASCII)
        assert_eq!(crc24q(b"123456789", 0, 72), 0xCDE703);
        assert_eq!(crc24q(&[0u8; 8], 0, 64), 0);

        // leading zeros do not modify the CRC
        let data = [0x00, 0x0A, 0xBC, 0xDE, 0xF0];
        assert_eq!(crc24q(&data, 0, 40), crc24q(&data, 12, 28));
    }

    #[test]
    fn real_ublox_pages_bit_exact() {
        for (word_type, words) in UBLOX_E08.iter() {
            let (even, odd) = ublox_to_parts(words);

            let page = GalileoInavPage::decode_parts(&even, &odd).unwrap_or_else(|e| {
                panic!("word type {}: {}", word_type, e);
            });

            assert_eq!(page.page_type, GalileoInavPageType::Nominal);
            assert_eq!(page.word.word_type(), *word_type);

            // bit exact re-encoding (CRC included)
            let (reencoded_even, reencoded_odd) = page.encode_parts().unwrap();
            assert_eq!(
                parts_to_ublox(&reencoded_even, &reencoded_odd),
                *words,
                "type {}",
                word_type
            );

            // and through the Message trait: the transmitted symbols
            // are those of the real page parts
            let encoded = encode(&page);
            let mut reader = BitReadStream::new(BitReadBuffer::new(&encoded, BigEndian));
            assert_eq!(
                fec::decode_part(&super::read_part_symbols(&mut reader).unwrap()).0,
                even
            );
            assert_eq!(
                fec::decode_part(&super::read_part_symbols(&mut reader).unwrap()).0,
                odd
            );

            let decoded = GalileoInavPage::decode(&encoded).unwrap();
            assert_eq!(decoded, page);
            assert_eq!(encode(&decoded), encoded);
        }
    }
}
