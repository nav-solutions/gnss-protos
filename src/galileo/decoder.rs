use crate::{
    bits::{get_bit, read_bits},
    galileo::{
        fec::{self, GALILEO_INAV_INVERTED_SYNC_PATTERN, GALILEO_INAV_PART_BYTES},
        page::read_part_symbols,
        GalileoInavPage, GALILEO_INAV_PART_SYMBOLS, GALILEO_INAV_SYNC_PATTERN,
        GALILEO_INAV_SYNC_SYMBOLS,
    },
    BigEndian,
};

use bitbuffer::{BitReadBuffer, BitReadStream};

#[cfg(feature = "log")]
use log::trace;

/// Size of the internal shift register (in bytes): 256 symbols,
/// the 250 most recent ones form the page part candidate.
const REGISTER_BYTES: usize = 32;

/// Position of the first symbol of the page part candidate in the shift register.
const WINDOW_START: usize = REGISTER_BYTES * 8 - GALILEO_INAV_PART_SYMBOLS;

/// Default maximal number of symbol errors corrected in a page part (out of 240).
/// On random symbols, the Viterbi decoder never ends with less than about 25 errors.
pub const GALILEO_INAV_DEFAULT_MAX_SYMBOL_ERRORS: usize = 20;

/// Maximal number of even page part candidates waiting for their odd page part.
const MAX_PENDING_EVEN_PARTS: usize = 4;

/// [GalileoInavDecoder] synchronizes to a Galileo I/NAV symbol stream (E1-B or E5b-I,
/// 250 symbols per second) and decodes [GalileoInavPage]s, wherever they are located
/// (any symbol alignment).
///
/// It works as a 250-symbol shift register, fed one symbol at a time
/// ([Self::push_symbol], typical of a tracking loop delivering one symbol every 4 ms)
/// or from a byte slice at any symbol offset ([Self::decode]).
/// Each time the register starts with a synchronization pattern (normal or inverted,
/// the carrier phase ambiguity is resolved), the page part is de-interleaved and
/// Viterbi decoded. An even page part immediately followed (250 symbols later)
/// by an odd page part forms a page, which is returned once its CRC is verified.
///
/// The 10-symbol synchronization pattern is a weak pattern, that random symbols
/// will regularly match: candidates for which the Viterbi decoder had to correct more
/// than [Self::with_max_symbol_errors] symbols are rejected. A false candidate
/// that would pass this test does not prevent the decoding of the real page parts:
/// every position is evaluated, and the CRC eventually rejects false pages.
///
/// The decoder has a small constant memory footprint and does not allocate.
#[derive(Debug, Copy, Clone)]
pub struct GalileoInavDecoder {
    /// Shift register: most recent symbol is the LSB of the last byte.
    symbols: [u8; REGISTER_BYTES],

    /// Number of symbols received since last reset (saturates at 250).
    fill: usize,

    /// Even page part candidates, with the number of symbols received since.
    evens: [Option<(usize, [u8; GALILEO_INAV_PART_BYTES])>; MAX_PENDING_EVEN_PARTS],

    /// Maximal number of corrected symbol errors in a page part.
    max_symbol_errors: usize,
}

impl Default for GalileoInavDecoder {
    /// Creates a new [GalileoInavDecoder], tolerating up to
    /// [GALILEO_INAV_DEFAULT_MAX_SYMBOL_ERRORS] symbol errors per page part.
    fn default() -> Self {
        Self {
            symbols: [0; REGISTER_BYTES],
            fill: 0,
            evens: [None; MAX_PENDING_EVEN_PARTS],
            max_symbol_errors: GALILEO_INAV_DEFAULT_MAX_SYMBOL_ERRORS,
        }
    }
}

impl GalileoInavDecoder {
    /// Copies and returns [GalileoInavDecoder] with updated maximal number of symbol
    /// errors (out of 240) that may be corrected in a page part. Larger values
    /// improve the sensitivity, but increase the number of false synchronizations
    /// that the CRC will have to reject.
    pub fn with_max_symbol_errors(mut self, max_symbol_errors: usize) -> Self {
        self.max_symbol_errors = max_symbol_errors;
        self
    }

    /// Drops the current synchronization state.
    pub fn reset(&mut self) {
        self.symbols = [0; REGISTER_BYTES];
        self.fill = 0;
        self.evens = [None; MAX_PENDING_EVEN_PARTS];
    }

    /// Stores a new even page part candidate (replacing the oldest one if need be).
    fn store_even(&mut self, part: [u8; GALILEO_INAV_PART_BYTES]) {
        let slot = self
            .evens
            .iter()
            .position(|even| even.is_none())
            .unwrap_or_else(|| {
                // all slots used: replace the oldest
                (0..MAX_PENDING_EVEN_PARTS)
                    .max_by_key(|i| self.evens[*i].map(|(age, _)| age).unwrap_or_default())
                    .unwrap_or_default()
            });

        self.evens[slot] = Some((0, part));
    }

    /// Pushes a new (hard) symbol into the decoder.
    /// Returns a [GalileoInavPage] when this symbol completed a valid page.
    pub fn push_symbol(&mut self, symbol: bool) -> Option<GalileoInavPage> {
        let mut carry = symbol as u8;

        for byte in self.symbols.iter_mut().rev() {
            let msb = *byte >> 7;
            *byte = (*byte << 1) | carry;
            carry = msb;
        }

        for even in self.evens.iter_mut() {
            if let Some((age, _)) = even {
                *age += 1;

                if *age > GALILEO_INAV_PART_SYMBOLS {
                    // odd page part did not follow
                    *even = None;
                }
            }
        }

        if self.fill < GALILEO_INAV_PART_SYMBOLS {
            self.fill += 1;

            if self.fill < GALILEO_INAV_PART_SYMBOLS {
                return None;
            }
        }

        let sync = read_bits(&self.symbols, WINDOW_START, GALILEO_INAV_SYNC_SYMBOLS)?;

        if sync != GALILEO_INAV_SYNC_PATTERN && sync != GALILEO_INAV_INVERTED_SYNC_PATTERN {
            return None;
        }

        let mut stream = BitReadStream::new(BitReadBuffer::new(&self.symbols, BigEndian));
        stream.set_pos(WINDOW_START).ok()?;

        let encoded = read_part_symbols(&mut stream).ok()?;
        let (part, errors) = fec::decode_part(&encoded);

        if errors > self.max_symbol_errors {
            #[cfg(feature = "log")]
            trace!(
                "(Galileo I/NAV) sync candidate rejected: {} symbol errors",
                errors
            );
            return None;
        }

        let is_odd = get_bit(&part, 0);

        if !is_odd {
            self.store_even(part);
            return None;
        }

        // the even part must have started exactly 250 symbols before
        let slot = self
            .evens
            .iter()
            .position(|even| matches!(even, Some((age, _)) if *age == GALILEO_INAV_PART_SYMBOLS))?;

        let (_, even) = self.evens[slot].take()?;

        match GalileoInavPage::decode_parts(&even, &part) {
            Ok(page) => Some(page),
            #[cfg(feature = "log")]
            Err(e) => {
                trace!("(Galileo I/NAV) page rejected: {}", e);
                None
            },
            #[cfg(not(feature = "log"))]
            Err(_) => None,
        }
    }

    /// Feeds symbols from `slice`, starting at absolute symbol position `symbol_offset`,
    /// until a page is decoded or the slice is exhausted.
    ///
    /// Returns the number of symbols consumed, and the decoded page if any.
    /// The decoder state is preserved between calls: a page may span
    /// several slices. To decode a complete buffer:
    ///
    /// ```
    /// use gnss_protos::galileo::{GalileoInavDecoder, GALILEO_INAV_PAGE_SYMBOLS};
    ///
    /// let symbols = [0u8; 256]; // any input
    /// let mut decoder = GalileoInavDecoder::default();
    /// let mut position = 0; // in symbols!
    ///
    /// loop {
    ///     let (consumed, page) = decoder.decode(&symbols, position);
    ///     position += consumed;
    ///
    ///     if let Some(page) = page {
    ///         // a page ended at symbol `position`
    ///         // (it started at `position - GALILEO_INAV_PAGE_SYMBOLS`)
    ///     } else {
    ///         break; // slice exhausted
    ///     }
    /// }
    /// ```
    pub fn decode(
        &mut self,
        slice: &[u8],
        symbol_offset: usize,
    ) -> (usize, Option<GalileoInavPage>) {
        let total = slice.len() * 8;
        let mut pos = symbol_offset;

        while pos < total {
            let symbol = get_bit(slice, pos);
            pos += 1;

            if let Some(page) = self.push_symbol(symbol) {
                return (pos - symbol_offset, Some(page));
            }
        }

        (pos.saturating_sub(symbol_offset), None)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        bits::get_bit,
        galileo::{
            fec::test::lfsr,
            test_vectors::{ublox_page, UBLOX_E08},
            GalileoInavDecoder, GalileoInavPage, GalileoInavWord, GALILEO_INAV_PAGE_SYMBOLS,
        },
        BigEndian, Message,
    };

    use bitbuffer::BitWriteStream;

    /// Builds a realistic sequence of pages, with evolving contents.
    fn stream_model(num_pages: usize) -> Vec<GalileoInavPage> {
        let types = [2, 4, 6, 7, 8, 17, 19, 16, 0, 1, 3, 5, 9, 10, 18, 20, 0, 63];

        (0..num_pages)
            .map(|i| {
                let mut page = GalileoInavPage::model(types[i % types.len()]);
                page.osnma = i as u64;
                page.ssp = i as u8;

                if let GalileoInavWord::Ephemeris4(word) = &mut page.word {
                    word.svid = (i % 36) as u8 + 1;
                }

                page
            })
            .collect()
    }

    /// Streams the pages contiguously, after `offset` symbols of noise,
    /// followed by a few symbols of noise.
    fn encode_stream(pages: &[GalileoInavPage], offset: usize) -> Vec<u8> {
        let mut seed = 0xACE1_2345;
        let mut symbols = Vec::new();

        {
            let mut writer = BitWriteStream::new(&mut symbols, BigEndian);

            for _ in 0..offset {
                writer.write_bool(lfsr(&mut seed) & 0x1 > 0).unwrap();
            }

            for page in pages.iter() {
                writer.write(page).unwrap();
            }

            for _ in 0..13 {
                writer.write_bool(lfsr(&mut seed) & 0x1 > 0).unwrap();
            }
        }

        symbols
    }

    fn decode_all(
        decoder: &mut GalileoInavDecoder,
        buffer: &[u8],
    ) -> Vec<(usize, GalileoInavPage)> {
        let mut ret = Vec::new();
        let mut position = 0;

        loop {
            let (consumed, page) = decoder.decode(buffer, position);
            position += consumed;

            match page {
                Some(page) => ret.push((position - GALILEO_INAV_PAGE_SYMBOLS, page)),
                None => return ret,
            }
        }
    }

    #[test]
    fn synchronization_any_symbol_offset() {
        let pages = stream_model(8);

        for offset in 0..64 {
            let buffer = encode_stream(&pages, offset);
            let mut decoder = GalileoInavDecoder::default();
            let decoded = decode_all(&mut decoder, &buffer);

            assert_eq!(decoded.len(), pages.len(), "offset={}", offset);

            for (i, (position, page)) in decoded.iter().enumerate() {
                assert_eq!(
                    *position,
                    offset + i * GALILEO_INAV_PAGE_SYMBOLS,
                    "offset={}",
                    offset
                );
                assert_eq!(*page, pages[i], "offset={} page #{}", offset, i);
            }
        }
    }

    #[test]
    fn inverted_stream() {
        let pages = stream_model(6);

        for offset in [0, 3, 11] {
            let buffer: Vec<u8> = encode_stream(&pages, offset).iter().map(|b| !b).collect();

            let mut decoder = GalileoInavDecoder::default();
            let decoded: Vec<_> = decode_all(&mut decoder, &buffer)
                .into_iter()
                .map(|(_, p)| p)
                .collect();

            assert_eq!(decoded, pages);
        }
    }

    #[test]
    fn symbol_by_symbol_and_split_slices() {
        let pages = stream_model(5);
        let buffer = encode_stream(&pages, 7);

        // symbol by symbol
        let mut decoder = GalileoInavDecoder::default();
        let mut decoded = Vec::new();

        for pos in 0..buffer.len() * 8 {
            if let Some(page) = decoder.push_symbol(get_bit(&buffer, pos)) {
                decoded.push(page);
            }
        }

        assert_eq!(decoded, pages);

        // small chunks: pages span many slices
        let mut decoder = GalileoInavDecoder::default();
        let mut decoded = Vec::new();

        for chunk in buffer.chunks(7) {
            let mut position = 0;
            loop {
                let (consumed, page) = decoder.decode(chunk, position);
                position += consumed;
                match page {
                    Some(page) => decoded.push(page),
                    None => break,
                }
            }
        }

        assert_eq!(decoded, pages);
    }

    #[test]
    fn symbol_errors_are_corrected() {
        let pages = stream_model(6);
        let mut buffer = encode_stream(&pages, 21);
        let mut seed = 0x7777_0001;

        // 8 random symbol errors per page part (sync patterns spared)
        for part in 0..2 * pages.len() {
            let start = 21 + part * 250 + 10;
            for _ in 0..8 {
                let pos = start + (lfsr(&mut seed) as usize) % 240;
                buffer[pos / 8] ^= 0x80 >> (pos % 8);
            }
        }

        let mut decoder = GalileoInavDecoder::default();
        let decoded: Vec<_> = decode_all(&mut decoder, &buffer)
            .into_iter()
            .map(|(_, p)| p)
            .collect();

        assert_eq!(decoded, pages);
    }

    #[test]
    fn corrupted_page_is_dropped() {
        let pages = stream_model(5);
        let mut buffer = encode_stream(&pages, 9);

        // corrupt the sync pattern of the odd part of page #2
        let pos = 9 + 2 * GALILEO_INAV_PAGE_SYMBOLS + 250 + 1;
        buffer[pos / 8] ^= 0x80 >> (pos % 8);

        let mut decoder = GalileoInavDecoder::default();
        let decoded: Vec<_> = decode_all(&mut decoder, &buffer)
            .into_iter()
            .map(|(_, p)| p)
            .collect();

        assert_eq!(decoded.len(), 4);
        assert_eq!(decoded[0], pages[0]);
        assert_eq!(decoded[1], pages[1]);
        assert_eq!(decoded[2], pages[3]);
        assert_eq!(decoded[3], pages[4]);
    }

    #[test]
    fn noise_only() {
        let mut seed = 0xDEAD_0042;
        let buffer: Vec<u8> = (0..4096).map(|_| lfsr(&mut seed) as u8).collect();

        let mut decoder = GalileoInavDecoder::default();
        assert!(decode_all(&mut decoder, &buffer).is_empty());

        // even when accepting many symbol errors, the CRC rejects false pages
        let mut decoder = GalileoInavDecoder::default().with_max_symbol_errors(240);
        assert!(decode_all(&mut decoder, &buffer).is_empty());
    }

    #[test]
    fn false_synchronization_does_not_block() {
        // accepting every synchronization candidate (any number of symbol errors):
        // many false page parts are accepted in the noise, and within the pages,
        // but they must not prevent the decoding of the real pages.
        let pages = stream_model(12);

        for offset in [0, 5, 333, 1000] {
            let buffer = encode_stream(&pages, offset);
            let mut decoder = GalileoInavDecoder::default().with_max_symbol_errors(240);

            let decoded: Vec<_> = decode_all(&mut decoder, &buffer)
                .into_iter()
                .map(|(_, p)| p)
                .collect();

            assert_eq!(decoded, pages, "offset={}", offset);
        }
    }

    #[test]
    fn real_ublox_stream() {
        let pages: Vec<GalileoInavPage> = UBLOX_E08
            .iter()
            .map(|(_, words)| ublox_page(words))
            .collect();

        let buffer = encode_stream(&pages, 13);

        let mut decoder = GalileoInavDecoder::default();
        let decoded = decode_all(&mut decoder, &buffer);

        assert_eq!(decoded.len(), pages.len());
        assert_eq!(decoded[0].0, 13);

        for (i, (_, page)) in decoded.iter().enumerate() {
            assert_eq!(*page, pages[i]);

            // bit exact
            let mut symbols = [0u8; 63];
            page.encode(&mut symbols).unwrap();

            let mut reference = [0u8; 63];
            pages[i].encode(&mut reference).unwrap();
            assert_eq!(symbols, reference);
        }
    }
}
