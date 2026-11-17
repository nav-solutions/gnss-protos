//! Galileo I/NAV spreading modulation (Galileo OS SIS ICD v2.1, §2.3 and §3.4).

use crate::{
    galileo::GALILEO_INAV_SYMBOL_RATE,
    modulation::{bpsk, packed_chip, set_packed_chip, Modulation, ModulationError},
};

/// E1-B primary code length (in chips): one code period lasts 4 ms.
pub const GALILEO_E1B_CODE_LEN: usize = 4092;

/// E1-B chipping rate (in chips per second).
pub const GALILEO_E1B_CHIP_RATE: usize = 1_023_000;

/// Number of hexadecimal characters of an E1-B primary code, as published in the ICD annex.
pub const GALILEO_E1B_CODE_HEX_LEN: usize = GALILEO_E1B_CODE_LEN / 4;

/// E5b-I primary code length (in chips): one code period lasts 1 ms.
pub const GALILEO_E5BI_CODE_LEN: usize = 10230;

/// E5b-I chipping rate (in chips per second).
pub const GALILEO_E5BI_CHIP_RATE: usize = 10_230_000;

/// E5b-I secondary code (CS4_1, `E` in hexadecimal), one chip per primary code period.
pub const GALILEO_E5BI_SECONDARY_CODE: [bool; 4] = [true, true, true, false];

/// Highest Galileo PRN number.
const MAX_PRN: u8 = 50;

/// Number of bytes needed to store the longest (packed) primary code.
const CODE_BYTES: usize = (GALILEO_E5BI_CODE_LEN + 7) / 8;

/// E5 base register length.
const E5_REGISTER_LEN: u32 = 14;

/// E5b-I base register 1 feedback polynomial (octal, ICD table 16).
const E5BI_POLY1: u32 = 0o64021;

/// E5b-I base register 2 feedback polynomial (octal, ICD table 16).
const E5BI_POLY2: u32 = 0o51445;

/// E5b-I base register 2 start values of PRN 1 to 50 (octal, ICD table 17).
/// Base register 1 always starts with all stages set.
const E5BI_REGISTER2_START: [u16; 50] = [
    0o07220, 0o26047, 0o00252, 0o17166, 0o14161, 0o02540, 0o01537, 0o26023, 0o01725, 0o20637,
    0o02364, 0o27731, 0o30640, 0o34174, 0o06464, 0o07676, 0o32231, 0o10353, 0o00755, 0o26077,
    0o11644, 0o11537, 0o35115, 0o20452, 0o34645, 0o25664, 0o21403, 0o32253, 0o02337, 0o30777,
    0o27122, 0o22377, 0o36175, 0o33075, 0o33151, 0o13134, 0o07433, 0o10216, 0o35466, 0o02533,
    0o05351, 0o30121, 0o14010, 0o32576, 0o30326, 0o37433, 0o26022, 0o35770, 0o06670, 0o12017,
];

/// Galileo signals that carry the I/NAV message.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GalileoInavSignal {
    /// E1-B (data component of E1 OS), 1575.42 MHz.
    E1B,

    /// E5b-I (data component of E5b), 1207.14 MHz.
    E5bI,
}

/// [Modulator] applies the spreading modulation of one Galileo satellite, on one
/// of the signals that carry the I/NAV message ([GalileoInavSignal]), to a symbol
/// stream that was previously encoded (for example with [crate::galileo::GalileoInavPage::encode_at]).
///
/// The input are the *channel symbols* (250 sym/s, sync patterns, FEC and interleaving
/// included), not the navigation bits: one page (2 s) is 500 symbols.
///
/// - **E5b-I**: each symbol (4 ms) is spread over 4 periods of the primary code
///   (10230 chips at 10.23 MHz), each period being multiplied by one chip of the
///   secondary code `1110`, that is 40920 chips per symbol.
///   The primary codes are generated (ICD §3.4.2), so [Modulator::e5b_i] only needs a PRN.
/// - **E1-B**: each symbol (4 ms) is spread over one period of the primary code
///   (4092 chips at 1.023 MHz). E1-B primary codes are *memory codes*: they can't be
///   generated and are published in the ICD annex, as 1023 hexadecimal characters,
///   that you provide to [Modulator::e1_b].
///   The E1 OS signal uses a CBOC(6,1,1/11) sub-carrier shared with the E1-C pilot.
///   That multi-level sub-carrier is not implemented: [Modulator::with_boc11] applies
///   the BOC(1,1) approximation (2 samples per chip), which is what most
///   receivers use as local replica. Without it, the output is pure BPSK(1).
///
/// The output is a baseband signal (1 sample per chip, or 2 with BOC(1,1)),
/// mapped to `+1` (logic 0) or `-1` (logic 1). The [Modulator] is stateless and does not
/// interpret the symbols.
///
/// ```
/// use gnss_protos::{
///     galileo::{Modulator, GALILEO_E5BI_CHIP_RATE},
///     GalileoInavPage, Modulation, GALILEO_INAV_PAGE_BYTES, GALILEO_INAV_PAGE_SYMBOLS,
/// };
///
/// // 1. encode a page (500 symbols, 2 s)
/// let page = GalileoInavPage::default();
///
/// let mut symbols = [0u8; GALILEO_INAV_PAGE_BYTES];
/// page.encode_at(&mut symbols, 0).unwrap();
///
/// // 2. spread it on E5b-I, like Galileo E11 would
/// let modulator = Modulator::e5b_i(11).unwrap();
///
/// let mut chips = vec![0i8; GALILEO_INAV_PAGE_SYMBOLS * modulator.samples_per_symbol()];
/// let size = modulator.modulate(&symbols, 0, GALILEO_INAV_PAGE_SYMBOLS, &mut chips).unwrap();
///
/// // 2 seconds of signal at 10.23 MHz
/// assert_eq!(size, 2 * GALILEO_E5BI_CHIP_RATE);
/// ```
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Modulator {
    /// PRN number
    prn: u8,

    /// Modulated signal
    signal: GalileoInavSignal,

    /// BOC(1,1) sub-carrier (E1-B only)
    boc11: bool,

    /// Primary code, packed MSB first (logic levels)
    code: [u8; CODE_BYTES],
}

impl Modulator {
    /// Creates an E5b-I [Modulator] for Galileo satellite `prn` (1 to 50).
    pub fn e5b_i(prn: u8) -> Result<Self, ModulationError> {
        if prn == 0 || prn > MAX_PRN {
            return Err(ModulationError::InvalidPrn(prn));
        }

        Ok(Self {
            prn,
            signal: GalileoInavSignal::E5bI,
            boc11: false,
            code: e5bi_code(E5BI_REGISTER2_START[prn as usize - 1] as u32),
        })
    }

    /// Creates an E1-B [Modulator] for Galileo satellite `prn` (1 to 50), from its
    /// primary code, as published in the ICD annex: [GALILEO_E1B_CODE_HEX_LEN]
    /// hexadecimal characters, first chip being the MSB of the first character.
    /// White spaces and line breaks are ignored, so you can paste the ICD table as is.
    pub fn e1_b(prn: u8, hex_code: &str) -> Result<Self, ModulationError> {
        if prn == 0 || prn > MAX_PRN {
            return Err(ModulationError::InvalidPrn(prn));
        }

        let mut code = [0u8; CODE_BYTES];
        let mut nibbles = 0;

        for c in hex_code.chars().filter(|c| !c.is_whitespace()) {
            let nibble = c.to_digit(16).ok_or(ModulationError::InvalidCode)?;

            if nibbles == GALILEO_E1B_CODE_HEX_LEN {
                return Err(ModulationError::InvalidCode);
            }

            for bit in 0..4 {
                set_packed_chip(
                    &mut code,
                    4 * nibbles + bit,
                    (nibble >> (3 - bit)) & 0x1 > 0,
                );
            }

            nibbles += 1;
        }

        if nibbles != GALILEO_E1B_CODE_HEX_LEN {
            return Err(ModulationError::InvalidCode);
        }

        Ok(Self {
            prn,
            signal: GalileoInavSignal::E1B,
            boc11: false,
            code,
        })
    }

    /// Applies the BOC(1,1) sub-carrier (E1-B only, without effect on E5b-I):
    /// each chip is followed by its opposite, doubling the sample rate.
    pub fn with_boc11(mut self) -> Self {
        self.boc11 = self.signal == GalileoInavSignal::E1B;
        self
    }

    /// Returns the PRN number this [Modulator] was built for.
    pub const fn prn(&self) -> u8 {
        self.prn
    }

    /// Returns the [GalileoInavSignal] this [Modulator] generates.
    pub const fn signal(&self) -> GalileoInavSignal {
        self.signal
    }

    /// Returns the primary code length, in chips.
    pub const fn code_len(&self) -> usize {
        match self.signal {
            GalileoInavSignal::E1B => GALILEO_E1B_CODE_LEN,
            GalileoInavSignal::E5bI => GALILEO_E5BI_CODE_LEN,
        }
    }

    /// Returns chip `index` (logic level, modulo the code length) of the primary code.
    pub fn chip(&self, index: usize) -> bool {
        packed_chip(&self.code, index % self.code_len())
    }

    /// Number of primary code periods per symbol.
    const fn codes_per_symbol(&self) -> usize {
        match self.signal {
            GalileoInavSignal::E1B => 1,
            GalileoInavSignal::E5bI => GALILEO_E5BI_SECONDARY_CODE.len(),
        }
    }

    /// Number of output samples per chip.
    const fn samples_per_chip(&self) -> usize {
        if self.boc11 {
            2
        } else {
            1
        }
    }
}

impl Modulation for Modulator {
    fn symbol_rate_hz(&self) -> f64 {
        GALILEO_INAV_SYMBOL_RATE as f64
    }

    fn sample_rate_hz(&self) -> f64 {
        let chip_rate = match self.signal {
            GalileoInavSignal::E1B => GALILEO_E1B_CHIP_RATE,
            GalileoInavSignal::E5bI => GALILEO_E5BI_CHIP_RATE,
        };

        (chip_rate * self.samples_per_chip()) as f64
    }

    fn samples_per_symbol(&self) -> usize {
        self.code_len() * self.codes_per_symbol() * self.samples_per_chip()
    }

    fn modulate_symbol(&self, symbol: bool, out: &mut [i8]) -> Result<usize, ModulationError> {
        let size = self.samples_per_symbol();

        if out.len() < size {
            return Err(ModulationError::WouldNotFit);
        }

        let period_samples = self.code_len() * self.samples_per_chip();

        for (k, period) in out[..size].chunks_exact_mut(period_samples).enumerate() {
            let secondary = match self.signal {
                GalileoInavSignal::E1B => false,
                GalileoInavSignal::E5bI => GALILEO_E5BI_SECONDARY_CODE[k],
            };

            let symbol = symbol ^ secondary;

            if self.boc11 {
                for (i, samples) in period.chunks_exact_mut(2).enumerate() {
                    let value = bpsk(symbol ^ packed_chip(&self.code, i));
                    samples[0] = value;
                    samples[1] = -value;
                }
            } else {
                for (i, sample) in period.iter_mut().enumerate() {
                    *sample = bpsk(symbol ^ packed_chip(&self.code, i));
                }
            }
        }

        Ok(size)
    }
}

/// Reverses the `E5_REGISTER_LEN` LSB of `value`.
const fn reverse(value: u32) -> u32 {
    let mut reversed = 0;
    let mut i = 0;

    while i < E5_REGISTER_LEN {
        reversed = (reversed << 1) | ((value >> i) & 0x1);
        i += 1;
    }

    reversed
}

/// One step of a 14-stage Fibonacci LFSR, stored with stage 14 (the output) as LSB.
/// Returns the output chip and the next register state.
#[inline]
fn lfsr_step(register: u32, taps: u32) -> (bool, u32) {
    let feedback = (register & taps).count_ones() & 0x1;
    let next = (feedback << (E5_REGISTER_LEN - 1)) | (register >> 1);
    (register & 0x1 > 0, next)
}

/// Generates an E5b-I primary code (ICD §3.4.2, figure 13): the sum (XOR) of two
/// 14-stage base registers, truncated to 10230 chips. Register 1 starts with all
/// stages set, register 2 with the PRN start value.
fn e5bi_code(register2_start: u32) -> [u8; CODE_BYTES] {
    // polynomials and start values are given with stage 1 as MSB
    let taps1 = reverse(E5BI_POLY1 >> 1);
    let taps2 = reverse(E5BI_POLY2 >> 1);

    let mut r1 = (1 << E5_REGISTER_LEN) - 1;
    let mut r2 = reverse(register2_start);

    let mut code = [0u8; CODE_BYTES];

    for i in 0..GALILEO_E5BI_CODE_LEN {
        let (c1, next1) = lfsr_step(r1, taps1);
        let (c2, next2) = lfsr_step(r2, taps2);

        set_packed_chip(&mut code, i, c1 ^ c2);

        r1 = next1;
        r2 = next2;
    }

    code
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{
        galileo::{GalileoInavPage, GALILEO_INAV_PAGE_BYTES, GALILEO_INAV_PAGE_SYMBOLS},
        modulation::test::chips_word,
    };

    /// (PRN, first 32 chips, last 32 chips) of the E5b-I primary codes, MSB first,
    /// generated with PocketSDR (T. Takasu), an independent implementation.
    const E5BI_VECTORS: [(u8, u32, u32); 50] = [
        (1, 0xC5BEA11B, 0x89929D31),
        (2, 0x4F62483A, 0x1A5BD802),
        (3, 0xFD548829, 0x19E74EE9),
        (4, 0x86277BAE, 0x54416E18),
        (5, 0x9E39D51E, 0x2822F0A4),
        (6, 0xEA7EDE7F, 0x664E2660),
        (7, 0xF283216A, 0x47E933D3),
        (8, 0x4FB0C9E1, 0xDE9D8739),
        (9, 0xF0AB6449, 0xB977A19B),
        (10, 0x79833B3A, 0xB731AB18),
        (11, 0xEC2D915F, 0x24778F6B),
        (12, 0x409B1134, 0x2DFCF911),
        (13, 0x397E16AE, 0x105278F4),
        (14, 0x1E0FCD6C, 0x8D0AC697),
        (15, 0xCB2F5A4B, 0x03D4D572),
        (16, 0xC1079AE9, 0x790DB55A),
        (17, 0x2D9BC6C5, 0x3B100C0D),
        (18, 0xBC514690, 0xE45A9B20),
        (19, 0xF848B06D, 0x60F4D8F4),
        (20, 0x4F01E8C6, 0x285E706B),
        (21, 0xB16C9B01, 0x08F2677A),
        (22, 0xB2827D68, 0x4DE5E32D),
        (23, 0x16C80914, 0x6D466425),
        (24, 0x7B570F8F, 0xE4C03A42),
        (25, 0x1969C089, 0xA180BE9F),
        (26, 0x512FA9AA, 0x2401EFB5),
        (27, 0x73F36B0B, 0x36893099),
        (28, 0x2D531725, 0x274E98D1),
        (29, 0xEC83904B, 0x221FF024),
        (30, 0x380374D2, 0x0D7E1E4D),
        (31, 0x46B4DEDB, 0xAD6B706E),
        (32, 0x6C01D916, 0x59C772BF),
        (33, 0x0E0BB666, 0xADB661AC),
        (34, 0x2708C751, 0xE76C1C83),
        (35, 0x265B5560, 0x93E92613),
        (36, 0xA68E1CB7, 0x6B059B2F),
        (37, 0xC3916EF4, 0x0905144E),
        (38, 0xBDC59558, 0xF4D4F162),
        (39, 0x1327D0DF, 0xC16F3524),
        (40, 0xEA921FC3, 0x43DF6961),
        (41, 0xD4586907, 0x2B68F7C8),
        (42, 0x3EB98A46, 0x42E5BBBD),
        (43, 0x9FDE1600, 0x82577E9C),
        (44, 0x2A04CABE, 0x92C634ED),
        (45, 0x3CA56F13, 0x0D4ABE3B),
        (46, 0x03928AF2, 0x1710654D),
        (47, 0x4FB5B9E9, 0xF6AEC4C3),
        (48, 0x101EC765, 0x688DADB2),
        (49, 0xC91D4F53, 0x25D28CDD),
        (50, 0xAFC22B34, 0x6B6CDBC2),
    ];

    #[test]
    fn e5bi_codes() {
        for (prn, first, last) in E5BI_VECTORS.iter() {
            let m = Modulator::e5b_i(*prn).unwrap();
            assert_eq!(
                chips_word(|n| m.chip(n), 0, 32),
                *first,
                "E{:02} (start)",
                prn
            );
            assert_eq!(
                chips_word(|n| m.chip(n), GALILEO_E5BI_CODE_LEN - 32, 32),
                *last,
                "E{:02} (end)",
                prn
            );
        }
    }

    #[test]
    fn invalid_prn() {
        for prn in [0, 51, 255] {
            assert_eq!(Modulator::e5b_i(prn), Err(ModulationError::InvalidPrn(prn)));
        }
    }

    #[test]
    fn e1b_hex_code() {
        // dummy code (not an ICD code): alternating nibbles, with white spaces
        let mut hex = String::new();

        for i in 0..GALILEO_E1B_CODE_HEX_LEN {
            hex.push(if i % 2 == 0 { 'A' } else { '3' });
            if i % 64 == 63 {
                hex.push('\n');
            }
        }

        let m = Modulator::e1_b(1, &hex).unwrap();

        assert_eq!(m.code_len(), GALILEO_E1B_CODE_LEN);
        assert_eq!(chips_word(|n| m.chip(n), 0, 8), 0xA3);
        assert_eq!(m.samples_per_symbol(), 4092);
        assert_eq!(m.sample_rate_hz(), 1.023e6);

        let boc = m.with_boc11();
        assert_eq!(boc.samples_per_symbol(), 2 * 4092);
        assert_eq!(boc.sample_rate_hz(), 2.046e6);

        let mut out = vec![0i8; boc.samples_per_symbol()];
        boc.modulate_symbol(false, &mut out).unwrap();
        // chip 0 is 1 (-1), followed by its opposite
        assert_eq!(&out[..4], &[-1, 1, 1, -1]);

        // invalid codes
        assert_eq!(
            Modulator::e1_b(1, &hex[..100]),
            Err(ModulationError::InvalidCode)
        );
        assert_eq!(
            Modulator::e1_b(1, &format!("{}0", hex)),
            Err(ModulationError::InvalidCode)
        );
        assert_eq!(
            Modulator::e1_b(1, &hex.replace('A', "G")),
            Err(ModulationError::InvalidCode)
        );
    }

    #[test]
    fn boc11_has_no_effect_on_e5bi() {
        let m = Modulator::e5b_i(1).unwrap();
        assert_eq!(m.with_boc11(), m);
    }

    #[test]
    fn page_modulation_and_despreading() {
        let page = GalileoInavPage::default();
        let mut symbols = [0u8; GALILEO_INAV_PAGE_BYTES];
        page.encode_at(&mut symbols, 0).unwrap();

        let m = Modulator::e5b_i(8).unwrap();
        assert_eq!(m.samples_per_symbol(), 40920);
        assert_eq!(m.symbol_rate_hz(), 250.0);

        let mut chips = vec![0i8; GALILEO_INAV_PAGE_SYMBOLS * m.samples_per_symbol()];
        m.modulate(&symbols, 0, GALILEO_INAV_PAGE_SYMBOLS, &mut chips)
            .unwrap();

        // correlate each code period against the primary code, then wipe the
        // secondary code off and accumulate over the symbol
        let mut despread = [0u8; GALILEO_INAV_PAGE_BYTES];

        for (i, symbol) in chips.chunks_exact(m.samples_per_symbol()).enumerate() {
            let mut acc = 0i32;

            for (k, period) in symbol.chunks_exact(GALILEO_E5BI_CODE_LEN).enumerate() {
                let corr: i32 = period
                    .iter()
                    .enumerate()
                    .map(|(n, c)| *c as i32 * bpsk(m.chip(n)) as i32)
                    .sum();

                acc += corr * bpsk(GALILEO_E5BI_SECONDARY_CODE[k]) as i32;
            }

            assert_eq!(acc.abs(), m.samples_per_symbol() as i32);
            crate::bits::set_bit(&mut despread, i, acc < 0);
        }

        assert_eq!(despread, symbols);
        assert_eq!(GalileoInavPage::decode_at(&despread, 0).unwrap(), page);
    }
}
