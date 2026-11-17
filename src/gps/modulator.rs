//! GPS / QZSS L1 C/A spreading modulation (IS-GPS-200 §3.2.1.3 and §3.3.2.3, IS-QZSS-PNT).

use crate::modulation::{bpsk, packed_chip, set_packed_chip, Modulation, ModulationError};

/// L1 C/A code length (in chips): one code period lasts 1 ms.
pub const GPS_L1_CA_CODE_LEN: usize = 1023;

/// L1 C/A chipping rate (in chips per second).
pub const GPS_L1_CA_CHIP_RATE: usize = 1_023_000;

/// GPS / QZSS LNAV data rate (in bits per second).
pub const GPS_DATA_RATE: usize = 50;

/// Number of C/A code periods per data bit (one data bit lasts 20 ms).
pub const GPS_L1_CA_CODES_PER_BIT: usize = 20;

/// Number of chips per data bit.
pub const GPS_L1_CA_CHIPS_PER_BIT: usize = GPS_L1_CA_CODES_PER_BIT * GPS_L1_CA_CODE_LEN;

/// Number of bytes needed to store a packed C/A code.
const CODE_BYTES: usize = (GPS_L1_CA_CODE_LEN + 7) / 8;

/// G2 delays (in chips) of GPS PRN 1 to 32 (IS-GPS-200 table 3-Ia).
const GPS_G2_DELAYS: [u16; 32] = [
    5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254, 255, 256, 257, 258, 469, 470, 471, 472, 473,
    474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862,
];

/// G2 delays (in chips) of QZSS PRN 193 to 202 (IS-QZSS-PNT table 3.2.2-2).
const QZSS_G2_DELAYS: [u16; 10] = [339, 208, 711, 189, 263, 537, 663, 942, 173, 900];

/// First QZSS L1 C/A PRN number.
const QZSS_FIRST_PRN: u8 = 193;

/// [Modulator] applies the L1 C/A spreading modulation of one GPS or QZSS
/// satellite, to a bit stream that was previously encoded (for example with
/// [crate::gps::GpsQzssFrame::encode_at]).
///
/// Each data bit (20 ms at 50 bit/s) is spread over 20 periods of the satellite
/// C/A code (1023 chips at 1.023 MHz), that is 20460 chips per data bit:
/// the output chip is `data ⊕ code`, mapped to `+1` (logic 0) or `-1` (logic 1).
///
/// The [Modulator] does not interpret the data: it applies to any bit sequence, so you may
/// also use it on your own (possibly corrupted) bit stream. It is stateless, so
/// a stream may be modulated in as many pieces as you want, one data bit at least.
///
/// The output is a baseband signal sampled at the chipping rate (1 sample per chip):
/// carrier, filtering and sample rate conversion are left to the transmitter.
///
/// The [Modulator] also gives access to the C/A code itself ([Self::chip]),
/// which is the local replica that a receiver correlates against.
///
/// ```
/// use gnss_protos::{
///     gps::{Modulator, GPS_L1_CA_CHIPS_PER_BIT},
///     GpsQzssFrame, GpsQzssHow, Modulation, GPS_FRAME_BITS,
/// };
///
/// // 1. encode a frame (300 bits, 6 s)
/// let frame = GpsQzssFrame::default()
///     .with_hand_over_word(GpsQzssHow::default().with_tow_seconds(259_956));
///
/// let mut bits = [0u8; 38];
/// frame.encode_at(&mut bits, 0).unwrap();
///
/// // 2. spread it, like GPS PRN 25 would
/// let modulator = Modulator::new(25).unwrap();
///
/// let mut chips = vec![0i8; GPS_FRAME_BITS * GPS_L1_CA_CHIPS_PER_BIT];
/// let size = modulator.modulate(&bits, 0, GPS_FRAME_BITS, &mut chips).unwrap();
///
/// // 6 seconds of signal at 1.023 MHz
/// assert_eq!(size, 6 * 1_023_000);
/// assert_eq!(modulator.sample_rate_hz(), 1.023e6);
/// ```
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Modulator {
    /// PRN number
    prn: u8,

    /// C/A code, packed MSB first (logic levels)
    code: [u8; CODE_BYTES],
}

impl Modulator {
    /// Creates a [Modulator] for one satellite, identified by its PRN number:
    /// 1 to 32 for GPS, 193 to 202 for QZSS.
    pub fn new(prn: u8) -> Result<Self, ModulationError> {
        let delay = match prn {
            1..=32 => GPS_G2_DELAYS[prn as usize - 1],
            193..=202 => QZSS_G2_DELAYS[(prn - QZSS_FIRST_PRN) as usize],
            _ => return Err(ModulationError::InvalidPrn(prn)),
        };

        Ok(Self {
            prn,
            code: ca_code(delay as usize),
        })
    }

    /// Returns the PRN number this [Modulator] was built for.
    pub const fn prn(&self) -> u8 {
        self.prn
    }

    /// Returns true when this [Modulator] emulates a QZSS satellite.
    pub const fn is_qzss(&self) -> bool {
        self.prn >= QZSS_FIRST_PRN
    }

    /// Returns chip `index` (logic level, modulo the code length) of the C/A code.
    pub fn chip(&self, index: usize) -> bool {
        packed_chip(&self.code, index % GPS_L1_CA_CODE_LEN)
    }

    /// Returns the C/A code, packed MSB first (1023 chips, the last bit is padding).
    pub const fn code(&self) -> &[u8; CODE_BYTES] {
        &self.code
    }
}

impl Modulation for Modulator {
    fn symbol_rate_hz(&self) -> f64 {
        GPS_DATA_RATE as f64
    }

    fn sample_rate_hz(&self) -> f64 {
        GPS_L1_CA_CHIP_RATE as f64
    }

    fn samples_per_symbol(&self) -> usize {
        GPS_L1_CA_CHIPS_PER_BIT
    }

    fn modulate_symbol(&self, bit: bool, out: &mut [i8]) -> Result<usize, ModulationError> {
        if out.len() < GPS_L1_CA_CHIPS_PER_BIT {
            return Err(ModulationError::WouldNotFit);
        }

        for period in out[..GPS_L1_CA_CHIPS_PER_BIT].chunks_exact_mut(GPS_L1_CA_CODE_LEN) {
            for (i, sample) in period.iter_mut().enumerate() {
                *sample = bpsk(bit ^ packed_chip(&self.code, i));
            }
        }

        Ok(GPS_L1_CA_CHIPS_PER_BIT)
    }
}

/// Generates the C/A code of a satellite, from its G2 delay (IS-GPS-200 figure 3-11).
///
/// - G1 = 1 + x^3 + x^10
/// - G2 = 1 + x^2 + x^3 + x^6 + x^8 + x^9 + x^10
///
/// Both registers start with all stages set. The code is G1 ⊕ G2, G2 being delayed.
fn ca_code(g2_delay: usize) -> [u8; CODE_BYTES] {
    let mut g1 = [0u8; CODE_BYTES];
    let mut g2 = [0u8; CODE_BYTES];

    // stage n (1..=10) is bit (n-1)
    let mut r1 = 0x3ffu16;
    let mut r2 = 0x3ffu16;

    let stage = |r: u16, n: u16| (r >> (n - 1)) & 0x1;

    for i in 0..GPS_L1_CA_CODE_LEN {
        set_packed_chip(&mut g1, i, stage(r1, 10) > 0);
        set_packed_chip(&mut g2, i, stage(r2, 10) > 0);

        let f1 = stage(r1, 3) ^ stage(r1, 10);

        let f2 = stage(r2, 2)
            ^ stage(r2, 3)
            ^ stage(r2, 6)
            ^ stage(r2, 8)
            ^ stage(r2, 9)
            ^ stage(r2, 10);

        r1 = ((r1 << 1) | f1) & 0x3ff;
        r2 = ((r2 << 1) | f2) & 0x3ff;
    }

    let mut code = [0u8; CODE_BYTES];

    for i in 0..GPS_L1_CA_CODE_LEN {
        let delayed = (i + GPS_L1_CA_CODE_LEN - g2_delay) % GPS_L1_CA_CODE_LEN;
        set_packed_chip(
            &mut code,
            i,
            packed_chip(&g1, i) ^ packed_chip(&g2, delayed),
        );
    }

    code
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{
        gps::{GpsQzssFrame, GpsQzssHow, GPS_FRAME_BITS},
        modulation::test::chips_word,
    };

    /// (PRN, first 32 chips, last 32 chips), MSB first, generated with PocketSDR
    /// (T. Takasu), an independent implementation.
    const CA_VECTORS: [(u8, u32, u32); 42] = [
        (1, 0xC83949E5, 0x1270E510),
        (2, 0xE4383E99, 0xA5A050C8),
        (3, 0xF2388527, 0x7E480A24),
        (4, 0xF938D8F8, 0x13BC2752),
        (5, 0x96C46C57, 0xA6826A72),
        (6, 0xCB46AC40, 0x7FD91779),
        (7, 0x967FD269, 0x4ED88664),
        (8, 0xCB1B735F, 0x8BF46172),
        (9, 0xE5A923C4, 0x696212F9),
        (10, 0xD1289C36, 0xDE54C600),
        (11, 0xE8B0D470, 0xC3B24140),
        (12, 0xFA1AE242, 0x8A38A330),
        (13, 0xFD29EB4A, 0x698473D8),
        (14, 0xFEB06FCE, 0x185A1BAC),
        (15, 0xFF7CAD8C, 0x20B52F96),
        (16, 0xFF9ACCAD, 0x3CC2B58B),
        (17, 0x9B8044FC, 0xF87F4FC0),
        (18, 0xCDE4B815, 0x50A785A0),
        (19, 0xE6D6C661, 0x04CBE090),
        (20, 0xF34FF95B, 0xAEFDD208),
        (21, 0xF98366C6, 0xFBE6CB44),
        (22, 0xFCE52908, 0x516B47E2),
        (23, 0x8CF7833E, 0xF322CD00),
        (24, 0xF1A101A3, 0x0F966250),
        (25, 0xF8F41ABA, 0x2B531368),
        (26, 0xFC5E9736, 0xB931ABF4),
        (27, 0xFE0BD1F0, 0xF000F7BA),
        (28, 0xFF217293, 0xD498599D),
        (29, 0x95E656ED, 0x3F06EE10),
        (30, 0xCAD7B11D, 0xB31B5548),
        (31, 0xE54F42E5, 0x751588E4),
        (32, 0xF2833B19, 0x9612E632),
        (193, 0x75C30281, 0x7B95FBD4),
        (194, 0x1E2E859C, 0xB493E27E),
        (195, 0x0615F32C, 0x5C2F6F49),
        (196, 0x4EB5B23F, 0xC718CCCC),
        (197, 0xA7C5FD4A, 0xA74EA830),
        (198, 0x4317FBB5, 0x85927363),
        (199, 0x8A084874, 0x287129D2),
        (200, 0xE630A740, 0x18BC7A8D),
        (201, 0x5E4AC2CB, 0xC7A372F2),
        (202, 0x76BCE724, 0x163C09BD),
    ];

    #[test]
    fn first_10_chips_octal() {
        // IS-GPS-200 table 3-Ia
        let octal = [
            0o1440, 0o1620, 0o1710, 0o1744, 0o1133, 0o1455, 0o1131, 0o1454, 0o1626, 0o1504,
        ];

        for (i, expected) in octal.iter().enumerate() {
            let m = Modulator::new(i as u8 + 1).unwrap();
            assert_eq!(chips_word(|n| m.chip(n), 0, 10), *expected, "PRN{}", i + 1);
        }
    }

    #[test]
    fn ca_codes() {
        for (prn, first, last) in CA_VECTORS.iter() {
            let m = Modulator::new(*prn).unwrap();
            assert_eq!(
                chips_word(|n| m.chip(n), 0, 32),
                *first,
                "PRN{} (start)",
                prn
            );
            assert_eq!(
                chips_word(|n| m.chip(n), 991, 32),
                *last,
                "PRN{} (end)",
                prn
            );
            assert_eq!(m.is_qzss(), *prn >= 193);
        }
    }

    #[test]
    fn ca_code_is_balanced() {
        // Gold codes: 512 ones, 511 zeros
        for (prn, _, _) in CA_VECTORS.iter() {
            let m = Modulator::new(*prn).unwrap();
            let ones = (0..GPS_L1_CA_CODE_LEN).filter(|i| m.chip(*i)).count();
            assert_eq!(ones, 512, "PRN{}", prn);
        }
    }

    #[test]
    fn invalid_prn() {
        for prn in [0, 33, 120, 192, 203, 255] {
            assert_eq!(Modulator::new(prn), Err(ModulationError::InvalidPrn(prn)));
        }
    }

    #[test]
    fn frame_modulation_and_despreading() {
        let frame = GpsQzssFrame::default()
            .with_hand_over_word(GpsQzssHow::default().with_tow_seconds(259_956));

        let mut bits = [0u8; 40];
        frame.encode_at(&mut bits, 3).unwrap();

        let m = Modulator::new(12).unwrap();
        let mut chips = vec![0i8; GPS_FRAME_BITS * GPS_L1_CA_CHIPS_PER_BIT];

        let size = m.modulate(&bits, 3, GPS_FRAME_BITS, &mut chips).unwrap();
        assert_eq!(size, chips.len());

        // A receiver correlates each data bit against the local C/A replica:
        // the sign of the correlation is the data bit.
        let mut despread = [0u8; 40];

        for (i, bit) in chips.chunks_exact(GPS_L1_CA_CHIPS_PER_BIT).enumerate() {
            let corr: i32 = bit
                .iter()
                .enumerate()
                .map(|(n, c)| *c as i32 * bpsk(m.chip(n)) as i32)
                .sum();

            assert_eq!(corr.abs(), GPS_L1_CA_CHIPS_PER_BIT as i32);
            crate::bits::set_bit(&mut despread, 3 + i, corr < 0);
        }

        assert_eq!(despread, bits);

        // Another satellite does not correlate
        let other = Modulator::new(13).unwrap();
        let corr: i32 = chips[..GPS_L1_CA_CODE_LEN]
            .iter()
            .enumerate()
            .map(|(n, c)| *c as i32 * bpsk(other.chip(n)) as i32)
            .sum();

        assert!(corr.abs() <= 65, "cross correlation {}", corr);
    }

    #[test]
    fn output_too_small() {
        let m = Modulator::new(1).unwrap();
        let mut out = [0i8; 100];
        assert_eq!(
            m.modulate_symbol(true, &mut out),
            Err(ModulationError::WouldNotFit)
        );
        assert_eq!(
            m.modulate(&[0u8; 1], 0, 9, &mut out),
            Err(ModulationError::NotEnoughBits)
        );
    }
}
