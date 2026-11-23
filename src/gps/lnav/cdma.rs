//! GPS / QZSS L1 C/A spreading modulation (IS-GPS-200 §3.2.1.3 and §3.3.2.3, IS-QZSS-PNT)

use crate::{
    modulation::{bpsk, packed_chip, set_packed_chip},
    Modulation, ModulationError,
};

#[cfg(doc)]
use crate::gps::lnav::{GpsQzssLnavFrame, GpsQzssLnavSuperframe};

const CHIP_RATE: usize = 1_023_000;
const DATA_RATE: usize = 50;
const CODES_PER_BIT: usize = 20;
const CODE_LEN: usize = 1023;

pub const GPS_L1_CA_CHIPS_PER_BIT: usize = CODES_PER_BIT * CODE_LEN;

/// Number of bytes needed to store a packed C/A code.
const CODE_BYTES: usize = (CODE_LEN + 7) / 8;

/// G2 delays (in chips) of GPS PRN 1 to 32 (IS-GPS-200 table 3-Ia).
const GPS_G2_DELAYS: [u16; 32] = [
    5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254, 255, 256, 257, 258, 469, 470, 471, 472, 473,
    474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862,
];

/// G2 delays (in chips) of QZSS PRN 193 to 202 (IS-QZSS-PNT table 3.2.2-2).
const QZSS_G2_DELAYS: [u16; 10] = [339, 208, 711, 189, 263, 537, 663, 942, 173, 900];

const QZSS_FIRST_PRN: u8 = 193;

/// [GpsQzssLnavModulator] applies the L1/CA CDMA modulation
/// to any bit sequence (regardless of its content).
///
/// The typical application is to modulate every 6s [GpsQzssLnavSubframe]s
/// which is the building block of the data stream.
/// But you could also modulate 30s [GpsQzssLnavFrame]s or 12'30s periods
/// with [GpsQzssLnavSuperframe]s.
///
/// The stream then becomes compatible with transmission over the air.
/// GPS/QZSS must be transmitted at 50 bits per second, one bit every 20ms,
/// one [GpsQzssLnavSubframe] every 6 seconds.
///
/// We propose [GpsQzssLnavTimeSync] to help you synchronize
/// your modulator.
///
/// Each bit (20ms at 50 bit/s) is spread over 20 periods of the satellite C/A code
/// (1023 chips at 1023 kHz), that is 20_460 chips per data bit.
/// The output chip is mapped to +1 or -1 BPSK logic.
///
/// [GpsQzssLnavModulator] does not interpret the data: it applies to any bit sequence, so you may
/// also use it on your own (possibly corrupted) bit stream. It is stateless, so
/// a stream may be modulated in as many pieces as you want, one data bit at least.
///
/// ```
/// use gnss_protos::{
///     gps::lnav::{
///         GpsQzssLnavModulator,
///         GpsQzssLnavSubframe,
///         GpsQzssLnavHow,
///         GPS_LNAV_SUBFRAME_BITS,
///         GPS_L1_CA_CHIPS_PER_BIT,
///     },
///     Message,
///     Modulation,
/// };
///
/// // 1. generate dumumy SBF
/// let sbf = GpsQzssLnavSubframe::default()
///     .with_hand_over_word(
///         GpsQzssLnavHow::default()
///             .with_tow_seconds(259_956)
///     );
///
/// // GPS/QZSS works one bit at a time
/// let mut bits = [0u8; 38];
///
/// sbf.encode(&mut bits).unwrap();
///
/// // 2. C/A spreading like SV25
/// let modulator = GpsQzssLnavModulator::from_satellite_id(25)
///     .unwrap();
///
/// let mut chips = vec![0i8; GPS_LNAV_SUBFRAME_BITS * GPS_L1_CA_CHIPS_PER_BIT];
///
/// let size = modulator.modulate(&bits, 0, GPS_LNAV_SUBFRAME_BITS, &mut chips)
///     .unwrap();
///
/// // 6 seconds of signal @ 1023kHz
/// assert_eq!(size, 6 * 1_023_000);
/// assert_eq!(modulator.sample_rate_hz(), 1.023E6);
/// ```
#[derive(Copy, Clone, PartialEq)]
pub struct GpsQzssLnavModulator {
    /// SV ID/PRN number
    prn: u8,

    /// C/A code
    ca_code: [u8; CODE_BYTES],
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

    for i in 0..CODE_LEN {
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

    for i in 0..CODE_LEN {
        let delayed = (i + CODE_LEN - g2_delay) % CODE_LEN;
        set_packed_chip(
            &mut code,
            i,
            packed_chip(&g1, i) ^ packed_chip(&g2, delayed),
        );
    }

    code
}

impl GpsQzssLnavModulator {
    /// Creates a [GpsQZssLnavModulator] for one satellite, identified by its PRN number:
    /// 1 to 32 for GPS, 193 to 202 for QZSS.
    pub fn from_satellite_id(prn: u8) -> Result<Self, ModulationError> {
        let delay = match prn {
            1..=32 => GPS_G2_DELAYS[prn as usize - 1],
            193..=202 => QZSS_G2_DELAYS[(prn - QZSS_FIRST_PRN) as usize],
            _ => return Err(ModulationError::InvalidPRN(prn)),
        };

        Ok(Self {
            prn,
            ca_code: ca_code(delay as usize),
        })
    }

    pub const fn prn(&self) -> u8 {
        self.prn
    }

    pub const fn is_qzss(&self) -> bool {
        self.prn >= QZSS_FIRST_PRN
    }
}

impl Modulation for GpsQzssLnavModulator {
    fn symbol_rate_hz(&self) -> f64 {
        DATA_RATE as f64
    }

    fn sample_rate_hz(&self) -> f64 {
        CHIP_RATE as f64
    }

    fn samples_per_symbol(&self) -> usize {
        GPS_L1_CA_CHIPS_PER_BIT
    }

    fn prn_number(&self) -> u8 {
        self.prn
    }

    fn period_samples(&self) -> usize {
        CODE_LEN
    }

    fn replica_sample(&self, index: usize) -> i8 {
        bpsk(packed_chip(&self.ca_code, index))
    }

    fn secondary_code(&self) -> &'static [bool] {
        &[false; CODES_PER_BIT]
    }

    /// Example:
    /// ```
    /// use gnss_protos::{
    ///     gps::lnav::{
    ///         GpsQzssLnavFrame,
    ///         GpsQzssLnavModulator,
    ///         GPS_LNAV_FRAME_BYTES,
    ///         GPS_L1_CA_CHIPS_PER_BIT,
    ///     },
    ///     Message,
    ///     Modulation,
    /// };
    ///
    /// // in this example we create a 30s GPS SV25 burst
    /// // this is the minimal size to allocate the burst
    /// let mut raw = [0; GPS_LNAV_FRAME_BYTES];
    ///
    /// // total size after scrambling
    /// let mut tx  = [0; GPS_LNAV_FRAME_BYTES * GPS_L1_CA_CHIPS_PER_BIT];
    ///
    /// // preset and deploy a modulator
    /// let mut modulator = GpsQzssLnavModulator::from_satellite_id(25)
    ///     .unwrap();
    ///
    /// // create a fancy burst full of zeros
    /// let frame = GpsQzssLnavFrame::default();
    ///
    /// // encode burst
    /// frame.encode(&mut raw).unwrap();
    ///
    /// // CDMA scrambling
    /// let modulated = modulator.modulate(&raw, raw.len(), raw.len(), &mut tx)
    ///     .unwrap_or_else(|e| {
    ///         panic!("CDMA scrambling failed with: {}", e);
    ///     });
    ///
    /// if modulated > 0 {
    ///     // we do not support partial modulation,
    ///     // so at this point you know the complete frame was modulated (1024 >37.5)
    /// }
    /// ```
    fn modulate_symbol(&self, bit: bool, out: &mut [i8]) -> Result<usize, ModulationError> {
        if out.len() < GPS_L1_CA_CHIPS_PER_BIT {
            return Err(ModulationError::WouldNotFit);
        }

        for period in out[..GPS_L1_CA_CHIPS_PER_BIT].chunks_exact_mut(CODE_LEN) {
            for (i, sample) in period.iter_mut().enumerate() {
                *sample = bpsk(bit ^ packed_chip(&self.ca_code, i));
            }
        }

        Ok(GPS_L1_CA_CHIPS_PER_BIT)
    }
}

#[cfg(test)]
pub(crate) mod test {
    /// Packs `count` (<= 32) chips of a code, starting at `start`, MSB first.
    pub(crate) fn chips_word(chip: impl Fn(usize) -> bool, start: usize, count: usize) -> u32 {
        (start..start + count).fold(0u32, |acc, i| (acc << 1) | chip(i) as u32)
    }
}
