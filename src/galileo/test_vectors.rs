//! Real Galileo I/NAV pages (E1-B), as reported by a u-blox receiver (UBX-RXM-SFRBX)
//! on 2025-04-25 (nav-solutions/data `UBX/16dBatt_no_interference_coldstart.ubx`).
//! Reference values were obtained with RTKLIB (convbin) from the same recording.

use crate::galileo::{fec::GALILEO_INAV_PART_BYTES, GalileoInavPage};

/// E08 pages, one per word type: (word type, UBX-RXM-SFRBX words).
pub(crate) const UBLOX_E08: [(u8, [u32; 8]); 16] = [
    (
        0,
        [
            0x00955555, 0x55555555, 0x55555555, 0x54ED8000, 0xBD366C08, 0x82338B2A, 0xAAAA6E81,
            0x8ACBC000,
        ],
    ),
    (
        1,
        [
            0x011EDD9C, 0x77109778, 0x0031DBAC, 0xAA04C000, 0x957CC289, 0x03E0BE6A, 0xAAAA5B36,
            0xCFCAC000,
        ],
    ),
    (
        2,
        [
            0x021EEF03, 0xDC1C09DF, 0xF8A0FC58, 0xC24D0000, 0xBDD25C94, 0x8587992A, 0xAAAA4686,
            0x2D410000,
        ],
    ),
    (
        3,
        [
            0x031EFFEF, 0x568A3BFE, 0x75C16749, 0x05BDC000, 0xB8DAED5A, 0xF935696A, 0xAAAA5404,
            0x128BC000,
        ],
    ),
    (
        4,
        [
            0x041EC800, 0x07000F76, 0x73FC461C, 0x87FF8000, 0x8F804986, 0x6BD3872A, 0xAAAA423B,
            0x4ECAC000,
        ],
    ),
    (
        5,
        [
            0x05406108, 0x20607C5E, 0xF00A76DE, 0x9C2A8000, 0xAAAAA922, 0x0742D4AA, 0xAAAA639E,
            0xFA410000,
        ],
    ),
    (
        6,
        [
            0x06000000, 0x01000000, 0x12783B89, 0xE24DC000, 0xA99A8888, 0x6F3A61AA, 0xAAAA58BE,
            0x160BC000,
        ],
    ),
    (
        7,
        [
            0x076EF664, 0x01200B71, 0xF3DD88CA, 0xFF074000, 0xA60551C7, 0x83D5CB6A, 0xAAAA5A42,
            0x13C10000,
        ],
    ),
    (
        8,
        [
            0x08600090, 0x0003400D, 0x015E0861, 0x499B8000, 0xB7F911D6, 0xA874822A, 0xAAAA7936,
            0xA24AC000,
        ],
    ),
    (
        9,
        [
            0x096EF68C, 0x1EFFE100, 0x183600B0, 0x10318000, 0x8BEEC143, 0xC3E9C42A, 0xAAAA6F45,
            0xF5C10000,
        ],
    ),
    (
        10,
        [
            0x0A61195F, 0xE1B3A5FD, 0x5FFFD0FF, 0x4CFDC000, 0xA78EE73B, 0x8C2907AA, 0xAAAA579C,
            0x094AC000,
        ],
    ),
    (
        16,
        [
            0x10295E7C, 0x55D6E0BC, 0x0F2ADBD2, 0xBFF88000, 0x8C501A4F, 0xF5A751EA, 0xAAAA6A49,
            0x93CAC000,
        ],
    ),
    (
        17,
        [
            0x112BC4F1, 0x1CD99145, 0xC6D26C9C, 0xCA3AC000, 0x9CBC9393, 0xFA556FAA, 0xAAAA4B62,
            0x21CBC000,
        ],
    ),
    (
        18,
        [
            0x128EE701, 0x585F0460, 0x70706391, 0x5955C000, 0x97506C83, 0xDB86AD2A, 0xAAAA6E0B,
            0x9ECBC000,
        ],
    ),
    (
        19,
        [
            0x138DFF29, 0xB61D1280, 0xA829DA7B, 0xD7348000, 0x8D0B14DB, 0x61C883EA, 0xAAAA657A,
            0x05410000,
        ],
    ),
    (
        20,
        [
            0x1466DFC8, 0x67DFF97A, 0x7510D3BE, 0x6553C000, 0xAE94F3F8, 0x27C233EA, 0xAAAA4143,
            0xE4C10000,
        ],
    ),
];

/// Splits a page as reported by u-blox receivers (UBX-RXM-SFRBX, Galileo E1-B / E5b-I)
/// into its two 120-bit page parts: 8 words of 32 bits, the even page part
/// in the 4 first words, the odd page part in the 4 last words (8 padding bits each).
pub(crate) fn ublox_to_parts(
    words: &[u32; 8],
) -> ([u8; GALILEO_INAV_PART_BYTES], [u8; GALILEO_INAV_PART_BYTES]) {
    let mut bytes = [0u8; 32];

    for (i, word) in words.iter().enumerate() {
        bytes[4 * i..4 * i + 4].copy_from_slice(&word.to_be_bytes());
    }

    let mut even = [0u8; GALILEO_INAV_PART_BYTES];
    let mut odd = [0u8; GALILEO_INAV_PART_BYTES];
    even.copy_from_slice(&bytes[..GALILEO_INAV_PART_BYTES]);
    odd.copy_from_slice(&bytes[16..16 + GALILEO_INAV_PART_BYTES]);

    (even, odd)
}

/// Mirror operation of [ublox_to_parts]. Padding bits are set to zero.
pub(crate) fn parts_to_ublox(
    even: &[u8; GALILEO_INAV_PART_BYTES],
    odd: &[u8; GALILEO_INAV_PART_BYTES],
) -> [u32; 8] {
    let mut bytes = [0u8; 32];
    bytes[..GALILEO_INAV_PART_BYTES].copy_from_slice(even);
    bytes[16..16 + GALILEO_INAV_PART_BYTES].copy_from_slice(odd);

    let mut words = [0u32; 8];

    for (i, word) in words.iter_mut().enumerate() {
        *word = u32::from_be_bytes([
            bytes[4 * i],
            bytes[4 * i + 1],
            bytes[4 * i + 2],
            bytes[4 * i + 3],
        ]);
    }

    words
}

/// Interprets a page as reported by u-blox receivers. The CRC is verified.
pub(crate) fn ublox_page(words: &[u32; 8]) -> GalileoInavPage {
    let (even, odd) = ublox_to_parts(words);
    GalileoInavPage::decode_parts(&even, &odd).unwrap()
}

/// Returns the E08 page of given word type.
pub(crate) fn ublox_e08(word_type: u8) -> [u32; 8] {
    UBLOX_E08
        .iter()
        .find(|(t, _)| *t == word_type)
        .map(|(_, words)| *words)
        .unwrap()
}

/// Real data, compared to the RINEX navigation record obtained with RTKLIB:
///
/// ```text
/// E08 2025 04 25 06 20 00 -.454849563539D-03 -.639488462184D-11  .000000000000D+00
///       .123000000000D+03 -.649062500000D+02  .374265589664D-08  .292229012703D+01
///      -.293739140034D-05  .380387064070D-03  .267662107944D-05  .544060424042D+04
///       .454800000000D+06  .130385160446D-07 -.166749088160D+01  .279396772385D-07
///       .969464814375D+00  .288687500000D+03 -.358660436333D+00 -.609382526085D-08
///      -.199294015681D-09  .513000000000D+03  .236300000000D+04  .000000000000D+00
///       .312000000000D+01  .000000000000D+00 -.698491930962D-08 -.791624188423D-08
/// ```
#[cfg(test)]
mod test {
    use super::{parts_to_ublox, ublox_e08, ublox_page, ublox_to_parts, UBLOX_E08};
    use crate::galileo::{GalileoInavPage, GalileoSignalHealth};
    use core::f64::consts::PI;

    fn decode(word_type: u8) -> GalileoInavPage {
        ublox_page(&ublox_e08(word_type))
    }

    #[test]
    fn ublox_parts_reciprocal() {
        for (_, words) in UBLOX_E08.iter() {
            let (even, odd) = ublox_to_parts(words);
            assert_eq!(parts_to_ublox(&even, &odd), *words);
        }
    }

    fn assert_close(value: f64, expected: f64, tolerance: f64, name: &str) {
        assert!(
            (value - expected).abs() <= tolerance,
            "{}: {:e} != {:e}",
            name,
            value,
            expected
        );
    }

    #[test]
    fn word0() {
        let word = decode(0).word.as_word0().unwrap();
        assert!(word.has_valid_time());
        assert_eq!(word.wn, 1339);
        assert!(word.tow < 604_800);
        assert_eq!(word.spare, [0x55; 11]);
    }

    #[test]
    fn word1() {
        let word = decode(1).word.as_word1().unwrap();
        assert_eq!(word.iodnav, 123);
        assert_eq!(word.toe, 454_800);
        assert_close(word.m0 * PI, 0.292229012703e1, 1e-11, "M0");
        assert_close(word.e, 0.380387064070e-3, 1e-14, "e");
        assert_close(word.sqrt_a, 0.544060424042e4, 1e-8, "sqrt(A)");
    }

    #[test]
    fn word2() {
        let word = decode(2).word.as_word2().unwrap();
        assert_eq!(word.iodnav, 123);
        assert_close(word.omega0 * PI, -0.166749088160e1, 1e-11, "Omega0");
        assert_close(word.i0 * PI, 0.969464814375, 1e-11, "i0");
        assert_close(word.omega * PI, -0.358660436333, 1e-11, "omega");
        assert_close(word.idot * PI, -0.199294015681e-9, 1e-20, "idot");
    }

    #[test]
    fn word3() {
        let word = decode(3).word.as_word3().unwrap();
        assert_eq!(word.iodnav, 123);
        assert_close(word.omega_dot * PI, -0.609382526085e-8, 1e-19, "Omega_dot");
        assert_close(word.dn * PI, 0.374265589664e-8, 1e-19, "dn");
        assert_close(word.cuc, -0.293739140034e-5, 1e-16, "cuc");
        assert_close(word.cus, 0.267662107944e-5, 1e-16, "cus");
        assert_eq!(word.crc, 288.6875);
        assert_eq!(word.crs, -64.90625);
        assert_close(word.sisa.meters().unwrap(), 3.12, 1e-9, "SISA");
    }

    #[test]
    fn word4() {
        let word = decode(4).word.as_word4().unwrap();
        assert_eq!(word.iodnav, 123);
        assert_eq!(word.svid, 8);
        assert_eq!(word.toc, 454_800);
        assert_close(word.cic, 0.130385160446e-7, 1e-18, "cic");
        assert_close(word.cis, 0.279396772385e-7, 1e-18, "cis");
        assert_close(word.af0, -0.454849563539e-3, 1e-14, "af0");
        assert_close(word.af1, -0.639488462184e-11, 1e-22, "af1");
        assert_eq!(word.af2, 0.0);
    }

    #[test]
    fn word5() {
        let word = decode(5).word.as_word5().unwrap();
        assert_close(word.bgd_e1_e5a, -0.698491930962e-8, 1e-19, "BGD E1-E5a");
        assert_close(word.bgd_e1_e5b, -0.791624188423e-8, 1e-19, "BGD E1-E5b");
        assert_eq!(word.e1b_health, GalileoSignalHealth::Ok);
        assert_eq!(word.e5b_health, GalileoSignalHealth::Ok);
        assert!(!word.e1b_dvs);
        assert!(!word.e5b_dvs);
        assert!(word.is_healthy());
        // RINEX GAL week 2363 is aligned to GPS weeks: GST = GPS - 1024
        assert_eq!(word.wn, 2363 - 1024);
        assert!(word.tow < 604_800);
        assert!(word.ai0 > 0.0);
    }

    #[test]
    fn word6() {
        let word = decode(6).word.as_word6().unwrap();
        assert_eq!(word.dt_ls, 18);
        assert_eq!(word.dt_lsf, 18);
        assert!(word.dn >= 1 && word.dn <= 7);
        assert!(word.a0.abs() < 1e-6);
    }

    #[test]
    fn gst_is_consistent() {
        // the 3 words carrying time are transmitted within the same sub-frame (30 s)
        let (wn0, tow0) = decode(0).word.gst().unwrap();
        let (wn5, tow5) = decode(5).word.gst().unwrap();
        let tow6 = decode(6).word.as_word6().unwrap().tow;
        assert_eq!(wn0, wn5);
        assert!(tow0.abs_diff(tow5) < 30);
        assert!(tow5.abs_diff(tow6) < 30);
    }
}
