/// GPS preamble (SYNC) byte
pub const GPS_PREAMBLE_BYTE: u8 = 0x8B;

/// GPS data word size (in bits)
pub const GPS_WORD_BITS: usize = 30;

/// GPS data word size (in bytes)
pub const GPS_WORD_BYTES: usize = 4;

/// Number of words in a frame
pub const GPS_WORDS_PER_FRAME: usize = 10;

/// Total GPS/QZSS frame size (in bits!)
pub const GPS_FRAME_BITS: usize = GPS_WORDS_PER_FRAME * GPS_WORD_BITS;

// /// Total GPS/QZSS subframe size (in bits!)
// pub(crate) const GPS_SUBFRAME_BITS: usize = (GPS_WORDS_PER_FRAME - 2) * GPS_WORD_BITS;

/// Total GPS/QZSS frame size (in bytes!)
pub const GPS_FRAME_BYTES: usize = (GPS_FRAME_BITS / 8) + 1;

// /// Parity bit mask (for each [GpsDataWord])
// pub(crate) const GPS_PARITY_MASK: u32 = 0x0000_003f;

// /// Number of parity bits for each [GpsDataWord]
// pub(crate) const GPS_PARITY_SIZE: usize = 6;

// /// L1 C/A code length
// pub const GPS_L1_CA_CODE_LEN: usize = 1023;

pub(crate) fn rad_to_semicircles(rad: f64) -> f64 {
    rad.to_degrees() * (2.0_f64.powi(31) / 90.0)
}

/// Writes a signed integer on `bits` bits (two's complement), clamped
/// to the range that this field can represent.
pub(crate) fn write_signed(
    stream: &mut bitbuffer::BitWriteStream<'_, bitbuffer::BigEndian>,
    value: i64,
    bits: usize,
) -> Result<(), bitbuffer::BitError> {
    let max = (1i64 << (bits - 1)) - 1;
    let min = -(1i64 << (bits - 1));
    let value = value.clamp(min, max);
    let mask = if bits == 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    };
    stream.write_int((value as u32) & mask, bits)
}

/// Writes an unsigned integer on `bits` bits, clamped
/// to the range that this field can represent.
pub(crate) fn write_unsigned(
    stream: &mut bitbuffer::BitWriteStream<'_, bitbuffer::BigEndian>,
    value: i64,
    bits: usize,
) -> Result<(), bitbuffer::BitError> {
    let max = if bits == 32 {
        u32::MAX as i64
    } else {
        (1i64 << bits) - 1
    };
    let value = value.clamp(0, max);
    stream.write_int(value as u32, bits)
}

/// Converts a physical value to its binary representation, using a 2^`pow2` scale factor.
pub(crate) fn to_scaled(value: f64, pow2: i32) -> i64 {
    (value * 2.0_f64.powi(-pow2)).round() as i64
}

/// Converts a binary value to its physical representation, using a 2^`pow2` scale factor.
pub(crate) fn from_scaled<T: Into<f64>>(value: T, pow2: i32) -> f64 {
    value.into() * 2.0_f64.powi(pow2)
}

// mod cdma;
// pub use cdma::GpsQzssModulator;

mod almanach;
pub use almanach::GpsQzssAlmanach;

mod decoder;
pub use decoder::GpsQzssDecoder;

mod errors;
pub use errors::GpsError;

pub mod link;

mod health;
pub use health::GpsQzssSatelliteHealth;

mod frame1;
pub use frame1::GpsQzssFrame1;

mod frame2;
pub use frame2::GpsQzssFrame2;

mod frame3;
pub use frame3::GpsQzssFrame3;

// mod frame4;
// pub use frame4::GpsQzssFrame4;

// mod frame5;
// pub use frame5::GpsQzssFrame5;

mod frame_id;
pub use frame_id::GpsQzssFrameId;

mod how;
pub use how::GpsQzssHow;

mod tlm;
pub use tlm::GpsQzssTelemetry;

mod frame;
pub use frame::GpsQzssFrame;

mod subframe;
pub use subframe::GpsQzssSubframe;

#[cfg(test)]
mod test {
    use crate::gps::{GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_WORDS_PER_FRAME};

    #[test]
    fn gps_properties() {
        assert_eq!(GPS_FRAME_BYTES, 38);
        assert_eq!(GPS_FRAME_BITS, 300);
        assert_eq!(GPS_WORDS_PER_FRAME, 10);
    }
}
