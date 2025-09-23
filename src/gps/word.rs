use crate::gps::{GpsDataByte, GpsError, GPS_PARITY_MASK};

/// [GpsDataWord] is used to pack [GpsDataByte]s.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GpsDataWord {
    /// [GpsDataByte]s stored with "big endianness"
    bytes: [GpsDataByte; 4],
}

impl Default for GpsDataWord {
    /// Creates a default null [GpsDataWord].
    fn default() -> Self {
        Self {
            bytes: Default::default(),
        }
    }
}

impl From<u32> for GpsDataWord {
    fn from(value: u32) -> GpsDataWord {
        GpsDataWord::from_be_bytes(&value.to_be_bytes())
    }
}

impl Into<u32> for GpsDataWord {
    fn into(self) -> u32 {
        self.value()
    }
}

impl GpsDataWord {
    /// Creates a [GpsDataWord] from a 30-bit slice stored as 4 big-endian bytes.
    pub fn from_be_bytes(slice: &[u8; 4]) -> Self {
        Self {
            bytes: [
                GpsDataByte::Byte(slice[0]),
                GpsDataByte::Byte(slice[1]),
                GpsDataByte::Byte(slice[2]),
                GpsDataByte::LsbPadded(slice[3]),
            ],
        }
    }

    /// Creates a [GpsDataWord] from a 30-bit slice stored as 4 little-endian bytes.
    pub fn from_le_bytes(slice: &[u8; 4]) -> Self {
        Self {
            bytes: [
                GpsDataByte::Byte(slice[3]),
                GpsDataByte::Byte(slice[2]),
                GpsDataByte::Byte(slice[1]),
                GpsDataByte::LsbPadded(slice[0]),
            ],
        }
    }

    /// Converts this [GpsDataWord] to [u32]
    pub fn value(&self) -> u32 {
        let mut value = self.bytes[3].as_u32();
        value |= self.bytes[2].as_u32() << 6;
        value |= self.bytes[1].as_u32() << 14;
        value |= self.bytes[0].as_u32() << 22;
        value
    }

    /// Evaluates the parity of this data word,
    /// using provided NIB which must be set to zero on initial cycle.
    /// 6-bit parity is encoded as [u8].
    pub fn parity(&self, rhs: Option<&Self>) -> u8 {
        let value = self.value();

        let (b29, b30) = if let Some(rhs) = rhs {
            let value = rhs.value() as u8; 
            ((value & 0x00000001) >> 3, (value & 0x00000002) >> 2)
        } else {
            (0, 0)
        };

        let (mut b0, mut b1, mut b2, mut b3, mut b4, mut b5)
                = (b29, b30, b29, b30, b29, b30);

        b0 ^= (value & 0x01) as u8;
        b0 ^= ((value & 0x02) >>1) as u8;
        b0 ^= ((value & 0x04) >>2) as u8;
        b0 ^= ((value & 0x10) >>4) as u8;
        b0 ^= ((value & 0x20) >>5) as u8;
        b0 ^= ((value & 0x200) >>9) as u8;
        b0 ^= ((value & 0x400) >>10) as u8;
        b0 ^= ((value & 0x800) >>11) as u8;
        b0 ^= ((value & 0x1000) >>12) as u8;
        b0 ^= ((value & 0x2000) >>13) as u8;
        b0 ^= ((value & 0x10000) >>16) as u8;
        b0 ^= ((value & 0x20000) >>17) as u8;
        b0 ^= ((value & 0x40000) >>19) as u8;
        b0 ^= ((value & 0x40000) >>22) as u8;
        
        b1 ^= (value & 0x02) as u8;

        b2 ^= (value & 0x01) as u8;
        
        b3 ^= (value & 0x02) as u8;
        
        b3 ^= (value & 0x01) as u8;
        
        b5 ^= (value & 0x04) as u8;

        let mut b = b0 & 0x1;

        b |= (b1 & 0x01) << 1;
        b |= (b2 & 0x01) << 2;
        b |= (b3 & 0x01) << 3;
        b |= (b4 & 0x01) << 4;
        b |= (b5 & 0x01) << 5;

        b
    }

    /// Verifies the parity of this [GpsDataWord], possibly invalidating this word.
    pub fn parity_check(&self, rhs: Option<&Self>) -> Result<(), GpsError> {
        let parity = self.parity(rhs);

        if (self.value() & GPS_PARITY_MASK) as u8 == parity {
            Ok(())
        } else {
            Err(GpsError::Parity)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn from_be_bytes() {
        for (bytes, value) in [
            ([0x8B, 0x12, 0x48, 0xCA], 0x22C49232),
            ([0x8B, 0xAA, 0xAA, 0xAA], 0x22EAAAAA),
            ([0x8B, 0xA5, 0xA5, 0xA5], 0x22E96969),
            ([0x8B, 0x5A, 0x5A, 0x5A], 0x22D69696),
        ] {
            let word = GpsDataWord::from_be_bytes(&bytes);
            let found = word.value();
            assert_eq!(
                found, value,
                "expected 0x{:08X}, got 0x{:08X}",
                value, found
            );
        }
    }

    #[test]
    fn from_le_bytes() {
        for (bytes, value) in [
            ([0xCA, 0x48, 0x12, 0x8B], 0x22C49232),
            ([0xAA, 0xAA, 0xAA, 0x8B], 0x22EAAAAA),
            ([0xA5, 0xA5, 0xA5, 0x8B], 0x22E96969),
            ([0x5A, 0x5A, 0x5A, 0x8B], 0x22D69696),
        ] {
            let word = GpsDataWord::from_le_bytes(&bytes);
            let found = word.value();
            assert_eq!(
                found, value,
                "expected 0x{:08X}, got 0x{:08X}",
                value, found
            );
        }
    }

    #[test]
    fn from_u32() {
        for (value, expected_value) in [
            (0x32563412u32, 0x32563412 >> 2),
            (0xF2563412u32, 0xF2563412 >> 2),
            (0xFFFF3412u32, 0xFFFF3412 >> 2),
            (0xFFFF34FFu32, 0xFFFF34FF >> 2),
        ] {
            let word = GpsDataWord::from(value);
            assert_eq!(word.value(), expected_value, "got 0x{:08X} expecting 0x{:08X}", word.value(), expected_value);
        }
    }

    #[test]
    fn parity_calc() {
        for (bin, b29, b30, expected) in [
            (
				0b101101001001011010010101110110u32,
				0,
				0,
				0b000000u8,
			),		 
        ] { 
			let word = GpsDataWord::from(bin << 6);

			let parity = if b29 == 0 && b30 == 0 {
				word.parity(None)
			} else {
				word.parity(None)
			};

	        assert_eq!(parity, expected, "got 0x{:02X}, expecting 0x{:02X}", parity, expected);	
        }
    }

    #[test]
    fn parity_ok_checker() {
        for (b29, b30, dword) in [
            (0, 0, 0x8B1248CA),
        ] {
            let word = GpsDataWord::from(dword);
            assert!(word.parity_check(None).is_ok(), "failed for 0x{:08X}", dword);
        }
    }

    #[test]
    fn parity_nok_checker() {
        for dword in [
            0x8B1248CA,
        ] {
            let word = GpsDataWord::from(dword);
            assert!(word.parity_check(None).is_err(), "failed for 0x{:08X}", dword);
        }
    }
}
