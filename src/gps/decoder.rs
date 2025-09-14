    #[test]
    fn test_tlm_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 3);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, true);
    }

    #[test]
    fn test_ublox_tlm_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::msb_padded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B), // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        let bytes = [0x22, 0xC1, 0x3E, 0x1B];
        let bytes = from_ublox_be_bytes(&bytes);

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);
    }

    #[test]
    fn test_tlmhow_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::msb_padded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B), // TLM
            GpsDataByte::MsbPadded(0x15),
            GpsDataByte::Byte(0x27),
            GpsDataByte::Byte(0xC9),
            GpsDataByte::Byte(0x73), // HOW
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        assert_eq!(decoder.how.alert, false);
        assert_eq!(decoder.how.anti_spoofing, true);
        assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);
    }

    #[test]
    fn test_ublox_tlmhow_decoding() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let bytes = [
            GpsDataByte::MsbPadded(0x22),
            GpsDataByte::Byte(0xC1),
            GpsDataByte::Byte(0x3E),
            GpsDataByte::Byte(0x1B), // TLM
            GpsDataByte::MsbPadded(0x15),
            GpsDataByte::Byte(0x27),
            GpsDataByte::Byte(0xC9),
            GpsDataByte::Byte(0x73), // HOW
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        assert_eq!(decoder.how.alert, false);
        assert_eq!(decoder.how.anti_spoofing, true);
        assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);

        let bytes = from_ublox_be_bytes(&[0x22, 0xC1, 0x3E, 0x1B, 0x15, 0x27, 0xC9, 0x73]);

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            let _ = decoder.consume_byte(byte);
        }

        assert_eq!(decoder.tlm.message, 0x13E);
        assert_eq!(decoder.tlm.integrity, false);
        assert_eq!(decoder.tlm.reserved_bits, false);

        assert_eq!(decoder.how.alert, false);
        assert_eq!(decoder.how.anti_spoofing, true);
        assert_eq!(decoder.how.frame_id, GpsQzssFrameId::Ephemeris1);
    }

    #[test]
    fn test_incomplete_frame() {
        #[cfg(all(feature = "std", feature = "log"))]
        init_logger();

        let mut found = false;

        let bytes = [
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // TLM
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.consume_byte(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded!");

        let bytes = [
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // TLM
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // HOW
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD3
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD4
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD5
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD6
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD7
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
            GpsDataByte::lsb_padded(0x00), // WORD8
            GpsDataByte::Byte(0x8B),
            GpsDataByte::Byte(0x00),
            GpsDataByte::Byte(0x0D),
        ];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Some(_) = decoder.consume_byte(byte) {
                found = true;
            }
        }

        assert!(!found, "Invalid GPS frame decoded");
    }
}
