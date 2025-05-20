#[cfg(feature = "log")]
use log::debug;

use crate::gps::{GpsError, GpsQzssFrame, State, GPS_MIN_SIZE, GPS_PREAMBLE_MASK};

impl GpsQzssFrame {
    /// Encodes this [GpsQzssFrame] into preallocated buffer in which
    /// the frame must fit completely. We consider the minimal
    /// allocation should be [GPS_MIN_SIZE].
    ///
    /// GPS is MSB first,
    /// least significant bit first inside each bytes,
    /// and uses 30 bit data words, NB: the output is not aligned
    /// to 32 bits.
    ///
    /// ## Input
    /// - buf: preallocated buffer.
    /// - size: available size in buffer. If this value
    /// is not up to date, the program may panic.
    /// If size available is greater than [GPS_MIN_SIZE], this will never happen.
    ///
    /// ## Output
    /// - Ok(size) : encoded size, when everything went fine
    pub fn encode(&self, buf: &mut [u8], size: usize) -> Result<usize, GpsError> {
        if size < GPS_MIN_SIZE {
            return Err(GpsError::WouldNotFit);
        }

        let mut ptr = 0;
        let mut binoffset = 0;

        let mut state = State::default();
        let mut prev_state = state;

        loop {
            if ptr == size {
                // would panic
                unreachable!("buffer to small");
            }

            let next_state = match state {
                State::Preamble => {
                    buf[ptr] = GPS_PREAMBLE_MASK;
                    State::TlmIntegrity
                },

                State::TlmIntegrity => {
                    // 14 -bit message
                    buf[ptr] = ((self.telemetry.message >> 8) & 0x3f) as u8;
                    buf[ptr + 1] = (self.telemetry.message & 0xff) as u8;

                    if self.telemetry.integrity {
                        buf[ptr + 1] |= 0x40; // integrity status flag
                    }

                    if self.telemetry.reserved_bits {
                        buf[ptr + 1] |= 0x80; // reserved bits
                    }

                    State::Parity
                },

                State::Parity => {
                    let parity = 0x00; // TODO
                    State::How
                },

                State::How => return Ok(0),
                _ => return Ok(0),
            };

            let bin_size = next_state.bin_size();

            let bytes = bin_size / 8;
            let binoffset = (bin_size * 8) - bytes;
            ptr += bytes;

            state = next_state;
            prev_state = state;

            #[cfg(feature = "log")]
            debug!("state={:?} | ptr={}", state, ptr);
        }

        Ok(size)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        gps::{
            GpsQzssFrame, GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
            GPS_MIN_SIZE,
        },
        prelude::Byte,
        GpsQzssDecoder, GPS_PREAMBLE_MASK,
    };

    #[cfg(feature = "log")]
    use crate::tests::init_logger;

    #[cfg(feature = "log")]
    use log::info;

    #[test]
    fn gps_qzss_frame1_encoding() {
        #[cfg(feature = "log")]
        init_logger();

        let frame = GpsQzssFrame {
            how: crate::GpsQzssHow {
                tow: 1,
                alert: true,
                anti_spoofing: false,
                frame_id: GpsQzssFrameId::Ephemeris1,
            },
            telemetry: GpsQzssTelemetry {
                message: 10,
                integrity: true,
                reserved_bits: false,
            },
            subframe: GpsQzssSubframe::Eph1(crate::GpsQzssFrame1 {
                week: 11,
                ca_or_p_l2: 4,
                ura: 5,
                health: 6,
                iodc: 7,
                toc: 8,
                tgd: 9.0,
                af2: 10.0,
                af1: 11.0,
                af0: 12.0,
                reserved_word4: 13,
                l2_p_data_flag: false,
                reserved_word5: 14,
                reserved_word6: 15,
                reserved_word7: 16,
            }),
        };

        let mut buffer = [0u8; GPS_MIN_SIZE];

        let encoded = frame.encode(&mut buffer, GPS_MIN_SIZE).unwrap();

        assert_eq!(
            buffer[0], GPS_PREAMBLE_MASK,
            "encoded first byte is not preamble!"
        );

        assert_eq!(buffer[1], 0x00);

        #[cfg(feature = "log")]
        info!("decoding!");

        let mut decoder = GpsQzssDecoder::default().without_parity_verification();

        for byte in buffer.iter() {
            let decoded = decoder.parse(Byte::byte(*byte));
        }
    }
}
