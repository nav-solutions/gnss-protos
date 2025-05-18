#[cfg(feature = "log")]
use log::error;

use crate::gps::{GpsQzssFrame, GPS_BITMASK, GpsQzssTelemetry, GpsQzssHow};

#[derive(Debug, Copy, Clone, Default)]
enum State {
    /// GPS TLM word
    #[default]
    Tlm,

    /// GPS How word
    How,
}

#[derive(Debug, Default)]
pub struct GpsQzssDecoder {
    /// Current [State]
    state: State,

    /// Internal pointer
    ptr: usize,

    /// Internal [u32] dword
    dword: u32,

    /// Latest [GpsQzssTelemetry]
    tlm: GpsQzssTelemetry,

    /// Latest [GpsQzssHow]
    how: GpsQzssHow,
}

impl GpsQzssDecoder {
    /// Parse a GPS/QZSS stream of bytes, which must be aligned to 32-bit with 2 MSB padding
    pub fn parse(&mut self, byte: u8) -> Option<GpsQzssFrame> {
        self.ptr += 1;

        self.dword <<= 8;
        self.dword |= byte as u32;

        if self.ptr == 4 {
            // 2 MSB padding
            self.dword &= GPS_BITMASK;            
            
            match self.state {
                State::Tlm => {
                    let tlm = GpsQzssTelemetry::decode(self.dword);

                    match tlm {
                        Ok(tlm) => {
                            self.tlm = tlm;
                            self.state = State::How;
                        },
                        #[cfg(feature = "log")]
                        Err(e) => {
                            error!("GPS: {}", e);
                        },
                        #[cfg(not(feature = "log"))]
                        Err(_) => {
                        },
                    }
                },
                State::How => {
                    let how = GpsQzssHow::decode(self.dword);

                    if let Ok(how) = how {
                        self.how = how;
                        self.state = State::Tlm;
                    }
                },
            }

            self.ptr = 0;
        }

        None
    }
}

#[cfg(test)]
mod test {
    use crate::gps::{GpsQzssDecoder};

    #[test]
    fn test_gps_decoder() {
        let bytes = [0x8B, 0x00, 0x00, 0x00];

        let mut decoder = GpsQzssDecoder::default();

        for byte in bytes {
            if let Ok(frame) = decoder.parse(byte) {

            }
        }
    }
}