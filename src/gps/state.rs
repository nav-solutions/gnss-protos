#[cfg(feature = "log")]
use log::{debug, error, trace};

use crate::{
    bitstream::{BitStream, Byte},
    gps::{
        frame1::{
            Word10 as Frame1Word10, Word3 as Frame1Word3, Word4 as Frame1Word4,
            Word5 as Frame1Word5, Word6 as Frame1Word6, Word7 as Frame1Word7, Word8 as Frame1Word8,
            Word9 as Frame1Word9,
        },
        frame2::{
            Word10 as Frame2Word10, Word3 as Frame2Word3, Word4 as Frame2Word4,
            Word5 as Frame2Word5, Word6 as Frame2Word6, Word7 as Frame2Word7, Word8 as Frame2Word8,
            Word9 as Frame2Word9,
        },
        GpsQzssFrame, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry,
    },
    GpsQzssFrameId,
};

#[derive(Debug, Copy, Clone, PartialEq, Default)]
pub enum State {
    #[default]
    Preamble,
    TlmIntegrity,
    Parity,
    How,
    HowParity,
    DataWord,
    DataWordParity,
}

impl State {
    pub fn bin_size(&self) -> usize {
        match self {
            Self::Preamble => 8,
            Self::TlmIntegrity => 16,
            Self::Parity => 6,
            Self::How => 30,
            Self::HowParity => 6,
            Self::DataWord => 30 - 6,
            Self::DataWordParity => 6,
        }
    }
}
