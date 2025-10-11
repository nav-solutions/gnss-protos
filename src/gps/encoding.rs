use crate::gps::{
    GpsDataWord, GpsQzssFrame, GpsQzssFrameId, GPS_FRAME_BITS, GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE,
    GPS_WORDS_PER_FRAME,
};

#[cfg(test)]
mod encoding {
    use std::fs::File;
    use std::io::{Read, Write};

    #[cfg(all(feature = "std", feature = "log"))]
    use crate::tests::init_logger;

    use log::info;

    use crate::gps::{
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrameId,
        GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS, GPS_FRAME_BYTES,
    };

