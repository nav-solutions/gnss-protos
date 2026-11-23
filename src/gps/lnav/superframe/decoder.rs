use crate::{
    bits::get_bit,
    gps::lnav::{GpsQzssLnavSubfDecoder, GpsQzssLnavSuperframe, GpsQzssLnavSuperframeBuilder},
};

/// [GpsQzssLnavSuperframeDecoder] synchronizes to a GPS / QZSS bit stream and decodes
/// 12'30" [GpsQzssLnavSuperframe]s, wherever they are located (any bit alignment).
///
/// It combines our [GpsQzssLnavSubfDecoder] (300-bit shift register, fed one bit at a time,
/// normal or inverted streams) with our [GpsQzssLnavSuperframeBuilder], which locks
/// to EPH-1 of Page-1 and verifies the continuity of the 125 subframes.
/// Discontinuities and corrupted subframes drop the current period: the decoder
/// then waits for the next Page-1 EPH-1.
///
/// ```
/// use gnss_protos::gps::lnav::GpsQzssLnavSuperframeDecoder;
///
/// let bytes = [0u8; 128]; // any input
/// let mut decoder = GpsQzssLnavSuperframeDecoder::default();
/// let mut position = 0; // in bits!
///
/// loop {
///     let (consumed, superframe) = decoder.decode(&bytes, position);
///     position += consumed;
///
///     if let Some(superframe) = superframe {
///         // a 12'30" period ended at bit `position`
///     } else {
///         break; // slice exhausted
///     }
/// }
/// ```
#[derive(Debug, Default, Copy, Clone)]
pub struct GpsQzssLnavSuperframeDecoder {
    decoder: GpsQzssLnavSubfDecoder,
    builder: GpsQzssLnavSuperframeBuilder,
}

impl GpsQzssLnavSuperframeDecoder {
    /// Copies and returns [GpsQzssLnavSuperframeDecoder] with parity verification (default).
    pub fn with_parity_verification(mut self) -> Self {
        self.decoder = self.decoder.with_parity_verification();
        self
    }

    /// Copies and returns [GpsQzssLnavSuperframeDecoder] without parity verification.
    /// Not recommended: the preamble alone (8 bits) is a weak synchronization
    /// pattern and data bits will regularly look like a preamble.
    pub fn without_parity_verification(mut self) -> Self {
        self.decoder = self.decoder.without_parity_verification();
        self
    }

    /// Drops the current synchronization state.
    pub fn reset(&mut self) {
        self.decoder.reset();
        self.builder.reset();
    }

    /// Returns the [GpsQzssLnavSuperframeBuilder] state, for example to monitor
    /// the time left before the current period completes.
    pub fn builder(&self) -> &GpsQzssLnavSuperframeBuilder {
        &self.builder
    }

    /// Pushes a new bit into the decoder.
    /// Returns a [GpsQzssLnavSuperframe] when this bit completes a valid 12'30" period.
    pub fn push_bit(&mut self, bit: bool) -> Option<GpsQzssLnavSuperframe> {
        let sbf = self.decoder.push_bit(bit)?;
        self.builder.push_sbf(sbf)
    }

    /// Feeds bits from `slice`, starting at absolute bit position `bit_offset`,
    /// until a 12'30" period is decoded or the slice is exhausted.
    ///
    /// Returns the number of bits consumed, and the decoded [GpsQzssLnavSuperframe] (if any).
    /// The decoder state is preserved between calls: a period may span several slices.
    pub fn decode(
        &mut self,
        slice: &[u8],
        bit_offset: usize,
    ) -> (usize, Option<GpsQzssLnavSuperframe>) {
        let total = slice.len() * 8;
        let mut pos = bit_offset;

        while pos < total {
            let bit = get_bit(slice, pos);
            pos += 1;

            if let Some(spf) = self.push_bit(bit) {
                return (pos - bit_offset, Some(spf));
            }
        }

        (pos.saturating_sub(bit_offset), None)
    }
}

#[cfg(test)]
mod test {
    use crate::{
        bits::get_bit,
        gps::lnav::{
            GpsQzssLnavSuperframe, GpsQzssLnavSuperframeDecoder, GPS_LNAV_SUBFRAME_BITS,
            GPS_LNAV_SUPERFRAME_BITS,
        },
        BigEndian, BitWriteStream,
    };

    fn stream(periods: &[GpsQzssLnavSuperframe], offset: usize) -> Vec<u8> {
        let mut bytes = Vec::new();
        {
            let mut writer = BitWriteStream::new(&mut bytes, BigEndian);
            for _ in 0..offset {
                writer.write_bool(true).unwrap();
            }
            for spf in periods {
                writer.write(spf).unwrap();
            }
            writer.write_int(0u8, 8).unwrap();
        }
        bytes
    }

    fn decode_all(
        decoder: &mut GpsQzssLnavSuperframeDecoder,
        bytes: &[u8],
    ) -> Vec<(usize, GpsQzssLnavSuperframe)> {
        let mut ret = Vec::new();
        let mut position = 0;
        loop {
            let (consumed, spf) = decoder.decode(bytes, position);
            position += consumed;
            match spf {
                Some(spf) => ret.push((position, spf)),
                None => return ret,
            }
        }
    }

    #[test]
    fn consecutive_periods() {
        let periods = [
            GpsQzssLnavSuperframe::model(750 * 4),
            GpsQzssLnavSuperframe::model(750 * 5),
            GpsQzssLnavSuperframe::model(750 * 6),
        ];

        for offset in [0, 1, 7, 13] {
            let bytes = stream(&periods, offset);
            let mut decoder = GpsQzssLnavSuperframeDecoder::default();
            let decoded = decode_all(&mut decoder, &bytes);

            assert_eq!(decoded.len(), 3, "offset={}", offset);

            for (k, (position, spf)) in decoded.iter().enumerate() {
                assert_eq!(*spf, periods[k], "offset={} period #{}", offset, k);
                assert_eq!(*position, offset + (k + 1) * GPS_LNAV_SUPERFRAME_BITS);
            }
        }
    }

    #[test]
    fn inverted_stream() {
        let periods = [GpsQzssLnavSuperframe::model(750)];
        let bytes: Vec<u8> = stream(&periods, 3).iter().map(|b| !b).collect();

        let mut decoder = GpsQzssLnavSuperframeDecoder::default();
        let decoded = decode_all(&mut decoder, &bytes);

        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].1, periods[0]);
    }

    #[test]
    fn bit_by_bit_and_reset() {
        let periods = [
            GpsQzssLnavSuperframe::model(0),
            GpsQzssLnavSuperframe::model(750),
        ];
        let bytes = stream(&periods, 5);

        let mut decoder = GpsQzssLnavSuperframeDecoder::default();
        let mut decoded = Vec::new();

        for pos in 0..bytes.len() * 8 {
            if let Some(spf) = decoder.push_bit(get_bit(&bytes, pos)) {
                decoded.push(spf);
            }
        }

        assert_eq!(decoded, periods);

        // interrupted in the middle of the first period: only the second one is decoded
        let mut decoder = GpsQzssLnavSuperframeDecoder::default();
        let half = 5 + GPS_LNAV_SUPERFRAME_BITS / 2;

        for pos in 0..half {
            assert!(decoder.push_bit(get_bit(&bytes, pos)).is_none());
        }

        assert!(!decoder.builder().is_empty());
        decoder.reset();
        assert!(decoder.builder().is_empty());

        // resume a few subframes later
        let resume = half + 3 * GPS_LNAV_SUBFRAME_BITS;
        let mut decoded = Vec::new();

        for pos in resume..bytes.len() * 8 {
            if let Some(spf) = decoder.push_bit(get_bit(&bytes, pos)) {
                decoded.push(spf);
            }
        }

        assert_eq!(decoded, vec![periods[1]]);
    }
}
