use crate::{
    bits::get_bit,
    gps::lnav::{frame::GpsQzssLnavFrameBuilder, GpsQzssLnavFrame, GpsQzssLnavSubfDecoder},
};

/// [GpsQzssLnavFrameDecoder] synchronizes to a GPS / QZSS bit stream and decodes
/// 30s [GpsQzssFrame]s, wherever they are located (any bit alignment).
///
/// It works as a 1500-bit shift register, fed one bit at a time
/// ([Self::push_bit], typical of a tracking loop delivering one navigation bit
/// every 20 ms) or from a byte slice at any bit offset ([Self::decode]).
/// Each time the register starts with a preamble (normal or inverted, the carrier
/// phase ambiguity is resolved), the 10 words are verified and interpreted.
///
/// The decoder has a small constant memory footprint and does not allocate.
#[derive(Default, Copy, Clone)]
pub struct GpsQzssLnavFrameDecoder {
    decoder: GpsQzssLnavSubfDecoder,
    builder: GpsQzssLnavFrameBuilder,
}

impl GpsQzssLnavFrameDecoder {
    /// Copies and returns [GpsQzssLnavFrameDecoder] with parity verification (default).
    pub fn with_parity_verification(mut self) -> Self {
        self.decoder = self.decoder.with_parity_verification();
        self
    }

    /// Copies and returns [GpsQzssDecoder] without parity verification.
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

    /// Returns the [GpsQzssLnavFrameBuilder] state, for example to monitor
    /// the time left before the current period completes.
    pub fn builder(&self) -> &GpsQzssLnavFrameBuilder {
        &self.builder
    }

    /// Pushes a new bit into the decoder.
    /// Returns a [GpsQzssFrame] when this bit completes a valid frame.
    pub fn push_bit(&mut self, bit: bool) -> Option<GpsQzssLnavFrame> {
        let sbf = self.decoder.push_bit(bit)?;
        self.builder.push_sbf(sbf)
    }

    /// Feeds bits from `slice`, starting at absolute bit position `bit_offset`,
    /// until a frame is decoded or the slice is exhausted.
    ///
    /// Returns the number of bits consumed, and the decoded subframe (if any).
    /// The decoder state is preserved between calls: a frame may span
    /// several slices. To decode a complete buffer:
    ///
    /// ```
    /// use gnss_protos::gps::lav::{
    ///     GpsQzssLnavFrameDecoder,
    ///     GpsQzssSubframe,
    ///     GPS_LNAV_FRAME_BYTES,
    /// };
    ///
    /// let bytes = [0u8; GPS_LNAV_FRAME_BYTES]; // any input
    /// let mut decoder = GpsQzssLnavFrameDecoder::default();
    /// let mut position = 0; // in bits!
    ///
    /// loop {
    ///     let (consumed, frame) = decoder.decode(&bytes, position);
    ///     position += consumed;
    ///
    ///     if let Some(frame) = frame {
    ///         // a frame ended at bit `position`
    ///         // it represents 1500 bits
    ///     } else {
    ///         break; // slice exhausted
    ///     }
    /// }
    /// ```
    pub fn decode(&mut self, slice: &[u8], bit_offset: usize) -> (usize, Option<GpsQzssLnavFrame>) {
        let total = slice.len() * 8;
        let mut pos = bit_offset;

        while pos < total {
            let bit = get_bit(slice, pos);
            pos += 1;

            if let Some(frame) = self.push_bit(bit) {
                return (pos - bit_offset, Some(frame));
            }
        }

        (pos.saturating_sub(bit_offset), None)
    }
}
