/// Uninterpreted (undocumented, classified, spare)
/// Data from GPS/QZSS Frame 4 pages.
pub struct GpsQzssFrame4Raw {
    /// 24-bit data
    pub data: u32,
}

impl GpsQzssFrame4Raw {
    pub(crate) fn to_word(&self) -> GpsDataWord {
        GpsDataWord::from((self.data & 0x00ffffff) <<2)

    }

    pub(crate) fn from_word(word: GpsDataWord) -> Self {
        Self {
            data: (word.value() & 0x00ffffff),
        }
    }
}
