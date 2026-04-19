use crate::{
    gps::{GpsQzssAlmanach, GpsQzssSatelliteHealth, GPS_WORD_BITS, GPS_WORD_BYTES},
    Message,
};

mod page25;
pub use page25::*;

use bitbuffer::{
    BigEndian, BitError, BitRead, BitReadBuffer, BitReadStream, BitWrite, BitWriteStream,
};

/// [GpsQzssFrame5] message interpretation
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GpsQzssFrame5 {
    /// [GpsQzssAlmanach] for satellite #1
    Page1(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #2
    Page2(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #3
    Page3(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #4
    Page4(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #5
    Page5(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #6
    Page6(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #7
    Page7(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #8
    Page8(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #9
    Page9(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #10
    Page10(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #11
    Page11(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #12
    Page12(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #13
    Page13(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #14
    Page14(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #15
    Page15(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #16
    Page16(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #17
    Page17(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #18
    Page18(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #19
    Page19(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #20
    Page20(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #21
    Page21(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #22
    Page22(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #23
    Page23(GpsQzssAlmanach),

    /// [GpsQzssAlmanach] for satellite #24
    Page24(GpsQzssAlmanach),

    /// [GpsQzssAlmanachStatus] gives satellite #1 (included) through #24 (included)
    /// health status and other general Almanach infos.
    Page25(GpsQzssFrame5Page25),
}

impl Default for GpsQzssFrame5 {
    /// Builds a default [GpsQzssFrame5::Page1]
    fn default() -> Self {
        Self::Page1(Default::default())
    }
}

impl Message<BigEndian> for GpsQzssFrame5 {
    fn encoding_size(&self) -> usize {
        8 * GPS_WORD_BYTES
    }

    fn encoding_bits(&self) -> usize {
        8 * GPS_WORD_BITS
    }
}

impl BitWrite<BigEndian> for GpsQzssFrame5 {
    fn write(&self, stream: &mut BitWriteStream<'_, BigEndian>) -> Result<(), BitError> {
        Ok(())
    }
}

impl BitRead<'_, BigEndian> for GpsQzssFrame5 {
    fn read(stream: &mut BitReadStream<'_, BigEndian>) -> Result<Self, BitError> {
        let page_id = stream.read_int::<u8>(2)?;
        let sat_id = stream.read_int::<u8>(6)?;

        match page_id {
            25 => {
                let toa = stream.read_int::<u8>(8)?;
                let wna = stream.read_int::<u8>(8)?;
                let sat_healths: [GpsQzssSatelliteHealth; 24] = Default::default();

                for i in 0..6 {
                    for j in 0..4 {
                        let value = stream.read_int::<u8>(6)?;
                        sat_healths[i * 4 + j] = GpsQzssSatelliteHealth::from(value);
                    }
                    let parity = stream.read_int::<u8>(6)?; // TODO (parity)
                }

                let reserved = stream.read_int::<u8>(3)?;
                let spare = stream.read_int::<u32>(19)?;
                let nib = stream.read_int::<u8>(2)?; // TODO (parity)
                let parity = stream.read_int::<u8>(6)?; // TODO (parity)

                Ok(Self::Page25(GpsQzssFrame5Page25 {
                    toa,
                    wna,
                    reserved,
                    spare,
                    sat_healths,
                }))
            },
            1..24 => {},
            _ => {
                // invalid page number
            },
        }
    }
}

impl GpsQzssFrame5 {
    /// Returns the page identification number of this [GpsQzssFrame5] interpretation
    pub fn page_id(&self) -> u8 {
        match self {
            Self::Page1(_) => 1,
            Self::Page2(_) => 2,
            Self::Page3(_) => 3,
            Self::Page4(_) => 4,
            Self::Page5(_) => 5,
            Self::Page6(_) => 6,
            Self::Page7(_) => 7,
            Self::Page8(_) => 8,
            Self::Page9(_) => 9,
            Self::Page10(_) => 10,
            Self::Page11(_) => 11,
            Self::Page12(_) => 12,
            Self::Page13(_) => 13,
            Self::Page14(_) => 14,
            Self::Page15(_) => 15,
            Self::Page16(_) => 16,
            Self::Page17(_) => 17,
            Self::Page18(_) => 18,
            Self::Page19(_) => 19,
            Self::Page20(_) => 20,
            Self::Page21(_) => 21,
            Self::Page22(_) => 22,
            Self::Page23(_) => 23,
            Self::Page24(_) => 24,
            Self::Page25(_) => 25,
        }
    }
}

#[cfg(test)]
mod frame5 {

    #[test]
    fn pagination_id() {
        for (page, id) in [
            (GpsQzssFrame5::Page1(Default::default()), 1),
            (GpsQzssFrame5::Page2(Default::default()), 2),
            (GpsQzssFrame5::Page4(Default::default()), 4),
            (GpsQzssFrame5::Page5(Default::default()), 5),
            (GpsQzssFrame5::Page8(Default::default()), 8),
            (GpsQzssFrame5::Page13(Default::default()), 13),
            (GpsQzssFrame5::Page24(Default::default()), 24),
            (GpsQzssFrame5::Page25(Default::default()), 25),
        ] {
            assert_eq!(page.page_id(), id, "returned invalid page-id for {}", page);
        }
    }
}
