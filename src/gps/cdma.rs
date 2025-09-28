use crate::gps::{GpsQzssFrame, GPS_BYTES_PER_FRAME, GPS_WORDS_PER_FRAME};

/// [GpsQzssModulator] applies the L1/CA CDMA modulation
/// to any bit sequence (regardless of its content).
/// The stream then becomes compatible with transmission.
/// GPS/QZSS must be transmitted at 50 bits per second, one [GpsQzssFrame]
/// every 6 seconds.
#[derive(Copy, Clone, PartialEq)]
pub struct GpsQzssModulator {
    /// SV ID
    sat_id: u8,

    /// single allocation
    r1: [i32; GPS_WORDS_PER_FRAME],
    
    /// single allocation
    g1: [i32; GPS_L1_CA_CODE_LEN],
    
    /// single allocation
    r2: [i32; GPS_WORDS_PER_FRAME],

    /// single allocation
    g2: [i32; GPS_L1_CA_CODE_LEN],
}

impl Default for GpsQzssModulator {
    /// Builds a default [GpsQzssModulator] ready to scramble like SV01
    fn default() -> Self {
        Self {
            sat_id: 1,
            r1: [-1i32; GPS_WORDS_PER_FRAME],
            r2: [-1i32; GPS_WORDS_PER_FRAME],
            g1: [0; GPS_L1_CA_CODE_LEN],
            g2: [0; GPS_L1_CA_CODE_LEN],
        }
    }
}

impl GpsQzssModulator {
    /// Creates a new [GpsQzssModulator] ready to scramble a stream of bits
    /// like a specific satellite.
    ///
    /// Although this method is infallible, you should use ID# between
    /// 1 and 32 here (both included), as GPS / QZSS does not supported other satellites.
    /// If you provid an incorrect ID, the modulator will be preset for SV01
    /// and you will obtain unexpected results.
    pub fn from_satellite_id(sat_id: u8) -> Self {
        let mut default = Self::default();

        if sat_id > 0 && sat_id < 33 {
            self.sat_id = sat_id;
        }
    }

    /// Encode read-only buffer into tx (transmission) buffer.
    ///
    /// You should preallocate the tx buffer with zeros to obtain
    /// a correct transmission.
    ///
    /// Correct GPS / QZSS transmission synchronous stream transmission.
    ///
    /// ## Input
    /// - rx: read-only buffer to encoded
    /// - tx: transmission, encoded buffer
    /// - size: number of bytes contained in rx buffer
    ///
    /// ## Returns
    /// - total number of _bits_ encoded in tx buffer.
    ///
    /// Since this method does not care for actual data content and simply applies
    /// the modulation logic, you can use it on your own buffer. Especially if
    /// you have a complete signal model (not only the [GpsQzssFrame] model that we provide).
    ///
    /// Example:
    /// ```
    /// use gnss_protos::{GpsQzssFrame, GpsQzssModulator};
    ///
    /// let mut tx = [0; 1024]; // at least 300 bits =37.5bytes
    ///
    /// // preset and deploy a modulator
    /// let mut modulator = GpsQzssModulator::from_satellite_id(25);
    ///
    /// // create a fancy GPS/QZSS frame
    /// let frame = GpsQzssFrame::default();
    ///
    /// // encode as bit stream ("readable", incompatible with transmission)
    /// let encoded = frame.encode_raw();
    ///
    /// // CDMA scramble this buffer
    /// let modulated = modulator.modulate(&encoded, encoded.len(), &mut tx);
    ///
    /// if modulated > 0 {
    ///     // we do not supported partial modulation,
    ///     // so at this point you know the complete frame was modulated (1024 >37.5)
    /// }
    /// ```
    pub fn modulate(rx: &[u8], rx_size: usize, tx: &mut [u8], tx_size: usize) -> usize {
        const TAPS : [usize; 32] = [
            5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
            252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
            473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
            861, 862,
        ];

        if tx_size < GPS_L1_CA_CODE_LEN {
            return 0; // must fit entirely
        }

        // reset
        for i in 0..GPS_WORDS_PER_FRAME {
            self.r1[i] = -1;
            self.r2[i] = -1;
        }

        for i in 0..GPS_L1_CA_SEQ_LEN {
            self.g1[i] = self.r1[GPS_WORDS_PER_FRAME -1];
            self.g2[i] = self.r2[GPS_WORDS_PER_FRAME -1];

            let c1 = self.r1[2] * self.r1[9];
            let c2 = self.r2[1] * self.r2[2] * self.r2[5] * self.r2[6] * self.r2[8] * self.r2[9];

            for j in 9..0 {
                self.r1[j] = self.r1[j -1];
                self.r2[j] = self.r2[j -1];
            }

            self.r1[0] = c1;
            self.r2[0] = c2;
        }
    
        let mut j = GPS_L1_CA_CODE_LEN - TAPS[self.sat_id -1];

        for i in 0..GPS_L1_CA_CODE_LEN {
            
            j += 1;
        }
        
        GPS_L1_CA_CODE_LEN
    }
}
