use crate::{
    gps::{
        GpsQzssFrameId, GpsQzssHow, GpsQzssSubframe, GpsQzssTelemetry, GPS_FRAME_BITS,
        GPS_FRAME_BYTES, GPS_PREAMBLE_BYTE,
    },
    BufferingError, Message,
};

/// GPS / QZSS interpreted frame.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct GpsQzssFrame {
    /// [GpsQzssTelemetry] describes the following frame and contains
    /// the sync-byter, therefore initiates a [GpsQzssFrame].
    pub telemetry: GpsQzssTelemetry,

    /// [GpsQzssHow] describes following frame.
    pub how: GpsQzssHow,

    /// [GpsQzssSubframe] depends on associated [GpsQzssHow].
    pub subframe: GpsQzssSubframe,
}

impl Message for GpsQzssFrame {
    fn encoding_size(&self) -> usize {
        GPS_FRAME_BYTES
    }

    fn encoding_bits(&self) -> usize {
        GPS_FRAME_BITS
    }

    /// Encodes this [GpsQzssFrame] as a 300 bit burst (38 bytes).
    /// Because [GpsQzssFrame] is not aligned to [u8], the very last byte contains 4 MSB padding bits, set to zeros
    /// (unsigned). If you leave it to that, any streaming/transmitter looses a little bit of efficiency
    /// any time a [GpsQzssFrame] is encoded/transmitted. The only solution then, is to manually remove this padding
    /// an truly concatenate your frames, but one can't expect easy processes when working with poorly designed and very old protocols.
    /// A true synchronous [GpsQzssFrame] emitter is supposed to transmit one frame every 6 seconds,
    /// that is 50 bits per second.
    /// NB: this [GpsQzssFrame] is not ready to transmit as-is and must be CDMA encoded

    /// Encodes this [GpsQzssFrame] into mutable [u8] buffer directly.
    fn encode(&self, dest: &mut [u8]) -> Result<usize, BufferingError> {
        let avail = dest.len();
        let needed_size = self.encoding_size();

        if avail < needed_size {
            return Err(BufferingError::StorageFull);
        }

        let subf = self.subframe.to_words();

        dest[0] = GPS_PREAMBLE_BYTE;
        dest[1] = ((self.telemetry.message & 0x3fc0) >> 6) as u8;
        dest[2] = (self.telemetry.message & 0x3f) as u8;
        dest[2] <<= 2; // TODO parity

        if self.telemetry.integrity {
            dest[2] |= 0x02;
        }

        if self.telemetry.reserved_bit {
            dest[2] |= 0x01;
        }

        let tow = self.how.tow * 2 / 3;

        dest[3] |= ((tow & 0x1_8000) >> 15) as u8;
        dest[4] = ((tow & 0x0_7f80) >> 7) as u8;
        dest[5] = (tow & 0x0_007f) as u8;
        dest[5] <<= 1;

        if self.how.alert {
            dest[5] |= 0x01;
        }

        if self.how.anti_spoofing {
            dest[6] |= 0x80;
        }

        dest[6] |= self.how.frame_id.encode() << 4;
        // dest[6] |= (parity & 0xf0) >> 4; // TODO parity
        // dest[7] |= (parity & 0x0f) << 4; // TODO parity

        // match self.how.frame_id {
        //     GpsQzssFrameId::Ephemeris1 => {
        //         let subf = self.subframe.as_eph1().unwrap_or_default();

        //         dest[7] |= ((subf.week & 0x3c0) >> 6) as u8;
        //         dest[8] = (subf.week & 0x03f) as u8;
        //         dest[8] <<= 2;
        //         dest[8] |= (subf.ca_or_p_l2) & 0x03;

        //         dest[9] = subf.ura & 0x0f;
        //         dest[9] <<= 4;
        //         dest[9] |= (subf.health & 0x3c) >> 2;

        //         dest[10] = subf.health & 0x03;
        //         dest[10] <<= 6;
        //         dest[10] |= (((subf.iodc & 0x300) >> 8) as u8) << 4;

        //         dest[11] <<= 6; // TODO (PAR)

        //         if subf.l2_p_data_flag {
        //             dest[11] |= 0x20;
        //         }

        //         dest[11] |= ((subf.reserved_word4 & 0x7c_0000) >> 18) as u8;
        //         dest[12] = ((subf.reserved_word4 & 0x03_fc00) >> 10) as u8;
        //         dest[13] = ((subf.reserved_word4 & 0x00_03fc) >> 2) as u8;

        //         dest[14] = (subf.reserved_word4 & 0x3) as u8;
        //         dest[14] <<= 6; // TODO

        //         dest[15] = ((subf.reserved_word5 & 0xff_0000) >> 16) as u8;
        //         dest[16] = ((subf.reserved_word5 & 0x00_ff00) >> 8) as u8;
        //         dest[17] = (subf.reserved_word5 & 0x00_00ff) as u8;
        //         dest[18] <<= 2; // TODO

        //         dest[18] |= ((subf.reserved_word6 & 0xc0_0000) >> 22) as u8;
        //         dest[19] = ((subf.reserved_word6 & 0x3f_c000) >> 14) as u8;
        //         dest[20] = ((subf.reserved_word6 & 0x00_3fc0) >> 6) as u8;
        //         dest[21] = (subf.reserved_word6 & 0x00_003f) as u8;
        //         dest[21] <<= 2;
        //         dest[22] <<= 4; // TODO

        //         dest[22] |= ((subf.reserved_word7 & 0xf000) >> 12) as u8;
        //         dest[23] = ((subf.reserved_word7 & 0x0ff0) >> 4) as u8;
        //         dest[24] = (subf.reserved_word7 & 0x000f) as u8;
        //         dest[24] <<= 4;

        //         let mut tgd = (subf.tgd * 2.0_f64.powi(31)).round() as u8;

        //         if subf.tgd < 0.0 {
        //             tgd += 128;
        //             tgd -= 1;
        //         }

        //         dest[24] |= (tgd & 0xf0) >> 4;

        //         dest[25] = tgd & 0x0f;
        //         dest[25] <<= 4; // TODO
        //         dest[26] <<= 6; // TODO
        //         dest[26] |= ((subf.iodc & 0x0fc) >> 2) as u8;

        //         dest[27] = (subf.iodc & 0x03) as u8;
        //         dest[27] <<= 6;

        //         let toc = subf.toc / 16;
        //         dest[27] |= ((toc & 0xfc00) >> 10) as u8;
        //         dest[28] = ((toc & 0x03fc) >> 2) as u8;
        //         dest[29] = (toc & 0x0003) as u8;
        //         dest[29] <<= 6; // TODO

        //         let af2 = (subf.af2 * 2.0_f64.powi(55)).round() as u8;
        //         let af1 = (subf.af1 * 2.0_f64.powi(43)).round() as u16;
        //         let af0 = (subf.af0 * 2.0_f64.powi(31)).round() as u32;

        //         dest[30] = af2;
        //         dest[31] = ((af1 & 0xff00) >> 8) as u8;
        //         dest[32] = (af1 & 0x00ff) as u8;
        //         dest[33] <<= 2; // TODO

        //         dest[33] |= ((af0 & 0x30_0000) >> 20) as u8;
        //         dest[34] = ((af0 & 0x0f_f000) >> 12) as u8;
        //         dest[35] = ((af0 & 0x00_0ff0) >> 4) as u8;
        //         dest[36] = (af0 & 0x00_000f) as u8;
        //         dest[36] <<= 4; // TODO

        //         dest[37] <<= 4;
        //     },
        //     GpsQzssFrameId::Ephemeris2 => {
        //         let subf = self.subframe.as_eph2().unwrap_or_default();

        //         dest[7] |= (subf.iode & 0xf0) >> 4;
        //         dest[8] |= subf.iode & 0x0f;
        //         dest[8] <<= 4;

        //         let crs = (subf.crs * 2.0_f64.powi(5)).round() as u16;
        //         dest[8] |= ((crs & 0xf000) >> 12) as u8;
        //         dest[9] |= ((crs & 0x0ff0) >> 4) as u8;
        //         dest[10] |= (crs & 0x000f) as u8;
        //         dest[10] <<= 4; // TODO

        //         let dn = (subf.dn * 2.0_f64.powi(43)).round() as u16;

        //         dest[11] |= ((dn & 0xfc00) >> 10) as u8;

        //         dest[12] |= ((dn & 0x03fc) >> 2) as u8;
        //         dest[13] = (dn & 0x0003) as u8;
        //         dest[13] <<= 6;

        //         let m0 = (subf.m0 * 2.0_f64.powi(31)).round() as u32;

        //         dest[13] |= (((m0 & 0xfc000000) >> 26) as u8) & 0x3f;
        //         dest[14] = ((m0 & 0x03000000) >> 24) as u8;
        //         dest[14] <<= 6; //TODO

        //         dest[15] |= ((m0 & 0x00ff0000) >> 16) as u8;
        //         dest[16] |= ((m0 & 0x0000ff00) >> 8) as u8;
        //         dest[17] |= (m0 & 0x000000ff) as u8;

        //         dest[18] <<= 2; // TODO

        //         let cuc = (subf.cuc * 2.0_f64.powi(29)).round() as u16;
        //         dest[18] |= ((cuc & 0xc000) >> 14) as u8;
        //         dest[19] |= ((cuc & 0x3fc0) >> 6) as u8;
        //         dest[20] |= (cuc & 0x003f) as u8;
        //         dest[20] <<= 2;

        //         let e = (subf.e * 2.0_f64.powi(33)).round() as u32;

        //         dest[20] |= ((e & 0xc0000000) >> 30) as u8;
        //         dest[21] |= ((e & 0x3f000000) >> 24) as u8;
        //         dest[21] <<= 2; // TODO
        //         dest[22] <<= 4; // TODO

        //         dest[22] |= ((e & 0x00f00000) >> 20) as u8;
        //         dest[23] |= ((e & 0x000ff000) >> 12) as u8;
        //         dest[24] |= ((e & 0x00000ff0) >> 4) as u8;
        //         dest[25] |= (e & 0x0000000f) as u8;
        //         dest[25] <<= 4; // TODO
        //         dest[26] <<= 2; // TODO

        //         let cus = (subf.cus * 2.0_f64.powi(29)).round() as u16;
        //         dest[26] |= ((cus & 0xfc00) >> 10) as u8;
        //         dest[27] |= ((cus & 0x03fc) >> 2) as u8;
        //         dest[28] |= (cus & 0x3) as u8;
        //         dest[28] <<= 6;

        //         let sqrt_a = (subf.sqrt_a * 2.0_f64.powi(19)).round() as u32;
        //         dest[28] |= ((sqrt_a & 0xfc000000) >> 26) as u8;
        //         dest[29] |= ((sqrt_a & 0x03000000) >> 24) as u8;
        //         dest[29] <<= 6; // TODO

        //         dest[30] |= ((sqrt_a & 0x00ff0000) >> 16) as u8;
        //         dest[31] |= ((sqrt_a & 0x0000ff00) >> 8) as u8;
        //         dest[32] |= (sqrt_a & 0x000000ff) as u8;

        //         let toe = (subf.toe / 16) as u16;

        //         dest[33] <<= 2; // TODO
        //         dest[33] |= ((toe & 0xc000) >> 14) as u8;
        //         dest[34] |= ((toe & 0x3fc0) >> 6) as u8;
        //         dest[35] |= (toe & 0x003f) as u8;
        //         dest[35] <<= 2;

        //         if subf.fit_int_flag {
        //             dest[35] |= 0x02;
        //         }

        //         dest[35] |= (subf.aodo & 0x10) >> 4;

        //         dest[36] |= subf.aodo & 0x0f;
        //         dest[36] <<= 4;

        //         dest[36] |= 0x00; // two non-information bits for parity calculations
        //         dest[37] |= 0x00;
        //         dest[37] <<= 4; // TODO
        //     },
        //     GpsQzssFrameId::Ephemeris3 => {
        //         let subf = self.subframe.as_eph3().unwrap_or_default();

        //         let cic = (subf.cic * 2.0_f64.powi(29)).round() as u16;

        //         dest[7] |= ((cic & 0xf000) >> 12) as u8;
        //         dest[8] |= ((cic & 0x0ff0) >> 4) as u8;

        //         dest[9] |= (cic & 0x000f) as u8;
        //         dest[9] <<= 4;

        //         let omega0 = (subf.omega0 * 2.0_f64.powi(31)).round() as u32;

        //         dest[9] |= (omega0 >> 28) as u8;
        //         dest[10] = ((omega0 >> 24) & 0x0f) as u8;
        //         dest[10] <<= 4; // TODO

        //         dest[11] |= ((omega0 & 0x00fc0000) >> 18) as u8;
        //         dest[12] = ((omega0 & 0x0003fc00) >> 10) as u8;
        //         dest[13] = ((omega0 & 0x000003fC) >> 2) as u8;
        //         dest[14] = (omega0 & 0x00000003) as u8;
        //         // TODO

        //         let cis = (subf.cis * 2.0_f64.powi(29)).round() as u32;

        //         dest[15] = ((cis & 0xff00) >> 8) as u8;
        //         dest[16] = (cis & 0x00ff) as u8;

        //         let i0 = (subf.i0 * 2.0_f64.powi(31)).round() as u32;

        //         dest[17] = ((i0 & 0xff000000) >> 24) as u8;
        //         dest[18] = ((i0 & 0x00c00000) >> 22) as u8; // TODO
        //         dest[19] = ((i0 & 0x003fc000) >> 14) as u8;
        //         dest[20] = ((i0 & 0x00003fc0) >> 6) as u8;
        //         dest[21] = ((i0 & 0x0000003f) >> 6) as u8;
        //         dest[21] <<= 2; // TODO

        //         let crc = (subf.crc * 2.0_f64.powi(5)).round() as u32;

        //         dest[22] |= ((crc & 0xf000) >> 12) as u8;
        //         dest[23] |= ((crc & 0x0ff0) >> 4) as u8;
        //         dest[24] |= (crc & 0x000f) as u8;

        //         let omega = (subf.omega * 2.0_f64.powi(31)).round() as u32;

        //         dest[24] |= ((omega & 0xf0000000) >> 28) as u8;
        //         dest[25] |= ((omega & 0x0f000000) >> 24) as u8; // TODO

        //         dest[26] = ((omega & 0x00fc0000) >> 18) as u8;
        //         dest[27] = ((omega & 0x0003fc00) >> 10) as u8;
        //         dest[28] = ((omega & 0x000003fc) >> 2) as u8;
        //         dest[29] = (omega & 0x00000003) as u8;
        //         dest[29] <<= 6; // TODO

        //         let omegadot = (subf.omega_dot * 2.0_f64.powi(43)).round() as u32;

        //         dest[30] = ((omegadot & 0x00ff0000) >> 16) as u8;
        //         dest[31] = ((omegadot & 0x0000ff00) >> 8) as u8;
        //         dest[32] = omegadot as u8;

        //         dest[33] = (subf.iode & 0xc0) >> 6; // TODO
        //         dest[34] = subf.iode & 0x3f;
        //         dest[34] <<= 2;

        //         let idot = (subf.idot * 2.0_f64.powi(43)).round() as u32;

        //         dest[34] |= ((idot & 0x3000) >> 12) as u8;
        //         dest[35] = ((idot & 0x0ff0) >> 4) as u8;
        //         dest[36] = (idot & 0xf) as u8;
        //         dest[36] <<= 4; // TODO
        //     },
        // }

        Ok(needed_size)
    }
}

impl GpsQzssFrame {
    /// Copies and returns with updated [GpsQzssHow].
    pub fn with_hand_over_word(mut self, how: GpsQzssHow) -> Self {
        self.how = how;
        self
    }

    /// Copies and returns with updated [GpsQzssTelemetry] data word
    pub fn with_telemetry(mut self, telemetry: GpsQzssTelemetry) -> Self {
        self.telemetry = telemetry;
        self
    }

    /// Copies and returns an updated [GpsQzssSubframe]
    pub fn with_subframe(mut self, subframe: GpsQzssSubframe) -> Self {
        self.subframe = subframe;

        match subframe {
            GpsQzssSubframe::Ephemeris1(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris1,
            GpsQzssSubframe::Ephemeris2(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris2,
            GpsQzssSubframe::Ephemeris3(_) => self.how.frame_id = GpsQzssFrameId::Ephemeris3,
        }

        self
    }

    #[cfg(test)]
    pub fn model(frame_id: GpsQzssFrameId) -> Self {
        Self::default()
            .with_telemetry(GpsQzssTelemetry::model())
            .with_hand_over_word(GpsQzssHow::model(frame_id))
            .with_subframe(GpsQzssSubframe::model(frame_id))
    }
}
