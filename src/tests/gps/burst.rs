use crate::{
    gps::{
        GpsQzssDecoder, GpsQzssFrame, GpsQzssFrame1, GpsQzssFrame2, GpsQzssFrame3, GpsQzssFrameId,
        GpsQzssSubframe, GpsQzssTelemetry,
    },
    tests::init_logger,
    Message,
};

#[test]
fn generate_burst_bin() {
    let mut fd = File::create("data/GPS/burst.bin").unwrap_or_else(|e| {
        panic!("Failed to create file: {}", e);
    });

    let mut eph_1 = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris1);
    let mut eph_2 = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris2);
    let mut eph_3 = GpsQzssFrame::model(GpsQzssFrameId::Ephemeris3);

    for i in 0..128 {
        // EPH-1
        let encoded = eph_1.encode_raw();
        fd.write(&encoded).unwrap_or_else(|e| {
            panic!("Failed to write eph-1 #{}: {}", i, e);
        });

        // EPH-2
        let encoded = eph_2.encode_raw();
        fd.write(&encoded).unwrap_or_else(|e| {
            panic!("Failed to write eph-2 #{}: {}", i, e);
        });

        // EPH-3
        let encoded = eph_3.encode_raw();
        fd.write(&encoded).unwrap_or_else(|e| {
            panic!("Failed to write eph-3 #{}: {}", i, e);
        });

        eph_1.telemetry.message += 1;
        eph_2.telemetry.message += 1;
        eph_3.telemetry.message += 1;

        eph_1.telemetry.integrity = !eph_1.telemetry.integrity;
        eph_2.telemetry.integrity = !eph_2.telemetry.integrity;
        eph_3.telemetry.integrity = !eph_3.telemetry.integrity;

        eph_1.telemetry.reserved_bit = !eph_1.telemetry.reserved_bit;
        eph_2.telemetry.reserved_bit = !eph_2.telemetry.reserved_bit;
        eph_3.telemetry.reserved_bit = !eph_3.telemetry.reserved_bit;

        eph_1.how.tow += 1;
        eph_2.how.tow += 1;
        eph_3.how.tow += 1;

        eph_1.how.alert = !eph_1.how.alert;
        eph_2.how.alert = !eph_2.how.alert;
        eph_3.how.alert = !eph_3.how.alert;

        eph_1.how.anti_spoofing = !eph_1.how.anti_spoofing;
        eph_2.how.anti_spoofing = !eph_2.how.anti_spoofing;
        eph_3.how.anti_spoofing = !eph_3.how.anti_spoofing;

        let subf1 = eph_1.subframe.as_mut_eph1().unwrap();
        let subf2 = eph_2.subframe.as_mut_eph2().unwrap();
        let subf3 = eph_3.subframe.as_mut_eph3().unwrap();

        subf1.ura += 1;
        subf1.week += 1;
        subf1.ca_or_p_l2 += 1;
        subf1.toc += 1;
        subf1.tgd += 1.0e-9;
        subf1.af0 += 1.0e-9;
        subf1.af1 += 1.0e-11;
        subf1.af2 += 1.0e-15;
        subf1.reserved_word4 += 1;
        subf1.reserved_word5 += 1;
        subf1.reserved_word6 += 1;
        subf1.reserved_word7 += 1;
        subf1.l2_p_data_flag = !subf1.l2_p_data_flag;

        subf2.toe += 16;
        subf2.aodo += 1;
        subf2.cuc += 1e-6;
        subf2.cus += 1e-6;
        subf2.e += 0.001;
        subf2.dn += 1e-9;
        subf2.iode += 1;
        subf2.fit_int_flag = !subf2.fit_int_flag;

        subf3.iode += 1;
        subf3.crc += 1.0;
        subf3.cic += 1.0e-6;
        subf3.cis += 1.0e-6;
        subf3.i0 += 0.001;
        subf3.idot += 1e-9;
    }
}
