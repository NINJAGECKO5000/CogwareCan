// SPDX-License-Identifier: GPL-3.0-only
//! Speeduino serial packet -> canonical gauges.

use cogware_can::speeduino::*;
use cogware_can::*;

mod common;
use common::bus;

/// A realtime payload in the master (202504) layout with recognisable values.
fn payload() -> [u8; FULL_PAYLOAD_LEN] {
    let mut d = [0u8; FULL_PAYLOAD_LEN];
    d[0] = 42; // secl
    d[4..6].copy_from_slice(&45u16.to_le_bytes()); // MAP 45 kPa
    d[6] = 25 + 40; // IAT 25 °C
    d[7] = 87 + 40; // CLT 87 °C
    d[9] = 138; // 13.8 V
    d[10] = 147; // AFR 14.7
    d[11] = 97; // egoCorrection 97 % = -3 % trim
    d[14..16].copy_from_slice(&3500u16.to_le_bytes()); // RPM
    d[16] = 15; // accelEnrich 30 % (2 % steps)
    d[17..19].copy_from_slice(&105u16.to_le_bytes()); // gammaEnrich 105 %
    d[22..24].copy_from_slice(&(-25i16).to_le_bytes()); // TPSdot -25 %/s
    d[24] = (-3i8) as u8; // advance -3°
    d[25] = 100; // TPS 50.0 % at 0.5 % per count
    d[33..35].copy_from_slice(&(-250i16).to_le_bytes()); // rpmDOT
    d[76..78].copy_from_slice(&2750u16.to_le_bytes()); // PW1 2.750 ms
    d[78..80].copy_from_slice(&2500u16.to_le_bytes()); // PW2
    d[90..92].copy_from_slice(&3500u16.to_le_bytes()); // dwell 3.5 ms
    d[95..97].copy_from_slice(&(-10i16).to_le_bytes()); // vvt1 -5.0°
    d[104..106].copy_from_slice(&88u16.to_le_bytes()); // VSS 88 km/h
    d[107] = 43; // fuel 43 psi
    d[120] = 0x5A; // SD status
    d
}

fn n_packet(data: &[u8]) -> Vec<u8> {
    let mut p = vec![b'n', 0x32, data.len() as u8];
    p.extend_from_slice(data);
    p
}

#[test]
fn n_packet_normalises_into_canonical_units() {
    let _bus = bus();
    let written = parse_n(&n_packet(&payload())).unwrap();
    assert_eq!(written, 69, "every mapped field should be written");

    assert_eq!(STA_TIME.get(), Some(42));
    assert_eq!(MAP.get(), Some(450));
    assert_eq!(IAT.as_f32(), Some(25.0));
    assert_eq!(CLNT.as_f32(), Some(87.0));
    assert_eq!(BAT_VOL.get(), Some(13_800));
    assert_eq!(AFR_PRI.as_f32(), Some(14.7));
    assert_eq!(EGO_CORRECT.as_f32(), Some(-3.0));
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(ACCEL_ENRICH.as_f32(), Some(30.0));
    assert_eq!(GAMME_E.as_f32(), Some(105.0));
    assert_eq!(TPS_DOT.as_f32(), Some(-25.0));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(-3.0));
    assert_eq!(TPS.as_f32(), Some(50.0));
    assert_eq!(RPM_DOT.get(), Some(-250));
    assert_eq!(PULSE_WIDTH1.as_f32(), Some(2.75));
    assert_eq!(PULSE_WIDTH2.get(), Some(2500));
    assert_eq!(DWELL.as_f32(), Some(3.5));
    assert_eq!(VVT_ANGLE.as_f32(), Some(-5.0));
    assert_eq!(VSS.as_f32(), Some(88.0));
    assert_eq!(FUEL_PRES.get(), Some(2965)); // 43 psi = 296.48 kPa, rounded
    assert_eq!(SD_STA.get(), Some(0x5A));
    // Channels no firmware broadcasts stay unset
    assert_eq!(INJ_ANGLE.get(), None);
    assert_eq!(IDLE_DUTY.get(), None);
    assert_eq!(STA_LAUNCH_CORRECT.get(), None);
}

#[test]
fn cold_intake_does_not_underflow() {
    let _bus = bus();
    let mut d = payload();
    d[6] = 30; // -10 °C
    parse_n(&n_packet(&d)).unwrap();
    assert_eq!(IAT.as_f32(), Some(-10.0));
}

#[test]
fn short_payload_fills_only_what_it_carries() {
    let _bus = bus();
    let d = payload();
    let written = parse_n(&n_packet(&d[..78])).unwrap();
    assert!(written < 69);
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(PULSE_WIDTH1.get(), Some(2750)); // offset 76..78, last bytes present
    assert_eq!(PULSE_WIDTH2.get(), None); // offset 78, absent
    assert_eq!(SD_STA.get(), None);
}

#[test]
fn explicit_layout_selection() {
    let _bus = bus();
    assert_eq!(speeduino::Layout::LATEST.payload_len(), FULL_PAYLOAD_LEN);
    parse_n_with(speeduino::Layout::V202504, &n_packet(&payload())).unwrap();
    assert_eq!(RPM.get(), Some(3500));
}

#[test]
fn n_header_is_validated() {
    let _bus = bus();
    let d = payload();
    assert_eq!(parse_n(&[b'n', 0x32]), Err(SpeeduinoError::Truncated));
    assert_eq!(parse_n(&[b'A', 0x32, 1, 0]), Err(SpeeduinoError::BadCommand(b'A')));
    assert_eq!(parse_n(&[b'n', 0x31, 1, 0]), Err(SpeeduinoError::BadFormat(0x31)));
    let mut p = n_packet(&d);
    p.truncate(N_HEADER_LEN + 10);
    assert_eq!(
        parse_n(&p),
        Err(SpeeduinoError::ShortPayload { declared: FULL_PAYLOAD_LEN, got: 10 })
    );
    assert_eq!(n_payload_len(&[b'n', 0x32, 119]), Ok(119));
    // Trailing bytes after the declared length are ignored, not an error.
    p = n_packet(&d);
    p.push(0xFF);
    assert!(parse_n(&p).is_ok());
}

#[test]
fn a_packet_uses_one_byte_header() {
    let _bus = bus();
    let mut p = vec![b'A'];
    p.extend_from_slice(&payload());
    parse_a(&p).unwrap();
    assert_eq!(STA_TIME.get(), Some(42));
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(parse_a(&[]), Err(SpeeduinoError::Truncated));
    assert_eq!(parse_a(b"n"), Err(SpeeduinoError::BadCommand(b'n')));
}
