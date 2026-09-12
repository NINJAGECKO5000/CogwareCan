// SPDX-License-Identifier: GPL-3.0-only
//! OBD2 mode 01 request/response.

use cogware_can::obd2::*;
use cogware_can::*;
use embedded_hal_0_2::can::Frame;

mod common;
use common::bus;

fn response(pid: u8, data: &[u8]) -> mcp2515::frame::CanFrame {
    let mut d = vec![2 + data.len() as u8, 0x41, pid];
    d.extend_from_slice(data);
    d.resize(8, 0x55);
    protocol::frame(0x7E8, &d).unwrap()
}

#[test]
fn request_frame_shape() {
    let f = request(0x0C).unwrap();
    assert_eq!(protocol::id_of(&f), Some(REQUEST_ID));
    assert_eq!(&f.data()[..3], &[0x02, 0x01, 0x0C]);
    assert_eq!(f.dlc(), 8);
    assert!(!protocol::is_gauge_range(REQUEST_ID));
}

#[test]
fn common_pids_decode() {
    let _bus = bus();
    assert_eq!(feed_response(&response(0x0C, &[0x36, 0xB0])), Some(1)); // 14000/4
    assert_eq!(RPM.get(), Some(3500));
    feed_response(&response(0x05, &[127]));
    assert_eq!(CLNT.as_f32(), Some(87.0));
    feed_response(&response(0x0B, &[45]));
    assert_eq!(MAP.as_f32(), Some(45.0));
    feed_response(&response(0x11, &[128]));
    assert_eq!(TPS.get(), Some(502)); // 128*100/255 = 50.2
    feed_response(&response(0x0E, &[153]));
    assert_eq!(CUR_SPARK_ADVANCE.as_f32(), Some(12.5)); // 153/2 - 64
    feed_response(&response(0x0D, &[88]));
    assert_eq!(VSS.as_f32(), Some(88.0));
    feed_response(&response(0x42, &[0x35, 0xE8]));
    assert_eq!(BAT_VOL.get(), Some(13_800));
    feed_response(&response(0x44, &[0x80, 0x00]));
    assert_eq!(AFR_TARGET.as_f32(), Some(14.7)); // lambda 1.0
    feed_response(&response(0x06, &[128]));
    assert_eq!(EGO_CORRECT.get(), Some(0)); // 128/1.28 - 100
    feed_response(&response(0x06, &[144]));
    assert_eq!(EGO_CORRECT.as_f32(), Some(12.5));
    feed_response(&response(0x01, &[0x83, 0, 0, 0]));
    assert_eq!(ERROR_COUNT.get(), Some(3)); // MIL on, 3 codes
    feed_response(&response(0x23, &[0x01, 0x2C]));
    assert_eq!(FUEL_PRES.as_f32(), Some(3000.0)); // 300*10 kPa
    feed_response(&response(0x78, &[0x01, 0x20, 0xD0]));
    assert_eq!(EGT1.as_f32(), Some(800.0)); // 8400/10 - 40
    feed_response(&response(0x1F, &[0x00, 0x78]));
    assert_eq!(STA_TIME.get(), Some(120));
    feed_response(&response(0x5C, &[137]));
    assert_eq!(OIL_TEMP.as_f32(), Some(97.0));
}

#[test]
fn rejects_non_responses() {
    let _bus = bus();
    // wrong ID, wrong mode, unknown PID, negative response, too short
    assert_eq!(feed_response(&protocol::frame(0x360, &[4, 0x41, 0x0C, 0x36, 0xB0]).unwrap()), None);
    assert_eq!(feed_response(&protocol::frame(0x7E8, &[4, 0x42, 0x0C, 0x36, 0xB0]).unwrap()), None);
    assert_eq!(feed_response(&response(0x99, &[1, 2])), None);
    assert_eq!(feed_response(&protocol::frame(0x7E8, &[3, 0x7F, 0x01, 0x12]).unwrap()), None);
    assert_eq!(feed_response(&protocol::frame(0x7E8, &[1, 0x41]).unwrap()), None);
    assert_eq!(RPM.get(), None);
    // Declared length shorter than the payload needed: field not written
    assert_eq!(feed_response(&protocol::frame(0x7E8, &[3, 0x41, 0x0C, 0x36, 0xB0]).unwrap()), Some(0));
}

#[test]
fn pids_are_unique_and_only_feed_common_gauges() {
    let mut seen = std::collections::HashSet::new();
    for p in PIDS {
        assert!(seen.insert(p.pid), "duplicate PID {:#x}", p.pid);
        for f in p.fields {
            assert_eq!(f.gauge.source, Source::Common, "PID {:#x} feeds {}", p.pid, f.gauge.name);
        }
    }
}
