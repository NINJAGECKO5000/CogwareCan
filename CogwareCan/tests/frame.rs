//! CAN framing and spec-table consistency.

use cogware_can::*;
use embedded_hal_0_2::can::{Frame, Id, StandardId};
use mcp2515::frame::CanFrame;

// Gauges are global statics; keep tests in this binary from interleaving.
mod common;
use common::bus;

fn std_frame(id: u16, data: &[u8]) -> CanFrame {
    CanFrame::new(Id::Standard(StandardId::new(id).unwrap()), data).unwrap()
}

#[test]
fn wire_roundtrips_every_width() {
    let cases: &[(Wire, i32)] = &[
        (Wire::U8, 200),
        (Wire::I8, -100),
        (Wire::U16, 60000),
        (Wire::I16, -1234),
        (Wire::I32, -70_000),
    ];
    for &(wire, v) in cases {
        let mut buf = [0u8; 4];
        let n = wire.encode(v, &mut buf);
        assert_eq!(n, wire.len());
        assert_eq!(wire.decode(&buf[..n]), Some(v), "{wire:?}");
    }
}

#[test]
fn wire_rejects_wrong_length_and_clamps() {
    assert_eq!(Wire::U16.decode(&[1]), None);
    assert_eq!(Wire::U16.decode(&[1, 2, 3]), None);
    let mut buf = [0u8; 4];
    Wire::U8.encode(300, &mut buf);
    assert_eq!(buf[0], 255);
    Wire::I16.encode(-40_000, &mut buf);
    assert_eq!(Wire::I16.decode(&buf[..2]), Some(i16::MIN as i32));
}

#[test]
fn two_byte_gauge_survives_the_frame() {
    let _bus = bus();
    PULSE_WIDTH2.set(0x1234);
    let f = PULSE_WIDTH2.to_frame().unwrap();
    assert_eq!(f.dlc(), 2);
    PULSE_WIDTH2.clear();
    assert_eq!(feed_frame(&f).unwrap().id, PULSE_WIDTH2.id);
    assert_eq!(PULSE_WIDTH2.get(), Some(0x1234));
}

#[test]
fn signed_gauge_keeps_its_sign_over_the_bus() {
    let _bus = bus();
    RPM_DOT.set(-100);
    let f = RPM_DOT.to_frame().unwrap();
    RPM_DOT.clear();
    feed_frame(&f).unwrap();
    assert_eq!(RPM_DOT.get(), Some(-100));
}

#[test]
fn oversized_frame_is_rejected_not_panicked() {
    let _bus = bus();
    RPM.set(1000);
    let f = std_frame(RPM.id, &[1, 2, 3, 4, 5, 6, 7, 8]);
    assert_eq!(
        feed_frame(&f),
        Err(FeedError::BadLength { id: RPM.id, expected: 2, got: 8 })
    );
    assert_eq!(RPM.get(), Some(1000), "bad frame must not touch the value");
}

#[test]
fn non_gauge_frames_are_rejected() {
    assert_eq!(feed_frame(&std_frame(0x7DF, &[0])), Err(FeedError::UnknownId(0x7DF)));
    let ack = protocol::ack_frame(0x2D).unwrap();
    assert!(protocol::is_ack(&ack));
    assert_eq!(feed_frame(&ack), Err(FeedError::UnknownId(protocol::MASTER_ACK_ID)));
    let req = protocol::request_frame(&[0x20, 0x2D]).unwrap();
    assert!(protocol::is_request(&req));
    assert_eq!(&req.data()[..req.dlc()], &[0x20, 0x2D]);
}

#[test]
fn id_map_is_partitioned() {
    assert!(protocol::is_reserved(protocol::MASTER_ACK_ID));
    assert!(protocol::is_reserved(protocol::CLIENT_REQUEST_ID));
    assert!(protocol::is_reserved(xfer::CMD_ID));
    assert!(protocol::is_reserved(xfer::DATA_ID));
    assert!(protocol::is_reserved(xfer::REPLY_ID));
    assert!(!protocol::is_gauge_range(protocol::RESERVED_ID_MAX));
    assert!(protocol::is_gauge_range(protocol::GAUGE_ID_MIN));
    assert!(!protocol::is_gauge_range(obd2::REQUEST_ID));
    let ctrl = [protocol::MASTER_ACK_ID, protocol::CLIENT_REQUEST_ID, xfer::CMD_ID, xfer::DATA_ID, xfer::REPLY_ID];
    let mut sorted = ctrl.to_vec();
    sorted.sort();
    sorted.dedup();
    assert_eq!(sorted.len(), ctrl.len(), "control IDs collide");
    for g in ALL_GAUGES {
        assert!(!protocol::is_reserved(g.id), "{} sits in the reserved range", g.name);
    }
}

#[test]
fn unset_gauge_produces_no_frame() {
    let _bus = bus();
    OIL_PRES.clear();
    assert!(OIL_PRES.to_frame().is_none());
    assert!(frame_for(OIL_PRES.id).is_none());
    assert!(frame_for(0x7DF).is_none());
}

#[test]
fn as_f32_applies_unit_scale() {
    let _bus = bus();
    CLNT.set(875);
    assert_eq!(CLNT.as_f32(), Some(87.5));
    CLNT.clear();
    assert_eq!(CLNT.as_f32(), None);
    assert_eq!(CLNT.get_or(-1), -1);
}

#[test]
fn table_enum_and_statics_agree() {
    let mut seen = std::collections::HashSet::new();
    for g in ALL_GAUGES {
        assert!(seen.insert(g.id), "duplicate CAN ID 0x{:X} ({})", g.id, g.name);
        let e = Gauge::from_id(g.id).unwrap_or_else(|| panic!("{} missing from enum", g.name));
        assert!(std::ptr::eq(e.data(), *g), "{} enum points at a different static", g.name);
        assert_eq!(e.id(), g.id);
        assert_eq!(gauge_by_id(g.id).unwrap().name, g.name);
        assert!(
            (protocol::GAUGE_ID_MIN..=protocol::GAUGE_ID_MAX).contains(&g.id),
            "{} id 0x{:X} outside gauge range",
            g.name,
            g.id
        );
    }
    assert!(Gauge::from_id(protocol::CLIENT_REQUEST_ID).is_none());
}
