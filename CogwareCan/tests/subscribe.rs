// SPDX-License-Identifier: GPL-3.0-only
//! Gauge subscriptions: what displays ask for and what the master sends back.

use cogware_can::subscribe::*;
use cogware_can::*;
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

mod common;
use common::bus;

const WANTED: [u8; 9] = [0x2D, 0x24, 0x26, 0x25, 0x29, 0x28, 0x35, 0x57, 0x70];

fn ids_of(f: &CanFrame) -> Vec<u8> {
    f.data()[..f.dlc()].to_vec()
}

#[test]
fn id_set_holds_any_gauge_id() {
    let mut s = IdSet::new();
    assert!(s.is_empty());
    assert!(s.insert(0x2D));
    assert!(!s.insert(0x2D), "second insert is not new");
    assert!(s.insert(0xFF));
    assert!(s.insert(0x00));
    assert_eq!(s.len(), 3);
    assert!(s.contains(0xFF) && !s.contains(0x2E));
    // Iteration is in ID order, which is CAN arbitration order.
    assert_eq!(s.iter().collect::<Vec<_>>(), vec![0x00, 0x2D, 0xFF]);
    s.remove(0x2D);
    assert_eq!(s.len(), 2);
    s.clear();
    assert!(s.is_empty());
    // Every one-byte ID fits, so there is no capacity to overflow.
    let mut all = IdSet::new();
    for id in 0..=255u8 {
        all.insert(id);
    }
    assert_eq!(all.len(), 256);
}

#[test]
fn publisher_records_and_acknowledges() {
    let _bus = bus();
    let mut p = Publisher::new();
    assert!(p.is_empty());

    let acks: Vec<CanFrame> = p.feed(&protocol::request_frame(&WANTED[..8]).unwrap()).collect();
    assert_eq!(acks.len(), 8, "one ack per requested ID");
    assert!(acks.iter().all(protocol::is_ack));
    assert_eq!(acks.iter().map(|f| f.data()[0]).collect::<Vec<_>>(), WANTED[..8].to_vec());
    assert_eq!(p.len(), 8);
    assert!(p.wanted().contains(0x2D));

    // A repeat request is acknowledged again, so a display that missed an ack
    // can retry, but nothing is recorded twice.
    let again: Vec<CanFrame> = p.feed(&protocol::request_frame(&WANTED[..8]).unwrap()).collect();
    assert_eq!(again.len(), 8);
    assert_eq!(p.len(), 8);
}

#[test]
fn publisher_ignores_everything_that_is_not_a_request() {
    let _bus = bus();
    let mut p = Publisher::new();
    for f in [
        protocol::ack_frame(0x2D).unwrap(),
        protocol::frame(Gauge::Rpm.id(), &[0, 1]).unwrap(),
        obd2::request(0x0C).unwrap(),
        protocol::frame(cogware_can::xfer::DATA_ID, &[0, 0, 1, 2, 3, 4]).unwrap(),
    ] {
        assert_eq!(p.feed(&f).count(), 0);
    }
    assert!(p.is_empty());
}

#[test]
fn unknown_ids_are_acknowledged_but_never_broadcast() {
    let _bus = bus();
    let mut p = Publisher::new();
    // 0xF0 names no gauge in this build, as it would not in a display running
    // an older spec. The master still answers, so that display stops asking.
    let acks: Vec<CanFrame> = p.feed(&protocol::request_frame(&[0x2D, 0xF0]).unwrap()).collect();
    assert_eq!(acks.len(), 2);
    assert_eq!(p.len(), 1, "only the real gauge is recorded");
    assert!(!p.wanted().contains(0xF0));
    assert!(!p.subscribe(0xF0));
    assert!(p.subscribe(0x24));
}

#[test]
fn broadcast_skips_gauges_that_were_never_written() {
    let _bus = bus();
    let mut p = Publisher::new();
    p.feed(&protocol::request_frame(&WANTED[..8]).unwrap()).count();

    assert_eq!(p.broadcast().count(), 0, "nothing set yet, so nothing to send");

    RPM.set(3500);
    MAP.set(950);
    let frames: Vec<CanFrame> = p.broadcast().collect();
    assert_eq!(frames.len(), 2);
    // Lowest ID first: MAP is 0x24, RPM is 0x2D.
    assert_eq!(
        frames.iter().map(|f| protocol::id_of(f).unwrap()).collect::<Vec<_>>(),
        vec![Gauge::Map.id(), Gauge::Rpm.id()]
    );
    // And the frames carry the values a display would read back.
    clear_all();
    for f in &frames {
        feed_frame(f).unwrap();
    }
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(MAP.as_f32(), Some(95.0));
}

#[test]
fn subscription_asks_in_frame_sized_chunks() {
    let mut s = Subscription::new(&WANTED);
    assert_eq!(s.pending(), 9);
    assert!(!s.is_complete());

    let reqs: Vec<CanFrame> = s.request().collect();
    assert_eq!(reqs.len(), 2, "nine IDs need two frames");
    assert!(reqs.iter().all(protocol::is_request));
    assert_eq!(ids_of(&reqs[0]), WANTED[..8].to_vec());
    assert_eq!(ids_of(&reqs[1]), WANTED[8..].to_vec());

    // Acknowledging shrinks the retry, so a resend does not re-ask for
    // everything the master already heard.
    for id in &WANTED[..8] {
        assert!(s.feed(&protocol::ack_frame(*id).unwrap()));
    }
    assert_eq!(s.pending(), 1);
    let retry: Vec<CanFrame> = s.request().collect();
    assert_eq!(retry.len(), 1);
    assert_eq!(ids_of(&retry[0]), vec![WANTED[8]]);

    assert!(s.feed(&protocol::ack_frame(WANTED[8]).unwrap()));
    assert!(s.is_complete());
    assert_eq!(s.request().count(), 0, "nothing left to ask for");
}

#[test]
fn subscription_ignores_acks_it_did_not_ask_for() {
    let mut s = Subscription::new(&WANTED);
    assert!(!s.feed(&protocol::ack_frame(0x5A).unwrap()), "not on our list");
    assert!(!s.feed(&protocol::request_frame(&[0x2D]).unwrap()), "not an ack");
    assert!(!s.feed(&protocol::frame(Gauge::Rpm.id(), &[0, 1]).unwrap()));
    assert_eq!(s.pending(), 9);
    assert!(s.feed(&protocol::ack_frame(0x2D).unwrap()));
    assert!(!s.feed(&protocol::ack_frame(0x2D).unwrap()), "already acknowledged");
    assert_eq!(s.pending(), 8);
    s.reset();
    assert_eq!(s.pending(), 9);
}

#[test]
fn display_and_master_agree_over_a_lossy_bus() {
    let _bus = bus();
    let mut master = Publisher::new();
    let mut display = Subscription::new(&WANTED);

    // Drop every third frame in each direction until the display is satisfied.
    let mut n = 0;
    let mut rounds = 0;
    while !display.is_complete() {
        rounds += 1;
        assert!(rounds < 20, "subscription never converged");
        for req in display.request() {
            n += 1;
            if n % 3 == 0 {
                continue; // request lost
            }
            for ack in master.feed(&req) {
                n += 1;
                if n % 3 == 0 {
                    continue; // ack lost
                }
                display.feed(&ack);
            }
        }
    }
    assert!(rounds > 1, "the lossy bus should have forced a retry");
    assert_eq!(master.len(), WANTED.len());

    // Now the master publishes and the display reads the values back.
    RPM.set(3500);
    CLNT.set(870);
    VSS.set(880);
    MASTERALIVE.set(7);
    let wire: Vec<CanFrame> = master.broadcast().collect();
    clear_all();
    for f in &wire {
        assert!(feed_frame(f).is_ok());
    }
    assert_eq!(RPM.get(), Some(3500));
    assert_eq!(CLNT.as_f32(), Some(87.0));
    assert_eq!(VSS.to(Unit::MPH).unwrap().round(), 55.0);
    assert_eq!(MASTERALIVE.get(), Some(7));
    assert_eq!(MAP.get(), None, "subscribed but never set by the ECU");
}

#[test]
fn the_guard_isolates_tests_from_each_other() {
    // Whatever a test leaves behind is gone before the next one starts, so
    // no test depends on which tests ran before it.
    {
        let _bus = bus();
        RPM.set(1234);
        MASTERALIVE.set(9);
        assert_eq!(ALL_GAUGES.iter().filter(|g| g.is_set()).count(), 2);
    }
    let _bus = bus();
    let dirty: Vec<&str> = ALL_GAUGES.iter().filter(|g| g.is_set()).map(|g| g.name).collect();
    assert!(dirty.is_empty(), "state survived the guard: {dirty:?}");
}
