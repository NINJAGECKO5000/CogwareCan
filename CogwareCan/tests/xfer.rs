// SPDX-License-Identifier: GPL-3.0-only
//! Master-to-node file transfer, driven frame by frame with a lossy "bus".

use cogware_can::protocol::NODE_BROADCAST;
use cogware_can::xfer::*;
use embedded_hal_0_2::can::Frame;
use cogware_can::{crc, protocol};
use mcp2515::frame::CanFrame;

fn image(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 7 % 251) as u8).collect()
}

/// Run a session to completion. `drop` decides which master frames (by
/// count) never reach the nodes. Returns the number of frames the master sent.
fn run(tx: &mut Sender, rxs: &mut [Receiver<BufferSink>], drop: &dyn Fn(usize) -> bool) -> usize {
    let mut sent = 0;
    let mut idle = 0;
    loop {
        match tx.state() {
            TxState::Done | TxState::Failed(_) => return sent,
            _ => {}
        }
        let f: Option<CanFrame> = tx.next_frame().or_else(|| {
            idle += 1;
            if idle > 3 { tx.resend() } else { None }
        });
        let Some(f) = f else {
            assert!(idle < 20, "sender stalled in {:?}", tx.state());
            continue;
        };
        idle = 0;
        sent += 1;
        if drop(sent) {
            continue;
        }
        for rx in rxs.iter_mut() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
    }
}

#[test]
fn clean_transfer_lands_and_commits() {
    let img = image(100);
    let mut buf = [0u8; 128];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 8, 1);
    let sent = run(&mut tx, std::slice::from_mut(&mut rx), &|_| false);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(rx.state(), RxState::Idle);
    assert!(rx.sink().committed);
    assert_eq!(rx.sink().len, 100);
    assert_eq!(&buf[..100], &img[..]);
    // 17 data blocks + begin + 3 checks + end + commit
    assert_eq!(sent, 17 + 1 + 3 + 1 + 1);
}

#[test]
fn lost_block_is_resent_from_the_gap() {
    let img = image(60);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 4, 1);
    // frame 1 = begin, 2..5 = blocks 0..3; drop block 1 (frame 3)
    run(&mut tx, std::slice::from_mut(&mut rx), &|n| n == 3);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(&buf[..60], &img[..]);
}

#[test]
fn lost_last_block_of_window_is_detected_by_check() {
    let img = image(60);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 4, 1);
    // drop block 3 (frame 5), the last one before the first check
    run(&mut tx, std::slice::from_mut(&mut rx), &|n| n == 5);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(&buf[..60], &img[..]);
}

#[test]
fn rewind_across_final_partial_block_is_exact() {
    let img = image(64); // 10 full blocks + 4 bytes
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 100, 1);
    // begin=1, blocks at 2..=12; drop block 9 (frame 11) so the rewind spans the partial block 10
    run(&mut tx, std::slice::from_mut(&mut rx), &|n| n == 11);
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(&buf[..64], &img[..]);
}

#[test]
fn corrupted_image_fails_crc_and_is_not_committed() {
    let img = image(30);
    let mut buf = [0u8; 32];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 100, 1);
    let corrupt = |f: &CanFrame| -> CanFrame {
        use embedded_hal_0_2::can::Frame;
        if protocol::id_of(f) == Some(DATA_ID) && f.data()[0] == 2 {
            let mut d = f.data()[..f.dlc()].to_vec();
            d[3] ^= 0xFF;
            protocol::frame(DATA_ID, &d).unwrap()
        } else {
            *f
        }
    };
    while !matches!(tx.state(), TxState::Done | TxState::Failed(_)) {
        if let Some(f) = tx.next_frame() {
            if let Some(r) = rx.feed(&corrupt(&f)) {
                tx.feed_reply(&r);
            }
        }
    }
    assert_eq!(tx.state(), TxState::Failed(Status::CrcFail));
    assert_eq!(rx.state(), RxState::Failed);
    assert!(!rx.sink().committed);
}

#[test]
fn node_refuses_oversized_image() {
    let img = image(200);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 8, 1);
    run(&mut tx, std::slice::from_mut(&mut rx), &|_| false);
    assert_eq!(tx.state(), TxState::Failed(Status::Refused));
    assert_eq!(rx.state(), RxState::Idle);
}

#[test]
fn other_nodes_ignore_a_session_not_addressed_to_them() {
    let img = image(20);
    let mut b7 = [0u8; 32];
    let mut b9 = [0u8; 32];
    let mut rxs = [Receiver::new(7, BufferSink::new(&mut b7)), Receiver::new(9, BufferSink::new(&mut b9))];
    let mut tx = Sender::new(7, &img, 1, 8, 1);
    run(&mut tx, &mut rxs, &|_| false);
    assert_eq!(tx.state(), TxState::Done);
    assert!(rxs[0].sink().committed);
    assert!(!rxs[1].sink().committed);
    assert_eq!(rxs[1].sink().len, 0);
}

#[test]
fn broadcast_updates_every_node_and_rewinds_for_the_slowest() {
    let img = image(90);
    let mut b7 = [0u8; 96];
    let mut b9 = [0u8; 96];
    let mut rxs = [Receiver::new(7, BufferSink::new(&mut b7)), Receiver::new(9, BufferSink::new(&mut b9))];
    let mut tx = Sender::new(NODE_BROADCAST, &img, 1, 5, 2);
    // Drop block 2 for everyone (frame 4); both nodes report it missing.
    run(&mut tx, &mut rxs, &|n| n == 4);
    assert_eq!(tx.state(), TxState::Done);
    assert!(rxs.iter_mut().all(|r| r.sink().committed));
    assert_eq!(&b7[..90], &img[..]);
    assert_eq!(&b9[..90], &img[..]);
}

#[test]
fn abort_resets_the_node() {
    let img = image(40);
    let mut buf = [0u8; 64];
    let mut rx = Receiver::new(7, BufferSink::new(&mut buf));
    let mut tx = Sender::new(7, &img, 1, 100, 1);
    let begin = tx.next_frame().unwrap();
    let ready = rx.feed(&begin).unwrap();
    tx.feed_reply(&ready);
    assert_eq!(rx.state(), RxState::Receiving);
    let abort = tx.abort().unwrap();
    let reply = rx.feed(&abort).unwrap();
    assert_eq!(rx.state(), RxState::Idle);
    tx.feed_reply(&reply);
    assert_eq!(tx.state(), TxState::Failed(Status::Aborted));
}

#[test]
fn crc_matches_reference() {
    assert_eq!(crc::crc32(b"123456789"), 0xCBF4_3926);
    let mut c = crc::Crc32::new();
    c.update(b"1234");
    c.update(b"56789");
    assert_eq!(c.finish(), 0xCBF4_3926);
}

#[test]
fn the_slot_and_kind_reach_the_sink() {
    let image = [0x5Au8; 40];
    let mut store = [0u8; 64];
    let mut tx = Sender::new(2, &image, kind::SCENE, 8, 1).with_slot(3);
    let mut rx = Receiver::new(2, BufferSink::new(&mut store));

    for _ in 0..64 {
        if let Some(f) = tx.next_frame() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
        if tx.state() == TxState::Done {
            break;
        }
    }
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(rx.image(), Image { size: 40, kind: kind::SCENE, slot: 3 });
    assert!(rx.sink().committed);
    assert_eq!(&rx.sink().buf[..40], &image[..]);
}

#[test]
fn a_begin_without_a_slot_byte_lands_in_slot_zero() {
    // Seven-byte Begin frames predate slots; a node must still accept one
    // rather than refusing the transfer outright.
    let mut store = [0u8; 16];
    let mut rx = Receiver::new(1, BufferSink::new(&mut store));
    let begin = protocol::frame(CMD_ID, &[1, Cmd::Begin as u8, 8, 0, 0, 0, kind::FIRMWARE]).unwrap();

    let reply = rx.feed(&begin).expect("the node answers");
    assert_eq!(reply.data()[1], Status::Ready as u8);
    assert_eq!(rx.image(), Image { size: 8, kind: kind::FIRMWARE, slot: 0 });
}
