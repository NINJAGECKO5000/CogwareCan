// SPDX-License-Identifier: GPL-3.0-only
//! Global mode, the update-in-progress flag, and the node message channel.

use cogware_can::bus::*;
use cogware_can::protocol::{self, NODE_BROADCAST};
use cogware_can::xfer;
use embedded_hal_0_2::can::Frame;

const MODE_SPORT: u8 = 1;
const MODE_TRACK: u8 = 2;

#[test]
fn a_node_speaks_on_its_own_id() {
    // The message ID *is* the node address, so two nodes can never collide on
    // one ID and the lower address wins arbitration.
    for node in 1..=protocol::NODE_MAX {
        let f = message(node, Opcode::Hello, &[]).unwrap();
        assert_eq!(protocol::id_of(&f), Some(node as u16));
        assert_eq!(NodeMsg::parse(&f).unwrap().node, node);
    }
    // The master and out-of-range addresses own no message ID.
    assert!(message(protocol::NODE_MASTER, Opcode::Hello, &[]).is_none());
    assert!(message(protocol::NODE_MAX + 1, Opcode::Hello, &[]).is_none());
    assert!(message(NODE_BROADCAST, Opcode::Hello, &[]).is_none());
    // Eight args plus the opcode would not fit a frame.
    assert!(message(1, Opcode::Error, &[0; 7]).is_some());
    assert!(message(1, Opcode::Error, &[0; 8]).is_none());
}

#[test]
fn a_node_asks_for_a_mode_and_the_master_announces_it() {
    let mut master = Bus::new();
    let mut display = BusView::new(3);
    assert_eq!(master.mode(), MODE_NORMAL);
    assert!(!display.is_synced(), "no state seen yet");

    let request = display.request_mode(MODE_SPORT).unwrap();
    let msg = master.feed(&request).unwrap();
    assert_eq!((msg.node, msg.opcode, msg.mode()), (3, Opcode::ModeRequest, Some(MODE_SPORT)));
    assert_eq!(master.mode(), MODE_SPORT);

    let state = master.announce().expect("the change is worth announcing");
    assert!(display.feed(&state));
    assert_eq!(display.mode(), MODE_SPORT);
    assert!(display.is_synced());
}

#[test]
fn the_state_is_announced_once_per_change() {
    let mut master = Bus::new();
    master.set_mode(MODE_TRACK);
    assert!(master.announce().is_some());
    assert!(master.announce().is_none(), "nothing changed since");

    // Asking for the mode it is already in is not a change.
    assert!(!master.set_mode(MODE_TRACK));
    assert!(master.announce().is_none());

    // A node that just booted asks for the state again.
    let msg = message(1, Opcode::Hello, &[]).unwrap();
    master.feed(&msg);
    assert!(master.announce().is_some());
}

#[test]
fn an_update_flips_the_flag_for_every_node_not_being_written() {
    let mut master = Bus::new();
    let mut target = BusView::new(2);
    let mut bystander = BusView::new(5);

    master.update_started(2);
    let state = master.announce().unwrap();
    for view in [&mut target, &mut bystander] {
        view.feed(&state);
        assert!(view.update_in_progress(), "every node knows why the bus is busy");
    }
    assert!(target.update_is_mine());
    assert!(!bystander.update_is_mine(), "it is not this node's image");

    master.update_finished();
    let state = master.announce().unwrap();
    target.feed(&state);
    bystander.feed(&state);
    assert!(!target.update_in_progress() && !bystander.update_in_progress());
}

#[test]
fn a_broadcast_update_is_every_nodes_business() {
    let mut master = Bus::new();
    master.update_started(NODE_BROADCAST);
    let state = master.announce().unwrap();
    let mut view = BusView::new(7);
    view.feed(&state);
    assert!(view.update_is_mine());
}

#[test]
fn the_mode_is_locked_while_an_update_runs() {
    let mut master = Bus::new();
    master.update_started(1);
    assert!(!master.set_mode(MODE_SPORT), "a scene must not swap mid-transfer");
    assert_eq!(master.mode(), MODE_NORMAL);

    // A node asking anyway is refused, but still told the mode it is in.
    master.feed(&message(4, Opcode::ModeRequest, &[MODE_TRACK]).unwrap());
    assert_eq!(master.mode(), MODE_NORMAL);
    let state = master.announce().expect("the asker is answered");
    assert_eq!(BusState::parse(&state).unwrap().mode, MODE_NORMAL);

    master.update_finished();
    assert!(master.set_mode(MODE_SPORT));
}

#[test]
fn progress_only_moves_the_state_a_whole_percent_at_a_time() {
    let mut master = Bus::new();
    master.update_started(1);
    master.announce();

    master.update_progress(0, 1000);
    assert!(master.announce().is_none(), "still zero percent");
    master.update_progress(9, 1000);
    assert!(master.announce().is_none(), "under one percent");
    master.update_progress(10, 1000);
    let f = master.announce().expect("one percent is worth a frame");
    assert_eq!(BusState::parse(&f).unwrap().progress, 1);

    master.update_progress(2000, 1000);
    assert_eq!(master.state().progress, 100, "done is clamped, never over");
}

#[test]
fn a_sender_percentage_feeds_the_announced_progress() {
    let image = [0xAAu8; 60];
    let mut store = [0u8; 64];
    let mut tx = xfer::Sender::new(1, &image, xfer::kind::SCENE, 64, 1);
    let mut rx = xfer::Receiver::new(1, xfer::BufferSink::new(&mut store));
    assert_eq!(tx.percent(), 0);

    // Drive the whole session; the announced percentage tracks what the node
    // has acknowledged, not what has merely been put on the wire.
    let mut bus = Bus::new();
    bus.update_started(1);
    for _ in 0..64 {
        if let Some(f) = tx.next_frame() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
        bus.update_progress(tx.progress().0 as u32, tx.progress().1 as u32);
        if tx.state() == xfer::TxState::Done {
            break;
        }
    }
    assert_eq!(tx.state(), xfer::TxState::Done);
    assert_eq!(tx.percent(), 100);
    assert_eq!(bus.state().progress, 100);
    assert_eq!(tx.image(), xfer::Image { size: 60, kind: xfer::kind::SCENE, slot: 0 });
}

#[test]
fn errors_and_offers_reach_the_master_intact() {
    let mut master = Bus::new();

    let err = BusView::new(6).report_error(0x42).unwrap();
    let msg = master.feed(&err).unwrap();
    assert_eq!((msg.node, msg.error()), (6, Some(0x42)));
    assert!(master.announce().is_none(), "an error changes no state");

    let offer = BusView::new(9).offer_update(xfer::kind::SCENE, 4096).unwrap();
    let msg = master.feed(&offer).unwrap();
    assert_eq!(msg.offer(), Some((xfer::kind::SCENE, 4096)));

    // An accessor only answers for its own opcode.
    assert!(msg.mode().is_none() && msg.error().is_none());
}

#[test]
fn a_state_frame_round_trips_and_junk_is_rejected() {
    let state = BusState { mode: 3, flags: FLAG_UPDATE | FLAG_ECU_LINK, progress: 55, target: 2 };
    let f = state.to_frame().unwrap();
    assert_eq!(f.dlc(), STATE_LEN);
    assert_eq!(BusState::parse(&f), Some(state));
    assert!(state.update_in_progress() && state.ecu_link() && !state.mode_locked());

    // Not a state frame, and a truncated one, are both refused.
    assert!(BusState::parse(&protocol::ack_frame(0x2D).unwrap()).is_none());
    assert!(BusState::parse(&protocol::frame(protocol::BUS_STATE_ID, &[1, 2]).unwrap()).is_none());

    // A gauge frame is not a node message, and an unknown opcode is refused.
    let mut master = Bus::new();
    assert!(master.feed(&protocol::frame(0x2D, &[1, 2]).unwrap()).is_none());
    assert!(master.feed(&protocol::frame(1, &[0xEE]).unwrap()).is_none());
}
