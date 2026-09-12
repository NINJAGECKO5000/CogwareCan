// SPDX-License-Identifier: GPL-3.0-only
//! Over-the-air update: the master pushes an image to every node on the bus.
//!
//!     cargo run --example ota_update

use cogware_can::protocol::NODE_BROADCAST;
use cogware_can::xfer::*;
use cogware_can::{crc, protocol};

/// The image to push. In firmware this comes off an SD card or USB.
fn firmware_image(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 31 % 251) as u8).collect()
}

fn main() {
    let image = firmware_image(240);
    println!("image: {} bytes, crc32 {:#010x}", image.len(), crc::crc32(&image));
    println!("transfer IDs: cmd {:#05x}, data {:#05x}, reply {:#05x}\n", CMD_ID, DATA_ID, REPLY_ID);

    // Two displays on the bus, addresses 7 and 9. Each stages the image in a
    // buffer; real firmware would write to a spare flash bank instead.
    let mut buf7 = [0u8; 256];
    let mut buf9 = [0u8; 256];
    let mut nodes = [
        Receiver::new(7, BufferSink::new(&mut buf7)),
        Receiver::new(9, BufferSink::new(&mut buf9)),
    ];

    // Broadcast to every node, checkpoint every 8 blocks, expect 2 replies.
    let mut master = Sender::new(NODE_BROADCAST, &image, 1, 8, 2);

    // Drop a few frames to show the recovery path. Block 3 and block 11 are
    // lost on the way out, as they would be on a noisy bus.
    let lost = [5usize, 14];

    let mut sent = 0;
    let mut idle = 0;
    let mut phase = String::new();
    loop {
        match master.state() {
            TxState::Done => break,
            TxState::Failed(s) => {
                println!("transfer failed: {s:?}");
                return;
            }
            s => {
                let name = format!("{s:?}");
                if name != phase {
                    let (done, total) = master.progress();
                    println!("  {name:<10} {done:>4}/{total} bytes acknowledged");
                    phase = name;
                }
            }
        }

        // The master emits frames; when it is waiting it emits nothing, and
        // firmware would resend on a timeout. Here idle ticks stand in.
        let frame = match master.next_frame() {
            Some(f) => {
                idle = 0;
                f
            }
            None => {
                idle += 1;
                assert!(idle < 10, "stalled");
                match master.resend() {
                    Some(f) => f,
                    None => continue,
                }
            }
        };
        sent += 1;

        if lost.contains(&sent) {
            println!("  ---- frame {sent} lost on the bus ----");
            continue;
        }

        // Every node sees every frame and answers only when addressed.
        for node in nodes.iter_mut() {
            if let Some(reply) = node.feed(&frame) {
                master.feed_reply(&reply);
            }
        }
    }

    println!("\ntransfer complete in {sent} frames");
    for (addr, node) in [7, 9].iter().zip(nodes.iter_mut()) {
        let sink = node.sink();
        println!(
            "  node {addr}: {} bytes staged, committed {}, matches source {}",
            sink.len,
            sink.committed,
            sink.buf[..image.len()] == image[..]
        );
    }

    // A node that is not addressed stays untouched, so a single display can
    // be updated without disturbing the others.
    println!("\ntargeting node 7 only");
    let mut b7 = [0u8; 256];
    let mut b9 = [0u8; 256];
    let mut only = [Receiver::new(7, BufferSink::new(&mut b7)), Receiver::new(9, BufferSink::new(&mut b9))];
    let mut m = Sender::new(7, &image, 1, 16, 1);
    while !matches!(m.state(), TxState::Done | TxState::Failed(_)) {
        if let Some(f) = m.next_frame() {
            for n in only.iter_mut() {
                if let Some(r) = n.feed(&f) {
                    m.feed_reply(&r);
                }
            }
        }
    }
    println!("  node 7 committed: {}", only[0].sink().committed);
    let idle_node = only[1].sink();
    println!("  node 9 committed: {}  ({} bytes seen)", idle_node.committed, idle_node.len);

    // A corrupted image never reaches commit.
    println!("\ncorrupted image");
    let mut bad = [0u8; 256];
    let mut node = Receiver::new(7, BufferSink::new(&mut bad));
    let mut m = Sender::new(7, &image, 1, 16, 1);
    while !matches!(m.state(), TxState::Done | TxState::Failed(_)) {
        if let Some(f) = m.next_frame() {
            let f = flip_a_bit(&f);
            if let Some(r) = node.feed(&f) {
                m.feed_reply(&r);
            }
        }
    }
    println!("  master: {:?}", m.state());
    println!("  node:   {:?}, committed {}", node.state(), node.sink().committed);
}

/// Corrupt one byte of the third data block, as line noise would.
fn flip_a_bit(f: &mcp2515::frame::CanFrame) -> mcp2515::frame::CanFrame {
    use embedded_hal_0_2::can::Frame;
    if protocol::id_of(f) == Some(DATA_ID) && f.data()[0] == 3 {
        let mut d = f.data()[..f.dlc()].to_vec();
        d[2] ^= 0x20;
        return protocol::frame(DATA_ID, &d).unwrap();
    }
    *f
}
