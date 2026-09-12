// SPDX-License-Identifier: GPL-3.0-only
//! The display bus end to end: a client subscribes, the server broadcasts,
//! the client renders. This is what RPSERVER and RPCLIENT do in firmware.
//!
//!     cargo run --example display_bus

use cogware_can::subscribe::{Publisher, Subscription};
use cogware_can::*;
use mcp2515::frame::CanFrame;

/// What one display wants to show. Firmware hard-codes its own list.
const WANTED: [u8; 6] = [
    Gauge::Rpm.id() as u8,
    Gauge::Map.id() as u8,
    Gauge::Clnt.id() as u8,
    Gauge::AfrPri.id() as u8,
    Gauge::BatVol.id() as u8,
    Gauge::Vss.id() as u8,
];

fn main() {
    println!("ID map");
    println!("  {:#05x}..={:#05x}  bus control (ack, subscribe, file transfer)", protocol::RESERVED_ID_MIN, protocol::RESERVED_ID_MAX);
    println!("  {:#05x}..={:#05x}  gauges", protocol::GAUGE_ID_MIN, protocol::GAUGE_ID_MAX);
    println!("  {:#05x}..=0x7ff  left clear for OBD2\n", obd2::REQUEST_ID);

    // ---- Subscription. The display asks for the gauge IDs it shows and
    // the master records and acknowledges them. Both halves are crate types,
    // so this is the same code the RPSERVER and RPCLIENT firmware runs.
    let mut master = Publisher::new();
    let mut display = Subscription::new(&WANTED);

    let mut rounds = 0;
    while !display.is_complete() {
        rounds += 1;
        for request in display.request() {
            for ack in master.feed(&request) {
                display.feed(&ack);
            }
        }
    }
    println!("display subscribed in {rounds} round(s): {:?}",
        display.wanted().iter().map(|i| Gauge::from_id(*i as u16).unwrap().data().name).collect::<Vec<_>>());
    println!("master is now publishing {} gauges\n", master.len());

    // ---- Server side. An ECU packet fills the table, then each subscribed
    // gauge becomes one frame.
    clear_all();
    RPM.set(3500);
    MAP.set(950);
    CLNT.set(870);
    AFR_PRI.set(1320);
    BAT_VOL.set(13_800);
    VSS.set(880);

    let broadcast: Vec<CanFrame> = master.broadcast().collect();
    println!("server built {} frames\n", broadcast.len());

    // ---- Client side. It has its own gauge table, filled only by the bus.
    clear_all();
    for f in &broadcast {
        match feed_frame(f) {
            Ok(g) => println!("  got {:<8} = {:>8.2} {}", g.name, g.as_f32().unwrap(), g.unit.symbol),
            Err(e) => println!("  rejected: {e:?}"),
        }
    }

    // ---- Render. The display picks whatever units it likes.
    println!("\n  ┌──────────────────────────┐");
    println!("  │ RPM   {:>6.0}  AFR {:>6.2} │", RPM.as_f32().unwrap(), AFR_PRI.as_f32().unwrap());
    println!("  │ BOOST {:>6.1}  CLT {:>5.0}° │", MAP.to(Unit::PSI).unwrap() - 14.7, CLNT.to(Unit::FAHRENHEIT).unwrap());
    println!("  │ MPH   {:>6.1}  BAT {:>6.2} │", VSS.to(Unit::MPH).unwrap(), BAT_VOL.as_f32().unwrap());
    println!("  └──────────────────────────┘");

    // ---- Anything else on the bus is rejected, never decoded by accident.
    println!("\nframes the display ignores");
    for (what, f) in [
        ("an ack", protocol::ack_frame(0x2D).unwrap()),
        ("a subscribe", protocol::request_frame(&[0x2D]).unwrap()),
        ("an OBD2 request", obd2::request(0x0C).unwrap()),
        ("a wrong-length gauge", protocol::frame(Gauge::Rpm.id(), &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap()),
    ] {
        println!("  {what:<22} {:?}", feed_frame(&f).err().unwrap());
    }
    println!("\nRPM survived the bad frame: {:?}", RPM.get());
}
